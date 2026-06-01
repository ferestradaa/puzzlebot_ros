#!/usr/bin/env python3
import heapq
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class SmoothPlanner(Node):
    def __init__(self):
        super().__init__('smooth_planner')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame',  'odom')
        self.declare_parameter('map_topic',    '/map')
        self.declare_parameter('robot_radius', 0.20)

        self.map: OccupancyGrid | None = None
        self.grid: np.ndarray | None = None  # bool array, True = ocupado
        self.inflate_pad: int = 0

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_topic = self.get_parameter('map_topic').value
        map_qos   = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_sub  = self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)
        self.path_pub = self.create_publisher(Path, 'smooth_path', 10)

    # ---------------------------------------------------------------------------
    # mapa
    # ---------------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg
        w, h     = msg.info.width, msg.info.height
        data     = np.array(msg.data, dtype=np.int8).reshape(h, w)
        # ocupado si >50 o desconocido (-1)
        self.grid = (data > 50) | (data < 0)
        r = self.get_parameter('robot_radius').value
        self.inflate_pad = int(math.ceil(r / msg.info.resolution))
        self.get_logger().info(f'mapa recibido {w}x{h}, pad={self.inflate_pad}')

    def _world_to_cell(self, wx: float, wy: float):
        info = self.map.info
        cx = int((wx - info.origin.position.x) / info.resolution)
        cy = int((wy - info.origin.position.y) / info.resolution)
        return cx, cy

    def _cell_to_world(self, cx: int, cy: int):
        info = self.map.info
        wx = info.origin.position.x + (cx + 0.5) * info.resolution
        wy = info.origin.position.y + (cy + 0.5) * info.resolution
        return wx, wy

    def _in_bounds(self, cx: int, cy: int) -> bool:
        h, w = self.grid.shape
        return 0 <= cx < w and 0 <= cy < h

    def _is_occupied(self, cx: int, cy: int) -> bool:
        if not self._in_bounds(cx, cy):
            return True
        return bool(self.grid[cy, cx])

    def _is_occupied_inflated(self, cx: int, cy: int) -> bool:
        pad = self.inflate_pad
        h, w = self.grid.shape
        x0 = max(cx - pad, 0);  x1 = min(cx + pad + 1, w)
        y0 = max(cy - pad, 0);  y1 = min(cy + pad + 1, h)
        patch = self.grid[y0:y1, x0:x1]
        # máscara circular
        ys, xs = np.mgrid[y0:y1, x0:x1]
        mask   = (xs - cx)**2 + (ys - cy)**2 <= pad**2
        return bool(patch[mask[: patch.shape[0], : patch.shape[1]]].any())

    def _line_of_sight(self, ax: int, ay: int, bx: int, by: int) -> bool:
        # Bresenham con grid inflado
        dx = abs(bx - ax); dy = abs(by - ay)
        sx = 1 if ax < bx else -1
        sy = 1 if ay < by else -1
        err = dx - dy
        x, y = ax, ay
        while True:
            if self._is_occupied_inflated(x, y):
                return False
            if x == bx and y == by:
                return True
            e2 = 2 * err
            if e2 > -dy: err -= dy; x += sx
            if e2 <  dx: err += dx; y += sy

    # ---------------------------------------------------------------------------
    # Theta*
    # ---------------------------------------------------------------------------

    def _theta_star(self, start, goal):
        sx, sy = start
        gx, gy = goal

        def h(x, y):
            return math.hypot(x - gx, y - gy)

        g_cost = {start: 0.0}
        parent = {start: start}
        open_  = [(h(sx, sy), start)]

        neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

        while open_:
            _, curr = heapq.heappop(open_)
            if curr == goal:
                path = []
                c = goal
                while c != parent[c]:
                    path.append(c)
                    c = parent[c]
                path.append(start)
                path.reverse()
                return path

            cx, cy = curr
            px, py = parent[curr]

            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                nb = (nx, ny)
                if not self._in_bounds(nx, ny) or self._is_occupied_inflated(nx, ny):
                    continue

                # path 2: line-of-sight desde el padre del nodo actual
                if self._line_of_sight(px, py, nx, ny):
                    ng = g_cost[parent[curr]] + math.hypot(nx - px, ny - py)
                    if ng < g_cost.get(nb, math.inf):
                        g_cost[nb] = ng
                        parent[nb] = parent[curr]
                        heapq.heappush(open_, (ng + h(nx, ny), nb))
                else:
                    # path 1: A* normal
                    ng = g_cost[curr] + math.hypot(dx, dy)
                    if ng < g_cost.get(nb, math.inf):
                        g_cost[nb] = ng
                        parent[nb] = curr
                        heapq.heappush(open_, (ng + h(nx, ny), nb))

        return []

    # ---------------------------------------------------------------------------
    # shortcut pruning
    # ---------------------------------------------------------------------------

    def _prune(self, path):
        if len(path) < 3:
            return path
        pruned = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            ax, ay = path[i]
            while j > i + 1 and not self._line_of_sight(ax, ay, path[j][0], path[j][1]):
                j -= 1
            pruned.append(path[j])
            i = j
        return pruned

    # ---------------------------------------------------------------------------
    # B-spline cúbico uniforme
    # ---------------------------------------------------------------------------

    def _bspline(self, pts, density=15):
        if len(pts) < 4:
            return pts
        # puntos fantasma en extremos
        cp = [pts[0]] + pts + [pts[-1]]
        result = []

        for i in range(1, len(cp) - 2):
            p0 = np.array(cp[i-1])
            p1 = np.array(cp[i])
            p2 = np.array(cp[i+1])
            p3 = np.array(cp[i+2])

            for k in range(density):
                t  = k / density
                t2 = t * t
                t3 = t2 * t
                b0 = (-t3 + 3*t2 - 3*t + 1) / 6.0
                b1 = ( 3*t3 - 6*t2      + 4) / 6.0
                b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6.0
                b3 = ( t3                    ) / 6.0
                result.append(b0*p0 + b1*p1 + b2*p2 + b3*p3)

        result.append(np.array(pts[-1]))
        return result

    # ---------------------------------------------------------------------------
    # goal callback
    # ---------------------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped):
        if self.map is None:
            self.get_logger().warn('sin mapa todavia')
            return

        global_frame = self.get_parameter('global_frame').value
        robot_frame  = self.get_parameter('robot_frame').value

        try:
            tf = self.tf_buffer.lookup_transform(global_frame, robot_frame, rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except TransformException as e:
            self.get_logger().error(f'tf: {e}')
            return

        start = self._world_to_cell(rx, ry)
        goal  = self._world_to_cell(msg.pose.position.x, msg.pose.position.y)

        if self._is_occupied_inflated(*start):
            self.get_logger().warn('start en obstaculo')
            return
        if self._is_occupied_inflated(*goal):
            self.get_logger().warn('goal en obstaculo')
            return

        raw    = self._theta_star(start, goal)
        if not raw:
            self.get_logger().warn('theta* sin solucion')
            return

        pruned = self._prune(raw)
        world  = [self._cell_to_world(cx, cy) for cx, cy in pruned]
        smooth = self._bspline(world, density=15)

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = global_frame

        for wx, wy in smooth:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'path: {len(raw)} raw -> {len(pruned)} pruned -> {len(smooth)} smooth')


def main():
    rclpy.init()
    rclpy.spin(SmoothPlanner())
    rclpy.shutdown()


if __name__ == '__main__':
    main()