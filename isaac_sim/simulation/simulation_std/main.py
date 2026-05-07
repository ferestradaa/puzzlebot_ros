"""
puzzlebot_sim.py  —  Isaac Sim 5.1 standalone para Puzzlebot
─────────────────────────────────────────────────────────────
Uso:
    ./python.sh puzzlebot_sim.py [--headless]

Variables de entorno:
    URDF_PATH   ruta al .urdf del puzzlebot
    USD_OUTPUT  ruta donde guardar el USD generado (se sobreescribe cada run)
"""

# ─── 0. BOOTSTRAP ────────────────────────────────────────────────────────────
import argparse, os, sys

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": args.headless,
    "width":    1280,
    "height":   720,
    "renderer": "RaytracedLighting",
})

import omni.usd
import omni.kit.commands
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf

# ─── 1. EXTENSIONES ──────────────────────────────────────────────────────────
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")

from isaacsim.asset.importer.urdf import _urdf

simulation_app.update()

# ─── 2. RUTAS ─────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", ".."))

URDF_PATH  = os.environ.get(
    "URDF_PATH",
    os.path.join(ROOT_DIR, "assets", "puzzlebot", "urdf", "puzzlebot.urdf")
)
USD_OUTPUT = os.environ.get(
    "USD_OUTPUT",
    os.path.join(_SCRIPT_DIR, "generated", "puzzlebot_v2.usd")
)
os.makedirs(os.path.dirname(USD_OUTPUT), exist_ok=True)

# ─── Constantes del robot (verificadas con debug) ─────────────────────────────
ROBOT_PRIM_PATH   = "/World/puzzlebot"
ARTICULATION_PATH = "/World/puzzlebot/base_footprint"  # tiene PhysicsArticulationRootAPI

CAMERA_LINK_PATH  = f"{ROBOT_PRIM_PATH}/camera_link"
LIDAR_LINK_PATH   = f"{ROBOT_PRIM_PATH}/lidar_link"

# Paths de sensores — Camera se crea en setup_sensors(), LiDAR lo crea
# IsaacSensorCreateRtxLidar y queda en lidar_link/Lidar/RPLidar_S2E (OmniLidar)
CAMERA_PRIM_PATH  = f"{CAMERA_LINK_PATH}/Camera"
LIDAR_SENSOR_PATH = f"{LIDAR_LINK_PATH}/Lidar/RPLidar_S2E"   # OmniLidar real

# Del URDF: wheel radius=0.051, base=2×0.09=0.18
WHEEL_RADIUS = 0.051
WHEEL_BASE   = 0.18

GRAPH_PATH = "/ActionGraph/ROS2"

# ─── 3. IMPORTAR URDF → USD ───────────────────────────────────────────────────
def import_urdf_to_usd(urdf_path: str, usd_output: str) -> str:
    importer = _urdf.acquire_urdf_interface()
    config   = _urdf.ImportConfig()

    config.set_fix_base(False)
    config.set_merge_fixed_joints(False)
    config.set_self_collision(False)
    config.set_import_inertia_tensor(True)
    config.set_create_physics_scene(False)
    config.set_default_drive_type(2)           # velocity drive
    config.set_default_drive_strength(1047198.0)
    config.set_default_position_drive_damping(1e7)
    config.set_distance_scale(1.0)
    config.set_density(0.0)
    config.set_convex_decomp(False)
    config.set_make_default_prim(True)

    urdf_dir      = os.path.dirname(os.path.abspath(urdf_path))
    urdf_filename = os.path.basename(urdf_path)

    parsed = importer.parse_urdf(urdf_dir, urdf_filename, config)
    result = importer.import_robot(urdf_dir, urdf_filename, parsed, config, usd_output)

    if not result:
        raise RuntimeError(f"Falló import_robot: {urdf_path}")

    print(f"[sim] URDF importado → {usd_output}  (prim raíz: {result})")
    return usd_output


# ─── 4. MUNDO ─────────────────────────────────────────────────────────────────
def setup_world() -> World:
    world = World(
        stage_units_in_meters = 1.0,
        physics_dt            = 1.0 / 60.0,
        rendering_dt          = 1.0 / 30.0,
    )
    world.scene.add_default_ground_plane()
    return world


# ─── 5. CARGAR ROBOT ──────────────────────────────────────────────────────────
def load_robot(usd_path: str):
    add_reference_to_stage(usd_path=usd_path, prim_path=ROBOT_PRIM_PATH)
    stage = omni.usd.get_context().get_stage()
    prim  = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    xform = UsdGeom.XformCommonAPI(prim)
    xform.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.05))


# ─── 6. SENSORES ─────────────────────────────────────────────────────────────
# El URDF solo define links físicos — los sensores se crean aquí.
#
# Cámara : prim tipo Camera en camera_link/Camera
# LiDAR  : OmniLidar creado con IsaacSensorCreateRtxLidar
#           queda en lidar_link/Lidar/RPLidar_S2E

def setup_sensors() -> bool:
    import omni.usd as _usd
    from pxr import UsdGeom, Gf

    stage = _usd.get_context().get_stage()

    # Verificar links
    for path in [CAMERA_LINK_PATH, LIDAR_LINK_PATH]:
        if not stage.GetPrimAtPath(path).IsValid():
            print(f"[sim] ERROR: link no encontrado: {path}")
            print("[sim] Prims del robot:")
            for p in stage.Traverse():
                sp = str(p.GetPath())
                if ROBOT_PRIM_PATH in sp:
                    print(f"  {sp}  [{p.GetTypeName()}]")
            return False

    # ── Cámara ───────────────────────────────────────────────────────────────
    # Parámetros del USDA original: focalLength=30.4, aperture 36.8×27.6
    camera_prim = stage.DefinePrim(CAMERA_PRIM_PATH, "Camera")
    camera_prim.GetAttribute("focalLength").Set(30.4)
    camera_prim.GetAttribute("horizontalAperture").Set(36.8)
    camera_prim.GetAttribute("verticalAperture").Set(27.6)
    camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.05, 50.0))
    # Orientación: Z-forward (Isaac) → eje óptico ROS (Z-forward, Y-down)
    cam_xform = UsdGeom.XformCommonAPI(camera_prim)
    cam_xform.SetRotate(
        Gf.Vec3f(-90.0, 0.0, -90.0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    print(f"[sim] Camera prim creado: {CAMERA_PRIM_PATH}")

    # ── LiDAR RTX ─────────────────────────────────────────────────────────────
    # path = nombre relativo (no path completo), parent = path completo del padre
    # Resultado: parent + "/Lidar" → luego el sensor RTX queda en Lidar/RPLidar_S2E
    ok, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path        = "/Lidar",
        parent      = LIDAR_LINK_PATH,
        config      = "RPLIDAR_S2E",
        translation = Gf.Vec3d(0.0, 0.0, 0.0),
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if ok and sensor is not None:
        print(f"[sim] LiDAR RTX creado. Sensor OmniLidar: {LIDAR_SENSOR_PATH}")
    else:
        print(f"[sim] ADVERTENCIA: IsaacSensorCreateRtxLidar falló — LiDAR no disponible")
        # El render product del LiDAR fallará también, pero el resto del script sigue
    return True


# ─── 7. RENDER PRODUCTS ───────────────────────────────────────────────────────
_rp_camera = None
_rp_lidar  = None

def setup_render_products():
    global _rp_camera, _rp_lidar
    import omni.replicator.core as rep

    _rp_camera = rep.create.render_product(CAMERA_PRIM_PATH, (640, 480))
    print(f"[sim] RenderProduct cámara: {_rp_camera.path}")

    # El OmniLidar es el sensor real — el render product se adjunta a ese prim
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(LIDAR_SENSOR_PATH).IsValid():
        _rp_lidar = rep.create.render_product(LIDAR_SENSOR_PATH, (1280, 720))
        print(f"[sim] RenderProduct LiDAR : {_rp_lidar.path}")
    else:
        print(f"[sim] ADVERTENCIA: {LIDAR_SENSOR_PATH} no existe — LiDAR sin render product")


# ─── 8. ACTION GRAPH ROS2 ────────────────────────────────────────────────────
#
# Paths verificados con debug:
#   ArticulationRoot : /World/puzzlebot/base_footprint
#   OmniLidar        : /World/puzzlebot/lidar_link/Lidar/RPLidar_S2E
#
# Nodos incluidos:
#   /odom            ← IsaacComputeOdometry → ROS2PublishOdometry
#   /joint_states    ← ROS2PublishJointState
#   /cmd_vel         → ROS2SubscribeTwist (leído en Python loop → diff drive)
#   /camera/image_raw ← ROS2CameraHelper (type="rgb")
#   /scan            ← ROS2RtxLidarHelper (type="laser_scan")
#   /tf              ← ROS2PublishTransformTree
#   /clock           ← ROS2PublishClock
#
# camera_info: ROS2CameraHelper no acepta type="camera_info" en 5.1
# Se omite por ahora — agregar cuando se confirme el nodo correcto.

def setup_ros2_graph() -> bool:
    rp_cam = _rp_camera.path if _rp_camera else ""
    rp_lid = _rp_lidar.path  if _rp_lidar  else ""

    nodes = [
        ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime",       "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ROS2Context",       "isaacsim.ros2.bridge.ROS2Context"),
        ("ComputeOdom",       "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("OdomPublisher",     "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("DiffDrive",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("ArticCtrl",         "isaacsim.core.nodes.IsaacArticulationController"),
        ("CameraHelper",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PublishTF",         "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("PublishClock",      "isaacsim.ros2.bridge.ROS2PublishClock"),
    ]
    if rp_lid:
        nodes.append(("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))

    set_values = [
        ("ROS2Context.inputs:useDomainIDEnvVar",     True),

        # Odometría — ArticulationRoot correcto
        ("ComputeOdom.inputs:chassisPrim",           [ARTICULATION_PATH]),
        ("OdomPublisher.inputs:topicName",           "odom"),
        ("OdomPublisher.inputs:odomFrameId",         "odom"),
        ("OdomPublisher.inputs:chassisFrameId",      "base_link"),

        # Joint states
        ("PublishJointState.inputs:topicName",       "joint_states"),
        ("PublishJointState.inputs:targetPrim",      [ARTICULATION_PATH]),

        # Cmd vel
        ("DiffDrive.inputs:topicName",               "cmd_vel"),

        # Articulation controller — ArticulationRoot correcto
        ("ArticCtrl.inputs:targetPrim",              [ARTICULATION_PATH]),

        # Cámara RGB
        ("CameraHelper.inputs:topicName",            "camera/image_raw"),
        ("CameraHelper.inputs:type",                 "rgb"),
        ("CameraHelper.inputs:frameId",              "camera_color_optical_frame"),
        ("CameraHelper.inputs:renderProductPath",    rp_cam),

        # TF
        ("PublishTF.inputs:topicName",               "tf"),
        ("PublishTF.inputs:targetPrims",             [ROBOT_PRIM_PATH]),

        # Clock
        ("PublishClock.inputs:topicName",            "clock"),
    ]
    if rp_lid:
        set_values += [
            ("LidarHelper.inputs:topicName",         "scan"),
            ("LidarHelper.inputs:type",              "laser_scan"),
            ("LidarHelper.inputs:frameId",           "lidar_link"),
            ("LidarHelper.inputs:renderProductPath", rp_lid),
            ("LidarHelper.inputs:fullScan",          True),
        ]

    connect = [
        # Tick → exec
        ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "DiffDrive.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticCtrl.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        # ComputeOdom exec chain
        ("ComputeOdom.outputs:execOut", "OdomPublisher.inputs:execIn"),
        # SimTime → timestamps
        ("ReadSimTime.outputs:simulationTime", "OdomPublisher.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        # Context → nodos ROS2
        ("ROS2Context.outputs:context", "OdomPublisher.inputs:context"),
        ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
        ("ROS2Context.outputs:context", "DiffDrive.inputs:context"),
        ("ROS2Context.outputs:context", "CameraHelper.inputs:context"),
        ("ROS2Context.outputs:context", "PublishTF.inputs:context"),
        ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
        # ComputeOdom → OdomPublisher data
        ("ComputeOdom.outputs:position",        "OdomPublisher.inputs:position"),
        ("ComputeOdom.outputs:orientation",     "OdomPublisher.inputs:orientation"),
        ("ComputeOdom.outputs:linearVelocity",  "OdomPublisher.inputs:linearVelocity"),
        ("ComputeOdom.outputs:angularVelocity", "OdomPublisher.inputs:angularVelocity"),
    ]
    if rp_lid:
        connect += [
            ("OnPlaybackTick.outputs:tick",     "LidarHelper.inputs:execIn"),
            ("ROS2Context.outputs:context",     "LidarHelper.inputs:context"),
        ]

    (ok, _, _, _) = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.SET_VALUES:   set_values,
            og.Controller.Keys.CONNECT:      connect,
        }
    )

    if not ok:
        print("[sim] ERROR: Falló la creación del Action Graph ROS2")
        return False

    print("[sim] Action Graph ROS2 OK")
    return True


# ─── 9. DIFFERENTIAL DRIVE (Python loop) ─────────────────────────────────────
# ROS2SubscribeTwist expone los outputs en el graph.
# Los leemos desde Python y calculamos velocidades de rueda.

_twist = {"v": 0.0, "w": 0.0}

def _read_twist_from_graph():
    try:
        lin = og.Controller.attribute(
            f"{GRAPH_PATH}/DiffDrive.outputs:linearVelocity"
        ).get()
        ang = og.Controller.attribute(
            f"{GRAPH_PATH}/DiffDrive.outputs:angularVelocity"
        ).get()
        _twist["v"] = float(lin[0]) if lin is not None and len(lin) > 0 else 0.0
        _twist["w"] = float(ang[2]) if ang is not None and len(ang) > 2 else 0.0
    except Exception:
        pass


def _apply_diff_drive(articulation_controller):
    from isaacsim.core.utils.types import ArticulationAction
    import numpy as np

    v = _twist["v"]
    w = _twist["w"]
    v_left  = (v - w * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    v_right = (v + w * WHEEL_BASE / 2.0) / WHEEL_RADIUS

    articulation_controller.apply_action(
        ArticulationAction(
            joint_velocities=np.array([v_left, v_right])
        )
    )


# ─── 10. MAIN ─────────────────────────────────────────────────────────────────
def main():
    print("[sim] Iniciando puzzlebot_sim...")

    if not os.path.exists(URDF_PATH):
        raise FileNotFoundError(f"URDF no encontrado: {URDF_PATH}")

    # URDF → USD
    usd_path = import_urdf_to_usd(URDF_PATH, USD_OUTPUT)

    # Mundo + robot
    world = setup_world()
    load_robot(usd_path)
    simulation_app.update()

    # Sensores (Camera prim + LiDAR RTX)
    if not setup_sensors():
        simulation_app.close()
        return
    simulation_app.update()

    # Render products
    setup_render_products()

    # Action Graph ROS2
    if not setup_ros2_graph():
        simulation_app.close()
        return

    # Primer reset
    world.reset()
    simulation_app.update()

    # Robot wrapper — usa el path de ArticulationRoot
    from isaacsim.core.api.robots import Robot
    robot = Robot(prim_path=ARTICULATION_PATH, name="puzzlebot")
    world.scene.add(robot)
    world.reset()

    articulation = robot.get_articulation_controller()

    print(f"[sim] DOF names: {robot.dof_names}")
    print(f"[sim] Simulación corriendo.")
    print(f"      URDF  : {URDF_PATH}")
    print(f"      USD   : {USD_OUTPUT}")
    print(f"      Pub   : /odom  /joint_states  /tf  /clock  /camera/image_raw  /scan")
    print(f"      Sub   : /cmd_vel")

    # ─── LOOP ────────────────────────────────────────────────────────────────
    step = 0
    while simulation_app.is_running():
        world.step(render=True)
        _read_twist_from_graph()
        _apply_diff_drive(articulation)

        if step % 300 == 0:
            print(f"[sim] step={step}  v={_twist['v']:.3f}  w={_twist['w']:.3f}")

        step += 1

    print("[sim] Cerrando...")
    simulation_app.close()


if __name__ == "__main__":
    main()