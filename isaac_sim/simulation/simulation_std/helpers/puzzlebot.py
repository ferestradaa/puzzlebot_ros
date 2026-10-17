from isaacsim.core.utils.types import ArticulationAction
import numpy as np
from isaacsim.asset.importer.urdf import _urdf
import os
import omni.usd as _usd
import omni.kit.commands
from pxr import UsdGeom, Gf
import omni.replicator.core as rep
import omni.usd
import omni.graph.core as og
from pxr import UsdPhysics
from pxr import PhysxSchema
from .textures import apply_realistic_textures


class Puzzlebot():
    def __init__(self, robot_prim_path, articulation_path,
                wheel_radius, wheel_base, camera_link_path,
                lidar_link_path, camera_prim_path, lidar_sensor_path,
                graph_path):
        
        self.robot_prim_path = robot_prim_path
        self.articulation_path = articulation_path 
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.camera_link_path = camera_link_path
        self.lidar_link_path = lidar_link_path
        self.camera_prim_path = camera_prim_path
        self.lidar_sensor_path = lidar_sensor_path
        self.graph_path = graph_path

        self._rp_camera = None
        self._rp_lidar  = None
        self._twist = {"v": 0.0, "w": 0.0}

        self.ANGULAR_GAIN = 5.0

        # Stages in the order they extend: carriage (forks) rides up the inner mast first,
        # then the inner mast lifts out of the outer one. Travel comes from the URDF limits.
        self.MAST_JOINTS = ["forks_joint", "inner_mast_joint"]
        self.MAST_MIN = -np.inf
        self.MAST_MAX = np.inf
        self._mast_target = 0.0
        self._mast_idx = None
        self._mast_lo = None
        self._mast_hi = None
        self.MAST_STIFFNESS = 1000.0
        self.MAST_DAMPING = 40.0

    
    def import_urdf_to_usd(self, urdf_path: str, usd_output: str) -> str:

        importer = _urdf.acquire_urdf_interface()
        config   = _urdf.ImportConfig()

        config.set_fix_base(False)
        config.set_merge_fixed_joints(False)
        config.set_self_collision(False)
        config.set_import_inertia_tensor(True)
        config.set_create_physics_scene(False)
        config.set_default_drive_type(2)           # velocity drive
        config.set_default_drive_strength(10.0)
        config.set_default_position_drive_damping(0.0)
        config.set_distance_scale(1.0)
        config.set_density(0.0)
        config.set_convex_decomp(False)
        config.set_make_default_prim(True)

        urdf_dir      = os.path.dirname(os.path.abspath(urdf_path))
        urdf_filename = os.path.basename(urdf_path)

        parsed = importer.parse_urdf(urdf_dir, urdf_filename, config)
        result = importer.import_robot(urdf_dir, urdf_filename, parsed, config, usd_output)

        if not result:
            raise RuntimeError(f"failed import_robot: {urdf_path}")
        from pxr import Usd, UsdGeom
        stage = Usd.Stage.Open(usd_output)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.GetRootLayer().Save()

        print(f"[sim] URDF imported as {usd_output}  (prim root: {result})")
        return usd_output

        

    def fix_wheel_drives_live(self):
        stage = omni.usd.get_context().get_stage()

        wheel_links = [
            f"{self.robot_prim_path}/left_wheel_link",
            f"{self.robot_prim_path}/right_wheel_link",
        ]
        for wp in wheel_links:
            prim = stage.GetPrimAtPath(wp)
            if not prim.IsValid():
                print(f"[sim] wheel link not found: {wp}")
                continue
            rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb_api.CreateMaxAngularVelocityAttr().Set(50.0)
            rb_api.CreateMaxLinearVelocityAttr().Set(1000.0)
            max_ang = rb_api.GetMaxAngularVelocityAttr().Get()
            print(f"[sim] {wp} maxAngularVelocity = {max_ang}")

        joints = [
            f"{self.robot_prim_path}/joints/base_to_left_wheel",
            f"{self.robot_prim_path}/joints/base_to_right_wheel",
        ]
        for joint_path in joints:
            prim = stage.GetPrimAtPath(joint_path)
            if not prim.IsValid():
                print(f"[sim] Joint NOT found {joint_path}")
                continue

            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.GetTypeAttr().Set("velocity")
            drive.GetMaxForceAttr().Set(1000.0)
            drive.GetDampingAttr().Set(20000)
            drive.GetStiffnessAttr().Set(0.0)

            physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
            physx_joint.CreateJointFrictionAttr().Set(0.0)
            physx_joint.CreateMaxJointVelocityAttr().Set(1000.0)

    def fix_mast_drive_live(self, stiffness=1000.0, damping=40.0, max_force=50.0):
        """Position drives for every prismatic stage of the forklift mast (see MAST_JOINTS)."""
        stage = omni.usd.get_context().get_stage()
        for name in self.MAST_JOINTS:
            prim = stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/{name}")
            if not prim.IsValid() or not prim.IsA(UsdPhysics.PrismaticJoint):
                print(f"[sim] {name} not a prismatic joint, skipping mast drive")
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            drive.GetTypeAttr().Set("force")
            drive.GetStiffnessAttr().Set(stiffness)
            drive.GetDampingAttr().Set(damping)
            drive.GetMaxForceAttr().Set(max_force)
            drive.GetTargetPositionAttr().Set(0.0)
            drive.GetTargetVelocityAttr().Set(0.0)
            # the URDF friction (0.5) is a coefficient on the constraint force of the cantilevered
            # load, which exceeds what the drive can push, so the stage would stall
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr().Set(0.0)

    def fix_fork_collision_live(self, links=("forks_link",)):
        """Turn off collision on the visual mesh of the given links.

        The URDF importer leaves PhysicsCollisionAPI on <link>/visuals as well as on
        <link>/collisions. For the forks the visual mesh then cooks to a convex hull that
        fills the gap between the two tines, so the fork cannot enter a pallet's slots and
        pushes it instead. The real tine boxes live in <link>/collisions and are kept.
        A link is only touched if it also has a <link>/collisions prim to fall back on."""
        stage = omni.usd.get_context().get_stage()
        for link in links:
            visuals = stage.GetPrimAtPath(f"{self.robot_prim_path}/{link}/visuals")
            collisions = stage.GetPrimAtPath(f"{self.robot_prim_path}/{link}/collisions")
            if not visuals.IsValid() or not collisions.IsValid():
                print(f"[sim] {link}: visuals/collisions not found, collision left as is")
                continue
            UsdPhysics.CollisionAPI.Apply(visuals).GetCollisionEnabledAttr().Set(False)
            print(f"[sim] collision disabled on {visuals.GetPath()}")

    def apply_textures(self):
        apply_realistic_textures(self.robot_prim_path)

    def fix_caster_wheel(self):
        from pxr import UsdShade, UsdPhysics, PhysxSchema
        stage = omni.usd.get_context().get_stage()

        stage.DefinePrim("/World/PhysicsMaterials", "Scope")  # idempotente si ya existe
        mat_prim = stage.DefinePrim("/World/PhysicsMaterials/CasterMat", "Material")
        mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
        mat.CreateStaticFrictionAttr().Set(0.0)
        mat.CreateDynamicFrictionAttr().Set(0.0)
        mat.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat_prim).CreateFrictionCombineModeAttr().Set("min")

        caster_prim = stage.GetPrimAtPath(f"{self.robot_prim_path}/caster_wheel_link")
        if not caster_prim.IsValid():
            print(f"[sim] caster_wheel_link no encontrado")
            return

        binding = UsdShade.MaterialBindingAPI.Apply(caster_prim)
        binding.Bind(
            UsdShade.Material(mat_prim),
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics"
        )
        print(f"[sim] caster friction=0 binded in: {caster_prim.GetPath()}")

        joint_prim = stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/chassis_to_caster")
        if joint_prim.IsValid():
            PhysxSchema.PhysxJointAPI.Apply(joint_prim).CreateJointFrictionAttr().Set(0.0)



    def fix_wheel_friction(self):
        from pxr import UsdShade, UsdPhysics, PhysxSchema
        stage = omni.usd.get_context().get_stage()

        stage.DefinePrim("/World/PhysicsMaterials", "Scope")
        mat_prim = stage.DefinePrim("/World/PhysicsMaterials/WheelMat", "Material")
        mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
        mat.CreateStaticFrictionAttr().Set(1.5)
        mat.CreateDynamicFrictionAttr().Set(1.5)
        mat.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat_prim).CreateFrictionCombineModeAttr().Set("multiply")

        wheel_links = [
            f"{self.robot_prim_path}/left_wheel_link",
            f"{self.robot_prim_path}/right_wheel_link",
        ]
        for wp in wheel_links:
            prim = stage.GetPrimAtPath(wp)
            if not prim.IsValid():
                print(f"[sim] wheel link not found: {wp}")
                continue
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material(mat_prim),
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics"
            )
            print(f"[sim] WheelMat binded to: {wp}")
            
    def tune_solver(self):
        from pxr import PhysxSchema
        stage = omni.usd.get_context().get_stage()

        root = stage.GetPrimAtPath(self.articulation_path)
        art = PhysxSchema.PhysxArticulationAPI.Apply(root)
        art.CreateSolverPositionIterationCountAttr().Set(32)   
        art.CreateSolverVelocityIterationCountAttr().Set(4)
        art.CreateSleepThresholdAttr().Set(0.0)               
        art.CreateStabilizationThresholdAttr().Set(0.0)       

        bodies = [
            f"{self.robot_prim_path}/chassis_link",
            f"{self.robot_prim_path}/left_wheel_link",
            f"{self.robot_prim_path}/right_wheel_link",
        ]
        for bp in bodies:
            prim = stage.GetPrimAtPath(bp)
            if prim.IsValid():
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                rb.CreateSleepThresholdAttr().Set(0.0)
                rb.CreateStabilizationThresholdAttr().Set(0.0)

    def setup_sensors(self) -> bool:
        stage = _usd.get_context().get_stage()

        for path in [self.camera_link_path, self.lidar_link_path]:
            if not stage.GetPrimAtPath(path).IsValid():
                print(f"[sim] ERROR: link not found: {path}")
                return False

        camera_prim = stage.DefinePrim(self.camera_prim_path, "Camera")
        camera_prim.GetAttribute("focalLength").Set(30.4)
        camera_prim.GetAttribute("horizontalAperture").Set(36.8)
        camera_prim.GetAttribute("verticalAperture").Set(27.6)
        camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.05, 50.0))
        cam_xform = UsdGeom.XformCommonAPI(camera_prim)
        cam_xform.SetRotate(Gf.Vec3f(-90.0, 180.0, 90.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        print(f"[sim] Camera prim created: {self.camera_prim_path}")

        ok, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/Lidar",
            parent=self.lidar_link_path,
            config="Example_Rotary_2D",  
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )
        if ok and sensor is not None:
            self.lidar_sensor_path = str(sensor.GetPath())
            # Example_Rotary_2D ships with nearRangeM=1.0 / farRangeM=200, so anything closer than 1 m
            # (a rack beside the robot) comes out as -1 in /scan. Far range matches the RPLIDAR S2E;
            # near range is kept at 1 cm so anything touching the lidar housing still returns a hit.
            sensor_prim = stage.GetPrimAtPath(self.lidar_sensor_path)
            applied = []
            for a in sensor_prim.GetAttributes():
                for suffix, val in (("nearRangeM", 0.01), ("farRangeM", 30.0)):
                    if a.GetName().endswith(suffix):
                        a.Set(val)
                        applied.append(f"{a.GetName()}={val}")
            if applied:
                print(f"[sim] Lidar range set: {applied}")
            else:
                print("[sim] Lidar range attrs NOT found, near range stays at 1 m. Sensor attrs: "
                      f"{[a.GetName() for a in sensor_prim.GetAttributes() if 'sensor' in a.GetName().lower()]}")
            print(f"[sim] Lidar created: {self.lidar_sensor_path}")
        else:
            print(f"[sim] Lidar NOT available")
            self.lidar_sensor_path = ""
        return True
    


    def setup_render_products(self, cam_w, cam_h):
        self._rp_camera = rep.create.render_product(self.camera_prim_path, (cam_w, cam_h))
        print(f"[sim] RenderProduct camera: {self._rp_camera.path}")
        from isaacsim.ros2.bridge import read_camera_info

        camera_info, _ = read_camera_info(render_product_path=self._rp_camera.path)
        writer_cam_info = rep.writers.get("ROS2PublishCameraInfo")
        writer_cam_info.initialize (
            frameId = "camera_color_optical_frame", 
            topicName = "camera/camera_info", 
            width=camera_info.width,
            height=camera_info.height,
            projectionType=camera_info.distortion_model,
            k=camera_info.k.reshape([1, 9]),
            r=camera_info.r.reshape([1, 9]),
            p=camera_info.p.reshape([1, 12]),
        )

        writer_cam_info.attach([self._rp_camera])


        if self.lidar_sensor_path:
            self._rp_lidar = rep.create.render_product(self.lidar_sensor_path, [1, 1], name="Lidar")
            writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            writer.initialize(topicName="scan", frameId="laser")
            writer.attach([self._rp_lidar])
            print(f"[sim] Lidar writer attached: {self._rp_lidar.path}")


    def setup_ros2_graph(self) -> bool:
        rp_cam = self._rp_camera.path if self._rp_camera else ""
 
        nodes = [
            ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime",       "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ROS2Context",       "isaacsim.ros2.bridge.ROS2Context"),
            ("ComputeOdom",       "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("OdomPublisher",     "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("DiffDrive",         "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("CameraHelper",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("PublishClock",      "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("MastTarget",        "isaacsim.ros2.bridge.ROS2Subscriber"),

        ]

        set_values = [
            ("ROS2Context.inputs:useDomainIDEnvVar",     True),

            # odometria  ArticulationRoot 
            ("ComputeOdom.inputs:chassisPrim",           [self.articulation_path]),
            ("OdomPublisher.inputs:topicName",           "odom"),
            ("OdomPublisher.inputs:odomFrameId",         "odom"),
            ("OdomPublisher.inputs:chassisFrameId",      "base_link"),

            # Joint states
            ("PublishJointState.inputs:topicName",       "joint_states"),
            ("PublishJointState.inputs:targetPrim",      [self.articulation_path]),

            # Cmd vel
            ("DiffDrive.inputs:topicName",               "cmd_vel"),


            ("CameraHelper.inputs:topicName",            "camera/image_raw"),
            ("CameraHelper.inputs:type",                 "rgb"),
            ("CameraHelper.inputs:frameId",              "camera_color_optical_frame"),
            ("CameraHelper.inputs:renderProductPath",    rp_cam),

            # Clock
            ("PublishClock.inputs:topicName",            "clock"),

            # Forklift height target (std_msgs/Float32)
            ("MastTarget.inputs:topicName",              "forklift/height_target"),
            ("MastTarget.inputs:messagePackage",         "std_msgs"),
            ("MastTarget.inputs:messageName",            "Float32"),

        ]
        


        connect = [
            # Tick to exec
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "DiffDrive.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "MastTarget.inputs:execIn"),
            ("ROS2Context.outputs:context", "MastTarget.inputs:context"),
            # ComputeOdom exec chain
            ("ComputeOdom.outputs:execOut", "OdomPublisher.inputs:execIn"),
            # SimTime to timestamps
            ("ReadSimTime.outputs:simulationTime", "OdomPublisher.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            # Context to  nodos ROS2
            ("ROS2Context.outputs:context", "OdomPublisher.inputs:context"),
            ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
            ("ROS2Context.outputs:context", "DiffDrive.inputs:context"),
            ("ROS2Context.outputs:context", "CameraHelper.inputs:context"),
            ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
            # ComputeOdom to  OdomPublisher data
            ("ComputeOdom.outputs:position",        "OdomPublisher.inputs:position"),
            ("ComputeOdom.outputs:orientation",     "OdomPublisher.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity",  "OdomPublisher.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "OdomPublisher.inputs:angularVelocity"),

       
        ]


        (ok, _, _, _) = og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: nodes,
                og.Controller.Keys.SET_VALUES:   set_values,
                og.Controller.Keys.CONNECT:      connect,
            }
        )

        if not ok:
            print("[sim] ERROR: Action Graph ROS2 creation")
            return False

        # dynamic output holding the received Float32 payload
        og.Controller.create_attribute(
            f"{self.graph_path}/MastTarget", "outputs:data", og.Type(og.BaseDataType.FLOAT))

        print("[sim] Action Graph ROS2 OK")
        return True

    def _init_mast(self, robot):
        names = [n for n in self.MAST_JOINTS if n in robot.dof_names]
        if not names:
            print("[sim] no mast joints found in articulation")
            self._mast_idx = []
            return
        self._mast_idx = [robot.get_dof_index(n) for n in names]
        props = robot.dof_properties
        self._mast_lo = np.array([props["lower"][i] for i in self._mast_idx], dtype=float)
        self._mast_hi = np.array([props["upper"][i] for i in self._mast_idx], dtype=float)
        self.MAST_MIN = float(self._mast_lo.sum())
        self.MAST_MAX = float(self._mast_hi.sum())
        print(f"[sim] mast stages {names}: height range [{self.MAST_MIN:.3f}, {self.MAST_MAX:.3f}] m")

        # PhysX comes up with kp=0 for these DOFs after articulation init, so set them explicitly
        ctrl = robot.get_articulation_controller()
        kps, kds = ctrl.get_gains()
        kps, kds = np.array(kps, dtype=float), np.array(kds, dtype=float)
        kps[self._mast_idx] = self.MAST_STIFFNESS
        kds[self._mast_idx] = self.MAST_DAMPING
        ctrl.set_gains(kps=kps, kds=kds)

    def _split_height(self, height):
        """Total lift -> per-stage targets. 0 is the rest pose (every stage at 0); above it each stage
        fills to its upper limit before the next one moves, below it the same with the lower limits."""
        targets = np.zeros_like(self._mast_lo)
        remaining = abs(height)
        for i, (lo, hi) in enumerate(zip(self._mast_lo, self._mast_hi)):
            room = hi if height >= 0 else -lo
            take = float(np.clip(remaining, 0.0, max(room, 0.0)))
            targets[i] = take if height >= 0 else -take
            remaining -= take
        return targets

    def update_mast(self, robot):
        """Read /forklift/height_target (total fork height, metres), saturate it to the combined travel
        of all stages and send per-stage position targets through the articulation
        (USD edits don't reach PhysX once it runs)."""
        if self._mast_idx is None:
            self._init_mast(robot)
        if not self._mast_idx:
            return
        try:
            val = og.Controller.attribute(f"{self.graph_path}/MastTarget.outputs:data").get()
            if val is not None:
                self._mast_target = float(np.clip(val, self.MAST_MIN, self.MAST_MAX))
        except Exception:
            pass
        targets = self._split_height(self._mast_target)
        order = np.argsort(self._mast_idx)   # keep DOF indices ascending, with targets in matching order
        robot.apply_action(ArticulationAction(
            joint_positions=targets[order],
            joint_indices=np.array(self._mast_idx)[order]))

    def reset_mast(self):
        self._mast_target = 0.0
        self._mast_idx = None   # world.reset() wipes the PhysX gains, so force update_mast to set them again
        try:
            og.Controller.attribute(f"{self.graph_path}/MastTarget.outputs:data").set(0.0)
        except Exception:
            pass
    


    def _read_twist_from_graph(self):
        
        try:
            lin = og.Controller.attribute(
                f"{self.graph_path}/DiffDrive.outputs:linearVelocity"
            ).get()
            ang = og.Controller.attribute(
                f"{self.graph_path}/DiffDrive.outputs:angularVelocity"
            ).get()
            self._twist["v"] = float(lin[0]) if lin is not None and len(lin) > 0 else 0.0
            self._twist["w"] = float(ang[2]) * self.ANGULAR_GAIN if ang is not None and len(ang) > 2 else 0.0
        except Exception:
            pass


    def _apply_diff_drive(self, articulation_controller):
        v = self._twist["v"]
        w = self._twist["w"]
        v_left  = (v - w * self.wheel_base / 2.0) / self.wheel_radius
        v_right = (v + w * self.wheel_base / 2.0) / self.wheel_radius

        articulation_controller.apply_action(
            ArticulationAction(
                joint_velocities=np.array([v_left, v_right])
            )
        )