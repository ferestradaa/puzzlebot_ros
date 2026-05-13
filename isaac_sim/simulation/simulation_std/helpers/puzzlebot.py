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

        # configurar drives y quitar caps de joint
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
            drive.GetMaxForceAttr().Set(2.0)
            drive.GetDampingAttr().Set(200)
            drive.GetStiffnessAttr().Set(0.0)

            physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
            physx_joint.CreateJointFrictionAttr().Set(0.0)
            physx_joint.CreateMaxJointVelocityAttr().Set(1000.0)

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

        # bind en el link directamente, los collision children heredan
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
        mat.CreateStaticFrictionAttr().Set(1.0)
        mat.CreateDynamicFrictionAttr().Set(1.0)
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
            print(f"[sim] Lidar created: {self.lidar_sensor_path}")
        else:
            print(f"[sim] Lidar NOT available")
            self.lidar_sensor_path = ""
        return True
    


    def setup_render_products(self, cam_w, cam_h):
        self._rp_camera = rep.create.render_product(self.camera_prim_path, (cam_w, cam_h))
        print(f"[sim] RenderProduct camera: {self._rp_camera.path}")

        if self.lidar_sensor_path:
            self._rp_lidar = rep.create.render_product(self.lidar_sensor_path, [1, 1], name="Lidar")
            writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            writer.initialize(topicName="scan", frameId="lidar_link")
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
        ]

        set_values = [
            ("ROS2Context.inputs:useDomainIDEnvVar",     True),

            # odometria  ArticulationRoot correcto
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
        ]


        connect = [
            # Tick to exec
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "DiffDrive.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
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

        print("[sim] Action Graph ROS2 OK")
        return True
    


    def _read_twist_from_graph(self):
        try:
            lin = og.Controller.attribute(
                f"{self.graph_path}/DiffDrive.outputs:linearVelocity"
            ).get()
            ang = og.Controller.attribute(
                f"{self.graph_path}/DiffDrive.outputs:angularVelocity"
            ).get()
            self._twist["v"] = float(lin[0]) if lin is not None and len(lin) > 0 else 0.0
            self._twist["w"] = float(ang[2]) if ang is not None and len(ang) > 2 else 0.0
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