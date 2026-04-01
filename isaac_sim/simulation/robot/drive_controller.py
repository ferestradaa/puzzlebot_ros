
#this class reads twist message sent and set velocities to wheels throught isaacs API
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # sube a simulation/


import yaml 
from pxr import UsdPhysics 
from omni.isaac.core.utils.stage import get_current_stage

from sim_utils.math_utils import twist_to_wheel_velocities, rad_to_deg

class DriveController:
    def __init__(self, config: dict):
        wheel_cfg = config["robot"]
        joints_cfg = config["joints"]

        self.wheel_radius = wheel_cfg["wheel_radius"]
        self.wheel_base = wheel_cfg["wheel_base"]

        stage = get_current_stage()
        self.left_drive  = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joints_cfg["left"]), "angular")
        self.right_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joints_cfg["right"]), "angular")

        if not self.left_drive or not self.right_drive:
            l_prim = stage.GetPrimAtPath(joints_cfg["left"])
            r_prim = stage.GetPrimAtPath(joints_cfg["right"])
            raise RuntimeError(
                f"DriveController: Could not find joints.\n"
                f"  left  path='{joints_cfg['left']}' valid={l_prim.IsValid()} "
                f"type={l_prim.GetTypeName() if l_prim.IsValid() else 'N/A'}\n"
                f"  right path='{joints_cfg['right']}' valid={r_prim.IsValid()} "
                f"type={r_prim.GetTypeName() if r_prim.IsValid() else 'N/A'}"
            )
        self.stop()

    def set_twist(self, linear_x, angular_z):
        v_left, v_right = twist_to_wheel_velocities(linear_x, angular_z, self.wheel_radius, self.wheel_base)
        self.left_drive.GetTargetVelocityAttr().Set(rad_to_deg(v_left))
        self.right_drive.GetTargetVelocityAttr().Set(rad_to_deg(v_right))

    def stop(self):
        self.left_drive.GetTargetVelocityAttr().Set(0.0)
        self.right_drive.GetTargetVelocityAttr().Set(0.0)