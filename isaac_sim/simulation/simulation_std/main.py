import argparse, os, sys, yaml

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False, 
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
from isaacsim.core.api.objects import GroundPlane
from pxr import PhysxSchema, UsdPhysics
import omni.usd
from pxr import Sdf

from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from simulation_std.helpers.puzzlebot import Puzzlebot
import numpy as np

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")

simulation_app.update()

'''
def setup_world(world_usd_path) -> World:
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 30.0,
    )
    add_reference_to_stage(usd_path=world_usd_path, prim_path="/World")
    
    stage = omni.usd.get_context().get_stage()
    
    # Busca el prim con PhysxSceneAPI aplicado
    physx_scene_prim = None
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            physx_scene_prim = prim
            break
        
    if physx_scene_prim:
        attr = physx_scene_prim.GetAttribute("physxScene:sleepThreshold")
        if attr.IsValid():
            attr.Set(0.0)
        else:
            # Crear el atributo si no existe
            physx_scene_prim.CreateAttribute("physxScene:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        
    return world
'''

def setup_world() -> World:
    from pxr import UsdShade, UsdPhysics, PhysxSchema

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 30.0,
    )
    world.scene.add_default_ground_plane(
        z_position=0.0,
        static_friction=1.0,
        dynamic_friction=1.0,
        restitution=0.0,
    )

    stage = omni.usd.get_context().get_stage()

    # crear scope primero, luego el material
    stage.DefinePrim("/World/PhysicsMaterials", "Scope")
    mat_prim = stage.DefinePrim("/World/PhysicsMaterials/GroundMat", "Material")
    gnd_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    gnd_mat.CreateStaticFrictionAttr().Set(1.0)
    gnd_mat.CreateDynamicFrictionAttr().Set(1.0)
    gnd_mat.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(mat_prim).CreateFrictionCombineModeAttr().Set("average")

    # add_default_ground_plane crea el collision en este path
    ground_collision = stage.GetPrimAtPath("/World/defaultGroundPlane/CollisionPlane")
    if ground_collision.IsValid():
        UsdShade.MaterialBindingAPI.Apply(ground_collision).Bind(
            UsdShade.Material(mat_prim),
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics"
        )
        print("[sim] GroundMat bindeado a /World/defaultGroundPlane/CollisionPlane")
    else:
        # fallback: buscar el collision del ground plane
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if "defaultGroundPlane" in path and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    UsdShade.Material(mat_prim),
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                    materialPurpose="physics"
                )
                print(f"[sim] GroundMat bindeado a: {path}")
                break

    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            attr = prim.GetAttribute("physxScene:sleepThreshold")
            if attr.IsValid():
                attr.Set(0.0)
            else:
                prim.CreateAttribute("physxScene:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
            break

    return world

def load_robot(usd_path: str, robot_prim_path):
    
    add_reference_to_stage(usd_path=usd_path, prim_path=robot_prim_path)
    stage = omni.usd.get_context().get_stage()
    prim  = stage.GetPrimAtPath(robot_prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"No valid prim: {robot_prim_path}")
    xform = UsdGeom.XformCommonAPI(prim)
    xform.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.08))


def main():
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    ROOT_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", ".."))

    _CONFIG_PATH = os.path.join(_SCRIPT_DIR, "config.yaml")
    with open(_CONFIG_PATH) as f:
        _cfg=yaml.safe_load(f)

    URDF_PATH  = os.path.join(ROOT_DIR, _cfg["paths"]["urdf"])
    USD_OUTPUT = os.path.join(ROOT_DIR, _cfg["paths"]["usd_output"])
    os.makedirs(os.path.dirname(USD_OUTPUT), exist_ok=True)

    WORLD_USD_PATH = os.path.join(ROOT_DIR, _cfg["world"]["stage_path"])

    ROBOT_PRIM_PATH   = _cfg["robot"]["prim_path"]
    ARTICULATION_PATH = _cfg["robot"]["articulation_path"]
    WHEEL_RADIUS      = _cfg["robot"]["wheel_radius"]
    WHEEL_BASE        = _cfg["robot"]["wheel_base"]

    CAMERA_LINK_PATH  = f"{ROBOT_PRIM_PATH}/camera_link"
    LIDAR_LINK_PATH   = f"{ROBOT_PRIM_PATH}/lidar_link"
    CAMERA_PRIM_PATH  = f"{CAMERA_LINK_PATH}/Camera"
    LIDAR_SENSOR_PATH = f"{LIDAR_LINK_PATH}/Lidar/{_cfg['sensors']['lidar']['config']}"

    GRAPH_PATH = _cfg["ros2"]["graph_path"]

    puzzlebot = Puzzlebot(ROBOT_PRIM_PATH,
                          ARTICULATION_PATH, WHEEL_RADIUS, WHEEL_BASE, 
                          CAMERA_LINK_PATH, LIDAR_LINK_PATH,
                          CAMERA_PRIM_PATH, LIDAR_SENSOR_PATH,
                          GRAPH_PATH)
    

    if not os.path.exists(URDF_PATH):
        raise FileNotFoundError(f"URDF not found at: {URDF_PATH}")

    usd_path = puzzlebot.import_urdf_to_usd(URDF_PATH, USD_OUTPUT)
    

    world = setup_world()
    load_robot(usd_path, ROBOT_PRIM_PATH)
    simulation_app.update()

    puzzlebot.fix_wheel_drives_live()
    puzzlebot.fix_caster_wheel()
    #puzzlebot.tune_articulation_solver()


    if not puzzlebot.setup_sensors():
        simulation_app.close()
        return

    puzzlebot.setup_render_products()

    if not puzzlebot.setup_ros2_graph():
        simulation_app.close()
        return

    simulation_app.update()
    world.reset()


    '''
    from isaacsim.core.api.objects import DynamicCuboid
    import numpy as np
    world.scene.add(DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([1.5, 0.0, 0.1]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([1.0, 0.0, 0.0]),
    ))


    from isaacsim.core.api.robots import Robot
    robot = Robot(prim_path=ARTICULATION_PATH, name="puzzlebot")
    world.scene.add(robot)
    world.reset()
    '''

    wheeled_robot = WheeledRobot(
        prim_path=ARTICULATION_PATH,
        name="puzzlebot",
        wheel_dof_names=["base_to_left_wheel", "base_to_right_wheel"],
        create_robot=False,
        )
    
    world.scene.add(wheeled_robot)
    world.reset()

    diff_controller = DifferentialController(
        name="diff_ctrl",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
    )


    step = 0

    while simulation_app.is_running():
        world.step(render=True)

        puzzlebot._read_twist_from_graph()
        cmd = np.array([puzzlebot._twist["v"], puzzlebot._twist["w"]])

        action = diff_controller.forward(command=cmd)
        wheeled_robot.apply_wheel_actions(action)
        
        step += 1

    print("[sim] Closing...")
    simulation_app.close()


if __name__ == "__main__":
    main()   