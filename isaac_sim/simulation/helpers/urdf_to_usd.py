#Try running this script inside isaac sim scripts editor in order to create the usd file automatically if any change 
#has to be fetched 

from isaacsim.asset.importer.urdf import _urdf

def load_assets():
    urdf_path = "/home/ferestrada/puzz_ws/isaac_sim/assets/puzzlebot/urdf/puzzlebot.urdf"
    
    import os
    print(f"URDF existe: {os.path.exists(urdf_path)}")

    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = True

    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path="/tmp/robot.usd",
    )

    print(f"result: {result}, prim_path: {prim_path}")

    stage = omni.usd.get_context().get_stage()
    print(f"stage: {stage}")

    stage.GetPrimAtPath("/World").GetReferences().AddReference("/tmp/robot.usd")

load_assets()