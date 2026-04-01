


from pathlib import Path
import sys
import time


import yaml
from pathlib import Path
import threading
import rclpy

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
sys.path.insert(0, str(Path(__file__).parent)) 

from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.timeline import get_timeline_interface
import omni.physx

from ros2.cmd_vel_bridge import SimBridge



def load_config(path: str = "config.yaml") -> dict:
    # Resuelve config.yaml relativo al script, no al CWD
    script_dir = Path(__file__).parent
    config_path = script_dir / path

    with open(config_path, "r") as f:
        return yaml.safe_load(f)

def resolve_path(raw_path: str) -> str:
    """Convierte rutas relativas al directorio del script."""
    script_dir = Path(__file__).parent
    resolved = (script_dir / raw_path).resolve()
    return str(resolved)

def main():
    config = load_config()
    stage_path = resolve_path(config["usd"]["stage_path"])

    if not Path(stage_path).exists():
        raise FileNotFoundError(f"USD stage not found: {stage_path}")

    # Cargar stage y esperar con el mecanismo propio de Isaac Sim
    open_stage(stage_path)
    while is_stage_loading():
        simulation_app.update()

    # Validar que el stage está listo
    stage = omni.usd.get_context().get_stage()
    if not stage:
        raise RuntimeError("Stage failed to load")

    timeline = get_timeline_interface()
    timeline.play()
    simulation_app.update()  # un solo frame para activar physics

    rclpy.init()
    bridge = SimBridge(config)
    bridge.ensure_initialized() 
    spin_thread = threading.Thread(target = rclpy.spin, args=(bridge,), daemon= True)
    spin_thread.start()

    
    def on_physics_step(dt: float):
        bridge.step(dt)

    subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    target_frame_time = 1.0 / 60.0

    frame_count = 0
    fps_timer = time.perf_counter()

    try:
        while simulation_app.is_running():
            simulation_app.update()
    except KeyboardInterrupt:
        pass
    finally:
        subscription.unsubscribe() 
        bridge.drive.stop()
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()

if __name__ == "__main__":
    main()