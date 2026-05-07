"""
diagnose_nodes.py  —  Imprime atributos reales de todos los nodos OmniGraph
                       usados en puzzlebot_sim.py
Uso:
    ./python.sh diagnose_nodes.py

No modifica el stage ni corre simulación. Solo inspecciona y termina.
"""

# ─── 0. BOOTSTRAP ────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import sys
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

# ─── 1. URDF ImportConfig ────────────────────────────────────────────────────
print("\n" + "="*60)
print("URDF ImportConfig  (isaacsim.asset.importer.urdf._urdf)")
print("="*60)
try:
    from isaacsim.asset.importer.urdf import _urdf
    cfg = _urdf.ImportConfig()
    attrs = [a for a in dir(cfg) if not a.startswith("__")]
    for a in attrs:
        print(f"  {a}")
except Exception as e:
    print(f"  ERROR: {e}")

sys.stdout.flush()

# ─── 2. OmniGraph nodes ──────────────────────────────────────────────────────
import omni.graph.core as og

# Nodos a inspeccionar: (nombre_local, tipo_OGN)
NODES_TO_CHECK = [
    ("OnPlaybackTick",   "omni.graph.action.OnPlaybackTick"),
    ("ROS2Context",      "isaacsim.ros2.bridge.ROS2Context"),
    ("CameraHelper",     "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ("LidarHelper",      "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
    ("OdomPublisher",    "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ("DiffDrive",        "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ("ReadSimTime",      "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("ComputeOdom",      "isaacsim.core.nodes.IsaacComputeOdometry"),
    ("PublishJointState","isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("SubJointState",    "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
    ("ArticCtrl",        "isaacsim.core.nodes.IsaacArticulationController"),
    ("PublishTF",        "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ("PublishClock",     "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("PublishRawTF",     "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
]

# Crear un graph temporal de diagnóstico
DIAG_GRAPH = "/DiagGraph"
try:
    (ok, graph, nodes_created, _) = og.Controller.edit(
        {"graph_path": DIAG_GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                (name, ntype) for name, ntype in NODES_TO_CHECK
            ],
        }
    )
except Exception as e:
    print(f"\nError creando graph de diagnóstico: {e}")
    ok = False

if ok:
    for node_name, node_type in NODES_TO_CHECK:
        print("\n" + "="*60)
        print(f"NODE: {node_name}  [{node_type}]")
        print("="*60)
        node = og.Controller.node(f"{DIAG_GRAPH}/{node_name}")
        if node and node.is_valid():
            attrs = node.get_attributes()
            if attrs:
                # Separar inputs / outputs / otros
                inputs  = [a for a in attrs if a.get_name().startswith("inputs:")]
                outputs = [a for a in attrs if a.get_name().startswith("outputs:")]
                others  = [a for a in attrs if not a.get_name().startswith(("inputs:","outputs:"))]
                if inputs:
                    print("  INPUTS:")
                    for a in inputs:
                        try:
                            val = a.get()
                            print(f"    {a.get_name()}  =  {val!r}")
                        except Exception:
                            print(f"    {a.get_name()}")
                if outputs:
                    print("  OUTPUTS:")
                    for a in outputs:
                        print(f"    {a.get_name()}")
                if others:
                    print("  OTHER:")
                    for a in others:
                        print(f"    {a.get_name()}")
            else:
                print("  (sin atributos)")
        else:
            print(f"  *** NODO NO VÁLIDO / NO EXISTE EN ESTA VERSIÓN ***")
        sys.stdout.flush()

# ─── 3. World / Physics ──────────────────────────────────────────────────────
print("\n" + "="*60)
print("World constructor signature")
print("="*60)
try:
    import inspect
    from omni.isaac.core import World
    sig = inspect.signature(World.__init__)
    print(f"  World.__init__{sig}")
except Exception as e:
    print(f"  ERROR: {e}")

# ─── 4. Camera wrapper ───────────────────────────────────────────────────────
print("\n" + "="*60)
print("Camera / Sensor imports")
print("="*60)
for mod, cls in [
    ("omni.isaac.sensor", "Camera"),
    ("omni.isaac.sensor", "RotatingLidarPhysX"),
    ("isaacsim.sensors.camera", "Camera"),
    ("isaacsim.sensors.rtx", "LidarRtx"),
]:
    try:
        m = __import__(mod, fromlist=[cls])
        c = getattr(m, cls)
        print(f"  OK  {mod}.{cls}")
    except Exception as e:
        print(f"  FAIL {mod}.{cls}  →  {e}")

sys.stdout.flush()

# ─── 5. add_reference_to_stage ───────────────────────────────────────────────
print("\n" + "="*60)
print("add_reference_to_stage signature")
print("="*60)
try:
    import inspect
    from omni.isaac.core.utils.stage import add_reference_to_stage
    sig = inspect.signature(add_reference_to_stage)
    print(f"  add_reference_to_stage{sig}")
except Exception as e:
    print(f"  ERROR: {e}")

print("\n" + "="*60)
print("DIAGNÓSTICO COMPLETO")
print("="*60)
sys.stdout.flush()

simulation_app.close()