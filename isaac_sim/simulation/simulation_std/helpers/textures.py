"""Realistic materials for the puzzlebot visuals.

The importer gives the robot a handful of flat colours and a chassis mesh that
is a single prim (plates, standoffs, Jetson, motors, cables, lidar...). The
chassis is therefore split by topology into connected components, each one is
classified from its size/position, and every material class becomes a
GeomSubset with its own UsdPreviewSurface. Everything is authored on the live
stage, nothing in the generated usd is touched.
"""

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

# name: (base colour, metallic, roughness, clearcoat, opacity)
MATERIALS = {
    "acrylic":       ((0.82, 0.92, 1.00), 0.0, 0.05, 0.0, 0.22),
    "black_plastic": ((0.020, 0.020, 0.022), 0.0, 0.38, 0.0, 1.0),
    "gloss_black":   ((0.010, 0.010, 0.012), 0.0, 0.12, 1.0, 1.0),
    "pcb_black":     ((0.012, 0.012, 0.014), 0.0, 0.35, 0.4, 1.0),
    "motor_gray":    ((0.180, 0.185, 0.195), 0.5, 0.35, 0.1, 1.0),
    "forklift_black": ((0.012, 0.012, 0.014), 0.0, 0.45, 0.1, 1.0),
    "black_oxide":   ((0.050, 0.050, 0.055), 0.9, 0.40, 0.0, 1.0),
    "anodized":      ((0.030, 0.030, 0.036), 0.6, 0.33, 0.0, 1.0),
    "silver":        ((0.800, 0.810, 0.830), 0.6, 0.30, 0.0, 1.0),
    "aluminium":     ((0.850, 0.860, 0.880), 0.85, 0.32, 0.0, 1.0),
    "wire_black":    ((0.015, 0.015, 0.015), 0.0, 0.50, 0.0, 1.0),
    "wire_white":    ((0.800, 0.800, 0.780), 0.0, 0.50, 0.0, 1.0),
    "wheel_orange":  ((1.000, 0.230, 0.020), 0.0, 0.42, 0.2, 1.0),
    "chrome":        ((0.800, 0.810, 0.830), 0.7, 0.20, 0.0, 1.0),
}
WIRES = ["wire_black", "wire_white"]

# other links: (link, material)
LINK_MATERIALS = [
    ("left_wheel_link", "wheel_orange"),
    ("right_wheel_link", "wheel_orange"),
    ("caster_wheel_link", "chrome"),
    ("outer_mast_link", "forklift_black"),
    ("inner_mast_link", "forklift_black"),
    ("forks_link", "forklift_black"),
]


def _make_material(stage, root, name):
    color, metallic, roughness, clearcoat, opacity = MATERIALS[name]
    path = f"{root}/Looks/material_{name}"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(clearcoat)
    shader.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.05)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    if name == "acrylic":
        shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.49)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _components(mesh):
    """Connected components of the mesh: list of dicts with faces + bbox (m)."""
    import scipy.sparse as sp
    from scipy.sparse.csgraph import connected_components

    pts = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])

    # one edge per face corner: vertex i -> next vertex of the same face
    pos = np.arange(len(idx)) - np.repeat(starts, counts)
    nxt = np.arange(len(idx)) - pos + (pos + 1) % np.repeat(counts, counts)
    graph = sp.coo_matrix((np.ones(len(idx)), (idx, idx[nxt])), shape=(len(pts),) * 2)
    _, vert_label = connected_components(graph, directed=False)
    face_label = vert_label[idx[starts]]

    order = np.argsort(face_label, kind="stable")
    sorted_labels = face_label[order]
    first = np.searchsorted(sorted_labels, np.unique(sorted_labels))
    last = np.append(first[1:], len(sorted_labels))

    comps = []
    for a, b in zip(first, last):
        faces = order[a:b]
        verts = np.unique(idx[np.concatenate([np.arange(starts[f], starts[f] + counts[f]) for f in faces])])
        p = pts[verts]
        comps.append({"faces": faces, "n": len(faces), "mn": p.min(0), "mx": p.max(0)})
    comps.sort(key=lambda c: (-c["n"], tuple(np.round(c["mn"], 4))))
    return comps


def _classify(c, wire_counter):
    """Material name for one chassis component, from size and position (mesh frame, m)."""
    mm = 1000.0
    d = (c["mx"] - c["mn"]) * mm
    zmin = c["mn"][2] * mm
    xmin = c["mn"][0] * mm
    n = c["n"]
    diag = float(np.linalg.norm(d))

    if zmin >= 120:                                    # lidar
        if zmin >= 150 and n > 5000:
            return "gloss_black"
        return "black_plastic" if n > 10000 else "black_oxide"
    if d[0] > 100 and d[1] > 100 and d[2] < 10:        # the three plates
        return "acrylic"
    if max(d[0], d[1]) < 9 and d[2] > 35:              # standoffs
        return "aluminium"
    if n > 15000 and d[2] < 35 and zmin < 45:          # gear motors
        return "motor_gray"
    if d[2] <= 4.5 and d[0] > 15 and d[1] > 15:        # boards
        return "pcb_black"
    if n >= 9000 and zmin >= 80:                       # jetson heatsink
        return "anodized"
    if min(d[0], d[1]) < 10 and max(d[0], d[1]) > 20 and d[2] > 40:   # motor brackets
        return "black_oxide"
    if d[2] <= 12 and d[0] > 20 and d[1] > 20 and n > 2500:           # shield / driver boards
        return "pcb_black"
    if diag > 45 and min(d) < 14 and n < 4000:         # cables
        wire = WIRES[wire_counter[0] % len(WIRES)]
        wire_counter[0] += 1
        return wire
    if max(d) < 16:                                    # screws, nuts
        return "black_oxide"
    if zmin >= 83 and 1000 <= n < 6000:                # connectors on the boards
        return "silver"
    if xmin < -130 and zmin < 25:                      # caster holder
        return "black_plastic"
    return "black_plastic"


def _unshare(visuals):
    """visuals is an instance of the generated usd; its subtree is read-only until this."""
    visuals.SetInstanceable(False)


def _rebind(prim, material, strength=UsdShade.Tokens.strongerThanDescendants):
    """Replace any importer binding on `prim` and its descendants with `material`."""
    for p in Usd.PrimRange(prim):
        if p.GetRelationship("material:binding"):
            UsdShade.MaterialBindingAPI(p).Bind(material, strength)


def apply_chassis_materials(robot_prim_path):
    stage = omni.usd.get_context().get_stage()
    visuals = stage.GetPrimAtPath(f"{robot_prim_path}/chassis_link/visuals")
    if not visuals.IsValid():
        print("[tex] chassis visuals not found")
        return
    _unshare(visuals)
    mesh_prim = next((p for p in Usd.PrimRange(visuals) if p.IsA(UsdGeom.Mesh)), None)
    if mesh_prim is None:
        print("[tex] chassis mesh not found")
        return
    mesh = UsdGeom.Mesh(mesh_prim)

    # untouched faces fall back to black plastic instead of the importer's white
    # (the importer binds white strongerThanDescendants on an ancestor of the mesh,
    # so every binding under visuals is replaced and made weaker than the subsets)
    base = _make_material(stage, robot_prim_path, "black_plastic")
    _rebind(visuals, base, UsdShade.Tokens.weakerThanDescendants)
    for stale in [c for c in mesh_prim.GetChildren() if c.GetName().startswith("mat_") or c.GetName() == "acrylic_plates"]:
        stage.RemovePrim(stale.GetPath())

    wire_counter = [0]
    groups = {}
    for c in _components(mesh):
        groups.setdefault(_classify(c, wire_counter), []).append(c["faces"])

    for name, face_lists in groups.items():
        material = _make_material(stage, robot_prim_path, name)
        faces = np.sort(np.concatenate(face_lists))
        subset = UsdGeom.Subset.CreateGeomSubset(
            mesh, f"mat_{name}", UsdGeom.Tokens.face, Vt.IntArray(faces.tolist()), "materialBind")
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
    print("[tex] chassis: " + ", ".join(f"{k}={sum(len(f) for f in v)}" for k, v in groups.items()))


def apply_link_materials(robot_prim_path):
    stage = omni.usd.get_context().get_stage()
    for link, name in LINK_MATERIALS:
        visuals = stage.GetPrimAtPath(f"{robot_prim_path}/{link}/visuals")
        if not visuals.IsValid():
            print(f"[tex] {link}/visuals not found")
            continue
        _unshare(visuals)
        _rebind(visuals, _make_material(stage, robot_prim_path, name))


def apply_realistic_textures(robot_prim_path):
    """Acrylic plates + a realistic material on every other visual of the robot."""
    # With plain translucency the RTX real-time renderer draws the opaque parts that sit
    # in front of the acrylic (wheels, forks) behind it, as if they were displaced.
    # Fractional cutout opacity keeps the depth ordering correct.
    import carb.settings
    carb.settings.get_settings().set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
    apply_chassis_materials(robot_prim_path)
    apply_link_materials(robot_prim_path)
