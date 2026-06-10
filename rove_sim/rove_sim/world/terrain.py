"""GLB terrain -> PyBullet assets (cached): mesh-first, texture optional.

Per the rendering strategy: the environment is rendered as 3D MESH (geometry);
textures are optional in the main view (perf permitting) and are really for the
camera-feed renderer. So we keep the FULL scene (no missing chunks) as a handful
of colour-grouped mesh bodies, plus a ground-only collision mesh the robot drives
on. The forest GLB is Y-up, 341k faces, with 26 separate per-object textures
(NOT one atlas) — which is why concatenating to a single textured OBJ produces
wrong textures / UV static. Grouping by material into solid-colour bodies avoids
that entirely; per-object textures are exported too (manifest `tex`) for when the
camera sensor wants them.
"""
from __future__ import annotations

import json
import os
import re

# --- ground (drivable surface) classification: collision + flat-spot ----------
GROUND_KEYWORDS = ("grass", "rock", "dirt", "road", "cliff", "ground",
                   "terrain", "mud", "cobblestone", "gravel")
EXCLUDE_KEYWORDS = ("aerial", "vegetation", "decal", "puddle", "far")

# --- visual category -> colour (the whole scene, mesh-first) -------------------
CATEGORY_COLORS = {
    "ground":  [0.52, 0.43, 0.32, 1],
    "grass":   [0.34, 0.45, 0.24, 1],
    "rock":    [0.50, 0.47, 0.42, 1],
    "trunk":   [0.36, 0.27, 0.18, 1],
    "foliage": [0.27, 0.40, 0.20, 1],
    "metal":   [0.55, 0.55, 0.58, 1],
}
MANIFEST_VERSION = 9

# Alpha-mesh cutout (bake foliage leaf-silhouettes into geometry) is implemented
# in _alpha_cutout, but on this asset the leaf detail is finer than geometry can
# represent -- it shatters canopies/litter into angular shards that look worse
# than clean opaque cards. PyBullet can't alpha-blend billboards (the way game
# engines render foliage), so we keep cards opaque here and do the real
# transparent-gap/leaf-hit logic in the lidar raycast (per-ray texture-alpha
# lookup at the hit UV) where it's both correct and free. Flip to True to A/B it.
_CUTOUT_FOLIAGE = True


def _flatten_texture(png: str) -> bool:
    """Composite a texture's alpha away so PyBullet can render it.

    PyBullet's renderer IGNORES per-texel texture alpha (loadTexture drops the
    channel), so glTF alphaMode=BLEND cards (foliage, grass, leaf litter, the
    road overlay) render their TRANSPARENT texels as opaque -- usually black,
    giving the "black holes" + flat-colour cards. We can't get true cutout in
    pybullet, so the next best thing: fill the transparent texels with the mean
    of the opaque ones (a few dilation passes to keep edges sensible) and drop
    the alpha. No more black; the texture reads as its real colours everywhere.
    Returns True if it rewrote the file. Idempotent (RGB images are left as-is).
    """
    import numpy as np
    from PIL import Image
    im = Image.open(png)
    if im.mode not in ("RGBA", "LA", "P"):
        return False
    im = im.convert("RGBA")
    arr = np.asarray(im).astype(np.float32)
    rgb, a = arr[..., :3], arr[..., 3]
    opaque = a > 16.0
    if opaque.all() or not opaque.any():
        Image.fromarray(arr[..., :3].astype(np.uint8), "RGB").save(png)
        return True
    fill = rgb[opaque].mean(axis=0)
    out = rgb.copy()
    out[~opaque] = fill
    # a few dilation passes so opaque colours bleed into the holes (better than a
    # flat mean at card edges); cheap on these <=2k textures.
    mask = opaque.copy()
    for _ in range(4):
        if mask.all():
            break
        nbr = np.zeros_like(out); cnt = np.zeros(mask.shape, np.float32)
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            sh = np.roll(np.where(mask[..., None], out, 0), (dy, dx), (0, 1))
            sm = np.roll(mask, (dy, dx), (0, 1)).astype(np.float32)
            nbr += sh; cnt += sm
        grow = (~mask) & (cnt > 0)
        out[grow] = (nbr[grow] / cnt[grow, None])
        mask |= grow
    Image.fromarray(out.astype(np.uint8), "RGB").save(png)
    return True


def _alpha_cutout(mesh, target_uv: float = 0.03, max_level: int = 3,
                  thresh: int = 128, max_tris: int = 2_000_000):
    """Bake a foliage card's texture-alpha silhouette into real geometry.

    PyBullet can't render per-texel texture alpha, so flat BLEND foliage/grass
    cards otherwise show as opaque rectangles. Here we ADAPTIVELY subdivide each
    triangle (in UV space) until its UV footprint is ~target_uv, then DROP the
    sub-triangles whose centre samples a transparent texel. Result: leaf-shaped
    geometry -- opaque where there are leaves, empty (sky shows through) in the
    gaps -- which is also the right collision for a lidar that should pass
    through the gaps. The level cap keeps already-fine canopies cheap (they just
    get culled) while splitting the big 2-tri grass/bush quads a couple of times.
    Returns a new triangle-soup Trimesh sharing the original material, or None.
    """
    import numpy as np
    import trimesh
    vis = getattr(mesh, "visual", None)
    uv = getattr(vis, "uv", None)
    mat = getattr(vis, "material", None)
    img = getattr(mat, "baseColorTexture", None) if mat else None
    if uv is None or img is None or len(uv) != len(mesh.vertices):
        return None
    alpha = np.asarray(img.convert("RGBA"))[..., 3]
    H, W = alpha.shape
    if alpha.min() >= thresh:                       # fully opaque -> nothing to cut
        return None
    F = mesh.faces
    tri = mesh.vertices[F].astype(np.float32)       # (N,3,3) triangle soup
    tuv = np.asarray(uv, np.float32)[F]             # (N,3,2)

    def split(arr):                                 # (M,3,k) -> (4M,3,k) midpoint 1:4
        a, b, c = arr[:, 0], arr[:, 1], arr[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        return np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b, bc], 1),
                               np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])

    for _ in range(max_level):
        e = np.stack([np.linalg.norm(tuv[:, 1] - tuv[:, 0], axis=1),
                      np.linalg.norm(tuv[:, 2] - tuv[:, 1], axis=1),
                      np.linalg.norm(tuv[:, 0] - tuv[:, 2], axis=1)], 1).max(1)
        sel = e > target_uv
        if not sel.any() or len(tri) + 3 * int(sel.sum()) > max_tris:
            break
        tri = np.concatenate([tri[~sel], split(tri[sel])])
        tuv = np.concatenate([tuv[~sel], split(tuv[sel])])

    cuv = tuv.mean(1)                               # centroid UV -> texel
    px = np.clip((cuv[:, 0] * (W - 1)).astype(int), 0, W - 1)
    py = np.clip(((1.0 - cuv[:, 1]) * (H - 1)).astype(int), 0, H - 1)
    keep = alpha[py, px] >= thresh
    tri, tuv = tri[keep], tuv[keep]
    if len(tri) == 0:
        return None
    V = tri.reshape(-1, 3)
    out = trimesh.Trimesh(vertices=V, faces=np.arange(len(V)).reshape(-1, 3),
                          process=False)
    out.visual = trimesh.visual.TextureVisuals(uv=tuv.reshape(-1, 2), material=mat)
    return out


def _suffix(name: str) -> str:
    # "Background_Tree_Atlas.025_Dirt_Road_0" -> "dirt_road_0"; the canopy meshes
    # ".001_Background_Tree_Atlas_0" keep "background_tree_atlas_0" (-> foliage).
    parts = re.split(r"\.\d+_", name, maxsplit=1)
    return (parts[1] if len(parts) > 1 else name).lower()


def _is_ground(name: str) -> bool:
    n = _suffix(name).replace("background", "")     # canopy suffix has "background"
    if any(k in n for k in EXCLUDE_KEYWORDS):
        return False
    return any(k in n for k in GROUND_KEYWORDS)


def _category(name: str) -> str:
    n = _suffix(name)
    if any(k in n for k in ("trunk", "log", "wood")):
        return "trunk"
    if any(k in n for k in ("tree", "bush", "fallen", "leaves", "leaf", "maple",
                            "birch", "oak", "foliage", "background")):
        return "foliage"
    if any(k in n for k in ("grass", "vegetation", "aerial")):
        return "grass"
    if any(k in n for k in ("rock", "cliff", "gravel", "broken", "terrain", "decal")):
        return "rock"
    if any(k in n for k in ("metal", "fence")):
        return "metal"
    return "ground"


def _flat_spot(verts, cell=10.0):
    """Open, flat, well-populated patch to spawn on (not a basin floor)."""
    import numpy as np

    ij = np.floor(verts[:, :2] / cell).astype(np.int64)
    keys = ij[:, 0] * 100000 + ij[:, 1]
    zmax = float(verts[:, 2].max())
    best = None
    for key in np.unique(keys):
        m = keys == key
        if m.sum() < 60:
            continue
        zs = verts[m, 2]
        i, j = int(ij[m][0, 0]), int(ij[m][0, 1])
        cx, cy = (i + 0.5) * cell, (j + 0.5) * cell
        med = float(np.median(zs))
        score = float(zs.std()) + 0.30 * (zmax - med) + 0.003 * float(np.hypot(cx, cy))
        if best is None or score < best[0]:
            best = (score, cx, cy, med)
    if best is None:
        c = verts.mean(axis=0)
        return float(c[0]), float(c[1]), float(c[2])
    return best[1], best[2], best[3]


def build_assets(src: str, out_dir: str, collision_faces: int = 60000) -> str:
    """GLB -> cached terrain manifest (JSON). Returns the manifest path.

    manifest = {collision: obj, visual: [{obj, color, tex|null}], spawn_z_hint}.
    """
    import numpy as np
    import trimesh
    import fast_simplification

    name = os.path.splitext(os.path.basename(src))[0]
    cache = os.path.join(out_dir, name)
    manifest = os.path.join(cache, "manifest.json")
    if os.path.exists(manifest):
        try:
            d = json.load(open(manifest))
            if d.get("version") == MANIFEST_VERSION \
                    and d.get("collision_faces") == collision_faces:
                return manifest
        except Exception:
            pass
    os.makedirs(cache, exist_ok=True)

    scene = trimesh.load(src, force="scene")
    R = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])  # Y-up -> Z-up
    items, ground = [], []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph.get(node)
        m = scene.geometry[gname].copy()
        m.apply_transform(T)
        m.apply_transform(R)
        items.append((gname, m))
        if _is_ground(gname):
            ground.append(m)
    if not ground:
        ground = [m for _, m in items]
    bx, by, bz = _flat_spot(trimesh.util.concatenate(ground).vertices)
    off = [-bx, -by, -bz]

    # collision: ground-only, decimated, plain geometry
    gm = trimesh.util.concatenate(ground)
    gm.apply_translation(off)
    if len(gm.faces) > collision_faces:
        v, f = fast_simplification.simplify(
            gm.vertices, gm.faces, target_reduction=1.0 - collision_faces / len(gm.faces))
        gm = trimesh.Trimesh(vertices=v, faces=f, process=False)
    coll = os.path.join(cache, "collision.obj")
    trimesh.Trimesh(vertices=gm.vertices, faces=gm.faces, process=False).export(coll)

    # visual: full scene, grouped by MATERIAL (one body per source texture). The
    # forest GLB has 26 PBR materials, each its own UV-mapped image and ~1:1 with
    # a geometry, so a per-material group has a single consistent texture -- that's
    # what makes a faithful textured export possible (concatenating ACROSS
    # materials is what scrambled UVs in the old colour-only build). Each entry is
    # exported with its texture (UVs preserved) for the textured view + camera
    # feeds, and carries a flat fallback `color` (material base colour) for the
    # cheap no-texture view.
    groups: dict = {}
    for gname, m in items:
        # skip the billboard/overlay cards (aerial grass, decals, puddles, far
        # backdrop) -- they're flat sprites that float as "umbrellas" over the
        # ground, which is jarring once textured. Same set excluded from collision.
        if any(k in _suffix(gname) for k in EXCLUDE_KEYWORDS):
            continue
        mat = getattr(getattr(m, "visual", None), "material", None)
        key = getattr(mat, "name", None) or _category(gname)
        groups.setdefault(key, []).append((gname, m))
    visual = []
    for key, members in groups.items():
        gname0 = members[0][0]
        meshes = [m.copy() for _, m in members]
        for mm in meshes:
            mm.apply_translation(off)
        merged = meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)
        cat = _category(gname0)
        safe = re.sub(r"[^A-Za-z0-9_.-]", "_", str(key))
        cdir = os.path.join(cache, cat, safe)
        os.makedirs(cdir, exist_ok=True)
        objp = os.path.join(cdir, "mesh.obj")
        # The VISUAL stays the opaque merged mesh (clean, not shard-y, ~4x lighter
        # than the cutout). For foliage/grass we ALSO bake the alpha-cutout leaf
        # silhouette as a geometry-only mesh used ONLY for LIDAR collision -- so the
        # lidar's rays pass through the leaf gaps and return on the leaves, while
        # the render shows solid foliage. (pybullet can't render texture alpha, so
        # we can't do true cutout in the VIEW; the cutout lives in the raycast.)
        collision_obj = None
        if _CUTOUT_FOLIAGE and cat in ("grass", "foliage"):
            try:
                cut = _alpha_cutout(merged)
                if cut is not None:
                    cutp = os.path.join(cdir, "mesh_cutout.obj")
                    trimesh.Trimesh(vertices=cut.vertices, faces=cut.faces,
                                    process=False).export(cutp)
                    collision_obj = cutp
                    print(f"    cutout-collision {key}: {len(cut.faces)} faces")
            except Exception as e:
                print(f"    cutout {key} skipped: {e}")
        texp = None
        try:
            merged.export(objp)                       # writes obj + mtl + texture png
            pngs = [f for f in os.listdir(cdir) if f.lower().endswith(".png")]
            if not pngs:
                # trimesh didn't emit one (some PBR paths): save the image ourselves
                mat = getattr(merged.visual, "material", None)
                img = (getattr(mat, "baseColorTexture", None)
                       or getattr(mat, "image", None)) if mat else None
                if img is not None:
                    img.save(os.path.join(cdir, "texture.png")); pngs = ["texture.png"]
            if pngs:
                texp = os.path.join(cdir, pngs[0])
                # pybullet can't render texture alpha -> composite it away so the
                # BLEND cards (foliage/road overlay) don't punch black holes.
                try:
                    _flatten_texture(texp)
                except Exception:
                    pass
        except Exception:
            # fall back to a geometry-only export with the category colour
            trimesh.Trimesh(vertices=merged.vertices, faces=merged.faces,
                            process=False).export(objp)
        # flat fallback colour: the material's base colour if we can read it, else
        # the category colour.
        color = CATEGORY_COLORS.get(cat, [0.5, 0.5, 0.5, 1])
        mat = getattr(getattr(merged, "visual", None), "material", None)
        mc = getattr(mat, "main_color", None) if mat else None
        if mc is not None:
            color = [float(c) / 255.0 if c > 1 else float(c) for c in mc[:3]] + [1.0]
        visual.append({"obj": objp, "tex": texp, "color": color,
                       "category": cat, "material": str(key),
                       "collision_obj": collision_obj})
    data = {"version": MANIFEST_VERSION, "collision": coll, "visual": visual,
            "collision_faces": collision_faces, "offset": off}
    json.dump(data, open(manifest, "w"))
    return manifest
