#!/usr/bin/env python3
"""Import original local texture images recorded in a terrain manifest.

Use an original model download or the author's packed source images. Web viewer
texture derivatives may have rearranged blocks and are not source assets.
Workers never download images. All source files are checked before any asset is
replaced; colour and opacity receive the same explicit UV-orientation transform.
"""

import argparse
import hashlib
import io
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import Image, ImageOps
from rove_sim.world.mesh_assets import asset_path
from rove_sim.world.terrain import MANIFEST_VERSION


def prepare_texture(entry, source_dir):
    spec = entry["source_images"]
    images = {}
    hashes = {}
    for channel in ("diffuse", "opacity"):
        name = spec.get(channel)
        if not name:
            continue
        path = (source_dir / name).resolve()
        if not path.is_relative_to(source_dir.resolve()):
            raise ValueError(f"Source image leaves texture directory: {name}")
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        expected = entry.get("source_sha256", {}).get(name)
        if expected and digest != expected:
            raise ValueError(f"Source image checksum mismatch: {name}")
        with Image.open(io.BytesIO(payload)) as image:
            images[channel] = image.convert("RGBA" if channel == "diffuse" else "L")
        hashes[name] = digest
    image = images["diffuse"]
    if "opacity" in images:
        if images["opacity"].size != image.size:
            raise ValueError(f"Colour and opacity sizes differ: {entry['material']}")
        image.putalpha(images["opacity"])
    if spec.get("flip_y", False):
        image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    output = io.BytesIO()
    image.save(output, format="PNG")
    return output.getvalue(), hashes, image.getextrema()[3][0] < 255


def prepare_material_maps(entry, source_dir):
    prepared = []
    for channel in ("normal", "roughness"):
        name = entry["source_images"].get(channel)
        if not name:
            continue
        path = (source_dir / name).resolve()
        if not path.is_relative_to(source_dir.resolve()):
            raise ValueError(f"Source image leaves texture directory: {name}")
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        expected = entry.get("source_sha256", {}).get(name)
        if expected and digest != expected:
            raise ValueError(f"Source image checksum mismatch: {name}")
        with Image.open(io.BytesIO(payload)) as original:
            image = original.convert("RGB" if channel == "normal" else "L")
        image.thumbnail((1024, 1024), Image.Resampling.LANCZOS)
        if entry["source_images"].get("flip_y", False):
            image = ImageOps.flip(image)
            if channel == "normal":
                r, g, b = image.split()
                image = Image.merge("RGB", (r, ImageOps.invert(g), b))
        output = io.BytesIO()
        image.save(output, format="PNG")
        prepared.append((channel, output.getvalue(), name, digest))
    return prepared


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("source_dir", type=Path)
    args = parser.parse_args()
    data = json.loads(args.manifest.read_text())
    prepared = [
        (entry, *prepare_texture(entry, args.source_dir))
        for entry in data["visual"]
        if entry.get("source_images")
    ]
    materials = [
        (entry, prepare_material_maps(entry, args.source_dir)) for entry, *_ in prepared
    ]
    for entry, payload, hashes, masked in prepared:
        target = asset_path(args.manifest, entry["tex"])
        temp = target.with_name(target.stem + ".tmp.png")
        temp.write_bytes(payload)
        temp.replace(target)
        entry.setdefault("source_sha256", {}).update(hashes)
        entry["texture_sha256"] = hashlib.sha256(payload).hexdigest()
        entry["alpha_mode"] = "MASK" if masked else "OPAQUE"
        print(entry["material"], entry["alpha_mode"])
    for entry, maps in materials:
        for channel, payload, name, digest in maps:
            target = asset_path(args.manifest, entry["tex"])
            target = target.with_name(target.stem + "_" + channel + ".png")
            temp = target.with_name(target.stem + ".tmp.png")
            temp.write_bytes(payload)
            temp.replace(target)
            entry[channel + "_tex"] = str(target.relative_to(args.manifest.parent))
            entry[channel + "_sha256"] = hashlib.sha256(payload).hexdigest()
            entry["source_sha256"][name] = digest
    data["alpha_preserved"] = True
    data["version"] = MANIFEST_VERSION
    args.manifest.write_text(json.dumps(data, indent=2) + "\n")


if __name__ == "__main__":
    main()
