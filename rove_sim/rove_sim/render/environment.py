"""Local Radiance RGBE panoramas and precomputed diffuse environment lighting.

The nine-coefficient irradiance approximation follows Ramamoorthi/Hanrahan
(SIGGRAPH 2001). The bright sun is extracted separately so it can cast shadows.
No environment integration or asset download takes place in the frame loop.
"""

from pathlib import Path
import re
import numpy as np


def read_hdr(path):
    """Read standard, scanline-RLE Radiance RGBE files in linear RGB."""
    with Path(path).open("rb") as stream:
        if stream.readline().strip() not in (b"#?RADIANCE", b"#?RGBE"):
            raise ValueError("Not a Radiance HDR image")
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Truncated HDR header")
            if not line.strip():
                break
            header.append(line.strip())
        if b"FORMAT=32-bit_rle_rgbe" not in header:
            raise ValueError("Only Radiance RGBE format is supported")
        match = re.fullmatch(rb"([+-])Y (\d+) ([+-])X (\d+)", stream.readline().strip())
        if not match:
            raise ValueError("HDR requires Y/X scanline orientation")
        sy, height, sx, width = match.groups()
        height, width = int(height), int(width)
        if not 1 <= height <= 16384 or not 8 <= width <= 32767:
            raise ValueError("Invalid HDR dimensions")
        pixels = np.empty((height, width, 4), np.uint8)
        for row in range(height):
            marker = stream.read(4)
            if marker != bytes((2, 2, width >> 8, width & 255)):
                raise ValueError("Expected scanline-RLE HDR data")
            for channel in range(4):
                column = 0
                while column < width:
                    raw = stream.read(1)
                    if not raw or raw[0] == 0:
                        raise ValueError("Truncated or invalid HDR run")
                    code = raw[0]
                    count = code - 128 if code > 128 else code
                    if column + count > width:
                        raise ValueError("HDR run exceeds scanline")
                    payload = stream.read(1 if code > 128 else count)
                    if len(payload) != (1 if code > 128 else count):
                        raise ValueError("Truncated HDR pixels")
                    pixels[row, column : column + count, channel] = (
                        payload[0] if code > 128 else np.frombuffer(payload, np.uint8)
                    )
                    column += count
        rgb = np.ldexp(
            pixels[..., :3].astype(np.float32),
            pixels[..., 3, None].astype(np.int32) - 136,
        )
        rgb[pixels[..., 3] == 0] = 0
        if sy == b"+":
            rgb = rgb[::-1]
        if sx == b"-":
            rgb = rgb[:, ::-1]
        return np.ascontiguousarray(rgb, np.float32)


def basis(directions):
    directions = np.asarray(directions, dtype=np.float64)
    x, y, z = np.moveaxis(directions, -1, 0)
    return np.stack(
        [
            np.full_like(x, 0.282095),
            0.488603 * y,
            0.488603 * z,
            0.488603 * x,
            1.092548 * x * y,
            1.092548 * y * z,
            0.315392 * (3 * z * z - 1),
            1.092548 * x * z,
            0.546274 * (x * x - y * y),
        ],
        axis=-1,
    )


def environment_lighting(rgb, rotation_deg=0, strength=1):
    """Extract a shadow-casting sun and normalized, diffuse sky lighting.

    Azimuth rotation applies to both the visible sky and light directions.
    Exposure is normalized against the non-solar sky, retaining HDR colour and
    angular distribution rather than using tonemapped JPG lighting.
    """
    # Integrate an area-filtered copy, retaining the full panorama for the
    # visible sky. Bounds startup RAM independently of HDRI resolution.
    h, w = rgb.shape[:2]
    if w > 512:
        from PIL import Image

        shape = (512, max(1, round(h * 512 / w)))
        rgb = np.stack(
            [
                np.asarray(
                    Image.fromarray(rgb[..., c]).resize(shape, Image.Resampling.BOX)
                )
                for c in range(3)
            ],
            axis=-1,
        )
        h, w = rgb.shape[:2]
    theta = (np.arange(h) + 0.5) * np.pi / h
    phi = (np.arange(w) + 0.5) * 2 * np.pi / w - np.pi + np.radians(rotation_deg)
    z = np.broadcast_to(np.cos(theta)[:, None], (h, w))
    directions = np.stack(
        [np.sin(theta)[:, None] * np.cos(phi), np.sin(theta)[:, None] * np.sin(phi), z],
        axis=-1,
    )
    weights = np.broadcast_to(
        np.sin(theta)[:, None] * 2 * np.pi * np.pi / (h * w), (h, w)
    )
    luminance = rgb @ [0.2126, 0.7152, 0.0722]
    cap = max(float(np.percentile(luminance[z > 0], 99.5)), 1e-6)
    ambient = rgb * np.minimum(1, cap / np.maximum(luminance, 1e-6))[..., None]
    excess = rgb - ambient
    solar = excess @ [0.2126, 0.7152, 0.0722] * weights * (z > 0)
    sun = np.sum(directions * solar[..., None], axis=(0, 1))
    norm = np.linalg.norm(sun)
    sun = sun / norm if norm > 1e-8 else np.array([0, 0, 1.0])
    average = np.sum((ambient @ [0.2126, 0.7152, 0.0722]) * weights * (z > 0)) / (
        2 * np.pi
    )
    exposure = 0.55 * strength / max(float(average), 1e-6)
    coefficients = np.einsum("hwk,hwc,hw->kc", basis(directions), ambient, weights)
    coefficients *= (
        np.array([1, 2 / 3, 2 / 3, 2 / 3, 1 / 4, 1 / 4, 1 / 4, 1 / 4, 1 / 4])[:, None]
        * exposure
    )
    sun_colour = (
        np.sum(excess * (weights * (z > 0))[..., None], axis=(0, 1)) * exposure / np.pi
    )
    return (
        coefficients.astype("f4"),
        sun.astype("f4"),
        sun_colour.astype("f4"),
        exposure,
    )
