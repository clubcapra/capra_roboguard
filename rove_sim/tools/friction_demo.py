"""Top-down friction-painting validation clip.

Paints material patches into the world FrictionField, then drives the robot
across them with the field overlaid (translucent colour per cell mu). The robot
visibly loses traction on the ice strip and grips on gravel -- emergent from the
per-contact mu lookup in the brush-track model.

    PYTHONPATH=. ../rove_sim_venv/bin/python tools/friction_demo.py --out media/friction.mp4
"""
import argparse
import subprocess

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw, ImageFont

from rove_sim import runtime
from rove_sim.world.friction import MATERIALS
from rove_sim.control import RoveControl, Tracks

W, H, FPS = 760, 760, 50
HALF = 12.0                   # world half-extent shown (m) -> ortho view
_FONT = ImageFont.load_default()
for _p in ("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
           "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf"):
    try:
        _FONT = ImageFont.truetype(_p, 20); break
    except Exception:
        pass


def _mu_color(mu, default):
    """low mu -> icy blue, nominal -> grey, high mu -> gravel green."""
    if abs(mu - default) < 1e-3:
        return None
    if mu < default:
        t = mu / default
        return (int(120 + 80 * t), int(170 + 60 * t), 255, 150)     # blue-ish
    t = min(1.0, (mu - default) / 0.4)
    return (int(120 - 60 * t), int(180 + 40 * t), int(90 - 40 * t), 150)  # green


def _overlay(field):
    """Render the painted cells to an RGBA image aligned to the ortho view
    (world [-HALF,HALF]^2 around origin -> WxH; world +y is up => flip rows)."""
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    px = W / (2 * HALF)
    s = field.cell * px / 2 + 1
    for j in range(field.ny):
        for i in range(field.nx):
            c = _mu_color(float(field.mu[j, i]), field.default)
            if c is None:
                continue
            wx, wy = field.cell_center(i, j)
            if abs(wx) > HALF or abs(wy) > HALF:
                continue
            u = (wx + HALF) * px
            v = (HALF - wy) * px
            draw.rectangle([u - s, v - s, u + s, v + s], fill=c)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/friction.mp4")
    args = ap.parse_args()

    sim = runtime.build("standard", mode="headless", world="mock",
                        world_overrides={"friction": {"origin": (-12, -12),
                                         "extent": (24, 24), "cell": 0.25}})
    f = sim.world.friction
    # robot front is -X; it drives toward -X. Lay an ICE strip then a GRAVEL strip
    # across the path (a wide lane in y).
    for gx in np.arange(-5.5, -2.0, 0.25):
        for gy in np.arange(-1.5, 2.0, 0.25):
            f.paint(gx, gy, 0.18, MATERIALS["ice"][0])
    for gx in np.arange(-9.5, -6.0, 0.25):
        for gy in np.arange(-1.5, 2.0, 0.25):
            f.paint(gx, gy, 0.18, MATERIALS["gravel"][0])

    for n, idx in sim.robot.link_index.items():
        if "Flipper" in n:
            p.changeVisualShape(sim.robot.body_id, idx, rgbaColor=[.9, .75, .1, 1])
        elif any(k in n for k in ("Base", "Section", "Joint", "robotiq", "finger", "knuckle")):
            p.changeVisualShape(sim.robot.body_id, idx, rgbaColor=[.85, .45, .1, 1])
        elif "Core" in n or "Drum" in n:
            p.changeVisualShape(sim.robot.body_id, idx, rgbaColor=[.25, .27, .3, 1])
    overlay = _overlay(f)

    proj = p.computeProjectionMatrix(-HALF, HALF, -HALF, HALF, 0.1, 100)
    view = p.computeViewMatrix([0, 0, 25], [0, 0, 0], [0, 1, 0])   # top-down, +y up
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "rawvideo", "-pixel_format", "rgba",
         "-video_size", f"{W}x{H}", "-framerate", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-loglevel", "error", args.out], stdin=subprocess.PIPE)

    speeds = []

    def frame(label):
        _, _, rgb, _, _ = p.getCameraImage(
            W, H, view, proj, renderer=sim.engine.camera_renderer_flag,
            lightDirection=[0.2, 0.3, 1.0], lightAmbientCoeff=0.7,
            lightDiffuseCoeff=0.5, shadow=0)
        base = Image.fromarray(np.reshape(rgb, (H, W, 4)).astype(np.uint8)).convert("RGBA")
        base.alpha_composite(overlay)
        d = ImageDraw.Draw(base)
        # bright marker at the robot's projected position (small + dark top-down)
        rp = p.getBasePositionAndOrientation(sim.robot.body_id)[0]
        px = W / (2 * HALF)
        ru, rv = (rp[0] + HALF) * px, (HALF - rp[1]) * px
        d.ellipse([ru - 11, rv - 11, ru + 11, rv + 11], outline=(255, 60, 60), width=4)
        d.rectangle([0, 0, W, 30], fill=(20, 20, 28))
        d.text((10, 5), label, fill=(245, 220, 120), font=_FONT)
        d.text((10, H - 26), "ice (blue)   gravel (green)   nominal (grey)",
               fill=(230, 230, 235), font=_FONT)
        ff.stdin.write(np.asarray(base.convert("RGBA")).astype(np.uint8).tobytes())

    sim.set_intent(RoveControl(tracks=Tracks(1.0, 1.0)))
    prev = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
    for _ in range(int(6.0 * sim.control_hz)):
        sim.step_control(1)
        cur = np.array(p.getBasePositionAndOrientation(sim.robot.body_id)[0])
        v = np.linalg.norm((cur - prev)[:2]) * sim.control_hz
        prev = cur
        speeds.append(v)
        mu_here = f.lookup(cur[0], cur[1])
        surf = "ICE" if mu_here < 0.3 else ("GRAVEL" if mu_here > 0.65 else "floor")
        frame(f"drive forward   surface={surf}  mu={mu_here:.2f}  v={v:4.1f} m/s")
    ff.stdin.close(); ff.wait()
    sim.disconnect()
    print(f"wrote {args.out}  max v={max(speeds):.2f}  min v on patch={min(speeds):.2f}")


if __name__ == "__main__":
    main()
