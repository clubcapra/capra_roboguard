"""FrictionField raster: paint / lookup / persistence (no pybullet)."""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim.world.friction import FrictionField, MATERIALS


def test_paint_lookup_and_default():
    f = FrictionField(origin=(-10, -10), extent=(20, 20), cell=0.5, default=0.6)
    assert abs(f.lookup(0, 0) - 0.6) < 1e-5          # unpainted -> default
    assert f.lookup(100, 100) == 0.6                 # outside grid -> default
    f.paint_material(2.0, 1.0, 1.0, "ice")
    assert abs(f.lookup(2.0, 1.0) - MATERIALS["ice"][0]) < 1e-6
    assert abs(f.lookup(8.0, 8.0) - 0.6) < 1e-5      # outside the brush -> default


def test_save_load_roundtrip():
    f = FrictionField(origin=(-5, -5), extent=(10, 10), cell=0.25, default=0.6)
    f.paint_material(0, 0, 0.8, "gravel")
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "fric.json")
        f.save(path)
        g = FrictionField.load(path)
    assert (g.nx, g.ny) == (f.nx, f.ny)
    assert abs(g.lookup(0, 0) - MATERIALS["gravel"][0]) < 1e-6
    assert abs(g.lookup(4, 4) - 0.6) < 1e-5
