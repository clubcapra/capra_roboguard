"""Check relocation links without starting physics or connecting to hardware.

Run from capra_roboguard: python3 -m unittest discover -s robot_doc -v
"""
import unittest
from pathlib import Path
import tomllib
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class ModelLinks(unittest.TestCase):
    def test_every_urdf_mesh_resolves(self):
        models = sorted(p for p in (ROOT / "robot_doc").rglob("*.urdf")
                        if ".rove_sim_cache" not in p.parts)
        self.assertEqual(len(models), 4)
        for model in models:
            for mesh in ET.parse(model).getroot().iter("mesh"):
                filename = mesh.attrib["filename"]
                with self.subTest(model=model.name, mesh=filename):
                    self.assertFalse(Path(filename).is_absolute())
                    self.assertTrue((model.parent / filename).is_file())

    def test_ik_configs_resolve_moved_models(self):
        for filename in ("engine.toml", "engine.standard.toml"):
            config = ROOT / "rove_ik_engine" / filename
            data = tomllib.loads(config.read_text())
            model = (config.parent / data["robot"]["urdf"]).resolve()
            with self.subTest(config=filename):
                self.assertTrue(model.is_relative_to(ROOT / "robot_doc"))
                self.assertTrue(model.is_file())


if __name__ == "__main__":
    unittest.main()
