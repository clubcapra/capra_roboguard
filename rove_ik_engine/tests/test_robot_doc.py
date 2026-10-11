"""Model relocation must preserve IK imports and visualizer mesh responses."""
import json
import unittest
from pathlib import Path
from types import SimpleNamespace

from engine.config import load
from engine.loader import load_robot
from engine.state import EngineState
from engine.transports.bus import StateBus
from engine.transports.ws import HttpWsServer

ROOT = Path(__file__).resolve().parents[2]


class RobotDocLinks(unittest.IsolatedAsyncioTestCase):
    async def test_visualizer_serves_the_loaded_caged_model_mesh(self):
        config = ROOT / "rove_ik_engine/engine.toml"
        project = load_robot(load(config))
        server = HttpWsServer(
            EngineState(project=project), StateBus(), "127.0.0.1:9101",
            input_enabled=False, input_path="/ovis", output_enabled=False,
            output_path="/state", ui_dir=None, data_dir=ROOT / "rove_ik_engine/data",
        )
        mesh = ROOT / "robot_doc/rove_caged_urdf/meshes/cage_2.glb"
        response = await server._http_mesh_by_stem(SimpleNamespace(match_info={"stem": "cage_2"}))
        self.assertEqual(response.body, mesh.read_bytes())
        listing = await server._http_meshes_list(None)
        self.assertEqual(json.loads(listing.text)["cage_2"]["size_bytes"], mesh.stat().st_size)

    def test_standard_export_and_legacy_urdf_fallback_load(self):
        config = load(ROOT / "rove_ik_engine/engine.standard.toml")
        exported = load_robot(config)
        self.assertTrue(exported.assets.mesh_data)
        config.robot.forgebot = "missing-export.forgebot"
        fallback = load_robot(config)
        self.assertTrue(fallback.scene.entities)
        self.assertTrue(fallback.assets.mesh_data)


if __name__ == "__main__":
    unittest.main()
