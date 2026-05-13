"""Exported ForgeBOT IK engine.

Headless server. Receives Ovis twists on a configured input transport,
runs the editor's IK solver against a loaded robot, and streams joint
state out on a configured output transport. Vendors `forgebot.core` so
its behaviour is bit-identical to the editor.
"""
