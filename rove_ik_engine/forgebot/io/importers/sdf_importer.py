"""SDFormat (SDF/Gazebo) importer.

SDF wraps everything in <sdf><world|model>. Links are flat (like URDF),
joints connect parent/child links by name, geometries are nested under
<visual>/<collision>. Pose is `x y z roll pitch yaw` in one element.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET

from ...core.model import (
    Entity,
    JointComponent,
    LinkComponent,
    Project,
    TransformComponent,
    new_entity_id,
)
from ...core.model.components.joint import JointDynamics, JointLimits
from ...core.model.components.link import Geometry, Inertial, InertiaTensor
from ...core.validation.rules import Diagnostic, Severity
from .base import BaseImporter, ImportOptions, ImportResult


SDF_TO_FORGEBOT_JOINT: dict[str, str] = {
    "revolute": "revolute",
    "revolute2": "revolute",
    "prismatic": "prismatic",
    "fixed": "fixed",
    "ball": "ball",
    "screw": "revolute",
    "universal": "revolute",
}


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    from math import cos, sin

    cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
    cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
    cr, sr = cos(roll * 0.5), sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def _parse_pose(elem: ET.Element | None) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    if elem is None or not elem.text:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    parts = [float(v) for v in elem.text.split()]
    if len(parts) != 6:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    xyz = (parts[0], parts[1], parts[2])
    quat = _rpy_to_quat(parts[3], parts[4], parts[5])
    return (xyz, quat)


def _localname(tag: str) -> str:
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag


def _parse_xyz_text(s: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not s:
        return default
    parts = [float(v) for v in s.split()]
    if len(parts) != 3:
        return default
    return (parts[0], parts[1], parts[2])


class SDFImporter(BaseImporter):
    def supported_extensions(self) -> list[str]:
        return [".sdf", ".world", ".xml"]

    def can_import(self, file_path: Path) -> bool:
        if file_path.suffix.lower() not in self.supported_extensions():
            return False
        try:
            tree = ET.parse(file_path)
        except ET.ParseError:
            return False
        return _localname(tree.getroot().tag) == "sdf"

    def import_file(self, file_path: Path, options: ImportOptions | None = None) -> ImportResult:
        diagnostics: list[Diagnostic] = []
        tree = ET.parse(file_path)
        root = tree.getroot()
        if _localname(root.tag) != "sdf":
            raise ValueError(f"not an SDF file (root is <{_localname(root.tag)}>)")

        # Find the first <model>, possibly nested under a <world>.
        model = root.find("model")
        if model is None:
            world = root.find("world")
            if world is not None:
                model = world.find("model")
        if model is None:
            raise ValueError("no <model> element found in SDF")

        project = Project()
        project.manifest.metadata.name = model.get("name", "Untitled SDF")

        link_id_by_name: dict[str, str] = {}

        for link_elem in model.findall("link"):
            ent = self._import_link(link_elem)
            project.scene.add(ent)
            link_id_by_name[ent.name] = ent.id

        for joint_elem in model.findall("joint"):
            self._import_joint(joint_elem, project, link_id_by_name, diagnostics)

        project.scene.roots = [
            eid for eid, e in project.scene.entities.items() if e.parent is None
        ]
        return ImportResult(project=project, diagnostics=diagnostics)

    def _import_link(self, link_elem: ET.Element) -> Entity:
        name = link_elem.get("name", "")
        eid = new_entity_id("link")
        link = LinkComponent(source={"sdf": {"name": name}})

        link_pose, link_quat = _parse_pose(link_elem.find("pose"))

        inertial_elem = link_elem.find("inertial")
        if inertial_elem is not None:
            link.inertial = self._import_inertial(inertial_elem)

        for vis_elem in link_elem.findall("visual"):
            geom = self._import_geometry(vis_elem)
            if geom is not None:
                link.visuals.append(geom)
        for col_elem in link_elem.findall("collision"):
            geom = self._import_geometry(col_elem)
            if geom is not None:
                link.collisions.append(geom)

        ent = Entity(id=eid, name=name)
        ent.attach(TransformComponent(position=link_pose, rotation=link_quat))
        ent.attach(link)
        return ent

    def _import_inertial(self, elem: ET.Element) -> Inertial:
        mass_elem = elem.find("mass")
        mass = float((mass_elem.text or "0").strip()) if mass_elem is not None and mass_elem.text else 0.0
        pose_xyz, pose_quat = _parse_pose(elem.find("pose"))
        inertia = InertiaTensor()
        i_elem = elem.find("inertia")
        if i_elem is not None:
            def _v(tag: str) -> float:
                child = i_elem.find(tag)
                return float((child.text or "0").strip()) if child is not None and child.text else 0.0
            inertia = InertiaTensor(
                ixx=_v("ixx"), iyy=_v("iyy"), izz=_v("izz"),
                ixy=_v("ixy"), ixz=_v("ixz"), iyz=_v("iyz"),
            )
        return Inertial(mass=mass, origin=pose_xyz, origin_rotation=pose_quat, inertia=inertia)

    def _import_geometry(self, container: ET.Element) -> Geometry | None:
        geom_elem = container.find("geometry")
        if geom_elem is None:
            return None
        pose_xyz, pose_quat = _parse_pose(container.find("pose"))
        g = Geometry(origin=pose_xyz, origin_rotation=pose_quat)
        material_elem = container.find("material")
        if material_elem is not None:
            script = material_elem.find("script/name")
            if script is not None and script.text:
                g.material = script.text.strip()

        for child in geom_elem:
            tag = _localname(child.tag)
            if tag == "box":
                size = _parse_xyz_text((child.find("size").text if child.find("size") is not None else None), (1.0, 1.0, 1.0))
                g.primitive = "box"
                g.primitive_params = {"x": size[0], "y": size[1], "z": size[2]}
            elif tag == "sphere":
                r_elem = child.find("radius")
                g.primitive = "sphere"
                g.primitive_params = {"radius": float((r_elem.text or "1").strip()) if r_elem is not None else 1.0}
            elif tag == "cylinder":
                r_elem = child.find("radius")
                l_elem = child.find("length")
                g.primitive = "cylinder"
                g.primitive_params = {
                    "radius": float((r_elem.text or "1").strip()) if r_elem is not None else 1.0,
                    "length": float((l_elem.text or "1").strip()) if l_elem is not None else 1.0,
                }
            elif tag == "mesh":
                uri_elem = child.find("uri")
                if uri_elem is not None and uri_elem.text:
                    g.mesh = Path(uri_elem.text.strip()).stem
        return g

    def _import_joint(
        self,
        joint_elem: ET.Element,
        project: Project,
        link_id_by_name: dict[str, str],
        diagnostics: list[Diagnostic],
    ) -> None:
        name = joint_elem.get("name", "")
        sdf_type = joint_elem.get("type", "fixed")
        forge_type = SDF_TO_FORGEBOT_JOINT.get(sdf_type, "fixed")

        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        parent_link_name = (parent_elem.text or "").strip() if parent_elem is not None else ""
        child_link_name = (child_elem.text or "").strip() if child_elem is not None else ""
        parent_eid = link_id_by_name.get(parent_link_name, "")
        child_eid = link_id_by_name.get(child_link_name, "")

        if not parent_eid or not child_eid:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="sdf.dangling_joint",
                    message=f"joint '{name}' references unknown link(s): "
                    f"parent={parent_link_name!r} child={child_link_name!r}",
                )
            )
            return

        eid = new_entity_id("joint")
        pose_xyz, pose_quat = _parse_pose(joint_elem.find("pose"))

        axis = (0.0, 0.0, 1.0)
        limits = None
        axis_elem = joint_elem.find("axis")
        if axis_elem is not None:
            xyz_elem = axis_elem.find("xyz")
            if xyz_elem is not None and xyz_elem.text:
                axis = _parse_xyz_text(xyz_elem.text, axis)
            limit_elem = axis_elem.find("limit")
            if limit_elem is not None:
                def _f(tag: str) -> float:
                    child = limit_elem.find(tag)
                    return float((child.text or "0").strip()) if child is not None and child.text else 0.0
                limits = JointLimits(
                    lower=_f("lower"), upper=_f("upper"),
                    effort=_f("effort"), velocity=_f("velocity"),
                )

        dynamics = None
        if axis_elem is not None:
            dyn_elem = axis_elem.find("dynamics")
            if dyn_elem is not None:
                def _g(tag: str) -> float:
                    child = dyn_elem.find(tag)
                    return float((child.text or "0").strip()) if child is not None and child.text else 0.0
                dynamics = JointDynamics(damping=_g("damping"), friction=_g("friction"))

        joint = JointComponent(
            type=forge_type,  # type: ignore[arg-type]
            axis=axis,
            parent_link=parent_eid,
            child_link=child_eid,
            limits=limits,
            dynamics=dynamics,
            source={"sdf": {"name": name, "type": sdf_type}},
        )
        ent = Entity(id=eid, name=name)
        ent.attach(TransformComponent(position=pose_xyz, rotation=pose_quat))
        ent.attach(joint)
        project.scene.add(ent, parent=parent_eid)
        if child_eid in project.scene.entities:
            project.scene.reparent(child_eid, eid)
