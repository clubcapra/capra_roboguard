"""URDF importer.

Maps URDF elements to ForgeBOT entities + components:
  <link>     -> entity + transform + link
  <joint>    -> entity + transform + joint   (transform = <origin> on joint)
  <visual>   -> link.visuals[]
  <collision>-> link.collisions[]
  <inertial> -> link.inertial
  <material> -> assets.materials[name]
  <mesh>     -> assets.mesh_files[stem] = <local resolved path>

Round-trip metadata is stashed on each component's `source` field so the
URDF exporter can reproduce names and ordering.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

from ...core.model import (
    Entity,
    JointComponent,
    LinkComponent,
    Manifest,
    Material,
    MeshAsset,
    Project,
    TransformComponent,
    new_entity_id,
)
from ...core.model.components.joint import JointDynamics, JointLimits
from ...core.model.components.link import Geometry, Inertial, InertiaTensor
from ...core.validation.rules import Diagnostic, Severity
from .base import BaseImporter, ImportOptions, ImportResult


URDF_TO_FORGEBOT_JOINT: dict[str, str] = {
    "revolute": "revolute",
    "continuous": "continuous",
    "prismatic": "prismatic",
    "fixed": "fixed",
    "floating": "floating",
    "planar": "planar",
}


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert RPY (URDF/ROS convention) to quaternion (x, y, z, w)."""
    from math import cos, sin

    cy, sy = cos(yaw * 0.5), sin(yaw * 0.5)
    cp, sp = cos(pitch * 0.5), sin(pitch * 0.5)
    cr, sr = cos(roll * 0.5), sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def _parse_origin(elem: ET.Element | None) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    if elem is None:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    xyz_str = elem.get("xyz", "0 0 0")
    rpy_str = elem.get("rpy", "0 0 0")
    xyz = tuple(float(v) for v in xyz_str.split())
    rpy = tuple(float(v) for v in rpy_str.split())
    if len(xyz) != 3 or len(rpy) != 3:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    return (xyz, _rpy_to_quat(*rpy))  # type: ignore[return-value]


def _parse_xyz(s: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not s:
        return default
    parts = [float(v) for v in s.split()]
    if len(parts) != 3:
        return default
    return (parts[0], parts[1], parts[2])


def _parse_geometry(geom: ET.Element, mesh_dir: Path | None) -> tuple[Geometry, str | None]:
    """Return (Geometry, mesh_local_path_or_None)."""
    g = Geometry()
    mesh_path: str | None = None
    for child in geom:
        tag = _localname(child.tag)
        if tag == "mesh":
            filename = child.get("filename", "")
            stem = Path(filename).stem
            g.mesh = stem
            scale_str = child.get("scale")
            if scale_str:
                s = _parse_xyz(scale_str, (1.0, 1.0, 1.0))
                g.scale = s
            mesh_path = _resolve_mesh_path(filename, mesh_dir)
        elif tag == "box":
            g.primitive = "box"
            sz = _parse_xyz(child.get("size"), (1.0, 1.0, 1.0))
            g.primitive_params = {"x": sz[0], "y": sz[1], "z": sz[2]}
        elif tag == "sphere":
            g.primitive = "sphere"
            g.primitive_params = {"radius": float(child.get("radius", "1") or 1.0)}
        elif tag == "cylinder":
            g.primitive = "cylinder"
            g.primitive_params = {
                "radius": float(child.get("radius", "1") or 1.0),
                "length": float(child.get("length", "1") or 1.0),
            }
    return g, mesh_path


def _resolve_mesh_path(filename: str, mesh_dir: Path | None) -> str | None:
    """URDF meshes use file:// or package:// or relative paths.

    For phase 1 we just strip schemes and resolve relative paths against the
    URDF's directory. Missing files are reported as diagnostics by the caller.
    """
    if not filename:
        return None
    s = filename
    if s.startswith("file://"):
        s = s[7:]
    if s.startswith("package://"):
        # Drop the package://<pkg>/ prefix; treat the rest as relative.
        s = s[len("package://"):].split("/", 1)[-1]
    p = Path(s)
    if not p.is_absolute() and mesh_dir is not None:
        p = mesh_dir / p
    return str(p)


def _localname(tag: str) -> str:
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag


class URDFImporter(BaseImporter):
    def supported_extensions(self) -> list[str]:
        return [".urdf", ".xml"]

    def can_import(self, file_path: Path) -> bool:
        if file_path.suffix.lower() not in self.supported_extensions():
            return False
        try:
            tree = ET.parse(file_path)
        except ET.ParseError:
            return False
        return _localname(tree.getroot().tag) == "robot"

    def import_file(self, file_path: Path, options: ImportOptions | None = None) -> ImportResult:
        opts = options or ImportOptions()
        diagnostics: list[Diagnostic] = []
        tree = ET.parse(file_path)
        root = tree.getroot()
        if _localname(root.tag) != "robot":
            raise ValueError(f"not a URDF file (root is <{_localname(root.tag)}>)")

        project = Project()
        project.manifest = Manifest()
        project.manifest.metadata.name = root.get("name", "Untitled URDF")

        mesh_dir = file_path.parent
        link_id_by_name: dict[str, str] = {}

        # Pass 1: links
        for link_elem in root.iter():
            if _localname(link_elem.tag) != "link":
                continue
            ent = self._import_link(link_elem, mesh_dir, project, diagnostics)
            project.scene.add(ent, parent=None)  # parent set in pass 2
            link_id_by_name[ent.name] = ent.id

        # Pass 2: joints
        for joint_elem in root.iter():
            if _localname(joint_elem.tag) != "joint":
                continue
            self._import_joint(joint_elem, project, link_id_by_name, diagnostics)

        # Pass 3: materials at robot scope
        for mat_elem in root.findall("material"):
            name, mat = self._import_material(mat_elem)
            if name:
                project.assets.materials[name] = mat

        # Pass 4: derive scene roots from parent pointers (any link with no parent is a root)
        project.scene.roots = [
            eid for eid, e in project.scene.entities.items() if e.parent is None
        ]

        return ImportResult(project=project, diagnostics=diagnostics)

    # ----- helpers -----

    def _import_link(
        self,
        link_elem: ET.Element,
        mesh_dir: Path,
        project: Project,
        diagnostics: list[Diagnostic],
    ) -> Entity:
        name = link_elem.get("name", "")
        eid = new_entity_id("link")
        link = LinkComponent(source={"urdf": {"name": name}})

        inertial_elem = link_elem.find("inertial")
        if inertial_elem is not None:
            link.inertial = self._import_inertial(inertial_elem)

        for visual_elem in link_elem.findall("visual"):
            geom_elem = visual_elem.find("geometry")
            if geom_elem is None:
                continue
            origin_xyz, origin_rot = _parse_origin(visual_elem.find("origin"))
            geom, mesh_path = _parse_geometry(geom_elem, mesh_dir)
            geom.origin = origin_xyz
            geom.origin_rotation = origin_rot
            mat_elem = visual_elem.find("material")
            if mat_elem is not None:
                mat_name = mat_elem.get("name")
                if mat_name:
                    geom.material = mat_name
                    if mat_elem.find("color") is not None or mat_elem.find("texture") is not None:
                        _, mat = self._import_material(mat_elem)
                        project.assets.materials[mat_name] = mat
            if mesh_path:
                self._register_mesh(project, geom.mesh, mesh_path, diagnostics)
            link.visuals.append(geom)

        for col_elem in link_elem.findall("collision"):
            geom_elem = col_elem.find("geometry")
            if geom_elem is None:
                continue
            origin_xyz, origin_rot = _parse_origin(col_elem.find("origin"))
            geom, mesh_path = _parse_geometry(geom_elem, mesh_dir)
            geom.origin = origin_xyz
            geom.origin_rotation = origin_rot
            if mesh_path:
                self._register_mesh(project, geom.mesh, mesh_path, diagnostics)
            link.collisions.append(geom)

        ent = Entity(id=eid, name=name)
        ent.attach(TransformComponent())  # link-local transform; gets adjusted by joints
        ent.attach(link)
        return ent

    def _register_mesh(
        self,
        project: Project,
        stem: str | None,
        local_path: str,
        diagnostics: list[Diagnostic],
    ) -> None:
        if not stem:
            return
        p = Path(local_path)
        if not p.is_file():
            diagnostics.append(
                Diagnostic(
                    severity=Severity.WARNING,
                    code="urdf.mesh_not_found",
                    message=f"mesh not found: {local_path}",
                )
            )
            return
        # Read bytes into the project so they survive past the importer's lifetime
        # (file may be temp). Also keep the original path for round-trip exports.
        try:
            data = p.read_bytes()
            project.assets.mesh_data[stem] = MeshAsset(
                suffix=p.suffix.lower() or ".stl",
                data=data,
            )
        except OSError as e:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.WARNING,
                    code="urdf.mesh_read_failed",
                    message=f"could not read mesh {local_path}: {e}",
                )
            )
        project.assets.mesh_files[stem] = local_path

    def _import_inertial(self, elem: ET.Element) -> Inertial:
        mass_elem = elem.find("mass")
        mass = float(mass_elem.get("value", "0") if mass_elem is not None else "0") or 0.0
        origin_xyz, origin_rot = _parse_origin(elem.find("origin"))
        inertia = InertiaTensor()
        i_elem = elem.find("inertia")
        if i_elem is not None:
            inertia = InertiaTensor(
                ixx=float(i_elem.get("ixx", "0") or 0),
                iyy=float(i_elem.get("iyy", "0") or 0),
                izz=float(i_elem.get("izz", "0") or 0),
                ixy=float(i_elem.get("ixy", "0") or 0),
                ixz=float(i_elem.get("ixz", "0") or 0),
                iyz=float(i_elem.get("iyz", "0") or 0),
            )
        return Inertial(mass=mass, origin=origin_xyz, origin_rotation=origin_rot, inertia=inertia)

    def _import_joint(
        self,
        joint_elem: ET.Element,
        project: Project,
        link_id_by_name: dict[str, str],
        diagnostics: list[Diagnostic],
    ) -> None:
        name = joint_elem.get("name", "")
        urdf_type = joint_elem.get("type", "fixed")
        joint_type = URDF_TO_FORGEBOT_JOINT.get(urdf_type, "fixed")

        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        parent_link_name = parent_elem.get("link", "") if parent_elem is not None else ""
        child_link_name = child_elem.get("link", "") if child_elem is not None else ""
        parent_eid = link_id_by_name.get(parent_link_name, "")
        child_eid = link_id_by_name.get(child_link_name, "")

        if not parent_eid or not child_eid:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.ERROR,
                    code="urdf.dangling_joint",
                    message=f"joint '{name}' references unknown link(s): "
                    f"parent={parent_link_name!r} child={child_link_name!r}",
                )
            )
            return

        eid = new_entity_id("joint")
        origin_xyz, origin_rot = _parse_origin(joint_elem.find("origin"))

        axis = (0.0, 0.0, 1.0)
        axis_elem = joint_elem.find("axis")
        if axis_elem is not None:
            axis = _parse_xyz(axis_elem.get("xyz"), axis)

        limits = None
        lim_elem = joint_elem.find("limit")
        if lim_elem is not None:
            limits = JointLimits(
                lower=float(lim_elem.get("lower", "0") or 0),
                upper=float(lim_elem.get("upper", "0") or 0),
                effort=float(lim_elem.get("effort", "0") or 0),
                velocity=float(lim_elem.get("velocity", "0") or 0),
            )

        dynamics = None
        dyn_elem = joint_elem.find("dynamics")
        if dyn_elem is not None:
            dynamics = JointDynamics(
                damping=float(dyn_elem.get("damping", "0") or 0),
                friction=float(dyn_elem.get("friction", "0") or 0),
            )

        joint = JointComponent(
            type=joint_type,  # type: ignore[arg-type]
            axis=axis,
            parent_link=parent_eid,
            child_link=child_eid,
            limits=limits,
            dynamics=dynamics,
            source={"urdf": {"name": name}},
        )

        ent = Entity(id=eid, name=name)
        ent.attach(TransformComponent(position=origin_xyz, rotation=origin_rot))
        ent.attach(joint)

        # Insert joint between parent and child links.
        # parent_link --[joint]--> child_link
        project.scene.add(ent, parent=parent_eid)
        # Reparent the child link under this joint.
        if child_eid in project.scene.entities:
            project.scene.reparent(child_eid, eid)

    def _import_material(self, elem: ET.Element) -> tuple[str, Material]:
        name = elem.get("name", "") or ""
        color_elem = elem.find("color")
        rgba = (0.8, 0.8, 0.8, 1.0)
        if color_elem is not None:
            parts = [float(v) for v in (color_elem.get("rgba", "") or "").split()]
            if len(parts) == 4:
                rgba = (parts[0], parts[1], parts[2], parts[3])
        tex_elem = elem.find("texture")
        texture = None
        if tex_elem is not None:
            tex_file = tex_elem.get("filename")
            if tex_file:
                texture = Path(tex_file).stem
        return name, Material(color=rgba, texture=texture)
