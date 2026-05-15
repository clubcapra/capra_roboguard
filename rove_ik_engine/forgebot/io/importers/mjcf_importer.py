"""MJCF (MuJoCo) importer.

MJCF differs from URDF mainly in shape: bodies are nested in XML, not flat.
A `<body>` becomes a link entity; nested `<body>` elements become children.
A `<joint>` inside a body is the joint that connects that body to its parent.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET

from ...core.model import (
    Entity,
    JointComponent,
    LinkComponent,
    Material,
    Project,
    TransformComponent,
    new_entity_id,
)
from ...core.model.components.joint import JointLimits
from ...core.model.components.link import Geometry, Inertial, InertiaTensor
from ...core.validation.rules import Diagnostic, Severity
from .base import BaseImporter, ImportOptions, ImportResult


MJCF_TO_FORGEBOT_JOINT: dict[str, str] = {
    "hinge": "revolute",
    "slide": "prismatic",
    "ball": "ball",
    "free": "floating",
}


def _parse_xyz(s: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not s:
        return default
    parts = [float(v) for v in s.split()]
    if len(parts) < 3:
        return default
    return (parts[0], parts[1], parts[2])


def _parse_quat(s: str | None) -> tuple[float, float, float, float]:
    """MuJoCo stores quaternions as (w, x, y, z); we use (x, y, z, w)."""
    if not s:
        return (0.0, 0.0, 0.0, 1.0)
    parts = [float(v) for v in s.split()]
    if len(parts) != 4:
        return (0.0, 0.0, 0.0, 1.0)
    w, x, y, z = parts
    return (x, y, z, w)


def _parse_size(s: str | None) -> list[float]:
    return [float(v) for v in (s or "").split()]


class MJCFImporter(BaseImporter):
    def supported_extensions(self) -> list[str]:
        return [".mjcf", ".xml"]

    def can_import(self, file_path: Path) -> bool:
        if file_path.suffix.lower() not in self.supported_extensions():
            return False
        try:
            tree = ET.parse(file_path)
        except ET.ParseError:
            return False
        return tree.getroot().tag == "mujoco"

    def import_file(self, file_path: Path, options: ImportOptions | None = None) -> ImportResult:
        diagnostics: list[Diagnostic] = []
        tree = ET.parse(file_path)
        root = tree.getroot()
        if root.tag != "mujoco":
            raise ValueError(f"not an MJCF file (root is <{root.tag}>)")

        project = Project()
        project.manifest.metadata.name = root.get("model", "Untitled MJCF")

        # Materials at <asset>
        asset_root = root.find("asset")
        if asset_root is not None:
            for mat in asset_root.findall("material"):
                name = mat.get("name", "")
                if not name:
                    continue
                rgba_str = mat.get("rgba", "0.8 0.8 0.8 1")
                parts = [float(v) for v in rgba_str.split()]
                rgba = (parts + [1.0] * 4)[:4]
                project.assets.materials[name] = Material(
                    color=(rgba[0], rgba[1], rgba[2], rgba[3])
                )

        worldbody = root.find("worldbody")
        if worldbody is None:
            diagnostics.append(
                Diagnostic(
                    severity=Severity.WARNING,
                    code="mjcf.no_worldbody",
                    message="MJCF has no <worldbody>; nothing to import",
                )
            )
            return ImportResult(project=project, diagnostics=diagnostics)

        # Recursively walk bodies. Each <body> becomes (joint?, link) parented appropriately.
        for body_elem in worldbody.findall("body"):
            self._import_body(body_elem, parent_id=None, project=project, diagnostics=diagnostics)

        project.scene.roots = [
            eid for eid, e in project.scene.entities.items() if e.parent is None
        ]
        return ImportResult(project=project, diagnostics=diagnostics)

    # ----- helpers -----

    def _import_body(
        self,
        body_elem: ET.Element,
        parent_id: str | None,
        project: Project,
        diagnostics: list[Diagnostic],
    ) -> None:
        body_name = body_elem.get("name", "") or f"body_{len(project.scene.entities)}"
        pos = _parse_xyz(body_elem.get("pos"))
        quat = _parse_quat(body_elem.get("quat"))

        # Build the link entity for this body.
        link_eid = new_entity_id("link")
        link_entity = Entity(id=link_eid, name=body_name)
        link_entity.attach(TransformComponent())  # body's offset goes on the joint(s) above it
        link_component = LinkComponent(source={"mjcf": {"name": body_name}})

        # Inertial
        inertial_elem = body_elem.find("inertial")
        if inertial_elem is not None:
            link_component.inertial = self._import_inertial(inertial_elem)

        # Geoms become visuals + collisions (MJCF doesn't separate them; we duplicate).
        for geom_elem in body_elem.findall("geom"):
            g = self._import_geom(geom_elem)
            link_component.visuals.append(g)
            link_component.collisions.append(g.model_copy(deep=True))

        link_entity.attach(link_component)

        # Joints: zero or more <joint> children mean this body is connected to its
        # parent through those joints (plural is rare). For zero joints, the body
        # is welded to its parent — we synthesize a "fixed" joint to represent
        # the body's pose offset. For one or more, the *first* joint carries
        # the offset; extras live as siblings (ball-of-joints case).
        joint_elems = body_elem.findall("joint")

        if parent_id is None:
            # Top-level body: just attach to root with the body's pose on the link's transform.
            link_entity.components["transform"] = TransformComponent(position=pos, rotation=quat)
            project.scene.add(link_entity, parent=None)
            current_link_parent = link_eid
        elif not joint_elems:
            # Welded under parent: synthesize a fixed joint with the body's offset.
            joint_eid = self._make_synthetic_fixed_joint(
                project, parent_id, link_eid, pos, quat, body_name
            )
            project.scene.add(link_entity, parent=joint_eid)
            current_link_parent = link_eid
        else:
            # First joint carries the body's pos/quat offset; subsequent joints chain.
            first_joint = joint_elems[0]
            first_joint_eid = self._make_joint_entity(
                project, first_joint, parent_id, link_eid, pos, quat
            )
            project.scene.add(link_entity, parent=first_joint_eid)
            current_link_parent = link_eid

            # Extra joints would mean concatenated DOFs; rare. We log and ignore extras
            # (would need an intermediate massless link to model properly).
            for extra in joint_elems[1:]:
                diagnostics.append(
                    Diagnostic(
                        severity=Severity.WARNING,
                        code="mjcf.multi_joint_body",
                        message=f"body '{body_name}' has multiple <joint>s — only the first is imported",
                    )
                )

        # Recurse into nested bodies.
        for child_body in body_elem.findall("body"):
            self._import_body(child_body, current_link_parent, project, diagnostics)

    def _import_inertial(self, elem: ET.Element) -> Inertial:
        mass = float(elem.get("mass", "0") or 0)
        pos = _parse_xyz(elem.get("pos"))
        quat = _parse_quat(elem.get("quat"))
        diaginertia = _parse_size(elem.get("diaginertia"))
        ixx = diaginertia[0] if len(diaginertia) > 0 else 0.0
        iyy = diaginertia[1] if len(diaginertia) > 1 else 0.0
        izz = diaginertia[2] if len(diaginertia) > 2 else 0.0
        return Inertial(
            mass=mass,
            origin=pos,
            origin_rotation=quat,
            inertia=InertiaTensor(ixx=ixx, iyy=iyy, izz=izz),
        )

    def _import_geom(self, elem: ET.Element) -> Geometry:
        gtype = elem.get("type", "sphere")
        size = _parse_size(elem.get("size"))
        pos = _parse_xyz(elem.get("pos"))
        quat = _parse_quat(elem.get("quat"))
        material = elem.get("material")
        mesh_attr = elem.get("mesh")
        g = Geometry(material=material, origin=pos, origin_rotation=quat)
        if mesh_attr:
            g.mesh = mesh_attr
        elif gtype == "box":
            # MuJoCo size for box is half-extents.
            sx = (size[0] if len(size) > 0 else 0.5) * 2
            sy = (size[1] if len(size) > 1 else 0.5) * 2
            sz = (size[2] if len(size) > 2 else 0.5) * 2
            g.primitive = "box"
            g.primitive_params = {"x": sx, "y": sy, "z": sz}
        elif gtype == "sphere":
            r = size[0] if len(size) > 0 else 1.0
            g.primitive = "sphere"
            g.primitive_params = {"radius": r}
        elif gtype in ("cylinder", "capsule"):
            r = size[0] if len(size) > 0 else 0.5
            l = (size[1] if len(size) > 1 else 0.5) * 2
            g.primitive = "cylinder"
            g.primitive_params = {"radius": r, "length": l}
        return g

    def _make_synthetic_fixed_joint(
        self,
        project: Project,
        parent_link_eid: str,
        child_link_eid: str,
        pos: tuple[float, float, float],
        quat: tuple[float, float, float, float],
        body_name: str,
    ) -> str:
        joint_eid = new_entity_id("joint")
        joint_entity = Entity(id=joint_eid, name=f"{body_name}_attach")
        joint_entity.attach(TransformComponent(position=pos, rotation=quat))
        joint_entity.attach(
            JointComponent(
                type="fixed",
                parent_link=parent_link_eid,
                child_link=child_link_eid,
                source={"mjcf": {"synthetic": True}},
            )
        )
        project.scene.add(joint_entity, parent=parent_link_eid)
        return joint_eid

    def _make_joint_entity(
        self,
        project: Project,
        joint_elem: ET.Element,
        parent_link_eid: str,
        child_link_eid: str,
        body_pos: tuple[float, float, float],
        body_quat: tuple[float, float, float, float],
    ) -> str:
        joint_eid = new_entity_id("joint")
        name = joint_elem.get("name", "") or f"joint_{len(project.scene.entities)}"
        mjcf_type = joint_elem.get("type", "hinge")
        forge_type = MJCF_TO_FORGEBOT_JOINT.get(mjcf_type, "fixed")

        axis = _parse_xyz(joint_elem.get("axis"), (0.0, 0.0, 1.0))

        limits = None
        range_str = joint_elem.get("range")
        if range_str:
            parts = [float(v) for v in range_str.split()]
            if len(parts) == 2:
                limits = JointLimits(lower=parts[0], upper=parts[1])

        joint_entity = Entity(id=joint_eid, name=name)
        joint_entity.attach(TransformComponent(position=body_pos, rotation=body_quat))
        joint_entity.attach(
            JointComponent(
                type=forge_type,  # type: ignore[arg-type]
                axis=axis,
                parent_link=parent_link_eid,
                child_link=child_link_eid,
                limits=limits,
                source={"mjcf": {"name": name, "type": mjcf_type}},
            )
        )
        project.scene.add(joint_entity, parent=parent_link_eid)
        return joint_eid
