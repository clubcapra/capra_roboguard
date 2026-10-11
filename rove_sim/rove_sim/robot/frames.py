"""URDF link poses for mesh rendering and sensor self-occlusion."""

import pybullet as p


def base_link_pose(body_id):
    """Convert Bullet's base centre-of-mass pose to the URDF base-link pose.

    Visual origins are relative to the link, while getBasePositionAndOrientation
    returns the inertial frame. Child links already expose their link frames in
    getLinkState()[4:6]. The inverse inertial transform matters even for a fixed
    base and may contain both a translation and a rotation.
    """
    position, orientation = p.getBasePositionAndOrientation(body_id)
    inertial_position, inertial_orientation = p.getDynamicsInfo(body_id, -1)[3:5]
    return p.multiplyTransforms(
        position,
        orientation,
        *p.invertTransform(inertial_position, inertial_orientation)
    )
