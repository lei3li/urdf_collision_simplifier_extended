"""Analyse mesh geometry to auto-select the best collision primitive.

Supported primitives: box, sphere, cylinder.
The heuristic is intentionally conservative — it defaults to *box* unless the
mesh clearly fits a simpler shape.
"""

from __future__ import annotations

import numpy as np
import trimesh


# ---------------------------------------------------------------------------
# Thresholds (tuned for typical humanoid robot link meshes)
# ---------------------------------------------------------------------------
SPHERE_ASPECT_MAX = 1.35  # max OBB aspect ratio to consider sphere
SPHERE_FILL_MIN = 0.45    # min mesh-vol / bounding-sphere-vol
CYLINDER_ELONGATION_MIN = 1.8   # min longest/shortest OBB axis ratio
CYLINDER_CIRCULARITY_MIN = 0.70  # min cross-section circularity


def _obb_extents(mesh: trimesh.Trimesh) -> np.ndarray:
    """Return the three OBB half-extents sorted ascending."""
    obb = mesh.bounding_box_oriented
    return np.sort(obb.extents)


def _sphericity(mesh: trimesh.Trimesh) -> float:
    """Volume ratio of the mesh to its bounding sphere (0..1)."""
    try:
        r = mesh.bounding_sphere.primitive.radius
        sphere_vol = (4.0 / 3.0) * np.pi * r ** 3
        if sphere_vol <= 0:
            return 0.0
        return min(abs(mesh.volume) / sphere_vol, 1.0)
    except Exception:
        return 0.0


def _cross_section_circularity(mesh: trimesh.Trimesh) -> float:
    """Estimate how circular the cross-section is perpendicular to the longest OBB axis.

    Returns a value in [0, 1] where 1 means perfectly circular.
    """
    try:
        obb = mesh.bounding_box_oriented
        extents = obb.extents
        longest_idx = int(np.argmax(extents))

        obb_tf = obb.primitive.transform
        principal_axis = obb_tf[:3, longest_idx]

        center = mesh.centroid
        vertices = mesh.vertices - center
        proj = vertices @ principal_axis
        mid = (proj.min() + proj.max()) / 2.0
        band = 0.15 * (proj.max() - proj.min())
        mask = np.abs(proj - mid) < max(band, 1e-6)
        if mask.sum() < 3:
            return 0.0

        slice_pts = vertices[mask]

        perp_axes = np.delete(np.eye(3), longest_idx, axis=0)
        u = perp_axes[0] - perp_axes[0].dot(principal_axis) * principal_axis
        u /= np.linalg.norm(u) + 1e-12
        v = np.cross(principal_axis, u)
        v /= np.linalg.norm(v) + 1e-12

        coords_2d = np.column_stack([slice_pts @ u, slice_pts @ v])
        centroid_2d = coords_2d.mean(axis=0)
        dists = np.linalg.norm(coords_2d - centroid_2d, axis=1)
        if dists.max() < 1e-12:
            return 0.0

        r_mean = dists.mean()
        r_std = dists.std()
        circularity = 1.0 - (r_std / (r_mean + 1e-12))
        return float(np.clip(circularity, 0.0, 1.0))
    except Exception:
        return 0.0


# ---------------------------------------------------------------------------
# Primitive fitting helpers
# ---------------------------------------------------------------------------

def fit_sphere(mesh: trimesh.Trimesh, scale: float = 1.0, padding: float = 0.0, min_size: float = 0.001):
    """Return (radius, center_tf_4x4)."""
    bs = mesh.bounding_sphere
    radius = max(bs.primitive.radius * scale + padding, min_size / 2.0)
    tf = np.eye(4)
    tf[:3, 3] = bs.primitive.center
    return radius, tf


def fit_cylinder(
    mesh: trimesh.Trimesh,
    scale: float = 1.0,
    scale_x: float | None = None,
    scale_y: float | None = None,
    scale_z: float | None = None,
    padding: float = 0.0,
    padding_x: float | None = None,
    padding_y: float | None = None,
    padding_z: float | None = None,
    min_size: float = 0.001,
):
    """Fit a cylinder aligned to the longest OBB axis.

    Returns (radius, length, tf_4x4) where tf_4x4 places the cylinder so that
    the URDF convention (Z = cylinder axis) is satisfied.
    """
    obb = mesh.bounding_box_oriented
    extents = obb.extents
    longest_idx = int(np.argmax(extents))

    obb_tf = obb.primitive.transform

    radial_s = float(scale_x if scale_x is not None else scale)
    axial_s = float(scale_z if scale_z is not None else scale)
    radial_p = float(padding_x if padding_x is not None else padding)
    axial_p = float(padding_z if padding_z is not None else padding)

    length = max(extents[longest_idx] * axial_s + 2 * axial_p, min_size)

    radial_extents = np.delete(extents, longest_idx)
    radius = max(np.max(radial_extents) / 2.0 * radial_s + radial_p, min_size / 2.0)

    cyl_axis = obb_tf[:3, longest_idx]
    cyl_axis = cyl_axis / (np.linalg.norm(cyl_axis) + 1e-12)

    z_axis = np.array([0.0, 0.0, 1.0])
    tf = np.eye(4)
    tf[:3, 3] = obb_tf[:3, 3]

    dot = np.clip(np.dot(z_axis, cyl_axis), -1.0, 1.0)
    if abs(dot) > 1.0 - 1e-8:
        if dot < 0:
            tf[:3, :3] = np.diag([1.0, -1.0, -1.0])
    else:
        cross = np.cross(z_axis, cyl_axis)
        cross /= np.linalg.norm(cross) + 1e-12
        angle = np.arccos(dot)
        K = np.array([
            [0, -cross[2], cross[1]],
            [cross[2], 0, -cross[0]],
            [-cross[1], cross[0], 0],
        ])
        tf[:3, :3] = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

    return radius, length, tf


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def detect_best_primitive(mesh: trimesh.Trimesh) -> str:
    """Return ``'sphere'``, ``'cylinder'``, or ``'box'``."""
    extents = _obb_extents(mesh)  # ascending
    if extents[0] < 1e-12:
        return "box"

    aspect_max = extents[2] / extents[0]
    aspect_mid = extents[1] / extents[0]

    if aspect_max <= SPHERE_ASPECT_MAX and _sphericity(mesh) >= SPHERE_FILL_MIN:
        return "sphere"

    if aspect_max >= CYLINDER_ELONGATION_MIN:
        circ = _cross_section_circularity(mesh)
        if circ >= CYLINDER_CIRCULARITY_MIN:
            return "cylinder"

    return "box"
