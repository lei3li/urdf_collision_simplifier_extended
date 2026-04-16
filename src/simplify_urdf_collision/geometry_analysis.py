"""Analyse mesh geometry to auto-select the best collision primitive.

Supported primitives: box, sphere, cylinder.
"""

from __future__ import annotations

import re

import numpy as np
import trimesh


# ---------------------------------------------------------------------------
# Thresholds
# ---------------------------------------------------------------------------
SPHERE_ASPECT_MAX = 1.15
SPHERE_FILL_MIN = 0.55
_CYL_PREFERENCE_MARGIN = 0.10


# ---------------------------------------------------------------------------
# Semantic body-part classification
# ---------------------------------------------------------------------------

_BODY_PART_RULES: list[tuple[re.Pattern, str]] = [
    (re.compile(r"(torso|pelvis|base_link|waist|chest|trunk|body)", re.I), "box"),
    (re.compile(r"(hand|finger|thumb|palm|index|middle|ring|little|pinky)", re.I), "box"),
    (re.compile(r"(foot|ankle|sole)", re.I), "box"),
    (re.compile(r"(sensor|cam|imu|lidar|logo)", re.I), "box"),
]


def classify_body_part(link_name: str) -> str | None:
    """Return a preferred primitive based on the link name, or None for geometry analysis."""
    for pattern, prim in _BODY_PART_RULES:
        if pattern.search(link_name):
            return prim
    return None


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _obb_extents(mesh: trimesh.Trimesh) -> np.ndarray:
    """Return the three OBB extents sorted ascending."""
    return np.sort(mesh.bounding_box_oriented.extents)


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


# ---------------------------------------------------------------------------
# Minimum enclosing circle (Welzl)
# ---------------------------------------------------------------------------

def _min_enclosing_circle(points_2d: np.ndarray) -> tuple[np.ndarray, float]:
    """Return (center_2d, radius) for the smallest enclosing circle."""
    pts = points_2d.copy()
    rng = np.random.RandomState(42)
    rng.shuffle(pts)

    def _from1(p):
        return p, 0.0

    def _from2(p1, p2):
        return (p1 + p2) / 2.0, np.linalg.norm(p1 - p2) / 2.0

    def _from3(p1, p2, p3):
        ax, ay = p1; bx, by = p2; cx, cy = p3
        d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-14:
            d12 = np.linalg.norm(p1 - p2)
            d13 = np.linalg.norm(p1 - p3)
            d23 = np.linalg.norm(p2 - p3)
            if d12 >= d13 and d12 >= d23:
                return _from2(p1, p2)
            return _from2(p1, p3) if d13 >= d23 else _from2(p2, p3)
        ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
        uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
        c = np.array([ux, uy])
        return c, np.linalg.norm(p1 - c)

    def _inside(c, r, p):
        return np.linalg.norm(p - c) <= r * (1.0 + 1e-8)

    def _welzl(P, R, n):
        if n == 0 or len(R) == 3:
            if len(R) == 0: return np.zeros(2), 0.0
            if len(R) == 1: return _from1(R[0])
            if len(R) == 2: return _from2(R[0], R[1])
            return _from3(R[0], R[1], R[2])
        p = P[n - 1]
        c, r = _welzl(P, R, n - 1)
        if _inside(c, r, p):
            return c, r
        return _welzl(P, R + [p], n - 1)

    n = len(pts)
    if n > 500:
        try:
            from scipy.spatial import ConvexHull
            hull = ConvexHull(pts)
            pts = pts[hull.vertices]
            n = len(pts)
        except Exception:
            pass

    return _welzl(pts, [], n)


# ---------------------------------------------------------------------------
# Axis + projection helper
# ---------------------------------------------------------------------------

def _principal_projection(mesh: trimesh.Trimesh):
    """Project vertices onto the OBB principal (longest) axis.

    Returns (cyl_axis, center_3d, u, v, proj_along, coords_2d, mec_center, mec_radius).
    """
    obb = mesh.bounding_box_oriented
    longest_idx = int(np.argmax(obb.extents))
    obb_tf = np.array(obb.primitive.transform)

    cyl_axis = obb_tf[:3, longest_idx].copy()
    cyl_axis = cyl_axis / (np.linalg.norm(cyl_axis) + 1e-12)
    center_3d = obb_tf[:3, 3].copy()

    rel = np.asarray(mesh.vertices) - center_3d
    proj_along = rel @ cyl_axis

    z_axis = np.array([0.0, 0.0, 1.0])
    u = np.cross(cyl_axis, z_axis)
    u_norm = np.linalg.norm(u)
    if u_norm < 1e-8:
        u = np.cross(cyl_axis, np.array([1.0, 0.0, 0.0]))
        u_norm = np.linalg.norm(u)
    u = u / (u_norm + 1e-12)
    v = np.cross(cyl_axis, u)
    v = v / (np.linalg.norm(v) + 1e-12)

    coords_2d = np.column_stack([rel @ u, rel @ v])
    mec_center, mec_radius = _min_enclosing_circle(coords_2d)

    return cyl_axis, center_3d, u, v, proj_along, coords_2d, mec_center, mec_radius


def _build_axis_rotation(z_axis, cyl_axis):
    """Build a 3x3 rotation that aligns Z to *cyl_axis*."""
    dot = np.clip(np.dot(z_axis, cyl_axis), -1.0, 1.0)
    if abs(dot) > 1.0 - 1e-8:
        if dot < 0:
            return np.diag([1.0, -1.0, -1.0])
        return np.eye(3)
    cross = np.cross(z_axis, cyl_axis)
    cross /= np.linalg.norm(cross) + 1e-12
    angle = np.arccos(dot)
    K = np.array([
        [0, -cross[2], cross[1]],
        [cross[2], 0, -cross[0]],
        [-cross[1], cross[0], 0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


# ---------------------------------------------------------------------------
# Primitive fitting
# ---------------------------------------------------------------------------

def fit_sphere(mesh: trimesh.Trimesh, scale=1.0, padding=0.0, min_size=0.001):
    """Return (radius, tf_4x4)."""
    bs = mesh.bounding_sphere
    radius = max(bs.primitive.radius * scale + padding, min_size / 2.0)
    tf = np.eye(4)
    tf[:3, 3] = bs.primitive.center
    return radius, tf


def fit_cylinder(
    mesh: trimesh.Trimesh,
    scale=1.0, scale_x=None, scale_y=None, scale_z=None,
    padding=0.0, padding_x=None, padding_y=None, padding_z=None,
    min_size=0.001,
):
    """Return (radius, length, tf_4x4).  Z = cylinder axis."""
    cyl_axis, center_3d, u, v, proj, _, mec_c, mec_r = _principal_projection(mesh)

    radial_s = float(scale_x if scale_x is not None else scale)
    axial_s = float(scale_z if scale_z is not None else scale)
    radial_p = float(padding_x if padding_x is not None else padding)
    axial_p = float(padding_z if padding_z is not None else padding)

    length = max((proj.max() - proj.min()) * axial_s + 2 * axial_p, min_size)
    radius = max(mec_r * radial_s + radial_p, min_size / 2.0)

    cyl_center = center_3d + mec_c[0] * u + mec_c[1] * v
    cyl_center += ((proj.max() + proj.min()) / 2.0) * cyl_axis

    tf = np.eye(4)
    tf[:3, 3] = cyl_center
    tf[:3, :3] = _build_axis_rotation(np.array([0.0, 0.0, 1.0]), cyl_axis)
    return radius, length, tf


# ---------------------------------------------------------------------------
# Volume computations
# ---------------------------------------------------------------------------

def _cylinder_volume(mesh: trimesh.Trimesh) -> float:
    """Volume of the tightest bounding cylinder."""
    _, _, _, _, proj, _, _, mec_r = _principal_projection(mesh)
    length = proj.max() - proj.min()
    if length < 1e-12:
        return float("inf")
    return float(np.pi * mec_r ** 2 * length)


def _candidate_volumes(mesh: trimesh.Trimesh) -> dict[str, float]:
    """Compute bounding volume for each candidate primitive."""
    extents = _obb_extents(mesh)
    if extents[0] < 1e-12:
        return {"box": float("inf")}

    aspect_max = extents[2] / extents[0]

    vols: dict[str, float] = {}
    vols["box"] = float(np.prod(mesh.bounding_box_oriented.extents))
    vols["cylinder"] = _cylinder_volume(mesh)

    if aspect_max <= SPHERE_ASPECT_MAX and _sphericity(mesh) >= SPHERE_FILL_MIN:
        r = mesh.bounding_sphere.primitive.radius
        vols["sphere"] = (4.0 / 3.0) * np.pi * r ** 3

    return vols


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def detect_best_primitive(mesh: trimesh.Trimesh, link_name: str | None = None) -> str:
    """Return ``'box'``, ``'cylinder'``, or ``'sphere'``.

    If *link_name* is given, semantic body-part rules are checked first
    (torso/pelvis -> box, hand/finger -> box, foot/ankle -> box).
    Falls back to volume-based selection with cylinder preference.
    """
    if link_name is not None:
        hint = classify_body_part(link_name)
        if hint is not None:
            return hint

    vols = _candidate_volumes(mesh)

    cyl_vol = vols.get("cylinder", float("inf"))
    box_vol = vols.get("box", float("inf"))
    sph_vol = vols.get("sphere", float("inf"))

    if sph_vol < box_vol and sph_vol < cyl_vol:
        return "sphere"

    if cyl_vol <= box_vol:
        return "cylinder"
    if box_vol < cyl_vol * (1.0 - _CYL_PREFERENCE_MARGIN):
        return "box"
    return "cylinder"
