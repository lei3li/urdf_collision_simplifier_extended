#!/usr/bin/env python3
"""Simplify URDF collision meshes with bounding primitives (box, sphere, cylinder)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import transforms3d
import trimesh
import yaml
from urdf_parser_py.urdf import Box, Cylinder, Sphere

from .color import COLORS
from .geometry_analysis import (
    detect_best_primitive,
    fit_cylinder,
    fit_sphere,
)
from .urdf_handler import URDFHandler


# ---------------------------------------------------------------------------
# YAML config loading
# ---------------------------------------------------------------------------

def _load_config(path: str | None) -> dict:
    """Load a per-link override YAML, returning a normalised dict.

    Expected format::

        defaults:
          primitive_type: auto
          scale: 1.1
          padding: 0.005
          bbox_type: obb
          tight_fit: false

        overrides:
          left_hip_pitch_link:
            primitive_type: box
            scale: 1.2
          left_ankle_roll_link:
            primitive_type: cylinder
            padding: 0.01
    """
    if path is None:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _link_params(config: dict, link_name: str, cli_args: argparse.Namespace) -> dict:
    """Merge CLI defaults < YAML defaults < YAML per-link overrides."""
    base = {
        "primitive_type": cli_args.primitive_type,
        "bbox_type": cli_args.bbox_type,
        "scale": cli_args.scale,
        "scale_x": cli_args.scale_x,
        "scale_y": cli_args.scale_y,
        "scale_z": cli_args.scale_z,
        "padding": cli_args.padding,
        "padding_x": cli_args.padding_x,
        "padding_y": cli_args.padding_y,
        "padding_z": cli_args.padding_z,
        "min_size": cli_args.min_size,
        "tight_fit": cli_args.tight_fit,
    }
    yaml_defaults = config.get("defaults", {})
    if yaml_defaults:
        for k, v in yaml_defaults.items():
            if k in base and v is not None:
                base[k] = v

    link_overrides = config.get("overrides", {}) or {}
    link_cfg = link_overrides.get(link_name, {})
    if link_cfg:
        for k, v in link_cfg.items():
            if k in base and v is not None:
                base[k] = v

    return base


# ---------------------------------------------------------------------------
# Bounding box (existing logic, cleaned up)
# ---------------------------------------------------------------------------

def calculate_bounding_box(
    mesh: trimesh.Trimesh,
    bbox_type: str = "obb",
    scale: float = 1.0,
    scale_x: float | None = None,
    scale_y: float | None = None,
    scale_z: float | None = None,
    padding: float = 0.0,
    padding_x: float | None = None,
    padding_y: float | None = None,
    padding_z: float | None = None,
    min_size: float = 0.001,
    tight_fit: bool = False,
):
    """Calculate bounding box dimensions and transformation for a mesh.

    Returns:
        (bb_size, bb_tf): list of 3 floats and a 4x4 numpy array.
    """
    if bbox_type == "obb":
        if tight_fit:
            try:
                convex_mesh = mesh.convex_hull
                bb_tf = np.linalg.inv(np.asarray(convex_mesh.apply_obb()))
                bb_bounds = convex_mesh.bounding_box_oriented.bounds
                bb_size = [bb_bounds[1][0] * 2, bb_bounds[1][1] * 2, bb_bounds[1][2] * 2]
            except (ValueError, RuntimeError, np.linalg.LinAlgError):
                bb_tf = np.linalg.inv(np.asarray(mesh.apply_obb()))
                bb_bounds = mesh.bounding_box_oriented.bounds
                bb_size = [bb_bounds[1][0] * 2, bb_bounds[1][1] * 2, bb_bounds[1][2] * 2]
        else:
            bb_tf = np.linalg.inv(np.asarray(mesh.apply_obb()))
            bb_bounds = mesh.bounding_box_oriented.bounds
            bb_size = [bb_bounds[1][0] * 2, bb_bounds[1][1] * 2, bb_bounds[1][2] * 2]
    else:  # aabb
        if tight_fit:
            try:
                bb_bounds = mesh.convex_hull.bounds
            except (ValueError, RuntimeError):
                bb_bounds = mesh.bounds
        else:
            bb_bounds = mesh.bounds
        bb_size = [
            bb_bounds[1][0] - bb_bounds[0][0],
            bb_bounds[1][1] - bb_bounds[0][1],
            bb_bounds[1][2] - bb_bounds[0][2],
        ]
        center = (bb_bounds[0] + bb_bounds[1]) / 2.0
        bb_tf = np.eye(4)
        bb_tf[:3, 3] = center

    scales = [
        scale_x if scale_x is not None else scale,
        scale_y if scale_y is not None else scale,
        scale_z if scale_z is not None else scale,
    ]
    paddings = [
        padding_x if padding_x is not None else padding,
        padding_y if padding_y is not None else padding,
        padding_z if padding_z is not None else padding,
    ]
    for i in range(3):
        bb_size[i] = max(bb_size[i] * scales[i] + 2 * paddings[i], min_size)

    return bb_size, bb_tf


# ---------------------------------------------------------------------------
# Unified primitive fitting
# ---------------------------------------------------------------------------

def fit_primitive(mesh: trimesh.Trimesh, params: dict):
    """Fit a collision primitive to *mesh* according to *params*.

    Returns:
        (geometry, tf_4x4, description_str)
    """
    ptype = params["primitive_type"]
    if ptype == "auto":
        ptype = detect_best_primitive(mesh)

    scale = params["scale"]
    padding = params["padding"]
    min_size = params["min_size"]

    if ptype == "sphere":
        radius, tf = fit_sphere(mesh, scale=scale, padding=padding, min_size=min_size)
        geom = Sphere(radius=radius)
        desc = f"sphere r={radius:.4f}"
        return geom, tf, desc

    if ptype == "cylinder":
        radius, length, tf = fit_cylinder(
            mesh,
            scale=scale,
            scale_x=params.get("scale_x"),
            scale_y=params.get("scale_y"),
            scale_z=params.get("scale_z"),
            padding=padding,
            padding_x=params.get("padding_x"),
            padding_y=params.get("padding_y"),
            padding_z=params.get("padding_z"),
            min_size=min_size,
        )
        geom = Cylinder(radius=radius, length=length)
        desc = f"cylinder r={radius:.4f} l={length:.4f}"
        return geom, tf, desc

    # default: box
    bb_size, tf = calculate_bounding_box(
        mesh,
        bbox_type=params["bbox_type"],
        scale=scale,
        scale_x=params.get("scale_x"),
        scale_y=params.get("scale_y"),
        scale_z=params.get("scale_z"),
        padding=padding,
        padding_x=params.get("padding_x"),
        padding_y=params.get("padding_y"),
        padding_z=params.get("padding_z"),
        min_size=min_size,
        tight_fit=params.get("tight_fit", False),
    )
    geom = Box(bb_size)
    desc = f"box [{bb_size[0]:.4f}, {bb_size[1]:.4f}, {bb_size[2]:.4f}]"
    return geom, tf, desc


def calculate_volume_ratio(mesh: trimesh.Trimesh, bb_size: list[float]) -> float:
    """Calculate the ratio of bounding box volume to mesh volume."""
    try:
        mesh_volume = mesh.volume
        bb_volume = bb_size[0] * bb_size[1] * bb_size[2]
        if mesh_volume > 0:
            return bb_volume / mesh_volume
        return float("inf")
    except (ValueError, RuntimeError):
        return float("inf")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Simplify URDF collision meshes with bounding primitives.",
    )
    parser.add_argument("input_urdf", help="path to input URDF file")
    parser.add_argument("output_urdf", help="path to write simplified URDF")
    parser.add_argument(
        "-r", "--ros", action="store_true",
        help="resolve ros package:// paths in the URDF",
    )
    parser.add_argument(
        "-s", "--select", action="store_true",
        help="interactive mode: deselect collision models before processing",
    )
    parser.add_argument(
        "-e", "--exclude", nargs="+", default=[],
        help="link names to exclude from simplification",
    )

    # Primitive selection
    parser.add_argument(
        "--primitive-type", choices=["auto", "box", "sphere", "cylinder"], default="auto",
        help="collision primitive type (default: auto — picks best per link)",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="YAML config file with per-link overrides (see README for format)",
    )

    # Scaling / padding
    parser.add_argument("--scale", type=float, default=1.0, help="uniform scaling factor (default: 1.0)")
    parser.add_argument("--scale-x", type=float, default=None, help="X scaling (overrides --scale)")
    parser.add_argument("--scale-y", type=float, default=None, help="Y scaling (overrides --scale)")
    parser.add_argument("--scale-z", type=float, default=None, help="Z scaling (overrides --scale)")
    parser.add_argument("--padding", type=float, default=0.0, help="uniform padding in meters (default: 0.0)")
    parser.add_argument("--padding-x", type=float, default=None, help="X padding in meters")
    parser.add_argument("--padding-y", type=float, default=None, help="Y padding in meters")
    parser.add_argument("--padding-z", type=float, default=None, help="Z padding in meters")

    # Box-specific
    parser.add_argument(
        "--bbox-type", choices=["obb", "aabb"], default="obb",
        help="bounding box type: oriented (obb) or axis-aligned (aabb) (default: obb)",
    )
    parser.add_argument("--min-size", type=float, default=0.001, help="minimum primitive dimension in meters (default: 0.001)")
    parser.add_argument("--tight-fit", action="store_true", help="use convex hull for tighter fitting")
    parser.add_argument("-v", "--verbose", action="store_true", help="print detailed per-link information")

    args = parser.parse_args()

    if args.ros:
        import resource_retriever  # noqa: F401 — only needed for ROS paths

    config = _load_config(args.config)
    urdf_handler = URDFHandler(args.input_urdf, args.exclude)
    filename_dict = urdf_handler.get_filenames(args.select)

    # Print configuration summary
    print(f"\n{COLORS.OKBLUE}Configuration:{COLORS.ENDC}")
    print(f"  Primitive type: {args.primitive_type}")
    print(f"  Bounding box type: {args.bbox_type}")
    if args.tight_fit:
        print("  Tight-fit enabled (convex hull)")
    print(f"  Scale: {args.scale} (X: {args.scale_x or args.scale}, Y: {args.scale_y or args.scale}, Z: {args.scale_z or args.scale})")
    print(f"  Padding: {args.padding}m (X: {args.padding_x or args.padding}m, Y: {args.padding_y or args.padding}m, Z: {args.padding_z or args.padding}m)")
    print(f"  Min size: {args.min_size}m")
    if args.config:
        print(f"  Config file: {args.config}")
    print()

    n = len(filename_dict)
    for i, (link, collision_models) in enumerate(filename_dict.items(), 1):
        params = _link_params(config, link, args)
        print(f"{link} ({i}/{n})")

        for c in collision_models:
            resolved_filename = c.geometry.filename
            if args.ros:
                resolved_filename = resource_retriever.get_filename(
                    resolved_filename, use_protocol=False,
                )
            mesh = trimesh.load(resolved_filename)

            geom, prim_tf, desc = fit_primitive(mesh, params)

            original_rotation = transforms3d.euler.euler2mat(
                c.origin.rotation[0], c.origin.rotation[1], c.origin.rotation[2],
                axes="sxyz",
            )
            original_tf = transforms3d.affines.compose(
                T=c.origin.position, R=original_rotation, Z=[1, 1, 1],
            )

            new_affine = np.matmul(original_tf, prim_tf)
            T, R, _, _ = transforms3d.affines.decompose44(new_affine)
            c.origin.position = T
            c.origin.rotation = list(transforms3d.euler.mat2euler(R, axes="sxyz"))
            c.geometry = geom

            if args.verbose:
                print(f"    {resolved_filename}: {desc}")

    print(f"\n{COLORS.OKGREEN}Writing urdf to {args.output_urdf}{COLORS.ENDC}")
    urdf_handler.write_urdf(args.output_urdf)


if __name__ == "__main__":
    main()
