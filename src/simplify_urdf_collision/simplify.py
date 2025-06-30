#!/usr/bin/env python3

import argparse
from urdf_handler import URDFHandler
import os
import trimesh
import transforms3d
import numpy as np
from urdf_parser_py.urdf import Box 
from color import COLORS

TMP_MESH_FILENAME = "/tmp/temp_mesh.off"

def calculate_bounding_box(mesh, bbox_type="obb", scale=1.0, scale_x=None, scale_y=None, scale_z=None, 
                          padding=0.0, padding_x=None, padding_y=None, padding_z=None, min_size=0.001, tight_fit=False):
    """
    Calculate bounding box dimensions and transformation for a mesh.
    
    Args:
        mesh: trimesh mesh object
        bbox_type: "obb" for oriented bounding box, "aabb" for axis-aligned bounding box
        scale: uniform scaling factor
        scale_x, scale_y, scale_z: per-axis scaling factors (override uniform scale)
        padding: uniform padding in meters
        padding_x, padding_y, padding_z: per-axis padding in meters (override uniform padding)
        min_size: minimum size for any dimension
        tight_fit: use tighter fitting algorithm
    
    Returns:
        tuple: (bb_size, bb_tf) where bb_size is [x,y,z] dimensions and bb_tf is transformation matrix
    """
    if bbox_type == "obb":
        # Use oriented bounding box
        if tight_fit:
            # Use convex hull for tighter fit
            try:
                convex_mesh = mesh.convex_hull
                bb_tf = np.linalg.inv(np.matrix(convex_mesh.apply_obb()))
                bb_bounds = convex_mesh.bounding_box_oriented.bounds
                bb_size = [bb_bounds[1][0]*2, bb_bounds[1][1]*2, bb_bounds[1][2]*2]
            except:
                # Fallback to regular OBB if convex hull fails
                bb_tf = np.linalg.inv(np.matrix(mesh.apply_obb()))
                bb_bounds = mesh.bounding_box_oriented.bounds
                bb_size = [bb_bounds[1][0]*2, bb_bounds[1][1]*2, bb_bounds[1][2]*2]
        else:
            bb_tf = np.linalg.inv(np.matrix(mesh.apply_obb()))
            bb_bounds = mesh.bounding_box_oriented.bounds
            bb_size = [bb_bounds[1][0]*2, bb_bounds[1][1]*2, bb_bounds[1][2]*2]
    else:  # aabb
        # Use axis-aligned bounding box
        bb_tf = np.eye(4)  # No transformation needed for AABB
        if tight_fit:
            try:
                # Use convex hull for tighter AABB
                convex_mesh = mesh.convex_hull
                bb_bounds = convex_mesh.bounds
            except:
                bb_bounds = mesh.bounds
        else:
            bb_bounds = mesh.bounds
        bb_size = [bb_bounds[1][0] - bb_bounds[0][0], 
                   bb_bounds[1][1] - bb_bounds[0][1], 
                   bb_bounds[1][2] - bb_bounds[0][2]]
    
    # Apply scaling factors
    scales = [
        scale_x if scale_x is not None else scale,
        scale_y if scale_y is not None else scale,
        scale_z if scale_z is not None else scale
    ]
    
    # Apply padding
    paddings = [
        padding_x if padding_x is not None else padding,
        padding_y if padding_y is not None else padding,
        padding_z if padding_z is not None else padding
    ]
    
    # Calculate final dimensions
    for i in range(3):
        bb_size[i] = max(bb_size[i] * scales[i] + 2 * paddings[i], min_size)
    
    return bb_size, bb_tf

def calculate_volume_ratio(mesh, bb_size):
    """Calculate the ratio of bounding box volume to mesh volume."""
    try:
        mesh_volume = mesh.volume
        bb_volume = bb_size[0] * bb_size[1] * bb_size[2]
        if mesh_volume > 0:
            return bb_volume / mesh_volume
        else:
            return float('inf')
    except:
        return float('inf')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_urdf")
    parser.add_argument("output_urdf")
    parser.add_argument("-r", "--ros", action="store_true", help="when enabled, ros package paths in urdfs are resolved")
    parser.add_argument("-s", "--select", action="store_true", help="enable interactive deselecting of certain collision models not to be transformed into bounding boxes")
    # todo add option exclude links
    parser.add_argument("-e", "--exclude", nargs="+", default=[], help="names of links to be excluded from collision model simplification")
    parser.add_argument("--scale", type=float, default=1.0, help="uniform scaling factor for bounding box dimensions (default: 1.0)")
    parser.add_argument("--scale-x", type=float, default=None, help="scaling factor for X dimension (overrides --scale)")
    parser.add_argument("--scale-y", type=float, default=None, help="scaling factor for Y dimension (overrides --scale)")
    parser.add_argument("--scale-z", type=float, default=None, help="scaling factor for Z dimension (overrides --scale)")
    parser.add_argument("--padding", type=float, default=0.0, help="uniform padding to add to bounding box dimensions in meters (default: 0.0)")
    parser.add_argument("--padding-x", type=float, default=None, help="padding for X dimension in meters (overrides --padding)")
    parser.add_argument("--padding-y", type=float, default=None, help="padding for Y dimension in meters (overrides --padding)")
    parser.add_argument("--padding-z", type=float, default=None, help="padding for Z dimension in meters (overrides --padding)")
    parser.add_argument("--bbox-type", choices=["obb", "aabb"], default="obb", help="bounding box type: 'obb' for oriented bounding box, 'aabb' for axis-aligned (default: obb)")
    parser.add_argument("--min-size", type=float, default=0.001, help="minimum size for any bounding box dimension in meters (default: 0.001)")
    parser.add_argument("--tight-fit", action="store_true", help="use tighter fitting algorithm based on principal component analysis")
    parser.add_argument("-v", "--verbose", action="store_true", help="print detailed information about each collision model")
    args = parser.parse_args()
    if args.ros:
        import resource_retriever
    urdf_handler = URDFHandler(args.input_urdf, args.exclude)
    filename_dict = urdf_handler.get_filenames(args.select)
    
    # Print configuration
    print(f"{COLORS.OKBLUE}Configuration:{COLORS.ENDC}")
    print(f"  Bounding box type: {args.bbox_type}")
    if args.tight_fit:
        print(f"  Using tight-fit algorithm (convex hull based)")
    print(f"  Scaling: {args.scale} (X: {args.scale_x or args.scale}, Y: {args.scale_y or args.scale}, Z: {args.scale_z or args.scale})")
    print(f"  Padding: {args.padding}m (X: {args.padding_x or args.padding}m, Y: {args.padding_y or args.padding}m, Z: {args.padding_z or args.padding}m)")
    print(f"  Minimum size: {args.min_size}m")
    if args.verbose:
        print(f"  Verbose output enabled")
    print()
    
    i = 1
    n = len(filename_dict)
    for link, collision_models in filename_dict.items():
        print(f"{link} ({i}/{n})")
        for c in collision_models:
            # resolve filename if, uses package:// prefix if ros is enabled
            resolved_filename = c.geometry.filename
            if args.ros:
                resolved_filename = resource_retriever.get_filename(resolved_filename, use_protocol=False)
            mesh = trimesh.load(resolved_filename)

            # Calculate bounding box with new parameters
            bb_size, bb_tf = calculate_bounding_box(
                mesh, 
                bbox_type=args.bbox_type,
                scale=args.scale,
                scale_x=args.scale_x,
                scale_y=args.scale_y, 
                scale_z=args.scale_z,
                padding=args.padding,
                padding_x=args.padding_x,
                padding_y=args.padding_y,
                padding_z=args.padding_z,
                min_size=args.min_size,
                tight_fit=args.tight_fit
            )

            original_rotation = transforms3d.euler.euler2mat(c.origin.rotation[0], c.origin.rotation[1], c.origin.rotation[2], axes="sxyz")
            original_transformation_affine = transforms3d.affines.compose(T=c.origin.position, R=original_rotation, Z=[1,1,1])

            new_affine = np.matmul(original_transformation_affine, bb_tf)
            T,R,_,_ = transforms3d.affines.decompose44(new_affine)
            c.origin.position = T
            c.origin.rotation = list(transforms3d.euler.mat2euler(R, axes="sxyz"))
            b = Box(bb_size)
            c.geometry = b
            
            # Calculate and print volume ratio for feedback
            if args.verbose:
                volume_ratio = calculate_volume_ratio(mesh, bb_size)
                if volume_ratio != float('inf'):
                    print(f"    {resolved_filename}: [{bb_size[0]:.4f}, {bb_size[1]:.4f}, {bb_size[2]:.4f}] (volume ratio: {volume_ratio:.2f})")
                else:
                    print(f"    {resolved_filename}: [{bb_size[0]:.4f}, {bb_size[1]:.4f}, {bb_size[2]:.4f}]")
            elif len(collision_models) > 1:
                # Just print a short version when there are multiple collision models per link
                print(f"    Mesh {len([m for m in collision_models if m == c])}: [{bb_size[0]:.4f}, {bb_size[1]:.4f}, {bb_size[2]:.4f}]")
        i +=1
    print(f"{COLORS.OKGREEN}Writing urdf to {args.output_urdf}{COLORS.ENDC}")
    urdf_handler.write_urdf(args.output_urdf)
