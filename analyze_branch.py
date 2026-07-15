#!/usr/bin/env python3
import os
import sys
from pathlib import Path

try:
    import trimesh
except ImportError:
    print("Error: 'trimesh' library is not installed.")
    print("Please run: pip install trimesh")
    sys.exit(1)

# Find the workspace path dynamically
workspace_path = Path("/home/maya/Documents/projects/agri-grasp-ros2")

# Search recursively for the STL file
stl_files = list(workspace_path.glob("**/tree_branch.stl"))

if not stl_files:
    print(f"Could not find 'tree_branch.stl' under {workspace_path}")
    sys.exit(1)

stl_path = stl_files[0]
print(f"Found STL file at: {stl_path}")

# Load the STL mesh
mesh = trimesh.load(stl_path)

# Scale multiplier (mm to meters)
scale = 0.001

# Calculate physical metrics
bounds = mesh.bounds * scale
extents = mesh.extents * scale
bounding_box_center = bounds.mean(axis=0)

print("\n" + "="*50)
print("             MESH GEOMETRY ANALYSIS             ")
print("="*50)
print(f"File Name:     {stl_path.name}")
print(f"Is Watertight: {mesh.is_watertight}")
print("-"*50)
print("PHYSICAL DIMENSIONS (in meters):")
print(f"  Width  (X): {extents[0]:.4f} m ({extents[0]*1000:.1f} mm)")
print(f"  Length (Y): {extents[1]:.4f} m ({extents[1]*1000:.1f} mm)")
print(f"  Height (Z): {extents[2]:.4f} m ({extents[2]*1000:.1f} mm)")
print("-"*50)
print("BOUNDING BOX BOUNDARIES (Scaled to meters):")
print(f"  X-axis: [{bounds[0][0]:.4f} to {bounds[1][0]:.4f}]")
print(f"  Y-axis: [{bounds[0][1]:.4f} to {bounds[1][1]:.4f}]")
print(f"  Z-axis: [{bounds[0][2]:.4f} to {bounds[1][2]:.4f}]")
print("-"*50)
print("GEOMETRIC CENTER (Relative to STL's 0,0,0 coordinate):")
print(f"  X-offset: {bounding_box_center[0]:.4f}")
print(f"  Y-offset: {bounding_box_center[1]:.4f}")
print(f"  Z-offset: {bounding_box_center[2]:.4f}")
print("-"*50)
print("CORRECTIVE POSE OFFSET (Copy these values):")
print("Subtract these to center your visual and collision meshes:")
print(f"  Ideal Offset X: {-bounding_box_center[0]:.4f}")
print(f"  Ideal Offset Y: {-bounding_box_center[1]:.4f}")
print(f"  Ideal Offset Z: {-bounding_box_center[2]:.4f}")
print("="*50 + "\n")
