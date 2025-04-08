# 3D Surface Reconstruction from Point Cloud

This project provides a complete pipeline for 3D surface reconstruction from a raw point cloud using two standard methods: **Poisson Surface Reconstruction** and **Marching Cubes (Ball Pivoting)**. The system supports preprocessing, noise filtering, surface estimation, color transfer, and mesh simplification.

## Overview

The reconstruction pipeline includes:

- Loading and visualizing raw point cloud data.
- Outlier removal using DBSCAN clustering.
- Surface normal estimation.
- Mesh reconstruction using:
  - Poisson Surface Reconstruction
  - Marching Cubes (based on Ball Pivoting)
- Vertex density analysis and filtering (Poisson only).
- Mesh simplification via quadric decimation.
- Color transfer from point cloud to reconstructed mesh.
- Exporting final mesh to `.obj` format.

## Project Structure

```
.
├── main.py                          # Main entry point to select and run reconstruction
├── point_cloud_processor.py        # Point cloud preprocessing: load, filter, normal estimation
├── poisson_reconstructor.py        # Poisson reconstruction pipeline
├── marching_cubes_reconstructor.py # Marching Cubes pipeline using Ball Pivoting
├── utils.py                        # Utility functions for visualization and parameter estimation
├── wheel.ply                       # Example input point cloud
├── poisson_output.obj              # Output mesh (Poisson)
└── marching_output.obj             # Output mesh (Marching Cubes)
```

## Installation

Dependencies:

- Python 3.8+
- Open3D
- NumPy
- Matplotlib
- SciPy

Install using pip:

```bash
pip install open3d numpy matplotlib scipy
```

## Usage

Run the main script:

```bash
python main.py
```

You will be prompted to choose the reconstruction method:

```
Surface Reconstruction Options:
1 - Poisson Reconstruction
2 - Marching Cubes Reconstruction
Chọn 1 hoặc 2:
```

The script will:
- Load and display the original point cloud.
- Filter out noise using DBSCAN.
- Estimate normals.
- Run the selected reconstruction algorithm.
- Display and save the final mesh to `.obj` format.

## Key Components

### DBSCAN Filtering
Clusters the point cloud and retains only the largest cluster to remove noise and outliers.

### Normal Estimation
Normals are estimated using a hybrid KD-Tree radius search. Required for both reconstruction methods.

### Poisson Reconstruction
Generates a watertight mesh by solving a spatial Poisson equation. Supports density visualization and filtering.

### Marching Cubes (Ball Pivoting)
Creates a surface mesh using Open3D's Ball Pivoting algorithm with estimated voxel size.

### Mesh Simplification
Optional simplification of the final mesh using quadric error metrics to reduce complexity.

### Color Transfer
Nearest neighbor color mapping is used to transfer RGB information from the point cloud to mesh vertices.

## Parameter Tuning

- `eps` and `min_points` for DBSCAN
- `depth` for Poisson reconstruction
- `density threshold` for filtering Poisson mesh
- `simplification ratio` for mesh decimation

Automatic estimation of voxel size and isotropic percentile is provided by `utils.py`.

## Notes

- Replace `wheel.ply` with your own point cloud file as needed.
- All visualizations are rendered using Open3D's interactive viewer.

