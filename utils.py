import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

def visualize_object(obj, title="Open3D Visualization"):
    o3d.visualization.draw_geometries([obj], window_name=title)

def estimate_parameters(pcd):
    points = np.asarray(pcd.points)
    if len(points) < 2:
        raise ValueError("Point cloud too small.")

    k = min(30, len(points) - 1)
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k+1)
    voxel_size = np.median(distances[:, 1:]) * 0.6

    all_distances, _ = tree.query(points, k=4)
    nearest = all_distances[:, 1]
    cv = np.std(nearest) / np.mean(nearest)
    iso_percentile = np.clip(17 * (1 - cv), 5, 10)

    return voxel_size, iso_percentile
