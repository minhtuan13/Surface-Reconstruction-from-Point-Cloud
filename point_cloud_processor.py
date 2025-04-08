import open3d as o3d
import numpy as np

class PointCloudProcessor:
    def __init__(self, file_path):
        self.pcd = o3d.io.read_point_cloud(file_path)
        print("Loaded point cloud:", self.pcd)

    def show(self, window_name="Point Cloud"):
        o3d.visualization.draw_geometries([self.pcd], window_name=window_name)

    def filter_with_dbscan(self, eps=0.13, min_points=3):
        print("Filtering outliers using DBSCAN...")
        labels = np.array(self.pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
        if labels.max() < 0:
            print("No clusters found.")
            return
        largest_cluster = np.argmax(np.bincount(labels[labels >= 0]))
        indices = np.where(labels == largest_cluster)[0]
        self.pcd = self.pcd.select_by_index(indices)
        print("Filtered point cloud.")

    def estimate_normals(self, radius=0.01, max_nn=30):
        print("Estimating normals...")
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        self.pcd.normalize_normals()

    def get(self):
        return self.pcd
