import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

class PoissonReconstructor:
    def __init__(self, pcd):
        self.pcd = pcd
        self.mesh = None
        self.densities = None

    def reconstruct(self, depth=9):
        print("Running Poisson reconstruction...")
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug):
            self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.pcd, depth=depth)

    def show_density(self):
        print("Visualizing vertex density...")
        d = np.asarray(self.densities)
        cmap = plt.get_cmap('plasma')((d - d.min()) / (d.max() - d.min()))
        self.mesh.vertex_colors = o3d.utility.Vector3dVector(cmap[:, :3])
        o3d.visualization.draw_geometries([self.mesh], window_name="Vertex Density")

    def remove_low_density(self, threshold=0.01):
        print("Removing low-density vertices...")
        d = np.asarray(self.densities)
        mask = d < np.quantile(d, threshold)
        self.mesh.remove_vertices_by_mask(mask)

    def simplify(self, ratio=0.7):
        print("Simplifying mesh...")
        target = int(len(self.mesh.triangles) * ratio)
        self.mesh = self.mesh.simplify_quadric_decimation(target)
    
    def transfer_colors_from_pcd(self):
        print("Transferring colors from point cloud to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        mesh_colors = []

        for v in self.mesh.vertices:
            [_, idx, _] = pcd_tree.search_knn_vector_3d(v, 1)
            mesh_colors.append(self.pcd.colors[idx[0]])

        self.mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_colors)

    def save(self, filename):
        o3d.io.write_triangle_mesh(filename, self.mesh)
        print(f"Mesh saved as {filename}")

    def show(self):
        o3d.visualization.draw_geometries([self.mesh], window_name="Poisson Mesh")
