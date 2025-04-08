import numpy as np
import open3d as o3d
import skimage.measure
from scipy.spatial import cKDTree
from utils import visualize_object, estimate_parameters

class MarchingCubesReconstructor:
    def __init__(self, pcd):
        self.pcd = pcd
        self.mesh = None

    def reconstruct(self):
        points = np.asarray(self.pcd.points)
        voxel_size, iso_level_percentile = estimate_parameters(self.pcd)

        # voxel_size= 0.01 
        # iso_level_percentile = 2

        print(f"Parameters: voxel_size={voxel_size}, iso_level_percentile={iso_level_percentile}")

        if len(points) > 200_000:
            print("Downsampling point cloud...")
            self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size)
            points = np.asarray(self.pcd.points)

        mins, maxs = points.min(axis=0), points.max(axis=0)
        x, y, z = np.meshgrid(
            np.arange(mins[0], maxs[0], voxel_size),
            np.arange(mins[1], maxs[1], voxel_size),
            np.arange(mins[2], maxs[2], voxel_size),
            indexing='ij'
        )

        tree = cKDTree(points)
        grid_points = np.vstack([x.ravel(), y.ravel(), z.ravel()]).T
        distances, _ = tree.query(grid_points)
        scalar_field = distances.reshape(x.shape)
        iso_level = np.percentile(distances, iso_level_percentile)

        verts, faces, _, _ = skimage.measure.marching_cubes(scalar_field, level=iso_level)
        verts = verts * voxel_size + mins

        self.mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(verts),
            o3d.utility.Vector3iVector(faces)
        )
        self.mesh.compute_vertex_normals()

    def save(self, filename):
        o3d.io.write_triangle_mesh(filename, self.mesh)
        print(f"Marching Cubes mesh saved as {filename}")


    def transfer_colors_from_pcd(self):
        print("Transferring colors from point cloud to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        mesh_colors = []

        for v in self.mesh.vertices:
            [_, idx, _] = pcd_tree.search_knn_vector_3d(v, 1)
            mesh_colors.append(self.pcd.colors[idx[0]])

        self.mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_colors)

    def show(self):
        visualize_object(self.mesh, "Marching Cubes Mesh")
