from point_cloud_processor import PointCloudProcessor
from poisson_reconstructor import PoissonReconstructor
from marching_cubes_reconstructor import MarchingCubesReconstructor

def main():
    print("Surface Reconstruction Options:")
    print("1 - Poisson Reconstruction")
    print("2 - Marching Cubes Reconstruction")
    choice = input("Chọn 1 hoặc 2: ").strip()

    processor = PointCloudProcessor("wheel.ply")  # Thay bằng file của bạn
    processor.show("Original Point Cloud")
    processor.filter_with_dbscan()
    processor.show("Filtered Point Cloud")
    processor.estimate_normals()

    if choice == "1":
        recon = PoissonReconstructor(processor.get())
        recon.reconstruct()
        recon.show_density()
        recon.remove_low_density()
        recon.simplify()
        recon.transfer_colors_from_pcd()
        recon.show()
        recon.save("poisson_output.obj")


    elif choice == "2":
        recon = MarchingCubesReconstructor(processor.get())
        recon.reconstruct()
        recon.transfer_colors_from_pcd()

        recon.show()
        recon.save("marching_output.obj")
    else:
        print("Invalid choice.")

if __name__ == "__main__":
    main()
