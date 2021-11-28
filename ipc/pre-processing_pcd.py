import open3d as o3d
# read the capture
pcd = o3d.io.read_point_cloud("/pcd.ply")
# downsample the point cloud
pcd = pcd.voxel_down_sample(voxel_size=0.001)
# make a box for cropping
box = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.4, -0.38, -1.28), max_bound=(0.4, 0.1, -0.6))
# crop out the plant
pcd_plant = pcd.crop(box)
# remove outliers by average distance between points
pcd_plant, ind = pcd_plant.remove_statistical_outlier(nb_neighbors=20, std_ratio=5.0)
pcd_cleaned = pcd_plant.select_by_index(ind)
# remove radius outliers
pcd_plant, ind = pcd_plant.remove_radius_outlier(nb_points=5, radius=0.01)
pcd_cleaned = pcd_plant.select_by_index(ind)
# write to new PLY file
o3d.io.write_point_cloud("pcd_cleaned.ply", pcd_plant)