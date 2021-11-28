import open3d as o3d
import copy
import numpy as np

source = o3d.io.read_point_cloud("/pcd1.ply")
num_pcds = 182

def combine(source):
	for i in range(num_pcds-1):
		target = o3d.io.read_point_cloud("/pcd%d.ply" %(i+2))

		# initiating parameters
		current_transformation = np.identity(4)
		threshold = 0.1
		sigma = 0.01  # standard deviation
		# estimate the normals
		source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=100))
		target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=100))
		# initiate point-to-plane improved
		loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
		transformation = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
		registration = o3d.pipelines.registration.registration_icp(source, target, threshold, current_transformation, transformation)
		# visualize
		source.transform(registration.transformation)

		# write new PLY file
		pcd1_points = np.asarray(source.points)
		pcd1_color = np.asarray(source.colors)
		pcd2_points = np.asarray(target.points)
		pcd2_color = np.asarray(target.colors)
		pcd_points = np.concatenate((pcd1_points, pcd2_points), axis=0)
		pcd_colors = np.concatenate((pcd1_color, pcd2_color), axis=0)

		pcd_combined = o3d.geometry.PointCloud()
		pcd_combined.points = o3d.utility.Vector3dVector(pcd_points)
		pcd_combined.colors = o3d.utility.Vector3dVector(pcd_colors)
		source = pcd_combined
	return source

source = combine(source)

o3d.visualization.draw_geometries([source])

#o3d.io.write_point_cloud("/pcd_combined.ply", source)