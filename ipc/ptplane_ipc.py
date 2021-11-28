import open3d as o3d
import numpy as np
# read the captures
source = o3d.io.read_point_cloud("/pcd1.ply")
target = o3d.io.read_point_cloud("/pcd2.ply")
# estimate the normals
source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=100))
target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=100))
# initiating parameters
current_transformation = np.identity(4)
threshold = 0.1
# initiate point to plane ICP
transformation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
registration = o3d.pipelines.registration.registration_icp(source, target, threshold, current_transformation, transformation)
# visualize the combined clouds
source.transform(registration.transformation)
o3d.visualization.draw_geometries([source, target])