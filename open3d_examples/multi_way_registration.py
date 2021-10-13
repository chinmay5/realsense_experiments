import open3d as o3d
import os
import numpy as np
from environment_setup import PROJECT_ROOT_DIR

def compute():
    def load_point_clouds(voxel_size=0.0):
        pcds = []
        for i in range(1, 4):
            pcd = o3d.io.read_point_cloud(os.path.join(PROJECT_ROOT_DIR, 'open3d_examples', "out%d.ply" % i), format='ply')
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            pcds.append(pcd_down)
        return pcds

    voxel_size = 0.001
    pcds_down = load_point_clouds(voxel_size)
    # o3d.visualization.draw_geometries(pcds_down)
    # print("Recompute the normal of the downsampled point cloud")
    # for pcd in pcds_down:
    #     o3d.geometry.estimate_normals(
    #         pcd,
    #         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
    #                                                           max_nn=30))

    def pairwise_registration(source, target):
        print("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            # o3d.pipelines.registration.TransformationEstimationPointToPlane())
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=4000))
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            # o3d.pipelines.registration.TransformationEstimationPointToPlane())
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=4000))
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp


    def full_registration(pcds, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        for source_id in range(n_pcds):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp = pairwise_registration(
                    pcds[source_id], pcds[target_id])
                print("Build o3d.pipelines.registration.PoseGraph")
                # The next part is very specific to our case. We have the two point clouds somewhat overalpping
                # while the first one is a bit different. Hence, the connection between the first and the other
                # two should be loop closure ones while between second and third should be just odometry
                if source_id == 0:  # loop closure case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
                else: # odometry case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph


    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 25  # 15
    max_correspondence_distance_fine = voxel_size * 1.5  # 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pcds_down,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    pcds = load_point_clouds(voxel_size)
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration.ply", pcd_combined_down)

if __name__ == '__main__':
    compute()
    # print("Load a ply point cloud, print it, and render it")
    # pcd = o3d.io.read_point_cloud(os.path.join(PROJECT_ROOT_DIR, 'open3d_examples', "out1.ply"))
    # print(pcd)
    # print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])
    #
    # print("Downsample the point cloud with a voxel of 0.05")
    # downpcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=0.05)
    # o3d.visualization.draw_geometries([downpcd])
    #
    # print("Recompute the normal of the downsampled point cloud")
    # o3d.geometry.estimate_normals(
    #     downpcd,
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
    #                                                       max_nn=30))
    # o3d.visualization.draw_geometries([downpcd])
    #
    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10, :])
    # print("")