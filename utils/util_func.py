import os
import open3d as o3d
import numpy as np
import json
from math import *


def rgbd2pc(rgb_path, depth_path, camera_intrinsic, vis=False, save_pcd=False, depth_scale=1000.):
    color_raw = o3d.io.read_image(rgb_path)
    depth_raw = o3d.io.read_image(depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,
                                                                     depth_raw,
                                                                     depth_scale,
                                                                     2.,
                                                                     convert_rgb_to_intensity=False)


    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        camera_intrinsic
    )
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    if vis:
        o3d.visualization.draw_geometries([pcd])
    if save_pcd:
        basename = os.path.basename(rgb_path)
        pcd_save_name = basename.split('.png')[0] + '.pcd'
        o3d.io.write_point_cloud(pcd_save_name, pcd)

    return pcd


def icp_registration(source_pcd,
                     target_pcd,
                     vis=False,
                     threshold=1.0,
                     voxel_size = 0.005,
                     outlier_removal=False,
                     save_path=None):
    # outlier removal
    if outlier_removal:
        source_pcd, outlier_index_source = source_pcd.remove_statistical_outlier(nb_neighbors=20,
                                                                                 std_ratio=2.0)
        target_pcd, outlier_index_target = target_pcd.remove_statistical_outlier(nb_neighbors=20,
                                                                                 std_ratio=2.0)

    source_pcd = o3d.geometry.PointCloud.voxel_down_sample(source_pcd, voxel_size)
    target_pcd = o3d.geometry.PointCloud.voxel_down_sample(target_pcd, voxel_size)

    trans_init = np.identity(4)

    reg_p2p = o3d.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())

    transformation = reg_p2p.transformation

    evaluation = o3d.registration.evaluate_registration(source_pcd, target_pcd, threshold, transformation)
    print(evaluation)

    if vis:
        # color the pcds
        #source_pcd.paint_uniform_color([1, 0.706, 0])
        #target_pcd.paint_uniform_color([0, 0.651, 0.929])

        # transform the source pcd
        source_pcd.transform(transformation)

        # visualization
        visualizer = o3d.visualization.Visualizer()
        visualizer.create_window()

        visualizer.add_geometry(source_pcd)
        visualizer.add_geometry(target_pcd)

        # visualizer.update_geometry(source_pcd)
        visualizer.poll_events()
        visualizer.update_renderer()
        visualizer.run()

    if save_path:
        ann_dict = dict()
        ann_dict['transformation4x4'] = transformation.tolist()
        with open(save_path + '/' + 'ann.json', 'w') as f:
            json.dump(ann_dict, f)

    return transformation


def Scaling(cloud_in, scale):
    """
    multiply scaling factor to the input point cloud.
    input(s):
        cloud_in: point cloud to be scaled.
        scale: scaling factor
    output(s):
        cloud_out:
    """
    cloud_np = np.asarray(cloud_in.points)
    cloud_np *= scale
    cloud_out = o3d.geometry.PointCloud()
    cloud_out.points = o3d.utility.Vector3dVector(cloud_np)

    return cloud_out


def ComputeTransformationMatrixAroundCentroid(_cloud_in, _roll, _pitch, _yaw, _trans=None):
    """ offset center """
    try:
        np_in = np.asarray(_cloud_in.points)
    except:
        np_in = np.asarray(_cloud_in.vertices)
    center = np.mean(np_in, axis=0)
    offset = np.identity(4)
    offset[0:3, 3] -= center

    """ rotation """
    rot = RPY2Matrix4x4(_roll, _pitch, _yaw, _trans)

    """ reverse offset """
    reverse = np.identity(4)
    reverse[0:3, 3] = center

    final = np.dot(reverse, np.dot(rot, offset))

    return final


def RPY2Matrix4x4(roll, pitch, yaw, trans=None):
    rot = np.identity(4)
    if roll < -3.141:
        roll += 6.282
    elif 3.141 < roll:
        roll -= 6.282
    if pitch < -3.141:
        pitch += 6.282
    elif 3.141 < pitch:
        pitch -= 6.282
    if yaw < -3.141:
        yaw += 6.282
    elif 3.141 < yaw:
        yaw -= 6.282

    rot[0, 0] = cos(yaw) * cos(pitch)
    rot[0, 1] = -sin(yaw) * cos(roll) + (cos(yaw) * sin(pitch) * sin(roll))
    rot[0, 2] = sin(yaw) * sin(roll) + (cos(yaw) * sin(pitch) * cos(roll))
    rot[1, 0] = sin(yaw) * cos(pitch)
    rot[1, 1] = cos(yaw) * cos(roll) + (sin(yaw) * sin(pitch) * sin(roll))
    rot[1, 2] = -cos(yaw) * sin(roll) + (sin(yaw) * sin(pitch) * cos(roll))
    rot[2, 0] = -sin(pitch)
    rot[2, 1] = cos(pitch) * sin(roll)
    rot[2, 2] = cos(pitch) * cos(roll)
    if trans is not None:
        rot[3, 0] = trans[0]
        rot[3, 1] = trans[1]
        rot[3, 2] = trans[2]
    else:
        rot[3, 0] = rot[3, 1] = rot[3, 2] = 0.0
    rot[3, 3] = 1.0

    return rot


def RotateAnyAxis(v1, v2, step):
    ROT = np.identity(4)

    axis = v2 - v1
    axis = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)

    step_cos = cos(step)
    step_sin = sin(step)

    ROT[0][0] = axis[0] * axis[0] + (axis[1] * axis[1] + axis[2] * axis[2]) * step_cos
    ROT[0][1] = axis[0] * axis[1] * (1 - step_cos) + axis[2] * step_sin
    ROT[0][2] = axis[0] * axis[2] * (1 - step_cos) - axis[1] * step_sin
    ROT[0][3] = 0

    ROT[1][0] = axis[1] * axis[0] * (1 - step_cos) - axis[2] * step_sin
    ROT[1][1] = axis[1] * axis[1] + (axis[0] * axis[0] + axis[2] * axis[2]) * step_cos
    ROT[1][2] = axis[1] * axis[2] * (1 - step_cos) + axis[0] * step_sin
    ROT[1][3] = 0

    ROT[2][0] = axis[2] * axis[0] * (1 - step_cos) + axis[1] * step_sin
    ROT[2][1] = axis[2] * axis[1] * (1 - step_cos) - axis[0] * step_sin
    ROT[2][2] = axis[2] * axis[2] + (axis[0] * axis[0] + axis[1] * axis[1]) * step_cos
    ROT[2][3] = 0

    ROT[3][0] = (v1[0] * (axis[1] * axis[1] + axis[2] * axis[2]) - axis[0] * (v1[1] * axis[1] + v1[2] * axis[2])) * (1 - step_cos) + \
                (v1[1] * axis[2] - v1[2] * axis[1]) * step_sin

    ROT[3][1] = (v1[1] * (axis[0] * axis[0] + axis[2] * axis[2]) - axis[1] * (v1[0] * axis[0] + v1[2] * axis[2])) * (1 - step_cos) + \
                (v1[2] * axis[0] - v1[0] * axis[2]) * step_sin

    ROT[3][2] = (v1[2] * (axis[0] * axis[0] + axis[1] * axis[1]) - axis[2] * (v1[0] * axis[0] + v1[1] * axis[1])) * (1 - step_cos) + \
                (v1[0] * axis[1] - v1[1] * axis[0]) * step_sin
    ROT[3][3] = 1

    return ROT.T


def transform_coordinates_3d(coordinates, RT):
    assert coordinates.shape[0] == 3
    coordinates = np.vstack([coordinates, np.ones((1, coordinates.shape[1]), dtype=np.float32)])
    new_coordinates = RT @ coordinates
    new_coordinates = new_coordinates[:3, :] / new_coordinates[3, :]
    return new_coordinates


if __name__ == '__main__':
    pcd1 = o3d.io.read_point_cloud('../data/part1.ply')
    pcd2 = o3d.io.read_point_cloud('../data/part2.ply')

    transformation = icp_registration(pcd1, pcd2, vis=True)

    print(transformation)
