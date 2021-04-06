import os
import json
import numpy as np
import copy
import open3d as o3d

from utils.util_func import rgbd2pc, icp_registration, Scaling


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def demo_manual_registration(source, target):
    print("Demo for manual ICP")
    # source = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_2.pcd")
    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source, picked_id_target = pick_points(source, target)
    # picked_id_source = [picked_id_source[i].index for i in range(len(picked_id_source))]
    # picked_id_source = pick_points(source)
    # picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print("")
    return reg_p2p.transformation


def pick_points(pcd1, pcd2):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    vis1 = o3d.visualization.VisualizerWithEditing()
    vis1.create_window('source')
    # pcd1.paint_uniform_color([1, 0.706, 0])
    # vis1.add_geometry(pcd1)
    # vis1.add_geometry(mesh)
    vis1.add_geometry(pcd1)
    vis1.update_renderer()
    vis1.run()  # user picks points
    vis1.destroy_window()
    vis2 = o3d.visualization.VisualizerWithEditing()
    vis2.create_window()
    vis2.add_geometry(pcd2)
    vis2.run()  # user picks points
    vis2.destroy_window()
    print("")
    return vis1.get_picked_points(), vis2.get_picked_points()


if __name__ == '__main__':
    path = 'examples/articulation_pose_label/data/'
    model_path = path + 'models/'
    rgb_path = path + 'color'
    depth_path = path + 'depth'
    categories = ['box']
    instances = ['box2']
    parts_total = [['base_link']]

    rgb_list = [x for x in os.listdir(rgb_path) if x.endswith('.jpg')]
    depth_list = [x for x in os.listdir(depth_path) if x.endswith('.png')]

    camera_intrinsic = o3d.io.read_pinhole_camera_intrinsic(path + 'camera_intrinsic.json')

    # assert len(rgb_list) == len(depth_list)
    save_path = path + 'trans0.json'

    if os.path.lexists(save_path):
        ann = json.load(open(save_path))
    else:
        ann = []
    for i, instance in enumerate(instances):
        if ann and i in [x['instance_id'] for x in ann]:
            continue
        ann_instance = dict(instance_id=i, instance_name=instance, category=categories[i])
        parts = parts_total[i]
        for part in parts:
            pcd_img = rgbd2pc(os.path.join(rgb_path, rgb_list[0]), os.path.join(depth_path, depth_list[0]), camera_intrinsic)

            pcd_model_path = model_path + '{}/{}/{}.ply'.format(categories[i], instance, part)
            pcd_model = o3d.io.read_point_cloud(pcd_model_path)
            # pcd_model = o3d.geometry.PointCloud.voxel_down_sample(pcd_model, 0.005)
            # o3d.visualization.draw_geometries([pcd_img])

            transformation = demo_manual_registration(pcd_model, pcd_img)
            ann_instance[part] = transformation.tolist()
        ann.append(ann_instance)

        with open(save_path, 'w') as f:
            json.dump(ann, f)
