import os
import json
import numpy as np
import copy
import open3d as o3d

from utils.util_func import rgbd2pc, icp_registration, Scaling, ComputeTransformationMatrixAroundCentroid, RotateAnyAxis

PART_CLASSES = {'box': ['BG', 'base_link', 'link1'],
                'stapler': ['BG', 'base_link', 'link1'],
                'cutter': ['BG', 'base_link', 'link1'],
                'drawer': ['BG', 'base_link', 'link1', 'link2', 'link3'],
                'scissor': ['BG', 'link1', 'link2']}


def generatePC(im_pc):
    global CLOUD_ROT
    global visualizer_3d
    global view_param

    visualizer_3d.clear_geometries()

    # visualizer_3d.remove_geometry(mesh_temp)

    if view_param is None:
        visualizer_3d.add_geometry(CLOUD_ROT)

        view_param = visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()

        visualizer_3d.clear_geometries()
    else:
        view_param = visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()

    # cloud_c = copy.deepcopy(CLOUD_ROT)
    # cloud_c, center = c3D.Centering(cloud_c)
    # np_cloud = np.asarray(cloud_c.points)

    # pc_temp = o3.geometry.PointCloud()
    # pc_temp.points = o3.utility.Vector3dVector(np_cloud)
    # pc_temp.transform(all_transformation)

    # mesh_temp = copy.deepcopy(MESH_ROT)
    # mesh_temp.transform(all_transformation)

    visualizer_3d.add_geometry(im_pc)
    # MESH_ROT = mesh_temp
    visualizer_3d.add_geometry(CLOUD_ROT)
    # del mesh_temp

    # visualizer_3d.update_geometry()
    # visualizer_3d.poll_events()
    # visualizer_3d.update_renderer()
    visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(view_param)
    visualizer_3d.run()
    #print(view_param.intrinsic.intrinsic_matrix)
    # visualizer_3d.destroy_window()
    #visualizer_3d.reset_view_point(False)

    # o3.visualization.draw_geometries([im_pc, mesh_temp], 'vis')


def fetch_joints_params(urdf_path):
    joint_ins = {}
    urdf_metas = json.load(open(urdf_path))['urdf_metas']
    for urdf_meta in urdf_metas:
        # joint_ins_key = urdf_meta['id']
        joint_ins_key = urdf_meta['object_name']
        if urdf_meta == []:
            continue
        joint_ins[joint_ins_key] = dict(xyz=[], axis=[], type=[], parent=[], child=[])
        joint_types = urdf_meta['joint_types']
        joint_parents = urdf_meta['joint_parents']
        joint_children = urdf_meta['joint_children']
        joint_xyz = urdf_meta['joint_xyz']
        joint_rpy = urdf_meta['joint_rpy']
        assert len(joint_types) == len(joint_parents) == len(joint_children) == len(joint_xyz) == len(joint_rpy)

        num_joints = len(joint_types)
        for n in range(num_joints):
            x, y, z = joint_xyz[n]
            joint_ins[joint_ins_key]['xyz'].append([y, z, x])
            r, p, y = joint_rpy[n]
            joint_ins[joint_ins_key]['axis'].append([p, y, r])
            joint_ins[joint_ins_key]['type'].append(joint_types[n])
            joint_ins[joint_ins_key]['parent'].append(joint_parents[n])
            joint_ins[joint_ins_key]['child'].append(joint_children[n])

    return joint_ins


# Pose refinement by ICP
def refine_registration(source, target, trans, voxel_size):
    global all_transformation
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-point ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(source, target,
                                 distance_threshold, trans,
                                 o3d.registration.TransformationEstimationPointToPoint())
    return result.transformation


if __name__ == '__main__':
    voxel_size = 0.003
    step = 0.01
    path = 'examples/articulation_pose_label/data/'
    model_path = path + 'models/'
    rgb_path = path + 'color'
    depth_path = path + 'depth'
    categories = ['box']
    instances = ['box2']
    all_joint_ins = fetch_joints_params(os.path.join(model_path, 'urdf_metas.json'))

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
        ann_instance = dict(instance_id=i, instance_name=instance)
        parts = PART_CLASSES[categories[i]][1:]
        if instance not in ['drawer11', 'drawer12', 'drawer13', 'drawer14', 'drawer15']:
            parts = PART_CLASSES[categories[i]][1:3]
        joint_ins = all_joint_ins[instance]
        for j, part in enumerate(parts):
            pcd_img = rgbd2pc(os.path.join(rgb_path, rgb_list[0]),
                              os.path.join(depth_path, depth_list[0]),
                              camera_intrinsic)

            pcd_model_path = model_path + '{}/{}/{}.ply'.format(categories[i], instance, part)
            pcd_model = o3d.io.read_point_cloud(pcd_model_path)
            pcd_model = o3d.geometry.PointCloud.voxel_down_sample(pcd_model, voxel_size)

            with open(save_path, 'r') as f:
                ann = json.load(f)
                if parts[j] in ann[i].keys():
                    init_trans = np.array(ann[i][parts[j]])
                else:
                    init_trans = np.array(ann[i][parts[0]])

            print('Use initial transformation\n', init_trans)
            all_transformation = np.dot(np.identity(4), init_trans)

            CLOUD_ROT = copy.deepcopy(pcd_model)
            CLOUD_ROT.transform(all_transformation)

            """transformation axis define"""
            if j == 0:
                use_prismatic = False
                use_revolute = False
            else:
                if joint_ins['type'][j-1] == 'prismatic':
                    use_prismatic = True
                elif joint_ins['type'][j-1] == 'revolute':
                    use_revolute = True

                line_xyz = np.array(joint_ins['xyz'][j-1])
                line_axis = np.array(joint_ins['axis'][j-1])
                start_tran = [(line_xyz[i] - line_axis[i]) for i in range(len(line_xyz))]
                end_tran = [(line_xyz[i] + line_axis[i]) for i in range(len(line_xyz))]
                line_pcd_tran = o3d.geometry.PointCloud()
                line_pcd_tran.points = o3d.utility.Vector3dVector([line_xyz, end_tran])

                CLOUD_AXIS = copy.deepcopy(line_pcd_tran)
                CLOUD_AXIS.transform(all_transformation)

            """Quit and save"""
            def press_s(vis):
                global visualizer_3d
                print('save anns')
                """ Save output files """
                ann[i][part] = all_transformation.tolist()
                with open(save_path, 'w') as f:
                    json.dump(ann, f)

            """ICP registration"""
            def press_i(vis):
                global all_transformation
                print('ICP start (coarse mode)')
                result_icp = refine_registration(CLOUD_ROT, pcd_img, np.identity(4), 10.0 * voxel_size)
                CLOUD_ROT.transform(result_icp)
                all_transformation = np.dot(result_icp, all_transformation)

                generatePC(pcd_img)


            def press_f(vis):
                global all_transformation
                print('ICP start (fine mode)')
                result_icp = refine_registration(CLOUD_ROT, pcd_img, np.identity(4), 3.0 * voxel_size)
                print(result_icp)
                CLOUD_ROT.transform(result_icp)
                all_transformation = np.dot(result_icp, all_transformation)

                generatePC(pcd_img)


            def press_1(vis):
                global all_transformation
                print('Rotation around roll axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, step, 0, 0)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_2(vis):
                global all_transformation
                print('Rotation around roll axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, -step, 0, 0)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_3(vis):
                global all_transformation
                print('Rotation around pitch axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, step, 0)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_4(vis):
                global all_transformation
                print('Rotation around pitch axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, -step, 0)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_5(vis):
                global all_transformation
                print('Rotation around yaw axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, 0, step)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_6(vis):
                global all_transformation
                print('Rotation around yaw axis')
                rotation = ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, 0, -step)
                CLOUD_ROT.transform(rotation)

                all_transformation = np.dot(rotation, all_transformation)
                generatePC(pcd_img)


            def press_7(vis):
                global all_transformation
                if use_prismatic:
                    print('Translation along axis')
                    axis = CLOUD_AXIS.points[1] - CLOUD_AXIS.points[0]
                    # transformation_step = c3D.ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, 0, 0, axis*step)
                    transformation_step = np.identity(4)
                    transformation_step[:3, 3] += axis * step

                    CLOUD_ROT.transform(transformation_step)

                    all_transformation = np.dot(transformation_step, all_transformation)
                    generatePC(pcd_img)
                else:
                    print('the object does not support prismatic')


            def press_8(vis):
                global all_transformation
                if use_prismatic:
                    print('Translation along axis')
                    axis = CLOUD_AXIS.points[1] - CLOUD_AXIS.points[0]
                    # transformation_step = c3D.ComputeTransformationMatrixAroundCentroid(CLOUD_ROT, 0, 0, 0, axis*step)
                    transformation_step = np.identity(4)
                    transformation_step[:3, 3] += axis * -step

                    CLOUD_ROT.transform(transformation_step)
                    # CLOUD_AXIS.transform(transformation_step)

                    all_transformation = np.dot(transformation_step, all_transformation)
                    generatePC(pcd_img)
                else:
                    print('the object does not support prismatic')


            def press_9(vis):
                global all_transformation
                if use_revolute:
                    print('Rotation along axis')
                    # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                    # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                    # rotation_step = np.identity(4)
                    # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * step)[0]

                    rotation_step = RotateAnyAxis(CLOUD_AXIS.points[0], CLOUD_AXIS.points[1], step)
                    CLOUD_ROT.transform(rotation_step)

                    all_transformation = np.dot(rotation_step, all_transformation)
                    generatePC(pcd_img)
                else:
                    print('the object does not support revolute')


            def press_0(vis):
                global all_transformation
                if use_revolute:
                    print('Rotation along axis')
                    # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                    # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                    # rotation_step = np.identity(4)
                    # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                    rotation_step = RotateAnyAxis(CLOUD_AXIS.points[0], CLOUD_AXIS.points[1], -step)
                    CLOUD_ROT.transform(rotation_step)

                    all_transformation = np.dot(rotation_step, all_transformation)
                    generatePC(pcd_img)
                else:
                    print('the object does not support revolute')


            def press_x_up(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[0, 3] += step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)


            def press_x_down(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[0, 3] += -step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)


            def press_y_up(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[1, 3] += step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)


            def press_y_down(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[1, 3] += -step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)


            def press_z_up(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[2, 3] += step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)


            def press_z_down(vis):
                global all_transformation
                print('Translate along x')
                # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
                # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
                # rotation_step = np.identity(4)
                # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

                offset = np.identity(4)
                offset[2, 3] += -step * 0.1

                CLOUD_ROT.transform(offset)

                all_transformation = np.dot(offset, all_transformation)
                generatePC(pcd_img)

            visualizer_3d = o3d.visualization.VisualizerWithKeyCallback()
            visualizer_3d.register_key_callback(ord("-"), press_s)
            visualizer_3d.register_key_callback(73, press_i)
            visualizer_3d.register_key_callback(70, press_f)
            visualizer_3d.register_key_callback(ord("1"), press_1)
            visualizer_3d.register_key_callback(ord("2"), press_2)
            visualizer_3d.register_key_callback(ord("3"), press_3)
            visualizer_3d.register_key_callback(ord("4"), press_4)
            visualizer_3d.register_key_callback(ord("5"), press_5)
            visualizer_3d.register_key_callback(ord("6"), press_6)
            visualizer_3d.register_key_callback(ord("7"), press_7)
            visualizer_3d.register_key_callback(ord("8"), press_8)
            visualizer_3d.register_key_callback(ord("9"), press_9)
            visualizer_3d.register_key_callback(ord("0"), press_0)
            visualizer_3d.register_key_callback(87, press_x_up)
            visualizer_3d.register_key_callback(83, press_x_down)
            visualizer_3d.register_key_callback(65, press_y_up)
            visualizer_3d.register_key_callback(68, press_y_down)
            visualizer_3d.register_key_callback(69, press_z_up)
            visualizer_3d.register_key_callback(67, press_z_down)
            visualizer_3d.create_window('3d_vis')
            view_param = None
            # view_param = visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
            # visualizer_3d.reset_view_point(False)
            # visualizer_3d.get_render_option().point_size = 3
            # visualizer_control = visualizer_3d.get_view_control()
            generatePC(pcd_img)

            # visualizer_3d.destroy_window()
            # del visualizer_3d
            # pcd_model = o3d.geometry.PointCloud.voxel_down_sample(pcd_model, 0.005)
            # o3d.visualization.draw_geometries([pcd_img])