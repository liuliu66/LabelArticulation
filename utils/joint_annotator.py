import os
import open3d as o3d
import json
import copy
import numpy as np

from .util_func import rgbd2pc, icp_registration, Scaling, ComputeTransformationMatrixAroundCentroid, RotateAnyAxis


class JointAnnotator():
    def __init__(self,
                 voxel_size=0.003,
                 step_rot = 0.01,
                 step_tran = 0.0001):
        self.voxel_size = voxel_size
        self.step_rot = step_rot
        self.step_tran = step_tran
        self.joint_transformation = np.identity(4)
        self.view_param = None
        self.trans_initialized = False

        self.arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.000005,
                                                            cone_radius=0.00001,
                                                            cylinder_height=0.0001,
                                                            cone_height=0.00002,
                                                            resolution=10,
                                                            cylinder_split=4,
                                                            cone_split=1)
        arrow_pts_num = np.asarray(self.arrow.vertices).shape[0]
        self.arrow.vertex_colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0]]).repeat(arrow_pts_num, axis=0))

    def init_annotator(self, model_to_be_annotated, init_joint_transformation=None):
        self.model_to_be_annotated = model_to_be_annotated
        if init_joint_transformation is not None:
            self.trans_initialized = True
            self.joint_transformation = init_joint_transformation

    def reset(self, model_to_be_annotated):
        self.model_to_be_annotated = model_to_be_annotated

        if self.trans_initialized:
            self.arrow.transform(self.joint_transformation)

    def begin_annotation(self, view_param):
        self.view_param = view_param
        if not self.trans_initialized:
            self.joint_transformation = self.demo_manual_location(self.model_to_be_annotated)

            self.arrow.transform(self.joint_transformation)

        self.visualizer_3d = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer_3d.register_key_callback(ord("1"), self.press_1)
        self.visualizer_3d.register_key_callback(ord("2"), self.press_2)
        self.visualizer_3d.register_key_callback(ord("3"), self.press_3)
        self.visualizer_3d.register_key_callback(ord("4"), self.press_4)
        self.visualizer_3d.register_key_callback(ord("5"), self.press_5)
        self.visualizer_3d.register_key_callback(ord("6"), self.press_6)
        self.visualizer_3d.register_key_callback(87, self.press_x_up)
        self.visualizer_3d.register_key_callback(83, self.press_x_down)
        self.visualizer_3d.register_key_callback(65, self.press_y_up)
        self.visualizer_3d.register_key_callback(68, self.press_y_down)
        self.visualizer_3d.register_key_callback(69, self.press_z_up)
        self.visualizer_3d.register_key_callback(67, self.press_z_down)

        self.visualizer_3d.create_window()
        self.visualizer_3d.add_geometry(self.model_to_be_annotated)
        self.visualizer_3d.add_geometry(self.arrow)
        self.visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(self.view_param)
        self.visualizer_3d.run()
        self.view_param = self.visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        self.visualizer_3d.destroy_window()

        return self.joint_transformation, self.view_param

    def generate_demo_img(self, view_point):
        vis_temp = o3d.visualization.Visualizer()
        vis_temp.create_window(visible=False)
        vis_temp.add_geometry(self.model_to_be_annotated)
        vis_temp.add_geometry(self.arrow)
        vis_temp.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        vis_temp.poll_events()
        vis_temp.update_renderer()
        demo_img = vis_temp.capture_screen_float_buffer(True)
        # vis_temp.run()
        vis_temp.destroy_window()

        return demo_img, view_point

    def reset_render(self):
        # vis.clear_geometries()
        self.view_param = self.visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        self.visualizer_3d.remove_geometry(self.arrow)

        # vis.update_geometry(material_pc)
        self.visualizer_3d.add_geometry(self.arrow)

        self.visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(self.view_param)
        return False

    def demo_manual_location(self, target):
        print("Demo for manual ICP")
        print("Visualization of two point clouds before manual alignment")
        transformation = np.identity(4)
        # draw_registration_result(source, target, np.identity(4))

        # pick points from two point clouds and builds correspondences
        picked_id_target = self.pick_point(target)
        assert len(picked_id_target) == 1, 'only one point for point location'
        # picked_id_source = [picked_id_source[i].index for i in range(len(picked_id_source))]
        # picked_id_source = pick_points(source)
        # picked_id_target = pick_points(target)
        translation = np.array(target.points)[picked_id_target[0]]
        transformation[:3, 3] = translation
        return transformation

    def pick_point(self, target):
        print("")
        print("   Press [shift + left click] to undo point picking")
        vis2 = o3d.visualization.VisualizerWithEditing()
        vis2.create_window()
        vis2.add_geometry(target)
        vis2.run()  # user picks points
        vis2.destroy_window()
        print("")
        return vis2.get_picked_points()

    def press_1(self, vis):
        print('Rotation around roll axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, self.step_rot, 0, 0)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_2(self, vis):
        print('Rotation around roll axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, -self.step_rot, 0, 0)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_3(self, vis):
        print('Rotation around pitch axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, 0, self.step_rot, 0)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_4(self, vis):
        print('Rotation around pitch axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, 0, -self.step_rot, 0)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_5(self, vis):
        print('Rotation around yaw axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, 0, 0, self.step_rot)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_6(self, vis):
        print('Rotation around yaw axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.arrow, 0, 0, -self.step_rot)
        self.arrow.transform(rotation)

        self.joint_transformation = np.dot(rotation, self.joint_transformation)
        self.reset_render()

    def press_x_up(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[0, 3] += self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def press_x_down(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[0, 3] += -self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def press_y_up(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[1, 3] += self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def press_y_down(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[1, 3] += -self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def press_z_up(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[2, 3] += self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def press_z_down(self, vis):
        print('Translate along x')
        # axis = CLOUD_AXIS_ROT.points[1] - CLOUD_AXIS_ROT.points[0]
        # rotation_vec = axis / sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        # rotation_step = np.identity(4)
        # rotation_step[:3, :3] = cv2.Rodrigues(rotation_vec * -step)[0]

        offset = np.identity(4)
        offset[2, 3] += -self.step_tran * 0.01

        self.arrow.transform(offset)

        self.joint_transformation = np.dot(offset, self.joint_transformation)
        self.reset_render()

    def save_joint_transformation(self, save_path):
        save_path = save_path.split(os.getcwd().replace('\\', '/'))[1][1:]

        ann_dict = {}
        ann_dict['joint_transformation'] = self.joint_transformation.tolist()
        with open(save_path + '/' + 'joint_transformation.json', 'w') as f_out:
            json.dump(ann_dict, f_out)


if __name__ == '__main__':
    voxel_size = 0.003
    step_rot = 0.01
    step_tran = 0.0001
    # load raw scanned model that to be segmented
    model_to_be_annotated = o3d.io.read_point_cloud('models_to_be_annotated/box5_before_align/box5.ply')
    # model_to_be_annotated = o3d.io.read_point_cloud('models_to_be_annotated/box5/base_link.ply')

    arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.000005,
                                                   cone_radius=0.00001,
                                                   cylinder_height=0.0001,
                                                   cone_height=0.00002,
                                                   resolution=10,
                                                   cylinder_split=4,
                                                   cone_split=1)
    arrow_pts_num = np.asarray(arrow.vertices).shape[0]
    arrow.vertex_colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0]]).repeat(arrow_pts_num, axis=0))

    o3d.visualization.draw_geometries([arrow, model_to_be_annotated])


