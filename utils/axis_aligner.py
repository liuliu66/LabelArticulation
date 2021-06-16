import os
import numpy as np
import copy
import open3d as o3d
import json

from .util_func import rgbd2pc, icp_registration, Scaling, ComputeTransformationMatrixAroundCentroid, RotateAnyAxis

MATERIAL_COLOR_MAP = {'color1': [0, 0.651, 0.929], 'color2': [1, 0.706, 0]}


class AxisAligner():
    def __init__(self,
                 voxel_size=0.003,
                 step=0.01):
        self.voxel_size = voxel_size
        self.step = step
        self.align_transformation = np.identity(4)
        self.view_param = None
        self.trans_initialized = None

        self.axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])

    def init_annotator(self, model_to_be_annotated, init_align_transformation=None):
        self.model_to_be_annotated = model_to_be_annotated
        if init_align_transformation is not None:
            self.trans_initialized = True
            self.align_transformation = init_align_transformation

    def reset(self, annotation_material, model_to_be_annotated, material_color):
        self.model_to_be_annotated = model_to_be_annotated
        # load annotation material, here is cube
        self.material_obj = o3d.io.read_triangle_mesh(annotation_material)

        self.material_pc = o3d.geometry.PointCloud()
        self.material_pc.points = self.material_obj.vertices
        self.material_pc.paint_uniform_color(MATERIAL_COLOR_MAP[material_color])

        self.material_pc.scale(0.02, np.array([0, 0, 0]))
        if self.trans_initialized:
            self.material_pc.transform(self.align_transformation)
            self.axis_pcd.transform(self.align_transformation)

    def begin_annotation(self, view_param):
        self.view_param = view_param
        if not self.trans_initialized:
            self.align_transformation = self.demo_manual_location(self.model_to_be_annotated)

            self.material_pc.transform(self.align_transformation)
            self.axis_pcd.transform(self.align_transformation)

        self.visualizer_3d = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer_3d.register_key_callback(70, self.press_f)
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

        self.visualizer_3d.register_key_callback(79, self.press_x_larger)
        self.visualizer_3d.register_key_callback(80, self.press_x_smaller)
        self.visualizer_3d.register_key_callback(75, self.press_y_larger)
        self.visualizer_3d.register_key_callback(76, self.press_y_smaller)
        self.visualizer_3d.register_key_callback(78, self.press_y_larger)
        self.visualizer_3d.register_key_callback(77, self.press_y_smaller)
        self.visualizer_3d.create_window()
        self.visualizer_3d.add_geometry(self.model_to_be_annotated)
        self.visualizer_3d.add_geometry(self.material_pc)
        self.visualizer_3d.add_geometry(self.axis_pcd)
        self.visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(self.view_param)
        self.visualizer_3d.run()
        self.view_param = self.visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        self.visualizer_3d.destroy_window()

        return self.align_transformation, self.view_param

    def generate_demo_img(self, view_point):
        vis_temp = o3d.visualization.Visualizer()
        vis_temp.create_window(visible=False)
        vis_temp.add_geometry(self.model_to_be_annotated)
        vis_temp.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        vis_temp.poll_events()
        vis_temp.update_renderer()
        # vis_temp.run()
        demo_img = vis_temp.capture_screen_float_buffer()
        vis_temp.destroy_window()

        return demo_img, view_point

    def reset_render(self):
        # vis.clear_geometries()
        self.view_param = self.visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        self.visualizer_3d.remove_geometry(self.material_pc)
        self.visualizer_3d.remove_geometry(self.axis_pcd)

        # vis.update_geometry(material_pc)
        self.visualizer_3d.add_geometry(self.material_pc)
        self.visualizer_3d.add_geometry(self.axis_pcd)

        self.visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(self.view_param)
        return False

    """ICP registration"""
    def press_f(self, vis):
        print('ICP start (fine mode)')
        result_icp = self.refine_registration(self.material_pc, self.model_to_be_annotated, np.identity(4), 3.0 * self.voxel_size)
        print(result_icp)
        self.material_pc.transform(result_icp)
        self.axis_pcd.transform(result_icp)
        self.align_transformation = np.dot(result_icp, self.align_transformation)

        self.reset_render()

    def press_1(self, vis):
        print('Rotation around roll axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, self.step, 0, 0)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_2(self, vis):
        print('Rotation around roll axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, -self.step, 0, 0)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_3(self, vis):
        print('Rotation around pitch axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, 0, self.step, 0)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_4(self, vis):
        print('Rotation around pitch axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, 0, -self.step, 0)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_5(self, vis):
        print('Rotation around yaw axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, 0, 0, self.step)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_6(self, vis):
        print('Rotation around yaw axis')
        rotation = ComputeTransformationMatrixAroundCentroid(self.material_pc, 0, 0, -self.step)
        self.material_pc.transform(rotation)
        self.axis_pcd.transform(rotation)

        self.align_transformation = np.dot(rotation, self.align_transformation)
        self.reset_render()

    def press_x_up(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[0, 3] += self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_x_down(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[0, 3] += -self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_y_up(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[1, 3] += self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_y_down(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[1, 3] += -self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_z_up(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[2, 3] += self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_z_down(self, vis):
        print('Translate along x')
        offset = np.identity(4)
        offset[2, 3] += -self.step * 0.01

        self.material_pc.transform(offset)
        self.axis_pcd.transform(offset)

        self.align_transformation = np.dot(offset, self.align_transformation)
        self.reset_render()

    def press_x_larger(self, vis):
        print('Enlarge along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='x', scale=1.1)

        self.reset_render()

    def press_x_smaller(self, vis):
        print('Reduce along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='x', scale=0.9)

        self.reset_render()

    def press_y_larger(self, vis):
        print('Enlarge along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='y', scale=1.1)

        self.reset_render()

    def press_y_smaller(self, vis):
        print('Reduce along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='y', scale=0.9)

        self.reset_render()

    def press_z_larger(self, vis):
        print('Enlarge along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='z', scale=1.1)

        self.reset_render()

    def press_z_smaller(self, vis):
        print('Reduce along x')
        self.material_pc = self.scale_xyz_axis(self.material_pc, axis='z', scale=0.9)

        self.reset_render()

    def refine_registration(self, source, target, trans, voxel_size):
        global transformation
        distance_threshold = voxel_size * 0.4
        print(":: Point-to-point ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.registration.registration_icp(source, target,
                                     distance_threshold, trans,
                                     o3d.registration.TransformationEstimationPointToPoint())
        return result.transformation

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

    def scale_xyz_axis(self, pc, axis='x', scale=1.):
        assert axis in ['x', 'y', 'z'], 'only xyz axis supported'
        pc_recover = pc.transform(np.linalg.inv(self.align_transformation))
        np_pc = np.array(pc_recover.points)
        np_colors = np.array(pc_recover.colors)

        if axis == 'x':
            np_pc[:, 0] = np_pc[:, 0] * scale
        elif axis == 'y':
            np_pc[:, 1] = np_pc[:, 1] * scale
        else:
            np_pc[:, 2] = np_pc[:, 2] * scale

        pc_scale = o3d.geometry.PointCloud()
        pc_scale.points = o3d.utility.Vector3dVector(np_pc)
        pc_scale.colors = o3d.utility.Vector3dVector(np_colors)
        pc_scale.transform(self.align_transformation)

        return pc_scale

    def save_align_transformation(self, save_path):
        save_path = save_path.split(os.getcwd().replace('\\', '/'))[1][1:]

        ann_dict = {}
        ann_dict['align_transformation'] = self.align_transformation.tolist()
        with open(save_path + '/' + 'align_transformation.json', 'w') as f_out:
            json.dump(ann_dict, f_out)


if __name__ == '__main__':
    voxel_size = 0.003
    step = 0.01
    # load raw scanned model that to be annotated
    model_to_be_annotated = o3d.io.read_point_cloud('models_to_be_annotated/box5_before_align/box5.ply')

    # load annotation material, here is cube
    material_obj = o3d.io.read_triangle_mesh('annotation_materials/cube.obj', print_progress=True)

    material_pc = o3d.geometry.PointCloud()
    material_pc.points = material_obj.vertices
    material_pc.paint_uniform_color([0, 0.651, 0.929])

    material_pc.scale(0.02, np.array([0, 0, 0]))

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.0001, origin=[0, 0, 0])