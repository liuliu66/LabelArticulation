import os
import open3d as o3d
import numpy as np
import copy
import json
from math import *
from PyQt5.QtCore import *

from utils.util_func import transform_coordinates_3d
from utils.axis_aligner import AxisAligner
from utils.part_segmentator import PartSegmentator
from utils.joint_annotator import JointAnnotator
from utils.animation import AnimationPlayer
from utils.urdf_exporter import URDFExporter


class Annotator():
    def __init__(self,
                 annotation_material_path,
                 save_path=None):
        self.model_to_be_annotated_path = None
        self.annotation_material_path = annotation_material_path
        self.annotation_material_list = [p for p in os.listdir(self.annotation_material_path) if p.endswith('.obj')]
        self.save_path = save_path
        self.temp_path = None

        self.model_to_be_annotated = None
        self.annotation_material = None
        self.view_param = None
        self.align_transformation = None
        self.joint_transformation = None
        self.demo_img_axis_align = None
        self.demo_img_part_segmentation = None
        self.demo_img_joint_annotation = None
        self.annotated_joint_infos = []
        self.material_color = 'color1'

        self.axis_aligner = AxisAligner()
        self.part_segmentator = PartSegmentator()
        self.joint_annotator = JointAnnotator()
        self.animation_player = AnimationPlayer()
        self.urdf_exporter = URDFExporter()

        self.current_ann_stage = "Axis Align"

    def init_annotator(self):
        self.current_material_index = 0

        self.model_to_be_annotated_path = self.model_to_be_annotated_path.split(os.getcwd().replace('\\', '/'))[1][1:]
        self.model_to_be_annotated_name = self.model_to_be_annotated_path.split('/')[-1].split('_')[0] + '.ply'
        self.model_to_be_annotated = o3d.io.read_point_cloud(os.path.join(self.model_to_be_annotated_path,
                                                                          self.model_to_be_annotated_name))
        self.demo_img_init, self.view_param = self.generate_demo_img()
        self.init_align_transformation_path = os.path.join(self.temp_path, 'align_transformation.json')
        if os.path.lexists(self.init_align_transformation_path):
            f = json.load(open(self.init_align_transformation_path))
            self.align_transformation = np.array(f['align_transformation'])
            # self.model_to_be_annotated.transform(self.align_transformation)
        self.init_joint_transformation_path = os.path.join(self.temp_path, 'joint_transformation.json')
        if os.path.lexists(self.init_joint_transformation_path):
            f = json.load(open(self.init_joint_transformation_path))
            self.joint_transformation = np.array(f['joint_transformation'])

        self.axis_aligner.init_annotator(self.model_to_be_annotated,
                                         init_align_transformation=self.align_transformation)
        self.part_segmentator.init_annotator(self.model_to_be_annotated,
                                             part_mesh_save_path=self.model_to_be_annotated_path)
        self.joint_annotator.init_annotator(self.model_to_be_annotated,
                                            init_joint_transformation=self.joint_transformation)
        # self.reset()

    def update_model(self, id):
        obj_name, obj_color = id.split('_')
        self.material_color = obj_color
        self.current_material_index = self.annotation_material_list.index(obj_name + '.obj')
        self.reset()

    def reset(self):
        self.annotation_material = self.annotation_material_path + '/' + self.annotation_material_list[self.current_material_index]

        self.axis_aligner.reset(self.annotation_material, self.model_to_be_annotated, self.material_color)
        self.part_segmentator.reset(self.model_to_be_annotated)
        self.joint_annotator.reset(self.model_to_be_annotated)

    def set_animation_info(self, parent_file, child_file, lower, upper, joint_type):
        self.animation_parent_mesh = o3d.io.read_point_cloud(parent_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        self.animation_child_mesh = o3d.io.read_point_cloud(child_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        self.animation_joint_lower = lower
        self.animation_joint_upper = upper
        self.animation_joint_type = joint_type

    def begin_annotation(self, stage):
        if stage == "axis align":
            self.align_transformation, self.view_param = self.axis_aligner.begin_annotation(self.view_param)
            self.axis_aligner.save_align_transformation(self.temp_path)
            self.demo_img_axis_align, self.view_param = self.axis_aligner.generate_demo_img(view_point=self.view_param)
        elif stage == "part segmentation":
            self.view_param = self.part_segmentator.begin_annotation(self.view_param)
            self.demo_img_part_segmentation, self.view_param = self.part_segmentator.generate_demo_img(view_point=self.view_param)
        elif stage == "joint annotation":
            self.joint_transformation, self.view_param = self.joint_annotator.begin_annotation(self.view_param)
            self.joint_annotator.save_joint_transformation(self.temp_path)
            self.demo_img_joint_annotation, self.view_param = self.joint_annotator.generate_demo_img(view_point=self.view_param)

    def generate_demo_img(self, view_point=None):
        axis_pcd_temp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.0001, origin=[0, 0, 0])

        vis_temp = o3d.visualization.Visualizer()
        vis_temp.create_window(visible=False)
        vis_temp.add_geometry(self.model_to_be_annotated)
        vis_temp.add_geometry(axis_pcd_temp)
        if view_point is None:
            view_point = vis_temp.get_view_control().convert_to_pinhole_camera_parameters()
        else:
            vis_temp.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        vis_temp.poll_events()
        vis_temp.update_renderer()
        demo_img = vis_temp.capture_screen_float_buffer(True)
        # vis_temp.run()
        vis_temp.destroy_window()

        return demo_img, view_point

    def play_animation(self):
        self.animation_player.play_animation(self.animation_parent_mesh,
                                             self.animation_child_mesh,
                                             self.animation_joint_lower,
                                             self.animation_joint_upper,
                                             self.animation_joint_type,
                                             self.joint_transformation,
                                             self.view_param)

    def saveann(self):
        save_path = self.model_to_be_annotated_path.split(os.getcwd().replace('\\', '/'))[1][1:]
        """Save annotation and quit"""
        """ Save output files """
        ann_dict = {}
        ann_dict['image_name'] = self.cimg.split(os.getcwd().replace('\\', '/'))[1][1:]
        ann_dict['joint_transformation'] = self.joint_transformation.tolist()
        ann_dict['joint_type'] = self.joint_type

        with open(save_path + '/' + 'joint.json', 'w') as f_out:
            json.dump(ann_dict, f_out)

    def record_joint_info(self, parent_file, child_file, lower, upper, joint_type):
        parent_name = os.path.basename(parent_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        child_name = os.path.basename(child_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        joint_name = parent_name.split('.')[0] + '|' + joint_type + '|' + child_name.split('.')[0]
        if all(joint_name not in v for v in self.annotated_joint_infos):
            joint_info = {}
            joint_info['name'] = joint_name
            joint_info['parent'] = parent_name
            joint_info['child'] = child_name
            joint_info['lower'] = float(lower) if lower != '' else 0
            joint_info['upper'] = float(upper) if upper != '' else 0
            joint_info['type'] = joint_type

            line_xyz = np.array([0., 0., 0.])
            line_axis = np.array([0., 0., 1.])

            end_point = [(line_xyz[i] + line_axis[i]) for i in range(len(line_xyz))]
            line_points = np.stack([line_xyz, end_point])
            line_points = transform_coordinates_3d(line_points.T, self.joint_transformation).T

            joint_info['xyz'] = line_points[0].tolist()
            joint_info['rpy'] = line_points[1].tolist()
            self.annotated_joint_infos.append(joint_info)

    def save_urdf(self, save_file_path):
        save_file_path = save_file_path.split(os.getcwd().replace('\\', '/'))[1][1:]
        self.urdf_exporter.export_urdf(save_file_path,
                                       joint_infos=self.annotated_joint_infos,
                                       part_path=os.path.join(self.model_to_be_annotated_path, 'part_meshes'))