import os
import sys
import json
import open3d as o3d
import cv2
import numpy as np
import time

from .util_func import Scaling, ComputeTransformationMatrixAroundCentroid, RotateAnyAxis, transform_coordinates_3d

sys.setrecursionlimit(100)


class AnimationPlayer():
    def __init__(self,
                 step=-0.01):
        self.step = step
        self.parent_mesh = None
        self.child_mesh = None
        self.lower = 0
        self.upper = 0
        self.joint_type = 'prismatic'

    def set_animation_info(self, parent_file, child_file, lower, upper, joint_type, joint_transformation):
        self.parent_mesh = o3d.io.read_point_cloud(parent_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        self.child_mesh = o3d.io.read_point_cloud(child_file.split(os.getcwd().replace('\\', '/'))[1][1:])
        self.lower = lower
        self.upper = upper
        self.joint_type = joint_type

    def play_animation(self, parent_mesh, child_mesh, lower, upper, joint_type, joint_transformation, view_point):
        arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.000005,
                                                       cone_radius=0.00001,
                                                       cylinder_height=0.0001,
                                                       cone_height=0.00002,
                                                       resolution=10,
                                                       cylinder_split=4,
                                                       cone_split=1)
        arrow_pts_num = np.asarray(arrow.vertices).shape[0]
        arrow.vertex_colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0]]).repeat(arrow_pts_num, axis=0))
        current_state = 0
        current_step = self.step
        lower = float(lower)
        if lower == 0:
            lower -= 0.05
        upper = float(upper)
        if upper == 0:
            upper += 0.05

        line_xyz = np.array([0., 0., 0.])
        line_axis = np.array([0., 0., 1.])

        start_point = [(line_xyz[i] - line_axis[i]) for i in range(len(line_xyz))]
        end_point = [(line_xyz[i] + line_axis[i]) for i in range(len(line_xyz))]
        line_points = np.stack([start_point, end_point])
        line_points = transform_coordinates_3d(line_points.T, joint_transformation).T

        arrow.transform(joint_transformation)

        def automatic_animation_callback(visualizer_3d):
            nonlocal current_state
            nonlocal current_step
            nonlocal lower
            nonlocal upper
            if joint_type == 'revolute':
                rotation_step = RotateAnyAxis(line_points[0], line_points[1], current_step)
                child_mesh.transform(rotation_step)
                current_state += current_step
                if current_state >= upper or current_state <= lower:
                    current_step = -current_step
            elif joint_type == 'prismatic':
                axis = line_points[1] - line_points[0]
                transformation_step = np.identity(4)
                transformation_step[:3, 3] += axis * current_step
                child_mesh.transform(transformation_step)
                current_state += current_step
                if current_state >= upper or current_state <= lower:
                    current_step = -current_step

            visualizer_3d.update_geometry(child_mesh)

        # generatePC([model_pcd_base, line_pcd])

        visualizer_3d = o3d.visualization.VisualizerWithKeyCallback()
        visualizer_3d.register_animation_callback(automatic_animation_callback)
        visualizer_3d.create_window('animation')
        # generatePC([model_pcd_base, line_pcd, model_pcd_part])
        for geom in [parent_mesh, child_mesh, arrow]:
            visualizer_3d.add_geometry(geom)

        # visualizer_3d.update_geometry(child_mesh)
        visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        visualizer_3d.run()

        return True
