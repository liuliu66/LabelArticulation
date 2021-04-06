import os
import open3d as o3d
import copy
import numpy as np
import seaborn as sns

# MESH_COLORS = [[0, 0.651, 0.929], [1, 0.706, 0]]


class PartSegmentator():
    def __init__(self):
        self.view_param = None
        self.save_path = None

    def init_annotator(self, model_to_be_annotated, part_mesh_save_path):
        self.model_to_be_annotated = model_to_be_annotated
        self.part_mesh_save_path = os.path.join(part_mesh_save_path, 'part_meshes')
        if not os.path.lexists(self.part_mesh_save_path):
            os.makedirs(self.part_mesh_save_path)

    def reset(self, model_to_be_annotated):
        self.model_to_be_annotated = model_to_be_annotated

    def begin_annotation(self, view_param):
        self.view_param = view_param
        self.view_param = self.crop_point_cloud(self.model_to_be_annotated, view_point=self.view_param)

        return self.view_param

    def generate_demo_img(self, view_point):
        part_mesh_file_list = [p for p in os.listdir(self.part_mesh_save_path) if p.endswith('.ply')]
        vis_temp = o3d.visualization.Visualizer()
        vis_temp.create_window(visible=False)
        if len(part_mesh_file_list) > 0:
            mesh_colors = sns.color_palette("bright", len(part_mesh_file_list))
            for i, part_mesh_file in enumerate(part_mesh_file_list):
                part_mesh = o3d.io.read_point_cloud(os.path.join(self.part_mesh_save_path, part_mesh_file))
                part_mesh.paint_uniform_color(mesh_colors[i])
                vis_temp.add_geometry(part_mesh)
        vis_temp.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        vis_temp.poll_events()
        vis_temp.update_renderer()
        # vis_temp.run()
        demo_img = vis_temp.capture_screen_float_buffer(True)
        vis_temp.destroy_window()

        return demo_img, view_point

    def crop_triangle_mesh(self, mesh, view_point):
        print("Demo for manual geometry cropping")
        print(
            "1) Press 'Y' twice to align geometry with negative direction of y-axis"
        )
        print("2) Press 'K' to lock screen and to switch to selection mode")
        print("3) Drag for rectangle selection,")
        print("   or use ctrl + left click for polygon selection")
        print("4) Press 'C' to get a selected geometry and to save it")
        print("5) Press 'F' to switch to freeview mode")
        visualizer_3d = o3d.visualization.VisualizerWithEditing()
        visualizer_3d.create_window()
        visualizer_3d.add_geometry(mesh)
        visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(view_point)
        visualizer_3d.run()
        view_point = visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        visualizer_3d.destroy_window()
        # o3d.visualization.draw_geometries_with_editing([mesh])

        return view_point

    def crop_point_cloud(self, pc, view_point):
        print("Demo for manual geometry cropping")
        print(
            "1) Press 'Y' twice to align geometry with negative direction of y-axis"
        )
        print("2) Press 'K' to lock screen and to switch to selection mode")
        print("3) Drag for rectangle selection,")
        print("   or use ctrl + left click for polygon selection")
        print("4) Press 'C' to get a selected geometry and to save it")
        print("5) Press 'F' to switch to freeview mode")
        visualizer_3d = o3d.visualization.VisualizerWithEditing()
        visualizer_3d.create_window()
        visualizer_3d.add_geometry(pc)
        visualizer_3d.get_view_control().convert_from_pinhole_camera_parameters(view_point)

        visualizer_3d.run()
        view_point = visualizer_3d.get_view_control().convert_to_pinhole_camera_parameters()
        visualizer_3d.destroy_window()
        # o3d.visualization.draw_geometries_with_editing([pc])

        return view_point


if __name__ == '__main__':
    # load raw scanned model that to be segmented
    # model_to_be_annotated = o3d.io.read_point_cloud('models_to_be_annotated/box5_before_align/box5.ply')
    model_to_be_annotated = o3d.io.read_triangle_mesh('models_to_be_annotated/box5_before_align/box5.obj')

    while True:
        o3d.visualization.draw_geometries_with_editing([model_to_be_annotated])


