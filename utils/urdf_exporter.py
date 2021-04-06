import os
import json
import open3d as o3d
import numpy as np
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, tostring, SubElement, Comment, ElementTree, XML
import xml.dom.minidom


class URDFExporter():
    def __init__(self):
        pass

    def export_urdf(self, save_file_name, joint_infos, part_path):
        root = Element('robot', name=os.path.basename(save_file_name).split('.')[0])
        link_list = []
        joint_list = []
        for joint_info in joint_infos:
            parent, joint_type, child = joint_info['name'].split('|')
            parent_element = Element('link', name=parent)
            parent_visual = SubElement(parent_element, 'visual')
            parent_mesh_file = os.path.join(part_path, joint_info['parent'])
            parent_point_cloud = o3d.io.read_point_cloud(parent_mesh_file)
            parent_origin_xyz = np.mean(np.array(parent_point_cloud.points), axis=0)
            parent_origin = SubElement(parent_visual, 'origin', rpy="0.0 0.0 0.0", xyz=' '.join(str(x) for x in parent_origin_xyz))
            parent_geometry = SubElement(parent_visual, 'geometry')
            parent_mesh = SubElement(parent_geometry, 'mesh', filename="part_meshes/{}".format(joint_info['parent']), scale="1 1 1")
            parent_inertial = SubElement(parent_element, 'inertial')
            node_inertial = XML(
                '''<inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="3.0"/><inertia ixx="100" ixy="100" ixz="100" iyy="100" iyz="100" izz="100"/></inertial>''')
            parent_inertial.extend(node_inertial)
            for mass in parent_inertial.iter('mass'):
                mass.set('value', "0.0")
            for inertia in parent_inertial.iter('inertia'):
                inertia.set('ixx', "0.0")
                inertia.set('ixy', "0.0")
                inertia.set('ixz', "0.0")
                inertia.set('iyy', "0.0")
                inertia.set('iyz', "0.0")
                inertia.set('izz', "0.0")
            link_list.append(parent_element)

            child_element = Element('link', name=child)
            child_visual = SubElement(child_element, 'visual')
            child_mesh_file = os.path.join(part_path, joint_info['child'])
            child_point_cloud = o3d.io.read_point_cloud(child_mesh_file)
            child_origin_xyz = np.mean(np.array(child_point_cloud.points), axis=0)
            child_origin = SubElement(child_visual, 'origin', rpy="0.0 0.0 0.0", xyz=' '.join(str(x) for x in child_origin_xyz))
            child_geometry = SubElement(child_visual, 'geometry')
            child_mesh = SubElement(child_geometry, 'mesh', filename="part_meshes/{}".format(joint_info['child']), scale="1 1 1")
            child_inertial = SubElement(child_element, 'inertial')
            node_inertial = XML(
                '''<inertial><origin rpy="0 0 0" xyz="0 0 0"/><mass value="3.0"/><inertia ixx="100" ixy="100" ixz="100" iyy="100" iyz="100" izz="100"/></inertial>''')
            child_inertial.extend(node_inertial)
            for mass in child_inertial.iter('mass'):
                mass.set('value', "0.0")
            for inertia in child_inertial.iter('inertia'):
                inertia.set('ixx', "0.0")
                inertia.set('ixy', "0.0")
                inertia.set('ixz', "0.0")
                inertia.set('iyy', "0.0")
                inertia.set('iyz', "0.0")
                inertia.set('izz', "0.0")
            link_list.append(child_element)

            # for joint
            joint = Element('joint', name=joint_info['name'], type=joint_type)
            joint_parent = SubElement(joint, "parent", link=parent)
            joint_child = SubElement(joint, "child", link=child)

            origin_xyz = ' '.join(str(x) for x in joint_info['xyz'])
            axis_xyz = ' '.join(str(x) for x in joint_info['rpy'])

            origin = SubElement(joint, "origin",
                                xyz=origin_xyz,
                                rpy="0 0 0")
            axis = SubElement(joint, "axis", xyz=axis_xyz)
            limit = SubElement(joint, "limit", effort="1.0", lower=str(joint_info['lower']), upper=str(joint_info['upper']), velocity="1000")
            joint_list.append(joint)

        # construct the trees
        root.extend(link_list)
        root.extend(joint_list)
        xml_string = xml.dom.minidom.parseString(tostring(root))
        xml_pretty_str = xml_string.toprettyxml()
        # print(xml_pretty_str)
        tree = ET.ElementTree(root)
        # save
        with open(save_file_name, "w") as f:
            print('writing to ', save_file_name)
            f.write(xml_pretty_str)
