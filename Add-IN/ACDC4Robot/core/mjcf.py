# -*- coding: utf-8 -*-
# Author: Nuofan - 12233200@mail.sustech.edu.cn
# Date: 20231124
# Functions for generating mjcf file

import adsk, adsk.core, adsk.fusion
from .link import Link
from .joint import Joint
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement
from . import math_operation as math_op
from . import utils
from ..commands.ACDC4Robot import constants

def get_mjcf_mesh(link: Link) -> Element:
    """
    Get mesh element for mjcf

    Return
    ---------
    mesh_ele: Element
        mesh element in mjcf
    """
    mesh_ele = ET.Element("mesh")
    name: str = link.get_name()
    file_name: str = "meshes/" + link.get_name() + ".stl"
    mesh_ele.attrib = {"name":name, "file": file_name, "scale": "0.001 0.001 0.001"}
    return mesh_ele

def get_mjcf_body(link: Link, parent_link: Link = None) -> Element:
    """
    Get body element for mjcf

    Return
    ---------
    body_ele: Element
        body element represent a link in mjcf
    """
    body_ele = Element("body")
    body_name: str = link.get_name()
    if parent_link is None:
        # a root of a body tree
        # pose = math_op.matrix3d_2_euler_xyz(link.pose) 
        pose = math_op.matrix3d_2_pose(link.pose)
        pos_attr = "{} {} {}".format(pose[0], pose[1], pose[2])
        euler_attr = "{} {} {}".format(pose[3], pose[4], pose[5])
    else:
        parent_frame: adsk.core.Matrix3D = parent_link.pose # parent_frame w.r.t world_frame
        child_frame: adsk.core.Matrix3D = link.pose # child_frame w.r.t world_frame
        parent_T_child: adsk.core.Matrix3D = math_op.coordinate_transform(parent_frame, child_frame)
        # pose = math_op.matrix3d_2_euler_xyz(parent_T_child)
        pose = math_op.matrix3d_2_pose(parent_T_child)
        pos_attr = "{} {} {}".format(pose[0], pose[1], pose[2])
        euler_attr = "{} {} {}".format(pose[3], pose[4], pose[5])

    body_ele.attrib = {"name": body_name, "pos": pos_attr, "euler": euler_attr}

    # insert parent joint if it exists
    parent_joint = link.get_parent_joint()
    if parent_joint is not None:
        joint_ele: Element = get_mjcf_joint(Joint(parent_joint))
        # undifined joint will welded two body
        if joint_ele is not None:
            body_ele.append(joint_ele)

    geom_ele: Element = get_mjcf_geom(link)
    inertial_ele: Element = get_mjcf_inertial(link)

    body_ele.append(geom_ele)
    body_ele.append(inertial_ele)

    return body_ele

def get_mjcf_geom(link: Link) -> Element:
    """
    Get geom element for mjcf

    Return
    ---------
    geom_ele: Element
        geom element in mjcf
    """
    geom_ele = Element("geom")
    geom_name = link.get_name() + "_geom"
    # geom_pos = "{} {} {}".format(link.get_pose_sdf()[0], link.get_pose_sdf()[1], link.get_pose_sdf()[2])
    # geom_euler = "{} {} {}".format(link.get_pose_sdf()[3], link.get_pose_sdf()[4], link.get_pose_sdf()[5])

    # geom should coincide with the body frame? It works
    geom_pos = "0 0 0"
    geom_euler = "0 0 0"

    visual_body = link.get_visual_body()
    col_body = link.get_collision_body()
    if (visual_body is None) and (col_body is None):
        geom_ele.attrib = {"name": geom_name, "type": "mesh", 
                    "mesh": link.get_name(), "pos": geom_pos, "euler": geom_euler}
    elif (visual_body is not None) and (col_body is not None):
        error_message = "mjcf will automatically generate geometry for collision. \n"
        error_message = error_message + link.get_name() + " does not need to set geometry for visual and collision seperately."
        utils.error_box(error_message)
        utils.terminate_box()
    elif (visual_body is None) and (col_body is not None):
        error_message = "mjcf will automatically generate geometry for collision. \n"
        error_message = error_message + link.get_name() + " does not need to set geometry for visual and collision seperately."
        utils.error_box(error_message)
        utils.terminate_box()
    elif (visual_body is not None) and (col_body is None):
        error_message = "mjcf will automatically generate geometry for collision. \n"
        error_message = error_message + link.get_name() + " does not need to set geometry for visual and collision seperately."
        utils.error_box(error_message)
        utils.terminate_box()
    
    return geom_ele
    

def get_mjcf_inertial(link: Link) -> Element:
    """
    Get inertial element for mjcf

    Return
    ---------
    inertial_ele: Element
        inertial element in mjcf
    """
    inertial_ele = Element("inertial")
    mass: str = str(link.get_mass())
    CoM: list = link.get_CoM_wrt_link()
    pos_att: str = "{} {} {}".format(CoM[0], CoM[1], CoM[2])
    # get error for euler attribute in inertial, so remove it
    # euler_att: str = "{} {} {}".format(CoM[3], CoM[4], CoM[5])
    inertial: list = link.get_inertia_mjcf()
    # 6 numbers in the following order: M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3).
    # which is ixx, iyy, izz, ixy, ixz, iyz
    fullinertia_att = "{} {} {} {} {} {}".format(inertial[0], inertial[1], inertial[2],
                                                 inertial[3], inertial[4], inertial[5])
    # inertial_ele.attrib = {"mass": mass, "pos": pos_att, 
    #                        "euler": euler_att, "fullinertia": fullinertia_att}
    inertial_ele.attrib = {"mass": mass, "pos": pos_att, 
                            "fullinertia": fullinertia_att}
    
    return inertial_ele

def get_mjcf_joint(joint: Joint) -> Element:
    """
    Get joint element for mjcf
    A joint creates motion degrees of freedom 
    between the body where it is defined and the body's parent.

    Return
    ---------
    joint_ele: Element
        joint element in mjcf
    """
    # If no joints are defined, the body is welded to its parent.
    joint_type = joint.get_mjcf_joint_type()
    if joint_type is not None:
        joint_ele: Element = ET.Element("joint")
        name_att = joint.get_name()
        pose = joint.get_sdf_origin()
        axis = joint.get_axis_mjcf()
        joint_ele.attrib = {"name": name_att, "type": joint_type, 
                            "axis": "{} {} {}".format(axis[0], axis[1], axis[2]),
                            "pos": "{} {} {}".format(pose[0], pose[1], pose[2])}


        limits = joint.get_limits()
        if limits is not None:
            lower, upper = limits
            joint_ele.attrib["range"] = "{} {}".format(lower, upper)
            joint_ele.attrib["limited"] = "true"
        return joint_ele
    else:
        return None
    

def get_mjcf(root_comp: adsk.fusion.Component, robot_name: str, dir: str) -> Element:
    """
    Get a mjcf format xml file contains robot model

    Parameters:
    root_comp: root_comp of the design
    robotName: robot name
    dir: directory of the mjcf file

    Return:
    mujoco_ele: Element
        root element of a mujoco xml file
    """
    root = ET.Element("mujoco", {"model": robot_name})

    # add compiler subelement of mujoco
    # the default eulerseq conventioin used in URDF corresponds to the default “xyz” in MJCF
    # Here we use an extrinsic X-Y-Z rotation
    # compiler_ele = ET.SubElement(root, "compiler", 
    #                             {"angle": "radian", "meshdir": (dir + "/meshes"), "eulerseq":"XYZ"})
    # Rotation matrix to euler in "xyz" seems have problem, use "XYZ" at temporary
    compiler_ele = ET.SubElement(root, "compiler", 
                                {"angle": "radian", "eulerseq":"XYZ"}) 
    
    # add asset subelement of mujoco
    asset_ele = ET.SubElement(root, "asset")

    # add worldbody subelement of mujoco
    worldbody_ele = ET.SubElement(root, "worldbody")

    # add light subelement of worldbody
    light_ele = ET.SubElement(worldbody_ele, "light")
    light_ele.attrib = {"directional":"true", "pos":"-0.5 0.5 3", "dir":"0 0 -1"}

    # add floor 
    floor = ET.SubElement(worldbody_ele, "geom", )
    floor.attrib = {"pos": "0 0 0", "size": "1 1 1", "type": "plane", "rgba": "1 0.83 0.61 0.5"}

    # add body elements to construct a robot
    parent_child_dict = {}
    all_occs = root_comp.allOccurrences

    joints = [j for j in root_comp.allJoints] + [j for j in root_comp.allAsBuiltJoints]
    for joint in joints:
        parent = joint.occurrenceTwo
        child = joint.occurrenceOne
        if parent is None:
            continue
        if parent.fullPathName not in parent_child_dict:
            parent_child_dict[parent.fullPathName] = []

        parent_child_dict[parent.fullPathName].append(child)
    
    # Function to recursively add body elements to mjcf
    def add_body_element(parent: adsk.fusion.Occurrence, parent_body_ele: Element) -> Element:
        children = parent_child_dict.get(parent.fullPathName, [])
        for child_occ in children:
            # Create mjcf element for child
            child_link = Link(child_occ)
            parent_link = Link(parent)
            child_ele = get_mjcf_body(child_link, parent_link)
            parent_body_ele.append(child_ele)
            # child_ele = ET.SubElement(parent_body_ele, "body")
            # child_ele_name: str = child_link.get_name()
            # child_ele.attrib = {"name": child_ele_name}

            # Recursively add children of this child_ele
            add_body_element(child_occ, child_ele)
    
    # traverse all the occs for body elements
    for occ in all_occs:
        if not occ.isLightBulbOn or not utils.component_has_bodies(occ.component):
            continue
        asset_ele.append(get_mjcf_mesh(Link(occ)))
        if occ.fullPathName not in [item.fullPathName for sublist in parent_child_dict.values() for item in sublist]:
            # parent_ele = ET.SubElement(worldbody_ele, "body", name=Link(occ).get_name())
            parent_ele = get_mjcf_body(Link(occ), None)
            worldbody_ele.append(parent_ele)
            add_body_element(occ, parent_ele)
    
    return root


