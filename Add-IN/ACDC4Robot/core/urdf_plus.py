# Add support for URDF+: An Enhanced URDF for Robots with Kinematic Loops
# https://arxiv.org/abs/2411.19753

import adsk, adsk.core, adsk.fusion
from .robot import Robot
from typing import List
from xml.etree.ElementTree import Element, SubElement
import xml.etree.ElementTree as ET
from .link import Link
from .joint import Joint
from . import math_operation as math_op
from . import utils
from ..commands.ACDC4Robot import constants
from .urdf import URDF

class URDF_PLUS(URDF):
    """
    Return an xml.etree.Element which contains robot model information in URDF+ format
    """

    def __init__(self, robot: Robot):
        super().__init__(robot)
        self.links: list[Link] = robot.get_links()
        self.tree_joints: list[Joint] = robot.get_tree_joints()
        self.loop_joints: list[Joint] = robot.get_loop_joints()

    def write_file(self, file_path):
        robot_ele = self.get_robot_ele()
        urdf_tree = ET.ElementTree(robot_ele)
        acdc4robot_info = self.get_acdc4robot_info()
        comment = ET.Comment(acdc4robot_info)
        robot_ele.append(comment)

        for link in self.links:
            link_ele = self.get_link_element(link)
            robot_ele.append(link_ele)
        
        for joint in self.tree_joints:
            joint_ele = self.get_tree_joint_element(joint)
            if joint_ele is not None:
                robot_ele.append(joint_ele)

        for loop in self.loop_joints:
            loop_ele = self.get_loop_joint_element(loop)
            robot_ele.append(loop_ele)
        
        # set indent to pretty the xml output
        ET.indent(urdf_tree, space="    ", level=0)

        urdf_tree.write(file_path, encoding="utf-8", xml_declaration=True)

    def get_tree_joint_element(self, joint: Joint):
        tree_joint_ele = self.get_joint_element(joint)
        return tree_joint_ele

    def get_loop_joint_element(self, loop: Joint):
        loop_ele = Element("loop")
        loop_ele.attrib = {"name": self.get_joint_name(loop),
                           "type": self.get_joint_type(loop)}
        
        # add predecessor element
        predecessor_ele = SubElement(loop_ele, "predecessor")
        predecessor_ele.attrib = {"name": self.get_link_name(self.get_joint_parent(loop))}
        pred_origin_ele = SubElement(predecessor_ele, "origin")

        # add successor element
        sucessor_ele = SubElement(loop_ele, "sucessor")
        sucessor_ele.attrib = {"name": self.get_link_name(self.get_joint_child(loop))}
        suc_origin_ele = SubElement(sucessor_ele, "origin")

        # get predecessor and successor origin info
        pred_link: Link = Link(loop.parent)
        suc_link: Link = Link(loop.child)
        pred_joint: adsk.fusion.Joint = pred_link.get_parent_joint()
        suc_joint: adsk.fusion.Joint = suc_link.get_parent_joint()

        ## get predecessor frame
        if pred_joint is None:
            parent_frame: adsk.core.Matrix3D = pred_link.pose
        else:
            pred_joint: Joint = Joint(pred_joint)
            pred_frame = pred_joint.get_joint_frame()
        
        ## get sucessor frame
        if suc_joint is None:
            suc_frame: adsk.core.Matrix3D = suc_link.pose
        else:
            suc_joint: Joint = Joint(suc_joint)
            suc_frame = suc_joint.get_joint_frame()

        loop_frame = loop.get_joint_frame()

        pred_transform = math_op.coordinate_transform(pred_frame, loop_frame)
        pred_origin = math_op.matrix3d_2_pose(pred_transform)
        suc_transform = math_op.coordinate_transform(suc_frame, loop_frame)
        suc_origin = math_op.matrix3d_2_pose(suc_transform)

        pred_origin_ele.attrib = {"xyz": f"{pred_origin[0]} {pred_origin[1]} {pred_origin[2]}",
                                  "rpy": f"{pred_origin[3]} {pred_origin[4]} {pred_origin[5]}"}
        suc_origin_ele.attrib = {"xyz": f"{suc_origin[0]} {suc_origin[1]} {suc_origin[2]}",
                                 "rpy": f"{suc_origin[3]} {suc_origin[4]} {suc_origin[5]}"}
        
        # add axis element
        # add axis1, urdf only has one axis for joint
        axis = self.get_joint_axis(loop)
        if axis is not None:
            axis_ele = SubElement(loop_ele, "axis")
            axis_ele.attrib = {"xyz": "{} {} {}".format(axis[0], axis[1], axis[2])}

        return loop_ele

    def get_robot_ele(self, ):
        """
        Root element of URDF+
        """
        robot_ele = Element("robot")
        robot_ele.attrib = {"name": self.robot.get_robot_name()}

        return robot_ele


    
