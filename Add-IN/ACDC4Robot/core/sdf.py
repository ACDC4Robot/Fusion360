# -*- coding: utf-8 -*-
# Author: Nuofan
# 20230811
# Contains functions for generating urdf
import adsk, adsk.core, adsk.fusion
from .link import Link
from .joint import Joint
from xml.etree.ElementTree import Element, SubElement
from . import math_operation as math_op
from . import utils
from ..commands.ACDC4Robot import constants

def get_link_name(link: Link) -> str:
    """
    Return
    ---------
    link_name: str
        link's full path name
    """
    name = link.get_name()
    return name

def get_link_pose(link: Link) -> list[float]:
    """
    Return
    ---------
    link_pose: [x, y, z, roll, pitch, yaw]
        Without @relative_to setting, link pose is expressed in parent element frame, 
        which is the model frame (world frame in Fusion360) by default
    """
    link_pose = math_op.matrix3d_2_pose(link.pose)
    return link_pose

def get_link_mass(link: Link) -> float:
    """
    Return
    ---------
    mass: float
        unit: kg
    """
    mass = link.phyPro.mass
    return mass

def get_link_CoM(link: Link) -> list[float]:
    """
    Return
    ---------
    CoM: [x, y, z, roll, pitch, yaw]
        Center of mass frame C relative to the link frame L
        unit: m, radian
    """
    # Let the orientation of center-of-mass frame C is same as link-frame L
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    w_CoM_x = link.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
    w_CoM_y = link.phyPro.centerOfMass.y
    w_CoM_z = link.phyPro.centerOfMass.z 
    w_Lo_x = link.pose.translation.x    # link-frame's origin point's x coordinat w.r.t world-frame
    w_Lo_y = link.pose.translation.y
    w_Lo_z = link.pose.translation.z
    # represent vector Lo-CoM which start from link-fram's origin point to CoM point w.r.t world-frame
    w_Lo_CoM = [[(w_CoM_x-w_Lo_x)*0.01], [(w_CoM_y-w_Lo_y)*0.01], [(w_CoM_z-w_Lo_z)*0.01]] # cm -> m
    # represent world-frame's orientation w.r.t link-frame
    L_R_w = [[link.pose.getCell(0, 0), link.pose.getCell(1, 0), link.pose.getCell(2, 0)],
                [link.pose.getCell(0, 1), link.pose.getCell(1, 1), link.pose.getCell(2, 1)],
                [link.pose.getCell(0, 2), link.pose.getCell(1, 2), link.pose.getCell(2, 2)]]

    L_Lo_CoM = math_op.change_orientation(L_R_w, w_Lo_CoM)
    CoM = [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]
    
    return CoM

def get_link_inertia(link: Link) -> list[float]:
    """
    Return
    ---------
    inertia_list: [ixx, iyy, izz, ixy, iyz, ixz]
        Inertia tensor elements w.r.t CoM frame, which has the same orientation with the link frame L
        unit: kg*m^2
    """
    (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = link.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
    # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
    x = link.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
    y = link.phyPro.centerOfMass.y * 0.01
    z = link.phyPro.centerOfMass.z * 0.01
    mass = link.phyPro.mass # unit: kg
    com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
    com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
    com_izz = w_izz*0.0001 - mass*(x**2+y**2)
    com_ixy = w_ixy*0.0001 + mass*(x*y)
    com_iyz = w_iyz*0.0001 + mass*(y*z)
    com_ixz = w_ixz*0.0001 + mass*(x*z)

    # Change inertia tensor to the orientation of link-frame L
    # reference: https://robot.sia.cn/CN/abstract/abstract374.shtml
    R = math_op.get_rotation_matrix(link.pose)
    R_T = math_op.matrix_transpose(R)
    inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                        [com_ixy, com_iyy, com_iyz],
                        [com_ixz, com_iyz, com_izz]]
    
    # moments of inertia has the same orientation of L-frame
    I = math_op.matrix_multi(math_op.matrix_multi(R_T, inertia_tensor), R)
    # I = math_op.matrix_multi(math_op.matrix_multi(R, inertia_tensor), R_T)
    inertia_list = [I[0][0], I[1][1], I[2][2], I[0][1], I[1][2], I[0][2]]

    return inertia_list # unit: kg*m^2

def get_link_visual_name(link: Link) -> str:
    """
    Return
    ---------
    visual_name: str
    """
    visual_name = link.get_name() + "_visual"
    return visual_name

def get_link_collision_name(link: Link) -> str:
    """
    Return:
    collision_name: str
    """
    collision_name = link.get_name() + "_collision"
    return collision_name

def get_link_mesh_origin(link: Link) -> list[float]:
    """
    Return:
    mesh_origin: [x, y, z, roll, pitch, yaw]
        unit: m, radian
    """
    collision_name = link.get_name() + "_collision"
    return collision_name

def get_link_visual_geo(link: Link) -> str:
    """
    A trimesh file for visual geometry
    Return:
    ---------
    mesh_loc: str
        the path to find mesh file
    """
    robot_name = constants.get_robot_name()
    visual_body = link.get_visual_body()
    col_body = link.get_collision_body()
    if (visual_body is None) and (col_body is None):
        # visual and collision geometry is same
        mesh_loc = "model://" + robot_name + "/meshes/" + link.get_name() + ".stl"
        return mesh_loc
    elif (visual_body is not None) and (col_body is not None):
        mesh_loc = "model://" + robot_name + "/meshes/" + link.get_name() + "_visual.stl"
        return mesh_loc
    elif (visual_body is None) and (col_body is not None):
        error_message = "Please set two bodies, one for visual and one for collision. \n"
        error_message = error_message + link.get_name() + " body for visual missing."
        utils.error_box(error_message)
        utils.terminate_box()
    elif (visual_body is not None) and (col_body is None):
        error_message = "Please set two bodies, one for visual and one for collision. \n"
        error_message = error_message + link.get_name() + " body for collision missing."
        utils.error_box(error_message)
        utils.terminate_box()

def get_link_collision_geo(link: Link) -> str:
    """
    A trimesh file for collision geometry
    Return:
    ---------
    mesh_loc: str
        the path to find mesh file
    """
    robot_name = constants.get_robot_name()
    visual_body = link.get_visual_body()
    col_body = link.get_collision_body()
    if (visual_body is None) and (col_body is None):
        # visual and collision geometry is same
        mesh_loc = "model://" + robot_name + "/meshes/" + link.get_name() + ".stl"
        return mesh_loc
    elif (visual_body is not None) and (col_body is not None):
        mesh_loc = "model://" + robot_name + "/meshes/" + link.get_name() + "_collision.stl"
        return mesh_loc
    elif (visual_body is None) and (col_body is not None):
        error_message = "Please set two bodies, one for visual and one for collision. \n"
        error_message = error_message + link.get_name() + " body for visual missing."
        utils.error_box(error_message)
        utils.terminate_box()
    elif (visual_body is not None) and (col_body is None):
        error_message = "Please set two bodies, one for visual and one for collision. \n"
        error_message = error_message + link.get_name() + " body for collision missing."
        utils.error_box(error_message)
        utils.terminate_box()

def get_link_element(link: Link) -> Element:
    """
    Return
    ---------
    link_ele: Element
        xml elements contains informations for link
    """
    link_ele = Element("link")
    link_ele.attrib = {"name": get_link_name(link)}

    # pose
    pose = SubElement(link_ele, "pose")
    pose.text = "{} {} {} {} {} {}".format(get_link_pose(link)[0], get_link_pose(link)[1],
                                           get_link_pose(link)[2], get_link_pose(link)[3],
                                           get_link_pose(link)[4], get_link_pose(link)[5])
    
    # inertial
    inertial = SubElement(link_ele, "inertial")
    mass = SubElement(inertial, "mass")
    mass.text = str(get_link_mass(link))
    CoM = SubElement(inertial, "pose")
    CoM.text = "{} {} {} {} {} {}".format(get_link_CoM(link)[0], get_link_CoM(link)[1],
                                          get_link_CoM(link)[2], get_link_CoM(link)[3],
                                          get_link_CoM(link)[4], get_link_CoM(link)[5])
    inertia = SubElement(inertial, "inertia")
    ixx = SubElement(inertia, "ixx")
    iyy = SubElement(inertia, "iyy")
    izz = SubElement(inertia, "izz")
    ixy = SubElement(inertia, "ixy")
    iyz = SubElement(inertia, "iyz")
    ixz = SubElement(inertia, "ixz")
    ixx.text = "{}".format(get_link_inertia(link)[0])
    iyy.text = "{}".format(get_link_inertia(link)[1])
    izz.text = "{}".format(get_link_inertia(link)[2])
    ixy.text = "{}".format(get_link_inertia(link)[3])
    iyz.text = "{}".format(get_link_inertia(link)[4])
    ixz.text = "{}".format(get_link_inertia(link)[5])

    # visual
    visual = SubElement(link_ele, "visual")
    visual_name = get_link_visual_name(link)
    visual_uri = get_link_visual_geo(link)
    visual.attrib = {"name": visual_name}
    vis_geo = SubElement(visual, "geometry")
    vis_geo_mesh = SubElement(vis_geo, "mesh")
    vis_geo_mesh_uri = SubElement(vis_geo_mesh, "uri")
    vis_geo_mesh_uri.text = visual_uri
    vis_geo_mesh_scale = SubElement(vis_geo_mesh, "scale")
    vis_geo_mesh_scale.text = "0.001 0.001 0.001"

    # collision
    collision = SubElement(link_ele, "collision")
    collision_name = get_link_collision_name(link)
    collision_uri = get_link_collision_geo(link)
    collision.attrib = {"name": collision_name}
    col_geo = SubElement(collision, "geometry")
    col_geo_mesh = SubElement(col_geo, "mesh")
    col_geo_mesh_uri = SubElement(col_geo_mesh, "uri")
    col_geo_mesh_uri.text = collision_uri
    col_geo_mesh_scale = SubElement(col_geo_mesh, "scale")
    col_geo_mesh_scale.text = "0.001 0.001 0.001"

    return link_ele

def get_joint_name(joint: Joint) -> str:
    """
    Return
    ---------
    joint_name: str
    """
    joint_name = joint.get_name()
    return joint_name

def get_joint_type(joint: Joint) -> str:
    """
    Return
    ---------
    joint_type: str
        fixed, revolute, prismatic, continuous
        currently, only support urdf joint types above
    """
    # currently, only support these three sdf joint type
    sdf_joint_type_list = ["fixed", "revolute", "prismatic"]
    if joint.joint.jointMotion.jointType <= 2:
        sdf_joint_type = sdf_joint_type_list[joint.joint.jointMotion.jointType]
        # # TODO:It seems continuous joint type has some problem with gazebo
        # if sdf_joint_type == "revolute" and (self.get_limits() is None):
        #     sdf_joint_type = "continuous"
    else:
        pass
    return sdf_joint_type


def get_joint_parent(joint: Joint) -> Link:
    """
    Return
    ---------
    parent_link: Link
        parent link of the joint
    """
    joint_parent: adsk.fusion.Occurrence = joint.parent
    parent_link: Link = Link(joint_parent)
    return parent_link

def get_joint_child(joint: Joint) -> Link:
    """
    Return
    ---------
    child_link: Link
        child link of the joint
    """
    joint_child: adsk.fusion.Occurrence = joint.child
    child_link: Link = Link(joint_child)
    return child_link

def get_joint_axis(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_axis: [x, y, z]
        joint axis w.r.t world frame
        if joint type is fixed, return None
    """
    # Fusion360 api returns joint axis w.r.t world frame
    if joint.joint.jointMotion.jointType == 0: # RigidJointType
        axis = None
        return axis
    elif joint.joint.jointMotion.jointType == 1: # RevoluteJointType
        axis = [round(i, 6) for i in joint.joint.jointMotion.rotationAxisVector.asArray()] # In Fusion360, returned axis is normalized
        return axis
    elif joint.joint.jointMotion.jointType == 2: # SliderJointType
        axis = [round(i, 6) for i in joint.joint.jointMotion.slideDirectionVector.asArray()] # In Fusion360, returned axis is normalized
        return axis

def get_joint_axis2(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_axis:2 [x, y, z]
    """
    pass

def get_joint_limit(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_limit: [lower, upper]
    """
    if joint.joint.jointMotion.jointType == 0: # RigidJointType
        return None
    elif joint.joint.jointMotion.jointType == 1: # RevoluteJointType
        max_enabled = joint.joint.jointMotion.rotationLimits.isMaximumValueEnabled
        min_enabled = joint.joint.jointMotion.rotationLimits.isMinimumValueEnabled
        if max_enabled and min_enabled:
            # unit: radians
            lower_limit = joint.joint.jointMotion.rotationLimits.minimumValue 
            upper_limit = joint.joint.jointMotion.rotationLimits.maximumValue
            return [round(lower_limit, 6), round(upper_limit, 6)]
        else:
            return None
    elif joint.joint.jointMotion.jointType == 2: # SliderJointType
        max_enabled = joint.joint.jointMotion.slideLimits.isMaximumValueEnabled
        min_enabled = joint.joint.jointMotion.slideLimits.isMinimumValueEnabled
        if max_enabled and min_enabled:
            lower_limit = joint.joint.jointMotion.slideLimits.minimumValue * 0.01 # cm -> m
            upper_limit = joint.joint.jointMotion.slideLimits.maximumValue * 0.01 # cm -> m
            return [round(lower_limit, 6), round(upper_limit, 6)]
        else:
            return None

def get_joint_limit2(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_limit2: [lower, upper]
    """
    pass

def get_joint_element(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_element: Element
        xml elements contains informations for joint
    """
    joint_ele = Element("joint")
    joint_ele.attrib = {"name": get_joint_name(joint), "type": get_joint_type(joint)}
    pose = SubElement(joint_ele, "pose")
    poseValues = joint.get_sdf_origin()
    pose.text = "{} {} {} {} {} {}".format(poseValues[0], poseValues[1],
                                           poseValues[2], poseValues[3],
                                           poseValues[4], poseValues[5])
    parent = SubElement(joint_ele, "parent")
    parent.text = "{}".format(get_joint_parent(joint).get_name())
    child = SubElement(joint_ele, "child")
    child.text = "{}".format(get_joint_child(joint).get_name())
    axis1 = get_joint_axis(joint)
    axis2 = get_joint_axis2(joint)
    if axis1 is not None:
        axis1_ele = SubElement(joint_ele, "axis")
        axis1_xyz = SubElement(axis1_ele, "xyz")
        # Fusion360 api returns joint axis w.r.t world frame,
        # which is the model frame here
        axis1_xyz.attrib = {"expressed_in": "__model__"}
        axis1_xyz.text = "{} {} {}".format(axis1[0], axis1[1], axis1[2])
        joint_limit1 = get_joint_limit(joint)
        if joint_limit1 is not None:
            limit = SubElement(axis1_ele, "limit")
            lower = SubElement(limit, "lower")
            lower.text = str(joint_limit1[0])
            upper = SubElement(limit, "upper")
            upper.text = str(joint_limit1[1])
    if axis2 is not None:
        pass

    return joint_ele
