# -*- coding: utf-8 -*-
"""
Get informations about joints from Fusion360 API

"""

from typing import Union
import adsk, adsk.fusion, adsk.core
from xml.etree.ElementTree import ElementTree, Element, SubElement
from ..commands.ACDC4Robot import constants
from . import utils
from . import math_operation as math_op

class Joint():
    """
    Joint class for joint
    """

    def __init__(self, joint: Union[adsk.fusion.Joint, adsk.fusion.AsBuiltJoint]) -> None:
        self.joint = joint
        self.name = joint.name
        try:
            self.parent = joint.occurrenceTwo # parent link of joint
            self.child = joint.occurrenceOne
        except Exception as e:
            textPalette: adsk.core.Palette = constants.get_text_palette()
            textPalette.writeText(f"Invalid joint: {joint.name}: {str(e)}")
            pass
        # self.parent = Link(joint.occurrenceTwo) # parent link of joint
        # self.child = Link(joint.occurrenceOne)

    def get_name(self):
        """
        Return:
        name: str
            joint's full path name
        """
        # joint names inside each occurrence are identical
        # but joint names in different occurrences can be the same, in which makes conflict
        # parent_name = utils.get_valid_filename(self.parent.fullPathName) # inorder to fix joint's name confliction
        # name = parent_name + "_" + utils.get_valid_filename(self.name)
        # output name is consistent with the name in Fusion
        name = utils.get_valid_filename(self.name)
        return name

    def get_full_path_name(self):
        """
        Return:
        path: str
            The joint name and parent component path
        """
        # joint names inside each occurrence are identical
        # but joint names in different occurrences can be the same, in which makes conflict
        if self.child:
            child_name = utils.get_valid_filename(self.child.fullPathName)
            return f"{child_name}_{self.get_name()}"
        return self.get_name()

    def get_parent(self):
        """
        Return name of parent link
        """
        if self.parent.component.name == "base_link":
            parent_name = "base_link"
        else:
            parent_name = utils.get_valid_filename(self.parent.fullPathName)

        return parent_name

    def get_child(self):
        """
        Return name of child link
        """
        if self.child.component.name == "base_link":
            child_name = "base_link"
        else:
            child_name = utils.get_valid_filename(self.child.fullPathName)

        return child_name

    def has_origin(self) -> bool:
        """
        Checks if the join has a parent origin geometry
        Return: bool
        """
        if hasattr(self.joint, 'geometry'):
            return self.joint.geometry is not None
        elif self.joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
            return self.joint.geometryOrOriginTwo.geometry is not None
        else:
            return self.joint.geometryOrOriginTwo.origin is not None

    def get_sdf_joint_type(self) -> str:
        """
        Currently support following joint type:
            fixed, revolute, prismatic, continuous
        
        Return:
        joint_type: str
            fixed, revolute, prismatic
        """
        # currently, only support these three sdf joint type
        sdf_joint_type_list = ["fixed", "revolute", "prismatic"]
        if self.joint.jointMotion.jointType <= 2:
            sdf_joint_type = sdf_joint_type_list[self.joint.jointMotion.jointType]
            # # TODO:It seems continuous joint type has some problem with gazebo
            # if sdf_joint_type == "revolute" and (self.get_limits() is None):
            #     sdf_joint_type = "continuous"
        else:
            pass
        return sdf_joint_type
    
    def get_mjcf_joint_type(self) -> str:
        """
        Return joint type for mjcf
        Return
        ---------
        joint_type: str
        """
        mjcf_joint_type_list = [None, "hinge", "slide"]
        if self.joint.jointMotion.jointType <= 2:
            mjcf_joint_type = mjcf_joint_type_list[self.joint.jointMotion.jointType]
        else:
            pass
        return mjcf_joint_type
    
    def get_urdf_joint_type(self) -> str:
        urdf_joint_type_list = ["fixed", "revolute", "prismatic"]
        if self.joint.jointMotion.jointType <= 2:
            urdf_joint_type = urdf_joint_type_list[self.joint.jointMotion.jointType]
            if urdf_joint_type == "revolute" and (self.get_limits() is None):
                urdf_joint_type = "continuous"
        else:
            # other joint types are not supported yet
            pass
        return urdf_joint_type

    def get_limits(self):
        """
        Return joint limits list: [lower_limit, upper_limit]
        or None
        """
        if self.joint.jointMotion.jointType == 0: # RigidJointType
            return None
        elif self.joint.jointMotion.jointType == 1: # RevoluteJointType
            max_enabled = self.joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = self.joint.jointMotion.rotationLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                # unit: radians
                upper_limit = self.joint.jointMotion.rotationLimits.maximumValue 
                lower_limit = self.joint.jointMotion.rotationLimits.minimumValue
                return [round(lower_limit, 6), round(upper_limit, 6)]
            else:
                return None
        elif self.joint.jointMotion.jointType == 2: # SliderJointType
            max_enabled = self.joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = self.joint.jointMotion.slideLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                upper_limit = self.joint.jointMotion.slideLimits.maximumValue * 0.01 # cm -> m
                lower_limit = self.joint.jointMotion.slideLimits.minimumValue * 0.01 # cm -> m
                return [round(lower_limit, 6), round(upper_limit, 6)]
            else:
                return None

    def get_sdf_origin(self):
        """
        From: http://sdformat.org/tutorials?tut=spec_model_kinematics&cat=specification&#jointpose
        For a joint with parent link frame `P` and child link frame `C`, 
        the joint `<pose>` tag specifies the pose `X_CJc` of a joint frame `Jc` rigidly attached to the child link. 
        """
        # I think the joint frame is defined by the parent joint origin in Fusion360
        # Because in Fusion360, a joint motion is defined by the axis in parent joint orign
        # and use offset to define the location of child joint origin w.r.t parent joint
        # I guess using parent joint origin would solve the offset problem?
        # I guess using child joint origin works just because they coincide together for all the text examples

        # get parent joint origin as the child joint
        if hasattr(self.joint, 'geometry'):
            w_P_Jc = self.joint.geometry.origin.asArray()
        elif self.joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
            w_P_Jc = self.joint.geometryOrOriginTwo.geometry.origin.asArray()
        else:
            w_P_Jc = self.joint.geometryOrOriginTwo.origin.asArray()

        # convert from cm to m
        w_P_Jc = [round(i*0.01, 6) for i in w_P_Jc] 
        # get child link frame's origin point w.r.t world frame
        w_P_Lc = [self.child.transform2.translation.x * 0.01, 
                  self.child.transform2.translation.y * 0.01,
                  self.child.transform2.translation.z * 0.01,]
        # vector from child link frame's origin point to child joint origin point w.r.t world frame
        w_V_LcJc = [[w_P_Jc[0]-w_P_Lc[0]],
                    [w_P_Jc[1]-w_P_Lc[1]],
                    [w_P_Jc[2]-w_P_Lc[2]]] # 3*1 vector

        w_T_Lc = self.child.transform2 # configuration of child-link-frame w.r.t world-frame w
        w_R_Lc = math_op.get_rotation_matrix(w_T_Lc) # rotation matrix of child-link-frame w.r.t world-frame w
        Lc_R_w = math_op.matrix_transpose(w_R_Lc) # rotation matrix of world-frame w.r.t child-link-frame Lc
        # vector from child link frame's origin point to child joint origin point w.r.t child-link-frame Lc
        Lc_V_LcJc = math_op.change_orientation(Lc_R_w, w_V_LcJc) # 3*1 array
        # assume the joint frame has the same oritation as child link frame
        # it seems that rpy of joint doesn't matter, so set them as 0
        sdf_origin = [Lc_V_LcJc[0][0], Lc_V_LcJc[1][0], Lc_V_LcJc[2][0], 0.0, 0.0, 0.0]

        return sdf_origin
    
    def get_joint_frame(self) -> adsk.core.Matrix3D:
        """
        Get the joint frame whose origin coincides with parent joint origin,
        and has same orientation with parent joint origin frame

        Return:
        joint_frame: adsk.core.Matrix3D
            a homogeneous matrix represents joint frame J in world frame W
            translation unit: cm
        """
        # get parent joint origin's coordinate w.r.t world frame
        if hasattr(self.joint, 'geometry'):
            geometry: adsk.fusion.JointGeometry = self.joint.geometry
            if geometry is None:
                return None
            w_P_J = geometry.origin.asArray()
        else:
            geometry: adsk.fusion.JointOrigin = self.joint.geometryOrOriginTwo
            if self.joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
                w_P_J = geometry.geometry.origin.asArray()
            else:
                w_P_J = geometry.origin.asArray()

        w_P_J = [round(i, 6) for i in w_P_J]
        
        # no matter jointGeometry or jointOrigin object, both have these properties
        zAxis: adsk.core.Vector3D = geometry.primaryAxisVector
        xAxis: adsk.core.Vector3D = geometry.secondaryAxisVector
        yAxis: adsk.core.Vector3D = geometry.thirdAxisVector

        origin = adsk.core.Point3D.create(w_P_J[0], w_P_J[1], w_P_J[2])

        joint_frame = adsk.core.Matrix3D.create()
        joint_frame.setWithCoordinateSystem(origin, xAxis, yAxis, zAxis)

        return joint_frame
    
    def get_urdf_origin(self):
        """
        Get joint origin, which is the transform from the parent frame to this joint
        """
        parent_link = self.parent

        # get parent_frame w.r.t world frame
        def get_parent_joint(link: adsk.fusion.Occurrence) -> adsk.fusion.Joint:
            joint_list: adsk.fusion.JointList = link.joints
            for j in joint_list:
                if j.occurrenceOne == link:
                    return j
                else:
                    continue
            return None
        
        parent_joint: adsk.fusion.Joint = get_parent_joint(parent_link)
        if parent_joint is None:
            # if the parent link does not have parent joint(which means the root link), 
            # then the parent link frame is the parent frame
            parent_frame: adsk.core.Matrix3D = parent_link.transform2
        else:
            if parent_joint.geometryOrOriginTwo == adsk.fusion.JointOrigin:
                w_P_J = parent_joint.geometryOrOriginTwo.geometry.origin.asArray()
            else:
                w_P_J = parent_joint.geometryOrOriginTwo.origin.asArray()

            w_P_J = [round(i, 6) for i in w_P_J]
            
            # no matter jointGeometry or jointOrigin object, both have these properties
            zAxis: adsk.core.Vector3D = parent_joint.geometryOrOriginTwo.primaryAxisVector
            xAxis: adsk.core.Vector3D = parent_joint.geometryOrOriginTwo.secondaryAxisVector
            yAxis: adsk.core.Vector3D = parent_joint.geometryOrOriginTwo.thirdAxisVector

            origin = adsk.core.Point3D.create(w_P_J[0], w_P_J[1], w_P_J[2])

            parent_frame = adsk.core.Matrix3D.create()
            parent_frame.setWithCoordinateSystem(origin, xAxis, yAxis, zAxis)
            
        
        # get joint frame w.r.t world frame
        joint_frame = self.get_joint_frame()

        # from_origin, from_xAxis, from_yAxis, from_zAxis = parent_frame.getAsCoordinateSystem()
        # to_origin, to_xAsix, to_yAxis, to_zAxis = joint_frame.getAsCoordinateSystem()

        transform = adsk.core.Matrix3D.create()
        # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis, 
        #                                         to_origin, to_xAsix, to_yAxis, to_zAxis)
        transform = math_op.coordinate_transform(parent_frame, joint_frame)

        joint_origin = math_op.matrix3d_2_pose(transform)

        return joint_origin

    def get_axes(self):
        """
        Return:
        ---------
        axis1: list or None
            list: [x1, y1, z1]
        axis2: list or None
            list: [x2, y2, z2]
        """
        # Fusion360 api returns joint axis w.r.t world frame
        if self.joint.jointMotion.jointType == 0: # RigidJointType
            axis1 = None
            axis2 = None
            return axis1, axis2
        elif self.joint.jointMotion.jointType == 1: # RevoluteJointType
            axis1 = [round(i, 6) for i in self.joint.jointMotion.rotationAxisVector.asArray()] # In Fusion360, returned axis is normalized
            axis2 = None
            return axis1, axis2
        elif self.joint.jointMotion.jointType == 2: # SliderJointType
            axis1 = [round(i, 6) for i in self.joint.jointMotion.slideDirectionVector.asArray()] # In Fusion360, returned axis is normalized
            axis2 = None
            return axis1, axis2
        elif self.joint.jointMotion.jointType == 3: # CylindricalJointType
            axis1 = [round(i, 6) for i in self.joint.jointMotion.rotationAxisVector.asArray()] # In Fusion360, returned axis is normalized
            axis2 = None
            return axis1, axis2
        elif self.joint.jointMotion.jointType == 4: # PinSlotJointType
            axis1 = [round(i, 6) for i in self.joint.jointMotion.rotationAxisVector.asArray()] # rotation axis
            axis2 = [round(i, 6) for i in self.joint.jointMotion.slideDirectionVector.asArray()] # slide axis
            return axis1, axis2
        elif self.joint.jointMotion.jointType == 5: # PlanarJointType
            axis1 = [round(i, 6) for i in self.joint.jointMotion.primarySlideDirectionVector.asArray()] # rotation axis
            axis2 = [round(i, 6) for i in self.joint.jointMotion.secondarySlideDirectionVector.asArray()] # slide axis
        elif self.joint.jointMotion.jointType == 6: # BallJointType
            pass
        
    def get_axes_urdf(self):
        """
        Return
        ---------
        axis1: list or None
            list: [x1, y1, z1], w.r.t joint-frame J
        axis2: list or None
            list: [x2, y2, z2], w.r.t joint-frame J
        """
        # both axes are expressed w.r.t world-frame w
        w_axis1, w_axis2 = self.get_axes()
        J_axis1, J_axis2 = None, None
        joint_frame: adsk.core.Matrix3D = self.get_joint_frame()
        w_R_J = math_op.get_rotation_matrix(joint_frame) # represent joint-frame J's orientation w.r.t world-frame w
        J_R_w = math_op.matrix_transpose(w_R_J)
        if w_axis1 is not None:
            w_axis1 = [[w_axis1[0]], [w_axis1[1]], [w_axis1[2]]] # from 1*3 to 3*1 list
            J_axis1 = math_op.change_orientation(J_R_w, w_axis1)
            J_axis1 = [J_axis1[0][0], J_axis1[1][0], J_axis1[2][0]]
        if w_axis2 is not None:
            w_axis2 = [[w_axis2[0]], [w_axis2[1]], [w_axis2[2]]]
            J_axis2 = math_op.change_orientation(J_R_w, w_axis2)
            J_axis2 = [J_axis2[0][0], J_axis2[1][0], J_axis2[2][0]]

        return J_axis1, J_axis2
    
    def get_axis_mjcf(self):
        """
        I guess the reference frame of the axis is the body contains the joint element,
        which is the child link of the joint

        Return:
        axis: list or None
        """
        w_axis1, _ = self.get_axes()
        C_axis = None
        child_link_frame: adsk.core.Matrix3D = self.child.transform2
        w_R_C = math_op.get_rotation_matrix(child_link_frame) 
        C_R_w = math_op.matrix_transpose(w_R_C)
        if w_axis1 is not None:
            w_axis1 = [[w_axis1[0]], [w_axis1[1]], [w_axis1[2]]] # from 1*3 to 3*1 list
            C_axis = math_op.change_orientation(C_R_w, w_axis1)
            C_axis = [C_axis[0][0], C_axis[1][0], C_axis[2][0]]
        
        return C_axis
