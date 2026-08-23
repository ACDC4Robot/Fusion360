# -*- coding: utf-8 -*-
"""
Get informations about joints from Fusion360 API

"""

from typing import Union
import adsk, adsk.fusion, adsk.core
from . import utils
from . import math_operation as math_op

class Joint():
    """
    Joint class for joint
    """

    JOINT_TYPE_NAMES = {
        0: "Rigid",
        1: "Revolute",
        2: "Slider",
        3: "Cylindrical",
        4: "Pin-Slot",
        5: "Planar",
        6: "Ball",
    }
    SUPPORTED_JOINT_TYPES = {0, 1, 2}

    def __init__(self, joint: Union[adsk.fusion.Joint, adsk.fusion.AsBuiltJoint]) -> None:
        self.joint = joint
        self.name = joint.name
        self.parent = None
        self.child = None
        try:
            self.parent = joint.occurrenceTwo # parent link of joint
            self.child = joint.occurrenceOne
        except Exception as e:
            utils.log(f"Invalid joint '{joint.name}': {str(e)}")
        # self.parent = Link(joint.occurrenceTwo) # parent link of joint
        # self.child = Link(joint.occurrenceOne)

    def is_valid(self) -> bool:
        """Return whether both joint endpoints are component occurrences."""
        return self.parent is not None and self.child is not None

    def get_joint_type_name(self) -> str:
        """Return a readable Fusion joint type for diagnostics."""
        try:
            joint_type = self.joint.jointMotion.jointType
        except Exception:
            return "Unknown"
        return self.JOINT_TYPE_NAMES.get(joint_type, f"Unknown({joint_type})")

    def _unsupported_joint_type_error(self) -> ValueError:
        return ValueError(
            f"Joint '{self.name}' uses unsupported Fusion type "
            f"'{self.get_joint_type_name()}'. Only Rigid, Revolute, and Slider "
            "joints can be exported. Recreate it with a supported type or model "
            "the additional degrees of freedom using multiple supported joints."
        )

    def _origin_geometry(self):
        """Return the geometry that defines the joint frame, or ``None``.

        Fusion exposes an ``AsBuiltJoint.geometry`` directly, while a regular
        joint can expose either ``JointGeometry`` or a ``JointOrigin`` wrapper.
        Comparing an instance to ``adsk.fusion.JointOrigin`` was always false;
        use ``isinstance`` and unwrap the wrapper instead.
        """
        try:
            if hasattr(self.joint, "geometry"):
                return self.joint.geometry
            geometry_or_origin = self.joint.geometryOrOriginTwo
            if isinstance(geometry_or_origin, adsk.fusion.JointOrigin):
                return geometry_or_origin.geometry
            return geometry_or_origin
        except (AttributeError, RuntimeError) as exc:
            utils.log(
                f"Joint '{self.name}': unable to read its origin geometry "
                f"({str(exc)})."
            )
            return None

    def require_endpoints(self):
        if self.is_valid():
            return
        raise ValueError(
            f"Joint '{self.name}' is not between two component occurrences. "
            "Move grounded/root bodies into a component such as 'base_link' "
            "and create the joint between component occurrences."
        )

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
        if self.child is not None:
            child_name = utils.get_valid_filename(self.child.fullPathName)
            return f"{child_name}_{self.get_name()}"
        return self.get_name()

    def get_parent(self):
        """
        Return name of parent link
        """
        self.require_endpoints()
        if self.parent.component.name == "base_link":
            parent_name = "base_link"
        else:
            parent_name = utils.get_valid_filename(self.parent.fullPathName)

        return parent_name

    def get_child(self):
        """
        Return name of child link
        """
        self.require_endpoints()
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
        return self._origin_geometry() is not None

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
        if self.joint.jointMotion.jointType in self.SUPPORTED_JOINT_TYPES:
            sdf_joint_type = sdf_joint_type_list[self.joint.jointMotion.jointType]
            # # TODO:It seems continuous joint type has some problem with gazebo
            # if sdf_joint_type == "revolute" and (self.get_limits() is None):
            #     sdf_joint_type = "continuous"
        else:
            raise self._unsupported_joint_type_error()
        return sdf_joint_type
    
    def get_mjcf_joint_type(self) -> str:
        """
        Return joint type for mjcf
        Return
        ---------
        joint_type: str
        """
        mjcf_joint_type_list = [None, "hinge", "slide"]
        if self.joint.jointMotion.jointType in self.SUPPORTED_JOINT_TYPES:
            mjcf_joint_type = mjcf_joint_type_list[self.joint.jointMotion.jointType]
        else:
            raise self._unsupported_joint_type_error()
        return mjcf_joint_type
    
    def get_urdf_joint_type(self) -> str:
        urdf_joint_type_list = ["fixed", "revolute", "prismatic"]
        if self.joint.jointMotion.jointType in self.SUPPORTED_JOINT_TYPES:
            urdf_joint_type = urdf_joint_type_list[self.joint.jointMotion.jointType]
            if urdf_joint_type == "revolute" and (self.get_limits() is None):
                urdf_joint_type = "continuous"
        else:
            raise self._unsupported_joint_type_error()
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
        self.require_endpoints()
        geometry = self._origin_geometry()
        if geometry is None:
            raise ValueError(
                f"Joint '{self.name}' has no usable origin geometry. Recreate "
                "the joint or define a valid joint origin before export."
            )
        w_P_Jc = geometry.origin.asArray()

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
        geometry = self._origin_geometry()
        if geometry is None:
            return None
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
        self.require_endpoints()
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
            parent_frame = Joint(parent_joint).get_joint_frame()
            if parent_frame is None:
                parent_frame = parent_link.transform2
            
        
        # get joint frame w.r.t world frame
        joint_frame = self.get_joint_frame()
        if joint_frame is None:
            if self.joint.jointMotion.jointType == 0:
                # Rigid as-built joints can legitimately have no explicit
                # origin. Their fixed transform is fully determined by the
                # two occurrence frames.
                transform = math_op.coordinate_transform(
                    parent_link.transform2, self.child.transform2
                )
                return math_op.matrix3d_2_pose(transform)
            raise ValueError(
                f"Joint '{self.name}' has no usable origin geometry. Recreate "
                "the joint or define a valid joint origin before export."
            )

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
            return axis1, axis2
        else: # BallJointType and unknown future joint types have no single axis
            return None, None
        
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
        if joint_frame is None:
            return w_axis1, w_axis2
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
        self.require_endpoints()
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
