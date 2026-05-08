# -*- coding: utf-8 -*-
"""
Get informations about links which are components from Fusion360 API

"""
import adsk, adsk.fusion, adsk.core
from xml.etree.ElementTree import ElementTree, Element, SubElement
from ..commands.ACDC4Robot import constants
from . import utils
from .joint import Joint
from . import math_operation as math_op

# TODO: support for dependent component
# TODO: support for external component

class Link():
    """
    Data abstruction for link element
    with some methods to access commonly used information

    Attributes:
        link: adsk.fusion.Occurrence
            occurrence in Fusion360 refers to linkage
        dir: str
            location of the exported files
        name: str
            identified name of the link
        pose: Matrix3D
            get by adsk.fusion.Occurrence.transform2
            Gets the 3d matrix data that defines this occurrences orientation and position in its assembly context
            the assembly context here is the root component's frame
        phyPro: adsk.fusion.PhysicalProperties
            object for accessing link's physical properties
    """

    dir = "" # location of the exported file
    def __init__(self, occurrence: adsk.fusion.Occurrence) -> None:
        """
        Initialize attributes for an object
        """
        # TODO: maybe this is a better and accurate way for this, not test
        # self.link = occurrence.createForAssemblyContext(occurrence)

        self.link: adsk.fusion.Occurrence = occurrence
        self.name = None
        # pose can have two meanings:
        # 1. the homogeneous matrix of link-frame L w.r.t world-frame w
        # 2. the coordinates of link-frame L
        self.pose: adsk.core.Matrix3D = occurrence.transform2
        self.phyPro = occurrence.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)

    def get_link_occ(self) -> adsk.fusion.Occurrence:
        """
        Return:
        occ: adsk.fusion.Occurrence
            occurrence the link instance referenced
        """
        return self.link

    def is_visible(self) -> bool:
        """
        Check if the occurrence has the light bulb on
        Return:
        bool
        """
        return self.link.isLightBulbOn

    def get_parent_joint(self) -> adsk.fusion.Joint:
        """
        Get the parent joint of a robot link
        Every robot link can only have one parent joint,
        but can have multiple child joints to connect child links

        Return:
            j: parent_joints
        """
        # TODO: in closed loop mechanism, there exits one assembly method that
        # make one link have two parent joint, write a instruction or fix this bug
        # text_palette = constants.get_text_palette()
        app = adsk.core.Application.get()
        root = adsk.fusion.Design.cast(app.activeProduct).rootComponent
        for joint in root.allJoints:
            if joint.occurrenceOne == self.link:
                return joint
        for join in root.allAsBuiltJoints:
            if join.occurrenceOne == self.link:
                return join
        return None

    def get_name(self) -> str:
        """
        Get the link valid name for link fullPathName
        Return:
        ---------
        name: str
        """
        if self.link.component.name == "base_link":
            name = "base_link"
        else:
            name = utils.get_valid_filename(self.link.fullPathName)

        return name

    def get_pose_sdf(self) -> list:
        """
        Return a pose(translation, rotation) expressed in the frame named by @relative_to.
        The first three components (x, y, z) represent the position of the element's origin(in the @relative_to frame)
        The rotation component represents the orientation of the element as either a sequence of Euler rotations(r, p, y)

        Return
        ---------
        pose: list
            [x, y, z, roll, pitch, yaw]
        """
        pose = math_op.matrix3d_2_pose(self.pose)
        return pose

    def get_inertia_sdf(self) -> list:
        """
        Return link's moments of inertia about Co(the link's center of mass)
        for the unit vectors Cx, Cy, Cz fixed in the center-of-mass-frame C.
        Note: the orientation of Cx, Cy, Cz relative to Lx, Ly, Lz which is specified
        by the `pose` tag.
        We set center-of-mass frame's orientation to be the same as link-frame L

        Return:
        ---------
        list: [ixx, iyy, izz, ixy, iyz, ixz]
            a inertia tensor w.r.t CoM frame which has the same orientation with parent frame
        """
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
        # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
        x = self.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
        y = self.phyPro.centerOfMass.y * 0.01
        z = self.phyPro.centerOfMass.z * 0.01
        mass = self.phyPro.mass # unit: kg
        com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
        com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
        com_izz = w_izz*0.0001 - mass*(x**2+y**2)
        com_ixy = w_ixy*0.0001 + mass*(x*y)
        com_iyz = w_iyz*0.0001 + mass*(y*z)
        com_ixz = w_ixz*0.0001 + mass*(x*z)

        # Change inertia tensor to the orientation of link-frame L
        # reference: https://robot.sia.cn/CN/abstract/abstract374.shtml
        R = math_op.get_rotation_matrix(self.pose)
        R_T = math_op.matrix_transpose(R)
        inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                          [com_ixy, com_iyy, com_iyz],
                          [com_ixz, com_iyz, com_izz]]

        # moments of inertia has the same orientation of L-frame
        I = math_op.matrix_multi(math_op.matrix_multi(R_T, inertia_tensor), R)
        # I = math_op.matrix_multi(math_op.matrix_multi(R, inertia_tensor), R_T)

        ixx = I[0][0]
        iyy = I[1][1]
        izz = I[2][2]
        ixy = I[0][1]
        iyz = I[1][2]
        ixz = I[0][2]

        return [ixx, iyy, izz, ixy, iyz, ixz] # unit: kg*m^2

    def get_initia_urdf(self):
        """
        Return [ixx, iyy, izz, ixy, iyz, ixz]
        the inertia tensor elements of the link w.r.t the CoM frame C
        CoM frame C has the same orientation with the parent frame

        Return:
        ---------
        list: [ixx, iyy, izz, ixy, iyz, ixz]
            a inertia tensor w.r.t CoM frame which has the same orientation with parent frame
        """
        # First, get the inertia tensor and CoM w.r.t the world frame
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
        x = self.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
        y = self.phyPro.centerOfMass.y * 0.01
        z = self.phyPro.centerOfMass.z * 0.01

        # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
        mass = self.phyPro.mass # unit: kg
        com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
        com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
        com_izz = w_izz*0.0001 - mass*(x**2+y**2)
        com_ixy = w_ixy*0.0001 + mass*(x*y)
        com_iyz = w_iyz*0.0001 + mass*(y*z)
        com_ixz = w_ixz*0.0001 + mass*(x*z)
        inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                          [com_ixy, com_iyy, com_iyz],
                          [com_ixz, com_iyz, com_izz]]

        # get the parent frame
        parent_joint = self.get_parent_joint()
        # parent frame for a link has parent joint, is the parent joint frame
        # else, is the link frame itselt
        if parent_joint is None:
            parent_frame: adsk.core.Matrix3D = self.pose
        else:
            joint = Joint(parent_joint)
            parent_frame: adsk.core.Matrix3D = joint.get_joint_frame()

        # Change inertia tensor to the orientation of parent frame


        # # Get eigenvectors, which are the principal axes of the inertia tensor
        # _, eigenvectors = math_op.eigenvalues_and_eigenvectors(inertia_tensor)

        # # Get the rotation matrix represents the orientation of the principal axes
        # R = math_op.matrix_transpose(eigenvectors)
        # R_T = eigenvectors

        R = math_op.get_rotation_matrix(parent_frame)
        R_T = math_op.matrix_transpose(R)

        # Express inertia tersor w.r.t CoM frame C with axes Cx, Cy, Cz aligned with principal inertia directions
        I = math_op.matrix_multi(math_op.matrix_multi(R_T, inertia_tensor), R)
        ixx = I[0][0]
        iyy = I[1][1]
        izz = I[2][2]
        ixy = I[0][1]
        iyz = I[1][2]
        ixz = I[0][2]

        return [ixx, iyy, izz, ixy, iyz, ixz] # unit: kg*m^2

    def get_inertia_mjcf(self) -> list:
        """
        Get inertia tensor as mjcf's body subelement
        6 numbers in the following order: M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3).
        which is ixx, iyy, izz, ixy, ixz, iyz

        Return
        ---------
        inertia: list
            [ixx, iyy, izz, ixy, ixz, iyz]
        """
        inertia = self.get_inertia_sdf()
        inertia = [inertia[0], inertia[1], inertia[2], inertia[3], inertia[5], inertia[4]]
        return inertia

    def get_mass(self) -> float:
        """
        Get mass of the link, which is same in urdf, sdf, mjcf

        Return:
        ---------
        mass: float
        """
        mass = self.phyPro.mass # kg
        return mass

    def get_CoM_wrt_link(self):
        """
        Get CoM frame w.r.t link frame in [x, y, z roll, pitch, yaw] representation

        Return:
        pose_CoM: [x, y, z, roll, pitch, yaw]
            Center-of-mass frame C w.r.t to the link-frame L which contains the CoM
        """
        # Let the orientation of center-of-mass frame C is same as link-frame L
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        w_CoM_x = self.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
        w_CoM_y = self.phyPro.centerOfMass.y
        w_CoM_z = self.phyPro.centerOfMass.z
        w_Lo_x = self.pose.translation.x    # link-frame's origin point's x coordinat w.r.t world-frame
        w_Lo_y = self.pose.translation.y
        w_Lo_z = self.pose.translation.z
        # represent vector Lo-CoM which start from link-fram's origin point to CoM point w.r.t world-frame
        w_Lo_CoM = [[(w_CoM_x-w_Lo_x)*0.01], [(w_CoM_y-w_Lo_y)*0.01], [(w_CoM_z-w_Lo_z)*0.01]] # cm -> m
        # represent world-frame's orientation w.r.t link-frame
        L_R_w = [[self.pose.getCell(0, 0), self.pose.getCell(1, 0), self.pose.getCell(2, 0)],
                 [self.pose.getCell(0, 1), self.pose.getCell(1, 1), self.pose.getCell(2, 1)],
                 [self.pose.getCell(0, 2), self.pose.getCell(1, 2), self.pose.getCell(2, 2)]]

        L_Lo_CoM = math_op.change_orientation(L_R_w, w_Lo_CoM)
        pose_CoM = [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]

        return pose_CoM

    def get_CoM_sdf(self):
        """
        Get CoM frame w.r.t link frame in [x, y, z, roll, pitch, yaw] representation

        Return:
        pose_CoM: [x, y, z, roll, pitch, yaw]
            Center-of-mass frame C relative to the link-frame L
        """
        # TODO: to avoid compatibility issues associated with the negative sign convention for product
        # of inertia, align CoM frame C's axes to the principal inertia directions, so that all the
        # products of inertia are zero

        # Let the orientation of center-of-mass frame C is same as link-frame L
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        w_CoM_x = self.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
        w_CoM_y = self.phyPro.centerOfMass.y
        w_CoM_z = self.phyPro.centerOfMass.z
        w_Lo_x = self.pose.translation.x    # link-frame's origin point's x coordinat w.r.t world-frame
        w_Lo_y = self.pose.translation.y
        w_Lo_z = self.pose.translation.z
        # represent vector Lo-CoM which start from link-fram's origin point to CoM point w.r.t world-frame
        w_Lo_CoM = [[(w_CoM_x-w_Lo_x)*0.01], [(w_CoM_y-w_Lo_y)*0.01], [(w_CoM_z-w_Lo_z)*0.01]] # cm -> m
        # represent world-frame's orientation w.r.t link-frame
        L_R_w = [[self.pose.getCell(0, 0), self.pose.getCell(1, 0), self.pose.getCell(2, 0)],
                 [self.pose.getCell(0, 1), self.pose.getCell(1, 1), self.pose.getCell(2, 1)],
                 [self.pose.getCell(0, 2), self.pose.getCell(1, 2), self.pose.getCell(2, 2)]]

        L_Lo_CoM = math_op.change_orientation(L_R_w, w_Lo_CoM)
        pose_CoM = [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]

        return pose_CoM

    def get_CoM_urdf(self):
        """
        Get CoM frame w.r.t link frame in [x, y, z, roll, pitch, yaw] representation
        In URDF, parent link frame L is the parent joint frame Jp
        Return:
        ---------
        pose_CoM: [x, y, z, roll, pitch, yaw]
        """
        if self.get_parent_joint() is None:
            # for the first link which does not have parent joint
            # do not know the CoM is w.r.t world frame or the link frame
            # here let it be w.r.t the link frame
            pose_CoM = self.get_CoM_sdf()
        else:
            textPalette: adsk.core.Palette = constants.get_text_palette()
            joint = Joint(self.get_parent_joint())
            textPalette.writeText("Parent joint: {}".format(joint.get_name()))
            parent_joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
            from_origin, from_xAxis, from_yAxis, from_zAxis = parent_joint_frame.getAsCoordinateSystem()
            textPalette.writeText("Parent joint frame: \n"+math_op.matrix3D_to_str(parent_joint_frame))

            w_CoM_x = self.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
            w_CoM_y = self.phyPro.centerOfMass.y
            w_CoM_z = self.phyPro.centerOfMass.z

            # Let the CoM-frame C has the same orientation with parent link frame
            CoM_frame_O: adsk.core.Point3D = adsk.core.Point3D.create()
            CoM_frame_O.set(w_CoM_x, w_CoM_y, w_CoM_z)
            CoM_frame: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
            CoM_frame_x: adsk.core.Vector3D = from_xAxis
            CoM_frame_y: adsk.core.Vector3D = from_yAxis
            CoM_frame_z: adsk.core.Vector3D = from_zAxis
            CoM_frame.setWithCoordinateSystem(CoM_frame_O, CoM_frame_x, CoM_frame_y, CoM_frame_z)
            to_origin, to_xAsix, to_yAxis, to_zAxis = CoM_frame.getAsCoordinateSystem()
            textPalette.writeText("CoM frame: \n" + math_op.matrix3D_to_str(CoM_frame))

            transform = adsk.core.Matrix3D.create()
            # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis,
            #                                     to_origin, to_xAsix, to_yAxis, to_zAxis)
            transform = math_op.coordinate_transform(parent_joint_frame, CoM_frame)
            textPalette.writeText("Transform matrix: \n" + math_op.matrix3D_to_str(transform))

            pose_CoM = math_op.matrix3d_2_pose(transform)

        return pose_CoM

    def get_collision_sdf(self):
        """
        Get colision info for sdf collision element
        Since mesh frame coincide with the parent link frame
        Use the default position for colision mesh
        Return:
        ---------
        col_name: a string for the <collision> element's name attribute
        geometry_uri: a uri to locate the collisioin's mesh geometry
        """
        col_name = str(self.get_name()) + "_collision"
        robot_name = constants.get_robot_name()
        geometry_uri = "model://" + robot_name + "/meshes/" + str(self.get_name()) + ".stl"
        return col_name, geometry_uri

    def get_collision_urdf(self):
        """
        Get colision info for urdf collision element
        Return:
        ---------
        col_name: a string for the <collision> element's name attribute
        mesh_loc: the path to the mesh file
        """
        col_name = self.get_name() + "_collision"
        # robot_name = constants.get_robot_name()s
        # mesh_loc = "package://" + robot_name + "/meshes/" + self.get_name() + ".stl"
        mesh_loc = "meshes/" + self.get_name() + ".stl"
        return col_name, mesh_loc

    def get_visual_sdf(self):
        """
        Get colision info for sdf collision element
        Since mesh frame coincide with the parent link frame
        Use the default position for visualization mesh
        Return:
        ---------
        visual_name: a string for the <visual> element's name attribute
        geometry_uri: a uri to locate the visual's mesh geometry
        """
        visual_name = self.get_name() + "_visual"
        robot_name = constants.get_robot_name()
        geometry_uri = "model://" + robot_name + "/meshes/" + str(self.get_name()) + ".stl"
        return visual_name, geometry_uri

    def get_visual_urdf(self):
        """
        Return:
            visual_name: a string for the <visual> element's name attribute
            mesh_loc: str
                the path to the mesh file
        """
        visual_name = self.get_name() + "_visual"
        robot_name = constants.get_robot_name()
        # mesh_loc = "package://" + robot_name + "/meshes/" + self.get_name() + ".stl"
        mesh_loc = "meshes/" + self.get_name() + ".stl"
        return visual_name, mesh_loc

    def get_visual_body(self) -> adsk.fusion.BRepBody:
        """
        get body that contains 'visual' in its name
        which means they are used as visual geometry

        Return:
        visual_geo: BRepBody
            the body represents visual geometry
        None
        """
        for body in self.link.bRepBodies:
            b_name: str = body.name
            if b_name.find('visual') != -1:
                visual_geo: adsk.fusion.BRepBody = body
                return visual_geo

        return None

    def get_collision_body(self) -> adsk.fusion.BRepBody:
        """
        get body that contains 'collision' in its name
        which means they are used as collision geometry

        Return:
        col_geo: BRepBody
            the body represents collision geometry
        None
        """
        for body in self.link.bRepBodies:
            b_name: str = body.name
            if b_name.find('collision') != -1:
                col_geo: adsk.fusion.BRepBody = body
                return col_geo

        return None
