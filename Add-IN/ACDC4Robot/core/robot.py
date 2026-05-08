# -*- coding: utf-8 -*-
from typing import List, Tuple
import adsk, adsk.fusion, adsk.core
from .link import Link
from .joint import Joint
from . import utils

class Robot():
    """
    Store robot link joint information into a bigraph structure
    """

    def __init__(self, robot: adsk.fusion.Design):
        self.robot = robot
        self.rootComp = robot.rootComponent
        self.name = self.get_robot_name()
        self.nodes = self.get_links()
        self.edges = self.get_joints()
        self.loop_joints, self.tree_joints = self.seperate_joints(self.edges)

    def get_robot_name(self, ) -> str:
        """
        Robot name is the root component name
        """
        robot_name = self.rootComp.name.split()[0]
        return robot_name

    def get_links(self, ) -> List[Link]:
        link_list = []
        occs: adsk.fusion.OccurrenceList = self.rootComp.allOccurrences

        # try to solve the nested components problem
        # but still not fully tested
        for occ in occs:
            if not utils.component_has_bodies(occ.component):
                continue
            # TODO: it seems use occ.joints.count will make it usable with occurrences? Test it
            if occ.component.joints.count > 0:
                # textPalette.writeText(str(occ.fullPathName))
                continue
            else:
                # Only occurrence contains zero joint and has zero childOccurrences 
                # can be seen as a link
                if occ.childOccurrences.count > 0:
                    # textPalette.writeText(str(occ.fullPathName))
                    # textPalette.writeText(str(occ.childOccurrences.count))
                    continue
                else:
                    # textPalette.writeText(str(occ.fullPathName))
                    # textPalette.writeText(str(occ.childOccurrences.count))
                    if occ.isLightBulbOn:
                        # only the occurrence light bulb on that the occurrence will be exported
                        link_list.append(Link(occ)) # add link objects into link_list

        return link_list
    
    def get_joints(self, ) -> List[Joint]:
        joint_list = []

        for joint in self.rootComp.allJoints:
            joint_list.append(Joint(joint)) # add joint objects into joint_list
        for joint in self.rootComp.allAsBuiltJoints:
            joint_list.append(Joint(joint)) # add joint objects into joint_list
        
        return joint_list

    def get_loop_joints(self,) -> List[Joint]:
        return self.loop_joints
    
    def get_tree_joints(self, ) -> List[Joint]:
        return self.tree_joints
    

    def get_graph(self, ):
        pass

    def seperate_joints(self, joint_list: List[Joint]) -> Tuple[List[Joint], List[Joint]]:
        """
        Seperate robot joints into two parts: loop joints, tree joints
        
        Returns:
        loop_joints: List[Joint]
            joints construct closed loop
            it should be noted by user explicitly by name it with prefix "loop"
        tree_joints: List[Joint]
            edges connects nodes(links) like a spanning tree
        """
        loop_joints = []
        tree_joints = []
        for joint in joint_list:
            if joint.name.startswith("loop"):
                loop_joints.append(joint)
            else:
                tree_joints.append(joint)
        return loop_joints, tree_joints

    def is_connected_graph(self, ):
        pass
