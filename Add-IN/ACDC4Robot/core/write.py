# -*- coding: utf-8 -*-
# Author: Nuofan
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, SubElement
from .link import Link
from .joint import Joint
from . import urdf as URDF
from . import sdf as SDF
from . import mjcf as MJCF
import adsk, adsk.core, adsk.fusion

def write_sdf(link_list: list[Link], joint_list: list[Joint], dir: str, robot_name: str):
    """
    Write all the joint and link elements to the sdf file
    """
    sdf_ele = Element("sdf")
    sdf_tree = ET.ElementTree(sdf_ele)
    sdf_ele.attrib = {"version": "1.7"}
    model = SubElement(sdf_ele, "model")
    model.attrib = {"name": robot_name}

    for link in link_list:
        # link_ele = link.get_link_sdf_element()
        link_ele = SDF.get_link_element(link)
        model.append(link_ele)
    
    for joint in joint_list:
        # joint_ele = joint.get_joint_sdf_element()
        joint_ele = SDF.get_joint_element(joint)
        model.append(joint_ele)

    # set indent to pretty the xml output
    ET.indent(sdf_tree, space="    ", level=0)

    sdf_file = dir + "/" + robot_name + ".sdf"
    sdf_tree.write(sdf_file, encoding="utf-8", xml_declaration=True)

def write_sdf_config(dir: str, model_name: str, author_name: str, description: str):
    """
    Write a config file for sdf
    """
    model = Element("model")
    config_tree = ET.ElementTree(model)
    name = SubElement(model, "name")
    name.text = "{}".format(model_name)
    author = SubElement(model, "author")
    author.text = "{}".format(author_name)
    version = SubElement(model, "version")
    version.text = "{}".format("1.0.0")
    sdf = SubElement(model, "sdf")
    sdf.attrib = {"version": "1.7"}
    sdf.text = "{}".format(str(model_name+".sdf"))
    des = SubElement(model, "description")
    des.text = "{}".format(description)

    # set indent to pretty the xml output and write the config file
    ET.indent(config_tree, space="    ", level=0)
    config_file = dir + "/model.config"
    config_tree.write(config_file, encoding="utf-8", xml_declaration=True) 
    
def write_urdf(link_list: list[Link], joint_list: list[Joint], dir: str, robot_name: str):
    """
    Write all the joint and link elements to the urdf file
    """
    robot_ele = Element("robot")
    robot_ele.attrib = {"name": robot_name}
    urdf_tree = ET.ElementTree(robot_ele)

    for link in link_list:
        # link_ele = link.get_link_urdf_element()
        link_ele = URDF.get_link_element(link)
        robot_ele.append(link_ele)
    
    for joint in joint_list:
        # joint_ele = joint.get_joint_urdf_element()
        joint_ele = URDF.get_joint_element(joint)
        if joint_ele is not None:
            robot_ele.append(joint_ele)

    # set indent to pretty the xml output
    ET.indent(urdf_tree, space="    ", level=0)

    urdf_file = dir + "/" + robot_name + ".urdf"
    urdf_tree.write(urdf_file, encoding="utf-8", xml_declaration=True)

def write_hello_pybullet(rdf_type: str, robot_name: str, save_dir: str):
    """
    Write a basic python script to automaticlly load robot description file
    Parameters:
    ---------
    rdf_type: str
        robot description type: urdf, sdf,
    robot_name: str
        the original Fusion360 file name
    save_dir: str
        the directory to save the python script
    """
    file_name = save_dir + "/hello_bullet.py"
    # template from: PyBullet Quickstart Guide - Hello PyBullet World
    pybullet_template: str = """import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
{} = p.{}("{}",cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   {})
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation({})
print(cubePos,cubeOrn)
p.disconnect()
"""
    if rdf_type == "URDF":
        rdf_file = robot_name + ".urdf"
        hello_pybullet = pybullet_template.format("robotID", "loadURDF", rdf_file, "flags=p.URDF_USE_INERTIA_FROM_FILE", "robotID")
    elif rdf_type == "SDFormat":
        rdf_file = robot_name + ".sdf"
        hello_pybullet = pybullet_template.format("robotIDs", "loadSDF", rdf_file, "", "robotIDs")

    with open(file_name, mode="w") as f:
        f.write(hello_pybullet)
        f.write("\n")

    
def write_mjcf(rootComp: adsk.fusion.Component, robotName: str, dir: str):
    """
    Write all elements in mjcf

    Parameters:
    ---------
    rootComp: rootComp of the design
    robotName: robot name
    dir: directory of the mjcf file
    """
    mjcf_ele = MJCF.get_mjcf(rootComp, robotName, dir)
    mjcf_tree = ET.ElementTree(mjcf_ele)

    # set indent to pretty the xml output
    ET.indent(mjcf_tree, space="    ", level=0)
    
    mjcf_file = dir + "/" + robotName + ".xml"
    mjcf_tree.write(mjcf_file, encoding="utf-8")
