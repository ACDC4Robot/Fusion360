# -*- coding: utf-8 -*-
# Author: nuofan
"""
Export robot description format files from Fusion360 design
"""

import adsk, adsk.core, adsk.fusion, traceback
import os, sys
from ...core.link import Link
from ...core.joint import Joint
from . import constants
from ...core import write
from ...core import utils
from ...core.robot import Robot
from ...core.urdf_plus import URDF_PLUS
import time

def get_link_joint_list(design: adsk.fusion.Design):
    """
    Get the link list and joint list to export

    Return:
    link_list: [Link]
        a list contains all the links that will be exported
    joint_list: [Joint]
        a list contains all the joint that will be exported
    """
    root = design.rootComponent
    link_list = []
    joint_list = []
    occs: adsk.fusion.OccurrenceList = root.allOccurrences
    
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

    for joint in root.allJoints:
        joint_list.append(Joint(joint)) # add joint objects into joint_list
    for joint in root.allAsBuiltJoints:
        joint_list.append(Joint(joint)) # add joint objects into joint_list

    return link_list, joint_list

def export_stl(design: adsk.fusion.Design, save_dir: str, links: list[Link]):
    """
    export each component's stl file into "save_dir/mesh"

    Parameters
    ---------
    design: adsk.fusion.Design
        current active design
    save_dir: str
        the directory to store the export stl file
    """
    # create a single exportManager instance
    export_manager = design.exportManager
    # set the directory for the mesh file
    try: os.mkdir(save_dir + "/meshes")
    except: pass
    mesh_dir = save_dir + "/meshes"

    for link in links:
        visual_body: adsk.fusion.BRepBody = link.get_visual_body()
        col_body: adsk.fusion.BRepBody = link.get_collision_body()
        if (visual_body is None) and (col_body is None):
            # export the whole occurrence
            mesh_name = mesh_dir + "/" + link.get_name()
            occ = link.get_link_occ()
            # obj_export_options = export_manager.createOBJExportOptions(occ, mesh_name)
            # obj_export_options.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits # set unit to mm
            # obj_export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            # export_manager.execute(obj_export_options)
            stl_export_options = export_manager.createSTLExportOptions(occ, mesh_name)
            stl_export_options.sendToPrintUtility = False
            stl_export_options.isBinaryFormat = True
            stl_export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            export_manager.execute(stl_export_options)
        elif (visual_body is not None) and (col_body is not None):
            # export visual and collision geometry seperately
            visual_mesh_name = mesh_dir + "/" + link.get_name() + "_visual"
            visual_exp_options = export_manager.createSTLExportOptions(visual_body, visual_mesh_name)
            visual_exp_options.sendToPrintUtility = False
            visual_exp_options.isBinaryFormat = True
            visual_exp_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            export_manager.execute(visual_exp_options)

            col_mesh_name = mesh_dir + "/" + link.get_name() + "_collision"
            col_exp_options = export_manager.createSTLExportOptions(col_body, col_mesh_name)
            col_exp_options.sendToPrintUtility = False
            col_exp_options.isBinaryFormat = True
            col_exp_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            export_manager.execute(col_exp_options)

        elif (visual_body is None) and (col_body is not None):
            error_message = "Please set two bodies, one for visual and one for collision. \n"
            error_message = error_message + "Body for visual missing."
            utils.error_box(error_message)
            utils.terminate_box()
        elif (visual_body is not None) and (col_body is None):
            error_message = "Please set two bodies, one for visual and one for collision. \n"
            error_message = error_message + "Body for collision missing."
            utils.error_box(error_message)
            utils.terminate_box()


def run():
    # Initialization
    app = adsk.core.Application.get()
    ui = app.userInterface
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)

    msg_box_title = "ACDC4Robot Message"
    
    # open a text palette for debuging
    textPalette = ui.palettes.itemById("TextCommands")
    if not textPalette.isVisible:
        textPalette.isVisible = True
    constants.set_text_palette(textPalette)

    try:
        # # Check the length unit of Fusion360
        # if design.unitsManager.defaultLengthUnits != "m":
        #     ui.messageBox("Please set length unit to 'm'!", msg_box_title)
        #     return 0 # exit run() function
        
        root = design.rootComponent # get root component
        allComp = design.allComponents
        robot_name = root.name.split()[0]
        constants.set_robot_name(robot_name)
        
        # Set the folder to store exported files
        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "Chose your folder to export"
        dialog_result = folder_dialog.showDialog() # show folder dialog
        save_folder = ""
        if dialog_result == adsk.core.DialogResults.DialogOK:
            save_folder = folder_dialog.folder
        else:
            ui.messageBox("ACDC4Robot was canceled", msg_box_title)
            return 0 # exit run() function
        
        save_folder = save_folder + "/" + robot_name
        try: os.mkdir(save_folder)
        except: pass

        ui.messageBox("Start ACDC4Robot Add-IN", msg_box_title)

        # get all the link & joint elements to export
        link_list, joint_list = get_link_joint_list(design)

        rdf = constants.get_rdf()
        simulator = constants.get_sim_env()

        if rdf == None:
            ui.messageBox("Robot description format is None.\n" +
                          "Please choose one robot description format", msg_box_title)
        elif rdf == "URDF":
            if simulator  == "None":
                ui.messageBox("Simulation environment is None.\n" +
                              "Please select a simulation environment.", msg_box_title)
            elif simulator in ["Gazebo", "PyBullet", "MuJoCo"]:
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                if simulator == 'PyBullet':
                    write.write_hello_pybullet(rdf, robot_name, save_folder)
                ui.messageBox(f"Finished exporting URDF for {simulator}.", msg_box_title)

        elif rdf == "SDFormat":
            if simulator == "None":
                ui.messageBox("Simulation environment is None.\n" + 
                              "Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # write a model cofig file
                author = constants.get_author_name()
                des = constants.get_model_description()
                write.write_sdf_config(save_folder, robot_name, author, des)
                # export stl files
                export_stl(design, save_folder, link_list)
                ui.messageBox("Finished exporting SDFormat for Gazebo.", msg_box_title)
            elif simulator == "PyBullet":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # export stl files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                write.write_hello_pybullet(rdf,robot_name, save_folder)
                ui.messageBox("Finished exporting SDFormat for PyBullet.", msg_box_title)
            
            elif simulator == "MuJoCo":
                ui.messageBox("MuJoCo does not support SDFormat. \n" +
                              "Please select PyBullet or Gazebo as simulation environment.", msg_box_title)

        elif rdf == "MJCF":
            if simulator == "None":
                ui.messageBox("Simulation environment is None. \n" +
                              "Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                ui.messageBox("Gazebo does not support MJCF. \n"+
                              "Please select MuJoCo for simulation.", msg_box_title)
            elif simulator == "PyBullet":
                ui.messageBox("PyBullet does not support MJCF. \n" +
                              "Please select MuJoCo for simulation.", msg_box_title)
            elif simulator == "MuJoCo":
                # write to .xml file
                write.write_mjcf(root, robot_name, save_folder)
                # export stl files
                export_stl(design, save_folder, link_list)
                time.sleep(0.1)
                ui.messageBox("Finished exporting MJCF for MuJoCo.", msg_box_title)
        
        elif rdf == "URDF+":
            robot = Robot(design)
            urdf_plus = URDF_PLUS(robot)
            urdf_plus_path = save_folder + "/{}.urdf".format(robot.get_robot_name())
            urdf_plus.write_file(urdf_plus_path)
            stl_list: list[Link] = robot.get_links()
            export_stl(design, save_folder, stl_list)
            time.sleep(0.1)
            ui.messageBox("Finished exporting URDF+.", msg_box_title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
