# -*- coding: utf-8 -*-
# Author: nuofan
"""
Export robot description format files from Fusion360 design
"""

import adsk, adsk.core, adsk.fusion, traceback
import os
from ...core.link import Link
from ...core.joint import Joint
from . import constants
from ...core import write
from ...core import preflight, utils
from ...core.robot import Robot
from ...core.urdf_plus import URDF_PLUS
from ... import i18n
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
    from ...core.link_discovery import discover_links
    occurrences, joints = discover_links(design.rootComponent)
    from ...core.serialization import validate_export_names
    validate_export_names(occurrences)
    return [Link(occurrence) for occurrence in occurrences], [Joint(joint) for joint in joints]


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
    os.makedirs(save_dir + "/meshes", exist_ok=True)
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
            error_message = i18n.translate("visual_body_missing")
            utils.error_box(error_message)
            utils.terminate_box()
        elif (visual_body is not None) and (col_body is None):
            error_message = i18n.translate("collision_body_missing")
            utils.error_box(error_message)
            utils.terminate_box()


def run():
    # Initialization
    app = adsk.core.Application.get()
    ui = app.userInterface
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)

    locale = i18n.get_locale()
    msg_box_title = i18n.translate("message_title", locale)
    
    # open a text palette for debuging
    textPalette = ui.palettes.itemById("TextCommands")
    if not textPalette.isVisible:
        textPalette.isVisible = True
    constants.set_text_palette(textPalette)

    try:
        if design is None:
            ui.messageBox(i18n.translate("active_document_not_design", locale), msg_box_title)
            return 0

        # # Check the length unit of Fusion360
        # if design.unitsManager.defaultLengthUnits != "m":
        #     ui.messageBox("Please set length unit to 'm'!", msg_box_title)
        #     return 0 # exit run() function
        
        root = design.rootComponent # get root component
        robot_name = utils.get_valid_filename(root.name) or "robot"
        constants.set_robot_name(robot_name)

        rdf = constants.get_rdf()
        simulator = constants.get_sim_env()

        preflight_report = None
        if rdf == "MJCF" and simulator == "MuJoCo":
            preflight_report = preflight.inspect_mjcf_design(design)
            report_text = preflight.format_report(preflight_report, locale=locale)
            textPalette.writeText(report_text)
            if not preflight_report["passed"]:
                ui.messageBox(
                    report_text + "\n\n" + i18n.translate("preflight_no_files", locale),
                    i18n.translate("preflight_title", locale),
                )
                return 0
        
        # Set the folder to store exported files
        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = i18n.translate("choose_export_folder", locale)
        dialog_result = folder_dialog.showDialog() # show folder dialog
        save_folder = ""
        if dialog_result == adsk.core.DialogResults.DialogOK:
            save_folder = folder_dialog.folder
        else:
            ui.messageBox(i18n.translate("canceled", locale), msg_box_title)
            return 0 # exit run() function
        
        save_folder = save_folder + "/" + robot_name
        os.makedirs(save_folder, exist_ok=True)

        if preflight_report is not None:
            preflight.write_report(save_folder, preflight_report)

        ui.messageBox(i18n.translate("start_export", locale), msg_box_title)

        # get all the link & joint elements to export
        link_list, joint_list = get_link_joint_list(design)

        if rdf in (None, "None"):
            ui.messageBox(i18n.translate("select_description_format", locale), msg_box_title)
        elif rdf == "URDF":
            if simulator  == "None":
                ui.messageBox(i18n.translate("select_simulation_environment", locale), msg_box_title)
            elif simulator in ["Gazebo", "PyBullet", "MuJoCo"]:
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                if simulator == 'PyBullet':
                    write.write_hello_pybullet(rdf, robot_name, save_folder)
                ui.messageBox(i18n.translate("finished_urdf", locale, simulator=simulator), msg_box_title)

        elif rdf == "SDFormat":
            if simulator == "None":
                ui.messageBox(i18n.translate("select_simulation_environment", locale), msg_box_title)
            elif simulator == "Gazebo":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # write a model cofig file
                author = constants.get_author_name()
                des = constants.get_model_description()
                write.write_sdf_config(save_folder, robot_name, author, des)
                # export stl files
                export_stl(design, save_folder, link_list)
                ui.messageBox(i18n.translate("finished_sdf_gazebo", locale), msg_box_title)
            elif simulator == "PyBullet":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # export stl files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                write.write_hello_pybullet(rdf,robot_name, save_folder)
                ui.messageBox(i18n.translate("finished_sdf_pybullet", locale), msg_box_title)
            
            elif simulator == "MuJoCo":
                ui.messageBox(i18n.translate("mujoco_no_sdf", locale), msg_box_title)

        elif rdf == "MJCF":
            if simulator == "None":
                ui.messageBox(i18n.translate("select_simulation_environment", locale), msg_box_title)
            elif simulator == "Gazebo":
                ui.messageBox(i18n.translate("gazebo_no_mjcf", locale), msg_box_title)
            elif simulator == "PyBullet":
                ui.messageBox(i18n.translate("pybullet_no_mjcf", locale), msg_box_title)
            elif simulator == "MuJoCo":
                # write to .xml file
                write.write_mjcf(root, robot_name, save_folder)
                # export stl files
                export_stl(design, save_folder, link_list)
                time.sleep(0.1)
                warning_count = len(preflight_report["warnings"]) if preflight_report else 0
                ui.messageBox(
                    i18n.translate("finished_mjcf", locale, warning_count=warning_count),
                    msg_box_title,
                )
        
        elif rdf == "URDF+":
            robot = Robot(design)
            urdf_plus = URDF_PLUS(robot)
            urdf_plus_path = save_folder + "/{}.urdf".format(robot.get_robot_name())
            urdf_plus.write_file(urdf_plus_path)
            stl_list: list[Link] = robot.get_links()
            export_stl(design, save_folder, stl_list)
            time.sleep(0.1)
            ui.messageBox(i18n.translate("finished_urdf_plus", locale), msg_box_title)
        
    except ValueError as exc:
        # Expected validation/export failures should be concise and actionable
        # in the dialog while retaining the traceback in Text Commands.
        if textPalette:
            textPalette.writeText(traceback.format_exc())
        if ui:
            error_text = i18n.localize_error_message(str(exc), locale)
            ui.messageBox(i18n.translate("export_stopped", locale, error=error_text), msg_box_title)
    except Exception:
        if textPalette:
            textPalette.writeText(traceback.format_exc())
        if ui:
            ui.messageBox(
                i18n.translate(
                    "export_failed", locale, error=traceback.format_exc(limit=3)
                ),
                msg_box_title,
            )
