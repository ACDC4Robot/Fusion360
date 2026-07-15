# -*- coding: utf-8 -*-
"""
contains common useful functions for this project
"""
import adsk, adsk.core, adsk.fusion
import re

## https://github.com/django/django/blob/master/django/utils/text.py
def get_valid_filename(s):
    """
    Return the given string converted to a string that can be used for a clean
    filename. Remove leading and trailing spaces; convert other spaces to
    underscores; and remove anything that is not an alphanumeric, dash,
    underscore, or dot.
    >>> get_valid_filename("john's portrait in 2004.jpg")
    'johns_portrait_in_2004.jpg'
    """
    # Replace the number `:#+` by `-`
    s = re.sub(r':.*?\+', '_', s)
    # Remove the number at the end of the full path name
    s = re.sub(r':.*$', '', s)

    s = str(s).strip().replace(' ', '-')
    return re.sub(r'(?u)[^-\w.]', '', s)


# Sanitizing full path names strips the occurrence counters (":1", ":2", ...),
# so two occurrences of the same component would collapse to the same link name
# and produce an invalid URDF/SDF/MJCF. This registry hands out a stable,
# unique name per raw path for the duration of one export run.
_assigned_names = {}
_used_names = set()

def reset_name_registry():
    """
    Clear the unique-name registry. Must be called at the start of every export
    run so names from a previous export do not leak into the next one.
    """
    _assigned_names.clear()
    _used_names.clear()

def get_unique_name(raw_name: str, fallback: str = "unnamed") -> str:
    """
    Return a sanitized name that is unique for this export run.
    The same raw_name always maps to the same unique name.

    Args:
    raw_name: str
        the raw identifier, e.g. an occurrence fullPathName
    fallback: str
        base name used when sanitizing removes every character
    """
    if raw_name in _assigned_names:
        return _assigned_names[raw_name]

    base = get_valid_filename(raw_name)
    if not base:
        base = fallback
    name = base
    counter = 2
    while name in _used_names:
        name = "{}_{}".format(base, counter)
        counter += 1
    _used_names.add(name)
    _assigned_names[raw_name] = name
    return name

def log(message: str):
    """
    Write a message to the Fusion text palette if available.
    Never raises: logging must not break an export.
    """
    try:
        from ..commands.ACDC4Robot import constants
        palette = constants.get_text_palette()
        if palette is not None:
            palette.writeText(str(message))
    except Exception:
        pass

def error_box(message: str):
    """
    a message box to show error message

    Args:
    message: str
        the message to show
    """
    app = adsk.core.Application.get()
    ui = app.userInterface
    box_title = "ACDC4Robot Error"
    buttons = adsk.core.MessageBoxButtonTypes.OKButtonType
    icon = adsk.core.MessageBoxIconTypes.WarningIconType

    _ = ui.messageBox(message, box_title, buttons, icon)

def terminate_box():
    """
    terminate currently running command
    """
    app = adsk.core.Application.get()
    ui = app.userInterface

    _ = ui.terminateActiveCommand()

def component_has_bodies(component: adsk.fusion.Component):
    """
    Check if the component has visible bodies

    Args:
    component: adsk.fusion.Component
        the component to check

    Returns:
    bool
        True if the component has bodies, False otherwise
    """
    if component.bRepBodies.count == 0 and component.meshBodies.count == 0:
        return False
    if not component.isBodiesFolderLightBulbOn:
        return False

    # Check if the component has visible bodies
    for body in component.bRepBodies:
        if body.isLightBulbOn:
            return True
    for body in component.meshBodies:
        if body.isLightBulbOn:
            return True

    return False
