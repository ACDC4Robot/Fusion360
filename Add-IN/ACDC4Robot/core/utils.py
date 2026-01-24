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

    s = str(s).strip().replace(' ', '_')
    return re.sub(r'(?u)[^-\w.]', '', s)

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