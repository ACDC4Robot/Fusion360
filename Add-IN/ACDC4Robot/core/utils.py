# -*- coding: utf-8 -*-
"""
contains common useful functions for this project
"""
import adsk, adsk.core, adsk.fusion
import re

from .. import i18n
from .serialization import export_name

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
    # Fusion occurrence paths contain instance suffixes such as ``Payload:1``
    # and use ``+`` between nested occurrences.  Removing everything after
    # ``:`` made repeated occurrences collide in MJCF and overwrite one
    # another's STL files.  Preserve hierarchy and instance identity.
    return export_name(s)


def log(message: str):
    """Write to Fusion's text palette when available, without breaking export."""
    try:
        from ..commands.ACDC4Robot import constants

        palette = constants.get_text_palette()
        if palette is not None:
            palette.writeText(str(message))
    except Exception:
        # Diagnostics are best-effort. A missing/closed palette must never turn
        # an otherwise actionable export error into a second exception.
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
    locale = i18n.get_locale()
    box_title = i18n.translate("error_title", locale)
    buttons = adsk.core.MessageBoxButtonTypes.OKButtonType
    icon = adsk.core.MessageBoxIconTypes.WarningIconType

    _ = ui.messageBox(
        i18n.localize_error_message(message, locale), box_title, buttons, icon
    )

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
