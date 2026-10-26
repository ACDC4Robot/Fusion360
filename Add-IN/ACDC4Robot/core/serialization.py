"""Lossless decimal URDF values and shared occurrence-preserving filenames."""
from decimal import Decimal, InvalidOperation
import re


def export_name(value):
    value = str(value).strip().replace('+', '__').replace(':', '_').replace(' ', '_')
    return re.sub(r'(?u)[^-\w.]', '', value)


def decimal_number(value):
    try:
        number = Decimal(str(value))
    except InvalidOperation as exc:
        raise ValueError(f'Invalid URDF numeric value: {value}') from exc
    if not number.is_finite():
        raise ValueError('URDF numeric values must be finite')
    if number == 0:
        return '0'
    result = format(number, 'f')
    return result.rstrip('0').rstrip('.') if '.' in result else result


def normalize_urdf_numbers(element):
    attributes = {'origin': ('xyz', 'rpy'), 'mass': ('value',),
                  'inertia': ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'),
                  'axis': ('xyz',), 'limit': ('lower', 'upper', 'effort', 'velocity'),
                  'mesh': ('scale',)}
    for node in element.iter():
        for key in attributes.get(node.tag, ()):
            if key in node.attrib:
                node.attrib[key] = ' '.join(decimal_number(v) for v in node.attrib[key].split())
    return element


def validate_export_names(occurrences):
    seen = {}
    for occurrence in occurrences:
        path = occurrence.fullPathName
        name = export_name(path)
        if not name or name in seen:
            raise ValueError(f'Invalid or duplicate export name {name}: {seen.get(name, "")} and {path}')
        seen[name] = path


def validate_xml_names(parent):
    """Check actual emitted identities before a description file is written.

    Legacy URDF and SDF use bare joint names, while class-based URDF prefixes
    them with the child path. Validate the output rather than assuming either
    naming convention when checking collisions.
    """
    seen = {}
    for node in parent:
        if node.tag not in ('link', 'joint', 'loop'):
            continue
        names = seen.setdefault(node.tag, set())
        name = node.get('name')
        if not name or name in names:
            raise ValueError(
                f'Invalid or duplicate {node.tag} export name {name!r}. '
                'Rename the corresponding Fusion objects so their exported names are unique.'
            )
        names.add(name)
