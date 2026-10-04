"""Occurrence-based rigid-link discovery shared by all exporters.

A jointed occurrence owns its rigid subtree. Ownership must never overlap;
subcomponents with their own joint endpoints require a different partition.
MJCF still applies its separate live-validated geometry preflight.
"""

def items(collection):
    try:
        return list(collection)
    except TypeError:
        return [collection.item(i) for i in range(collection.count)]


def unique_joints(root):
    # Names and entityToken strings are not identity keys. Fusion documents that
    # different token strings can identify the same entity. Compare API objects.
    result = []
    for joint in items(root.allJoints) + items(root.allAsBuiltJoints):
        if not any(joint is previous or joint == previous for previous in result):
            result.append(joint)
    return result


def discover_links(root):
    joints = unique_joints(root)
    occurrences = items(root.allOccurrences)
    endpoints = {}
    errors = []
    for joint in joints:
        for occurrence in (joint.occurrenceOne, joint.occurrenceTwo):
            if occurrence is None:
                errors.append(f"Joint {joint.name} connects to root/ground; use a base_link occurrence.")
                continue
            if not occurrence.isLightBulbOn:
                errors.append(f"Joint {joint.name} references hidden occurrence {occurrence.fullPathName}.")
            endpoints.setdefault(occurrence.fullPathName, occurrence)
    if not joints:
        return [occ for occ in occurrences if occ.isLightBulbOn
                and occ.childOccurrences.count == 0 and occ.component.bRepBodies.count], joints

    roots = list(endpoints)
    for path in roots:
        for other in roots:
            if path != other and path.startswith(other + '+'):
                errors.append(f"Overlapping link subtrees {other} and {path}; move joint endpoints to non-overlapping link components.")
    for occurrence in occurrences:
        if not occurrence.isLightBulbOn or not occurrence.component.bRepBodies.count:
            continue
        path = occurrence.fullPathName
        owners = [owner for owner in roots if path == owner or path.startswith(owner + '+')]
        if len(owners) != 1:
            errors.append(f"Visible geometry {path} must belong to exactly one jointed link subtree; found {len(owners)}.")
    if errors:
        raise ValueError('\n'.join(dict.fromkeys(errors)))
    return list(endpoints.values()), joints
