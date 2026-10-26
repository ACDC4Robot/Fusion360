"""Read-only Fusion joint/constraint probe. Run from Scripts and Add-Ins.

Writes local JSON only. It does not edit/suppress joints or constraints, export
geometry, save the design, or upload data. Names and occurrence paths are local
diagnostic data: review the report before sharing it.
"""

from datetime import datetime, timezone
import json
import platform
from pathlib import Path


# Optional absolute destination; leave None to choose a new file in Fusion.
OUTPUT_PATH = None


def _read(owner, attribute, convert=lambda value: value):
    try:
        return {"status": "ok", "value": convert(getattr(owner, attribute))}
    except AttributeError as exc:
        return {"status": "unavailable", "error": str(exc)}
    except Exception as exc:
        return {"status": "error", "error_type": type(exc).__name__, "error": str(exc)}


def _items(collection):
    try:
        iterator = iter(collection)
    except TypeError:
        if hasattr(type(collection), "__iter__"):
            raise
        return [collection.item(index) for index in range(collection.count)]
    return list(iterator)


def _path(occurrence):
    return None if occurrence is None else occurrence.fullPathName


def _identity(obj):
    return {name: _read(obj, name) for name in ("name", "objectType", "isValid")}


def _occurrence(occurrence):
    record = _identity(occurrence)
    record.update({
        "fullPathName": _read(occurrence, "fullPathName"),
        "assemblyContext": _read(occurrence, "assemblyContext", _path),
        "componentName": _read(occurrence, "component", lambda value: value.name),
        "isReferencedComponent": _read(occurrence, "isReferencedComponent"),
        "isLightBulbOn": _read(occurrence, "isLightBulbOn"),
    })
    return record


def _relationship(relationship):
    record = _identity(relationship)
    for name in ("isSuppressed", "healthState", "errorOrWarningMessage"):
        record[name] = _read(relationship, name, str if name == "healthState" else lambda value: value)
    for name in ("assemblyContext", "occurrenceOne", "occurrenceTwo"):
        record[name] = _read(relationship, name, _path)
    record["jointType"] = _read(relationship, "jointMotion", lambda value: str(value.jointType))
    return record


def _constraint(constraint):
    record = _relationship(constraint)
    # Record relationship types, without reading the referenced geometry.
    record["geometricRelationships"] = _collection(
        constraint, "geometricRelationships", lambda value: {"objectType": _read(value, "objectType")}
    )[0]
    return record


def _collection(owner, attribute, describe):
    try:
        collection = getattr(owner, attribute)
    except AttributeError as exc:
        return {"status": "unavailable", "stage": "property", "error": str(exc)}, []
    except Exception as exc:
        return {"status": "error", "stage": "property", "error_type": type(exc).__name__, "error": str(exc)}, []
    try:
        objects = _items(collection)
    except Exception as exc:
        return {"status": "error", "stage": "iteration", "error_type": type(exc).__name__, "error": str(exc)}, []
    return {"status": "ok", "count": len(objects), "items": [describe(obj) for obj in objects]}, objects


def _count(owner, attribute):
    def count(collection):
        value = getattr(collection, "count", None)
        return value if isinstance(value, int) else len(collection)
    return _read(owner, attribute, count)


def _component(component):
    record = _identity(component)
    record["counts"] = {name: _count(component, name) for name in ("occurrences", "bRepBodies", "meshBodies")}
    record["collections"] = {
        name: _collection(component, name, _constraint if "Constraints" in name else _relationship)[0]
        for name in (
            "allJoints", "allAsBuiltJoints", "joints", "asBuiltJoints",
            "allAssemblyConstraints", "assemblyConstraints",
        )
    }
    return record


def collect_report(app, design):
    """Collect API results, distinguishing empty, unsupported and failed reads."""
    root = design.rootComponent
    occurrences, _ = _collection(root, "allOccurrences", _occurrence)
    components, _ = _collection(design, "allComponents", _component)
    return {
        "schema": "acdc_joint_diagnostics/v1",
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "fusion_version": _read(app, "version"),
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "design_type": _read(design, "designType", str),
        "root": _component(root),
        "occurrences": occurrences,
        "components": components,
        "scope": "API metadata only; no geometry, design changes, design save, or upload.",
    }


def write_report(app, design, output_path):
    """Write a new local report; never overwrite an existing file."""
    report = collect_report(app, design)
    path = Path(output_path)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    return path


def run(context):
    import adsk.core
    import adsk.fusion

    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox("Open the assembly in Fusion's Design workspace first.", "Joint diagnostics")
            return
        output_path = OUTPUT_PATH
        if output_path is None:
            dialog = ui.createFileDialog()
            dialog.title = "Save local joint diagnostic JSON (choose a new filename)"
            dialog.filter = "JSON files (*.json)"
            dialog.initialFilename = "joint-diagnostics-" + datetime.now().strftime("%Y%m%d-%H%M%S") + ".json"
            if dialog.showSave() != adsk.core.DialogResults.DialogOK:
                return
            output_path = dialog.filename
        path = write_report(app, design, output_path)
        ui.messageBox(f"Saved local metadata report:\n{path}\n\nReview names and paths before sharing. The design was not changed.", "Joint diagnostics")
    except Exception as exc:
        ui.messageBox(f"Diagnostic report failed: {type(exc).__name__}: {exc}", "Joint diagnostics")
