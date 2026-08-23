#!/usr/bin/env python3
"""Build a deterministic, manually installable ACDC4Robot add-in archive."""

from __future__ import annotations

import hashlib
import json
import stat
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Add-IN" / "ACDC4Robot"
DIST = ROOT / "dist"
EXCLUDED_NAMES = {".DS_Store", ".env", "__pycache__"}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}
ZIP_TIMESTAMP = (2026, 8, 23, 0, 0, 0)


def _release_version() -> str:
    manifest = json.loads((SOURCE / "ACDC4Robot.manifest").read_text(encoding="utf-8"))
    return str(manifest["version"])


def _files() -> list[Path]:
    files = []
    for path in SOURCE.rglob("*"):
        relative = path.relative_to(SOURCE)
        if any(part in EXCLUDED_NAMES for part in relative.parts):
            continue
        if path.is_file() and path.suffix not in EXCLUDED_SUFFIXES:
            files.append(path)
    return sorted(files, key=lambda item: item.relative_to(SOURCE).as_posix())


def build() -> tuple[Path, str]:
    version = _release_version()
    DIST.mkdir(exist_ok=True)
    destination = DIST / f"ACDC4Robot-{version}.zip"

    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for source_path in _files():
            relative = source_path.relative_to(SOURCE)
            archive_path = (Path("ACDC4Robot") / relative).as_posix()
            info = zipfile.ZipInfo(archive_path, ZIP_TIMESTAMP)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (stat.S_IFREG | 0o644) << 16
            archive.writestr(info, source_path.read_bytes())

        for repository_path, archive_name in (
            (ROOT / "LICENSE", "ACDC4Robot/LICENSE"),
            (ROOT / "CHANGELOG.md", "ACDC4Robot/CHANGELOG.md"),
            (ROOT / "docs" / "MJCF_EXPORT.md", "ACDC4Robot/MJCF_EXPORT.md"),
        ):
            info = zipfile.ZipInfo(archive_name, ZIP_TIMESTAMP)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (stat.S_IFREG | 0o644) << 16
            archive.writestr(info, repository_path.read_bytes())

    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    checksum_path = destination.with_suffix(destination.suffix + ".sha256")
    checksum_path.write_text(f"{digest}  {destination.name}\n", encoding="utf-8")
    return destination, digest


if __name__ == "__main__":
    archive_path, sha256 = build()
    print(archive_path)
    print(f"sha256={sha256}")
