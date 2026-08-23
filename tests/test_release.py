import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "scripts"))

import build_addin_release  # noqa: E402


class ReleaseArchiveTests(unittest.TestCase):
    def test_archive_is_deterministic_and_private_path_free(self):
        with tempfile.TemporaryDirectory() as directory:
            original_dist = build_addin_release.DIST
            build_addin_release.DIST = Path(directory)
            try:
                first_path, first_digest = build_addin_release.build()
                first_bytes = first_path.read_bytes()
                second_path, second_digest = build_addin_release.build()
                self.assertEqual(first_digest, second_digest)
                self.assertEqual(first_bytes, second_path.read_bytes())

                with zipfile.ZipFile(second_path) as archive:
                    names = archive.namelist()
                    self.assertEqual(second_path.name, "ACDC4Robot-1.2.0.zip")
                    self.assertIn("ACDC4Robot/ACDC4Robot.manifest", names)
                    self.assertIn("ACDC4Robot/LICENSE", names)
                    self.assertIn("ACDC4Robot/MJCF_EXPORT.md", names)
                    self.assertFalse(any(".env" in name for name in names))
                    payload = b"".join(archive.read(name) for name in names)
                    private_home_prefix = b"/" + b"Users/"
                    self.assertNotIn(private_home_prefix, payload)
                    manifest = json.loads(
                        archive.read("ACDC4Robot/ACDC4Robot.manifest")
                    )
                    self.assertEqual(manifest["version"], "1.2.0")
                    self.assertIn("ACDC4Robot/i18n.py", names)
                    joint_source = archive.read("ACDC4Robot/core/joint.py")
                    self.assertIn(b"has no usable origin geometry", joint_source)
            finally:
                build_addin_release.DIST = original_dist


if __name__ == "__main__":
    unittest.main()
