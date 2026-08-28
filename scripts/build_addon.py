#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

"""Build a deterministic Blender Extension archive for Coaster Mixer."""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORY = PROJECT_ROOT / "coaster_mixer"
MANIFEST_PATH = SOURCE_DIRECTORY / "blender_manifest.toml"
REQUIRED_PROJECT_FILES = (PROJECT_ROOT / "LICENSE", PROJECT_ROOT / "README.md")
FIXED_TIMESTAMP = (2026, 1, 1, 0, 0, 0)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "dist",
        help="Directory receiving the extension ZIP (default: dist)",
    )
    return parser.parse_args()


def load_manifest() -> dict:
    with MANIFEST_PATH.open("rb") as manifest_file:
        manifest = tomllib.load(manifest_file)

    required_fields = {
        "schema_version",
        "id",
        "version",
        "name",
        "tagline",
        "maintainer",
        "type",
        "blender_version_min",
        "license",
    }
    missing = sorted(required_fields.difference(manifest))
    if missing:
        raise ValueError(f"Manifest is missing required fields: {', '.join(missing)}")
    if manifest["type"] != "add-on":
        raise ValueError("Manifest type must be 'add-on'")
    return manifest


def archive_sources() -> list[tuple[Path, str]]:
    sources = [
        (path, path.relative_to(SOURCE_DIRECTORY).as_posix())
        for path in SOURCE_DIRECTORY.rglob("*")
        if path.is_file() and path.suffix != ".pyc" and "__pycache__" not in path.parts
    ]
    sources.extend((path, path.name) for path in REQUIRED_PROJECT_FILES)
    return sorted(sources, key=lambda item: item[1])


def write_file(archive: ZipFile, source: Path, archive_name: str) -> None:
    info = ZipInfo(archive_name, date_time=FIXED_TIMESTAMP)
    info.compress_type = ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    archive.writestr(info, source.read_bytes())


def verify_archive(archive_path: Path, expected_files: set[str]) -> None:
    with ZipFile(archive_path) as archive:
        names = set(archive.namelist())
        missing = expected_files.difference(names)
        unexpected = names.difference(expected_files)
        if missing or unexpected:
            raise ValueError(
                f"Archive contents differ (missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)})"
            )
        bad_member = archive.testzip()
        if bad_member is not None:
            raise ValueError(f"Corrupt archive member: {bad_member}")


def main() -> int:
    arguments = parse_arguments()
    manifest = load_manifest()
    sources = archive_sources()
    for required_path in (MANIFEST_PATH, *REQUIRED_PROJECT_FILES):
        if not required_path.is_file():
            raise FileNotFoundError(required_path)

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    archive_path = arguments.output_dir / f"{manifest['id']}-{manifest['version']}.zip"
    with ZipFile(archive_path, "w") as archive:
        for source, archive_name in sources:
            write_file(archive, source, archive_name)

    verify_archive(archive_path, {archive_name for _, archive_name in sources})
    print(archive_path.resolve())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"build failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error
