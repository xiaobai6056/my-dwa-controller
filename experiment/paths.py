"""Explicit locations for historical experiment evidence stored outside Git."""
import os
from pathlib import Path


def external_directory(variable, purpose):
    value = os.environ.get(variable)
    if not value:
        raise SystemExit(f"Set {variable} to {purpose}; historical evidence is not bundled in this repository.")
    root = Path(value).expanduser().resolve()
    if not root.is_dir():
        raise SystemExit(f"{variable} is not an existing directory: {root}")
    return root


def data_root(label):
    root = external_directory("EXPERIMENT_DATA_ROOT", f"the external {label} evidence directory")
    if not (root / "input.json").is_file():
        raise SystemExit(f"EXPERIMENT_DATA_ROOT must contain the original input.json: {root}")
    return root


def previous_root():
    return external_directory("EXPERIMENT_PREVIOUS_ROOT", "the matching predecessor experiment evidence directory")
