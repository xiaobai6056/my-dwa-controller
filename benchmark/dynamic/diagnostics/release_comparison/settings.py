"""Explicit independent install selection and strict live mapping verification."""
from pathlib import Path
import os


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

WS=ARCHIVE_WS

def selection(variant):
    if variant not in ('unoptimized','release'):
        raise ValueError('variant must be unoptimized or release')
    prefix=WS/'install_compile_comparison_20261020'/variant
    return prefix,prefix/'lib/libmy_dwa_controller.so'

def mapping_matches(text,expected):
    paths=set()
    for line in text.splitlines():
        if 'libmy_dwa_controller.so' in line:
            fields=line.split(None,5)
            if len(fields)!=6 or fields[5].endswith(' (deleted)'):
                return False
            paths.add(str(Path(fields[5]).resolve()))
    return paths=={str(Path(expected).resolve())}
def completed_trial(returncode,summary):
    """Original trial returns 2 for valid non-successful navigation results."""
    return (returncode in (0,2) and summary.get('valid') is True
            and summary.get('status')=='finished'
            and summary.get('action_accepted') is True
            and summary.get('action_error') is None
            and summary.get('error') is None)