"""Reuse the existing test target's compile/link flags without modifying its sources."""
from pathlib import Path
import os

import shlex
import subprocess


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

here = Path(__file__).resolve().parent
build = (ARCHIVE_WS / 'build/my_dwa_controller')
flags = dict(line.split(' = ', 1) for line in
             (build / 'CMakeFiles/test_controller.dir/flags.make').read_text().splitlines()
             if line.startswith('CXX_') and ' = ' in line)
arguments = ['/usr/bin/c++', '-std=c++17']
for name in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
    arguments.extend(shlex.split(flags[name]))
subprocess.run(arguments + ['-c', str(here / 'terminal_latch_probe.cpp'),
                           '-o', str(here / 'terminal_latch_probe.o')], cwd=build, check=True)
link = shlex.split((build / 'CMakeFiles/test_controller.dir/link.txt').read_text())
link[1] = str(here / 'terminal_latch_probe.o')
link[link.index('-o') + 1] = str(here / 'terminal_latch_probe')
subprocess.run(link, cwd=build, check=True)
subprocess.run([str(here / 'terminal_latch_probe'),
                '--gtest_filter=ControllerTest.AuditTerminalLatchAgainstResetChecker',
                '--gtest_output=xml:' + str(here / 'terminal_latch_probe.xml')], check=True)
