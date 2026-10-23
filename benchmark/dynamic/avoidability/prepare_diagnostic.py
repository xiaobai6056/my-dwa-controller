"""New supervisor adapter; original runner/launch reused byte-for-byte where possible."""
from pathlib import Path
HERE=Path(__file__).resolve().parent
source=(HERE.parent/'timing_sweep/run_one.py').read_text()
source=source.replace("from rules import selection, mapping_matches, completed_trial","from rules import selection, mapping_matches, completed_trial")
source=source.replace("sys.path.insert(0, str(DYNAMIC.parent))","sys.path.insert(0, str(DYNAMIC.parent))\nsys.path.insert(0, str(DYNAMIC/'timing_sweep'))")
source=source.replace("if args.scenario != 'D01' or args.gui or args.preload:","if args.scenario != 'D01' or args.gui:")
source=source.replace("str(HERE / 'launch_scene.sh')","str(DYNAMIC / 'timing_sweep/launch_scene.sh')")
source=source.replace("str(HERE / 'trial_entry.py')","str(DYNAMIC / 'timing_sweep/trial_entry.py')")
source=source.replace("control_env['NAVPERF_DIR'] = str(output)","control_env['DECISION_DIR'] = str(output)\n        control_env['NAVPERF_MYDWA_LIB'] = str(plugin)")
source=source.replace("classification='d01_crossing_timing_scan_release'","classification='d01_observation_only_decision_diagnostic'")
source=source.replace("(DYNAMIC/'diagnostics/perf_probe.cpp')","(HERE/'decision_probe.cpp')")
source=source.replace("while runner.poll() is None and not interrupted:\n", "while runner.poll() is None and not interrupted:\n                if 'symbol lookup error' in (output/'launch.log').read_text():\n                    manifest['supervisor_error']='observer_link_failure'\n                    break\n")
(HERE/'run_diagnostic.py').write_text(source)