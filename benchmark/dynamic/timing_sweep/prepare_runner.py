"""Create a separate minimal supervisor/launcher from frozen comparison tools."""
from pathlib import Path
HERE=Path(__file__).resolve().parent
DYNAMIC=HERE.parent
base=DYNAMIC/'diagnostics/release_comparison'
s=(base/'run_one.py').read_text()
s=s.replace('DYNAMIC = HERE.parents[1]','DYNAMIC = HERE.parent')
s=s.replace('from settings import selection, mapping_matches, completed_trial','from rules import selection, mapping_matches, completed_trial')
s=s.replace("    parser.add_argument('--variant', choices=['unoptimized','release'], required=True)","    parser.add_argument('--start-time', type=float, choices=[2.,3.,4.,5.,6.], required=True)")
s=s.replace('selection(args.variant)','selection(args.controller)')
s=s.replace("if args.controller != 'mydwa' or args.scenario != 'D01' or args.gui: parser.error('comparison requires D01/mydwa/headless')","if args.scenario != 'D01' or args.gui or args.preload: parser.error('sweep requires D01/headless, no performance preload')")
s=s.replace("    env['NAVPERF_MYDWA_LIB'] = str(plugin)","    env['SWEEP_START_TIME'] = str(args.start_time)")
s=s.replace("str(DYNAMIC / 'run_trial.py'), '--scene', args.scenario,", "str(HERE / 'trial_entry.py'), '--start-time', str(args.start_time), '--scene', args.scenario,")
s=s.replace("classification='release_build_comparison_not_formal'","classification='d01_crossing_timing_scan_release'")
s=s.replace("if 'libmy_dwa_controller.so' not in maps: continue", "if plugin.name not in maps: continue")
s=s.replace('One actual dynamic navigation, optional GUI, separate V0.2 diagnostic records.','One actual D01 crossing timing trial; Release MyDWA/system DWB, headless.')
for name,text in [('run_one.py',s),('launch_scene.sh',(base/'launch_scene.sh').read_text())]:
    p=HERE/name
    if p.exists():raise RuntimeError('refusing to overwrite wrapper')
    p.write_text(text)