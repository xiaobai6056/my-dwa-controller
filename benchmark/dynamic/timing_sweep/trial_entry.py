#!/usr/bin/env python3
"""Run original dynamic trial, replacing only its in-memory scenario loader."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from rules import DYNAMIC,scene,selection

def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--start-time',type=float,required=True)
    parsed,remaining=parser.parse_known_args()
    out_index=remaining.index('--output')+1
    output=Path(remaining[out_index]).resolve()
    controller=remaining[remaining.index('--controller')+1]
    value=scene(parsed.start_time)
    (output.parent/'effective_scene.json').write_text(json.dumps(value,indent=2)+'\n')
    spec=importlib.util.spec_from_file_location('frozen_dynamic_trial',DYNAMIC/'run_trial.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original_loader=module.load
    module.load=lambda name:value if name=='D01' else original_loader(name)
    sys.argv=[str(DYNAMIC/'run_trial.py'),*remaining]
    code=module.main()
    # Preserve exact original runner summary; correct the hardcoded underlay hash
    # in the new experiment's metadata without touching old records or control.
    source=output/'summary.json'
    if source.exists():
        raw=source.read_text();(output/'summary_from_original_runner.json').write_text(raw)
        summary=json.loads(raw)
        summary['original_underlay_mydwa_sha256']=summary.pop('mydwa_library_sha256',None)
        _,plugin=selection(controller)
        summary['selected_plugin_path']=str(plugin)
        summary['selected_plugin_sha256']=hashlib.sha256(plugin.read_bytes()).hexdigest()
        if controller=='mydwa':summary['mydwa_library_sha256']=summary['selected_plugin_sha256']
        summary['effective_start_time_s']=parsed.start_time
        summary['effective_scene']=value
        source.write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        (output/'effective_scene.json').write_text(json.dumps(value,indent=2)+'\n')
    return code
if __name__=='__main__':raise SystemExit(main())