#!/usr/bin/env python3
"""Check exact forwarding/exception behavior and calibrate observer overhead."""
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from read_perf import read  # noqa: E402

fixture = HERE / 'perf_build' / 'fixture'
fixture.mkdir(parents=True, exist_ok=True)
build = Path(tempfile.mkdtemp(prefix='calibration_', dir=fixture))
probe = (HERE / 'perf_probe.cpp').read_text()
my = re.search(r'MY = "([^"]+)"', probe).group(1)
dw = re.search(r'DW = "([^"]+)"', probe).group(1)
common = '#include <cstdio>\n#include <iostream>\n#include <chrono>\n#include <stdexcept>\n#include "geometry_msgs/msg/twist_stamped.hpp"\n#include "geometry_msgs/msg/pose_stamped.hpp"\nusing P=geometry_msgs::msg::PoseStamped; using T=geometry_msgs::msg::Twist; using C=geometry_msgs::msg::TwistStamped;\n'
declarations = '\n'.join(f'extern "C" C {name}(void*, const P&, const T&, void*) asm("{symbol}");'
                         for name, symbol in [('mc', my), ('dc', dw)])+'\n'
fake = common+declarations+'''
static int calls=0;
static C original(const T& v) { ++calls; if(v.linear.x==-100) throw std::runtime_error("original exception"); C r; r.twist.linear.x=v.linear.x+1; r.twist.angular.z=v.angular.z-1; return r; }
extern "C" C mc(void*,const P&,const T&v,void*) {return original(v);}
extern "C" C dc(void*,const P&,const T&v,void*) {return original(v);}
extern "C" int original_calls() {return calls;}
'''
main = common+declarations+'''
extern "C" int original_calls();
int main() {
 P p; T v; v.linear.x=3; v.angular.z=7;
 const auto start=std::chrono::steady_clock::now();
 for(int i=0;i<2000;++i) {
  auto a=mc((void*)1,p,v,nullptr); auto b=dc((void*)1,p,v,nullptr);
  if(a.twist.linear.x!=4 || b.twist.angular.z!=6 || v.linear.x!=3) return 2;
 }
 v.linear.x=-100;
 for(auto f:{&mc,&dc}) {try {f((void*)1,p,v,nullptr);return 3;} catch(const std::runtime_error&e) {if(std::string(e.what())!="original exception")return 4;}}
 const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-start).count();
 if(original_calls()!=4002)return 5;
 std::cout << original_calls() << " " << ns << std::endl;
}
'''
# Keep the generated fixture source straightforward despite Python string escaping.

(build / 'fake.cpp').write_text(fake)
(build / 'main.cpp').write_text(main)
includes = ['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
subprocess.run(['g++','-std=c++17','-O2','-fPIC','-shared',*includes,str(build/'fake.cpp'),'-o',str(build/'libfake.so')],check=True)
subprocess.run(['g++','-std=c++17','-O2',*includes,str(build/'main.cpp'),'-L'+str(build),'-lfake','-Wl,-rpath,'+str(build),'-o',str(build/'check')],check=True)
measurements = []
for n in range(5):
    directory = build / f'run_{n}'
    directory.mkdir(exist_ok=False)
    env = dict(os.environ, NAVPERF_MYDWA_LIB=str(build/'libfake.so'), NAVPERF_DWB_LIB=str(build/'libfake.so'), NAVPERF_DIR=str(directory))
    values = list(map(int, subprocess.check_output([str(build/'check')],env=env).split()))
    bare = dict(calls=values[0], elapsed_ns=values[1])
    values = list(map(int, subprocess.check_output([str(build/'check')],env=dict(env,LD_PRELOAD=str(HERE/'perf_build/libnavperf.so'))).split()))
    wrapped = dict(calls=values[0], elapsed_ns=values[1])
    samples, integrity = read(next(directory.glob('perf_*.bin')))
    assert len(samples)==4002 and sum(r['threw'] for r in samples)==2
    assert {r['controller'] for r in samples}=={1,2}
    assert all(r['vx']==4 and r['wz']==6 for r in samples if not r['threw'])
    measurements.append(dict(bare=bare, wrapped=wrapped, overhead_ns_per_call=(wrapped['elapsed_ns']-bare['elapsed_ns'])/4002))
result = dict(forwarding='4002 exact original calls including 2 preserved exceptions per pair',
    repetitions=5, median_added_ns_per_call=statistics.median(r['overhead_ns_per_call'] for r in measurements),
    measurements=measurements, caveat='Synthetic empty functions, includes mmap setup amortization; not a universal error bound.')
(build / 'probe_calibration.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))