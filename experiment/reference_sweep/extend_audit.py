import pathlib
root=pathlib.Path('/home/user/ros2_experiments/reference_sweep_20261029')
p=root/'source/experiment/rolling/audit_reference.cpp';s=p.read_text()
s=s.replace('for (double key:{5.,6.,7.,8.,8.4,8.6,8.8,9.})','for (double key:{5.,6.,7.,8.,8.4,8.6,8.8,9.,10.,12.,14.,15.95})')
s=s.replace('const auto samples=sampleVelocities(plant.velocity,config);Json aligned=nullptr;',
'const auto samples=sampleVelocities(plant.velocity,config);Json aligned=nullptr,fastest=nullptr,ordinary_best=nullptr;')
anchor='''        auto check=examine(Json::array({cmd.v,cmd.omega}));if (!check["admitted"].get<bool>()) {continue;}'''
assert s.count(anchor)==1
s=s.replace(anchor,anchor+'''
        if (fastest.is_null() || check["command"][0]>fastest["command"][0] ||
          (check["command"][0]==fastest["command"][0] && check["guided_terms"]["total"]<fastest["guided_terms"]["total"])) {fastest=check;}
        if (ordinary_best.is_null() || check["ordinary_terms_same_action"]["total"]<ordinary_best["ordinary_terms_same_action"]["total"]) {ordinary_best=check;}''')
s=s.replace('      choices["most_heading_aligned_admitted"]=aligned;',
'''      choices["most_heading_aligned_admitted"]=aligned;
      choices["fastest_admitted"]=fastest;choices["best_ordinary_admitted"]=ordinary_best;''')
p.write_text(s)
