import pathlib,shutil,difflib
root=pathlib.Path('/home/user/ros2_experiments/reference_sweep_20261029')
folder=root/'source/experiment/rolling';parent=root.parent/'locked_reference_20261029/source/experiment/rolling'
p=folder/'reference.cpp';s=p.read_text()
s=s.replace('s_target=z.hi+.10,q_target=z.rear-.10','s_target=z.hi+forward_offset,q_target=z.rear-extra_clearance')
s=s.replace('<.37-1e-10','<.27+extra_clearance-1e-10')
s=s.replace('static_cast<long double>(w.heading)*s.heading_cost','static_cast<long double>(w.heading)*heading_multiplier*s.heading_cost')
p.write_text(s)
p=folder/'rolling.cpp';s=p.read_text();old=(parent/'rolling.cpp').read_text()
def replace(a,b):
    global s
    assert s.count(a)==1,a
    s=s.replace(a,b)
replace('  Json input;std::ifstream(argv[1])>>input;', '''  Json input;std::ifstream(argv[1])>>input;
  const auto reference_parameters=input.value("reference_parameters",Json::object());
  const double heading_multiplier=reference_parameters.value("heading_multiplier",1.);
  const double extra_clearance=reference_parameters.value("extra_clearance",.10);
  const double forward_offset=reference_parameters.value("forward_offset",.10);
  if (!std::isfinite(heading_multiplier) || heading_multiplier<=0 || !std::isfinite(extra_clearance) ||
      extra_clearance<.10 || !std::isfinite(forward_offset) || forward_offset<.10) {return 2;}''')
replace('      if (scheme=="B") {guide.observe', '''      guide.extra_clearance=extra_clearance;guide.forward_offset=forward_offset;
      if (scheme=="B") {guide.observe''')
replace('      if (scheme=="B" && guide.valid && reference_point && guide.phase!=ReferencePhase::Done) {',
'''      bool selected_guided=scheme=="B" && guide.valid && reference_point && guide.phase!=ReferencePhase::Done;
      if (selected_guided) {''')
replace('plant.pose,*reference_point);','plant.pose,*reference_point,heading_multiplier);')
replace('      if (scheme=="A" || guide.phase==ReferencePhase::Done) {',
'''      if (scheme=="A" || guide.phase==ReferencePhase::Done) {
        selected_guided=false;''')
replace('      Json weighted=nullptr;',
'''      const double effective_heading_weight=weights.heading*(selected_guided?heading_multiplier:1.);
      Json weighted=nullptr;''')
replace('weights.heading*selected_score["heading"].get<double>()','effective_heading_weight*selected_score["heading"].get<double>()')
replace('      Json state={{"reference",ref_state}',
'''      Json state={{"reference_parameters",{{"heading_multiplier",heading_multiplier},{"extra_clearance",extra_clearance},{"forward_offset",forward_offset}}},
        {"guided_scoring",selected_guided},{"effective_heading_weight",effective_heading_weight},{"score_is_held_proposal",hold_output},
        {"reference",ref_state}''')
# Verbatim inherited safety/plant helper blocks; score configuration is outside.
for a,b in [('      for (const auto & cmd:commands)','      const auto ordinary_ranked'),
            ('      Velocity2D selected{}','      auto risk=yield.update'),
            ('      if (!fallback_ok)','      if (entry<0')]:
    assert old[old.index(a):old.index(b)]==s[s.index(a):s.index(b)],a
p.write_text(s)
(root/'runner.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='locked_reference/rolling.cpp',tofile='reference_sweep/rolling.cpp')))
for n in ['rolling_model.cpp','rolling_model.hpp','strategy.hpp','replay_helpers.hpp']:
    assert (folder/n).read_bytes()==(parent/n).read_bytes(),n
print('Only configurable reference geometry, guided Heading coefficient and metadata changed; inherited safety blocks equal.')
