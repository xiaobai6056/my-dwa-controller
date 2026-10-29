"""Only extend the experimental runner's logging/horizon; keep gates intact."""
import pathlib,shutil,difflib
root=pathlib.Path('/home/user/ros2_experiments/score_ablation_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/score_ablation_20261029')
folder=root/'source/experiment/rolling'
for name in ['strategy.hpp','test_strategy.cpp']:shutil.copy2(local/name,folder/name)
old=(folder/'rolling.cpp').read_text();new=old
def replace(a,b):
    global new
    assert new.count(a)==1,a
    new=new.replace(a,b)
replace('#include "rolling_model.hpp"','#include "rolling_model.hpp"\n#include "strategy.hpp"')
replace('  if (argc<3) {return 2;}','  if (argc<4) {return 2;}\n  const std::string scheme=argv[3];const auto weights=strategy(scheme);')
replace('  TrajectoryScorer scorer(.26);DynamicFilter filter(.22,1.7,.05);','  TrajectoryScorer scorer(.26,weights);DynamicFilter filter(.22,1.7,.05);')
replace('  if (argc>3) {starts.clear();for (int i=3;i<argc;++i) {starts.push_back(std::stod(argv[i]));}}',
        '  if (argc>4) {starts.clear();for (int i=4;i<argc;++i) {starts.push_back(std::stod(argv[i]));}}')
replace('      Json state={{"t",t}', '''      Json selected_rear=nullptr;
      if (selected_rank>=0) {
        const auto & tr=candidates[ranked[selected_rank].index].nominal.trajectory;
        bool reaches=false;for (const auto & q:tr.poses) {if (q.x>=z.lo && q.x<=z.hi && q.y<z.rear) {reaches=true;break;}}
        selected_rear=reaches;
      }
      Json weighted=nullptr;
      if (!selected_score.is_null()) {
        weighted={{"heading",weights.heading*selected_score["heading"].get<double>()},
          {"path",weights.path*selected_score["path"].get<double>()},{"goal",weights.goal*selected_score["goal"].get<double>()},
          {"speed",weights.speed*selected_score["speed"].get<double>()},{"obstacle",weights.obstacle*selected_score["obstacle"].get<double>()}};
      }
      Json state={{"scheme",scheme},{"selected_rear_reaching",selected_rear},{"weighted_score",weighted},{"t",t}''')
replace('      if (entry>=0 && passed(plant.pose,z)) {pass=plant.time;failure="";break;}',
        '      if (pass<0 && entry>=0 && passed(plant.pose,z)) {pass=plant.time;failure="";}')
(folder/'rolling.cpp').write_text(new)
cmake=folder/'CMakeLists.txt';s=cmake.read_text();s+='\nadd_executable(test_strategy test_strategy.cpp)\ntarget_link_libraries(test_strategy my_dwa_controller::trajectory_scorer GTest::gtest_main)\nadd_test(NAME strategy_weights COMMAND test_strategy)\n';cmake.write_text(s)
(root/'runner.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='parent/rolling.cpp',tofile='experiment/rolling.cpp')))
# No helper, sampler, predictor, scoring implementation or gate block is changed.
assert old[:old.index('int main(')] .replace('#include "rolling_model.hpp"','#include "rolling_model.hpp"\n#include "strategy.hpp"') == new[:new.index('int main(')]
for a,b in [('      for (const auto & cmd:commands)', '      struct Ranked'),
            ('      Velocity2D selected{}','      Json selected_rear'),
            ('      if (!fallback_ok)', '      if (entry<0')]:
    oldend='      Json state=' if b=='      Json selected_rear' else b
    assert old[old.index(a):old.index(oldend)]==new[new.index(a):new.index(b)]
print('Prepared copied runner: only weights, telemetry and common pass-recording horizon extension')
