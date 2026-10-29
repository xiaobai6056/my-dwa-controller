"""Integrate only into the copied offline runner; retain every inherited hard gate."""
import pathlib,difflib
root=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
p=root/'source/experiment/rolling/rolling.cpp';old=p.read_text();new=old
def change(a,b):
    global new
    assert new.count(a)==1,a
    new=new.replace(a,b)
change('#include "strategy.hpp"','#include "strategy.hpp"\n#include "reference.hpp"\n#include "my_dwa_controller/crossing_yield.hpp"')
change('const auto weights=strategy(scheme);','const auto weights=strategy("A");\n  if (scheme!="A" && scheme!="B") {return 2;}')
change('    Json motion_starts=Json::array();Json totals=Json::object();',
       '    Json motion_starts=Json::array();Json totals=Json::object();\n    CrossingYield yield;ReferenceGuide guide;')
change('plant.time<14-1e-8','plant.time<16-1e-8')
change('''        scan=observation(source_time);z=exclusion(source_time);
        auto boxes=walls;boxes.push_back(actor(source_time));map=makeMap(plant.pose,boxes);last_scan=source_time;''',
'''        scan=observation(source_time);scan.previous_stamp_ns=last_scan<0?0:sourceNs(last_scan);
        scan.obstacles[0].previous_cluster_index=last_scan<0?-1:0;
        if (last_scan<0) {z.start(scan,0,1,0);}else {z.observe(scan,sourceNs(t));}
        auto boxes=walls;boxes.push_back(actor(source_time));fillCurrentRays(scan,plant.pose,boxes);
        map=makeMap(plant.pose,boxes);last_scan=source_time;''')
change('      const auto fallback=predictor.predict(plant.pose,plant.velocity,{},0);','')
change('      auto local=extractForwardPath(path,projection,2.0);',
'''      auto local=extractForwardPath(path,projection,2.0);
      const auto fallback=predictor.predict(plant.pose,plant.velocity,{},0);
      const auto risk_observation=yield.update(&scan,local,plant.pose,plant.velocity,
        fallback.trajectory,sourceNs(t),false);
      if (risk_observation.triggered) {yield.latchHold(true);guide=ReferenceGuide{};}
      if (scheme=="B") {guide.observe(&scan,z,plant.pose,walls,sourceNs(t));}''')
anchor='''      Json best_turn=nullptr,best_rotation=nullptr;int turns=0,rotations=0,rear_reaches=0;'''
change(anchor,'''      const auto ordinary_ranked=ranked;
      int ordinary_position=-1;
      for (size_t i=0;i<ordinary_ranked.size();++i) {
        const auto & c=candidates[ordinary_ranked[i].index];const auto cmd=c.nominal.trajectory.command;
        const auto qb=queuedBackup(plant,cmd);const double reserve=std::max(std::abs(plant.velocity.v),std::abs(cmd.v))*.05;
        if (!z.clear(qb,reserve) || clearance(qb,t,walls)<.05 || !evaluateTrajectory(qb,*map).collision_free) {continue;}
        ordinary_position=i;break;
      }
      const auto reference_point=guide.phase==ReferencePhase::Locked?guide.target:guide.proposal;
      if (scheme=="B" && guide.valid && reference_point && guide.phase!=ReferencePhase::Done) {
        for (auto & r:ranked) {r.score=referenceScore(r.score,candidates[r.index].nominal.trajectory,plant.pose,*reference_point);}
        std::stable_sort(ranked.begin(),ranked.end(),[&](const Ranked & a,const Ranked & b){
          if (a.score.total_cost!=b.score.total_cost) {return a.score.total_cost<b.score.total_cost;}
          return std::abs(candidates[a.index].nominal.trajectory.command.omega)<std::abs(candidates[b.index].nominal.trajectory.command.omega);
        });
      }
'''+anchor)
anchor='      Json selected_rear=nullptr;'
change(anchor,'''      auto risk=yield.update(&scan,local,plant.pose,plant.velocity,fallback.trajectory,
        sourceNs(t),ordinary_position>=0);
      risk.triggered=risk.triggered || risk_observation.triggered;
      risk.released=risk.released || risk_observation.released;
      risk.clear=risk_observation.clear;
      const bool ordinary_admitted=ordinary_position>=0;
      bool recovers_path=false;
      if (ordinary_admitted) {
        const auto & tr=candidates[ordinary_ranked[ordinary_position].index].nominal.trajectory;
        recovers_path=tr.command.v>0 && distanceToPath(tr.poses.back(),path).value()<=distanceToPath(plant.pose,path).value()+1e-9;
      }
      if (scheme=="B" && !risk.hold) {guide.riskReleasedWhileWaiting();}
      if (scheme=="B") {guide.complete(plant.pose,z,ordinary_admitted,recovers_path);}
      bool useful=false;
      if (selected_rank>=0 && reference_point && guide.valid) {
        const auto & tr=candidates[ranked[selected_rank].index].nominal.trajectory;
        const double desired=guide.desiredYaw(plant.pose),tau=2*std::acos(-1.);
        useful=(std::abs(std::remainder(desired-tr.poses.back().theta,tau))<std::abs(std::remainder(desired-plant.pose.theta,tau))-1e-9 ||
          z.across(tr.poses.back())<z.across(plant.pose)-1e-6) && (tr.command.v!=0 || tr.command.omega!=0);
      }
      const bool stopped=std::abs(plant.velocity.v)<.01 && std::abs(plant.velocity.omega)<.05;
      const bool permission=scheme=="B" && guide.approve(stopped,bypassActionAdmitted(selected_rank,selected),useful,fallback_ok);
      const bool risk_hold=yield.latchHold(risk.hold);
      if (scheme=="A" || guide.phase==ReferencePhase::Done) {
        ranked=ordinary_ranked;selected_rank=ordinary_position;
        if (ordinary_admitted) {
          const auto & tr=candidates[ordinary_ranked[ordinary_position].index].nominal.trajectory;
          selected=tr.command;const auto & s=ordinary_ranked[ordinary_position].score;
          selected_score={{"total",s.total_cost},{"goal",s.goal_cost},{"path",s.path_cost},{"speed",s.speed_cost},
            {"obstacle",s.obstacle_cost},{"heading",s.heading_cost}};
        }else {selected={};selected_score=nullptr;}
      }
      const auto proposed_command=selected;
      const bool hold_output=(scheme=="B" && guide.phase!=ReferencePhase::Done)?!permission:risk_hold;
      if (hold_output) {selected={};}
'''+anchor)
change('        selected_rear=reaches;','        selected_rear=reaches && !hold_output;')
change('      Json state={{"scheme",scheme}',
'''      Json ref_state={{"phase",int(guide.phase)},{"proposal",guide.proposal?Json::array({guide.proposal->x,guide.proposal->y}):Json(nullptr)},
        {"target",guide.target?Json::array({guide.target->x,guide.target->y}):Json(nullptr)},
        {"votes",guide.votes},{"lock_stamp",guide.lock_stamp},{"valid",guide.valid},
        {"permission",permission},{"aborted",guide.aborted},{"released",guide.released},
        {"desired_yaw",reference_point?Json(guide.desiredYaw(plant.pose)):Json(nullptr)}};
      Json state={{"reference",ref_state},{"proposed_candidate",{proposed_command.v,proposed_command.omega}},
        {"command_held",hold_output},{"ordinary_admitted",ordinary_admitted},{"ordinary_recovers_path",recovers_path},
        {"risk",{{"pending",risk.hold},{"hold",risk_hold},{"released",risk.released},{"triggered",risk.triggered},
          {"occupied",risk.occupied},{"clear",risk.clear},{"clear_scans",risk.clear_scans},{"entry",risk.entry},{"exit",risk.exit}}},
        {"scheme",scheme}''')
# Guard the inherited candidate safety pipeline and executed geometry checks.
for a,b in [('      for (const auto & cmd:commands)', '      struct Ranked'),
            ('      Velocity2D selected{}','      auto risk=yield.update'),
            ('      if (!fallback_ok)','      if (entry<0')]:
    old_b='      Json selected_rear' if b=='      auto risk=yield.update' else b
    before=old[old.index(a):old.index(old_b)];after=new[new.index(a):new.index(b)]
    if a=='      for (const auto & cmd:commands)':
        before=before.replace('      const auto fallback=predictor.predict(plant.pose,plant.velocity,{},0);','')
    assert before==after,a
p.write_text(new)
(root/'runner.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='ablation/rolling.cpp',tofile='reference/rolling.cpp')))
c=root/'source/experiment/rolling/CMakeLists.txt';c.write_text(c.read_text()+'\ntarget_link_libraries(rolling reference_model)\n')
print('Integrated new reference adapter; inherited candidate and executed safety blocks unchanged.')
