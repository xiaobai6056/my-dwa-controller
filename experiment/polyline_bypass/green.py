import pathlib,difflib
r=pathlib.Path('/home/user/ros2_experiments/polyline_bypass_20261029');p=r/'source/experiment/rolling'
h=p/'reference.hpp';s=h.read_text().replace('std::vector<Pose2D> line() const {return {};}','''std::vector<Pose2D> line() const {
    if (!polyline || !valid || phase==ReferencePhase::Done) {return {};}
    const auto a=phase==ReferencePhase::Locked?anchor:proposal_anchor;
    const auto b=phase==ReferencePhase::Locked?entry:proposal_entry;
    const auto c=phase==ReferencePhase::Locked?target:proposal;
    return a && b && c?std::vector<Pose2D>{*a,*b,*c}:std::vector<Pose2D>{};
  }''');h.write_text(s)
c=p/'reference.cpp';s=c.read_text().replace('#include "reference.hpp"','#include "reference.hpp"\n#include "my_dwa_controller/path_geometry.hpp"')
s=s.replace('  proposal.reset();','  proposal.reset();proposal_entry.reset();proposal_anchor.reset();',1)
s=s.replace('corner=world(z.along(robot),q_target);','corner=world(polyline?std::min(z.along(robot),z.lo-extra_clearance):z.along(robot),q_target);')
s=s.replace('  proposal=target_point;','  proposal=target_point;if (polyline) {proposal_entry=corner;proposal_anchor=robot;}')
s=s.replace('target=proposal;phase=','target=proposal;entry=proposal_entry;anchor=proposal_anchor;phase=')
s=s.replace('      z.along(p)<=z.hi || z.across(p)>=z.rear)', '      (!(polyline && reliably_clear) && (z.along(p)<=z.hi || z.across(p)>=z.rear)))')
s=s.replace('phase=ReferencePhase::Done;target.reset();proposal.reset();votes=0;released=true;', 'phase=ReferencePhase::Done;target.reset();proposal.reset();entry.reset();anchor.reset();proposal_entry.reset();proposal_anchor.reset();votes=0;released=true;')
s=s.replace('{phase=ReferencePhase::Waiting;target.reset();proposal.reset();votes=0;', '{phase=ReferencePhase::Waiting;target.reset();proposal.reset();entry.reset();anchor.reset();proposal_entry.reset();proposal_anchor.reset();votes=0;')
s=s.replace('  const auto ref=phase==ReferencePhase::Locked?target:proposal;', '''  if (polyline) {
    const auto path=line();if (path.empty()) {return p.theta;}
    const auto projection=projectOntoPath(p,path).value();const auto ahead=lookAheadOnPath(path,projection,.5).value();
    if (ahead.distance>1e-9) {return std::atan2(ahead.point.y-p.y,ahead.point.x-p.x);}
    return ahead.forward_heading.value_or(p.theta);
  }
  const auto ref=phase==ReferencePhase::Locked?target:proposal;''')
c.write_text(s)
c=p/'rolling.cpp';old=c.read_text();s=old
s=s.replace('  const double heading_multiplier=', '  const bool polyline=reference_parameters.value("polyline",false);\n  const double heading_multiplier=')
s=s.replace('      guide.extra_clearance=','      guide.polyline=polyline;guide.extra_clearance=')
s=s.replace('''for (auto & r:ranked) {r.score=referenceScore(r.score,candidates[r.index].nominal.trajectory,plant.pose,*reference_point,heading_multiplier);}''','''for (auto & r:ranked) {
          if (polyline) {
            const auto full=guide.line();const auto ref=extractForwardPath(full,projectOntoPath(plant.pose,full).value(),2.0);
            auto tuned=weights;tuned.heading*=heading_multiplier;
            r.score=TrajectoryScorer(.26,tuned).score(candidates[r.index].nominal.trajectory,ref,candidates[r.index].cost,true);
          } else {r.score=referenceScore(r.score,candidates[r.index].nominal.trajectory,plant.pose,*reference_point,heading_multiplier);}
        }''')
s=s.replace('guide.complete(plant.pose,z,ordinary_admitted,recovers_path);','guide.complete(plant.pose,z,ordinary_admitted,recovers_path,polyline && !risk.hold);')
s=s.replace('      Json ref_state=', '''      Json line_state=Json::array();for (const auto & q:guide.line()) {line_state.push_back({q.x,q.y});}
      Json ref_state=''')
s=s.replace('Json state={{"reference_parameters",','Json state={{"polyline",line_state},{"reference_parameters",')
for a,b in [('      for (const auto & cmd:commands)','      const auto ordinary_ranked'),('      Velocity2D selected{}','      auto risk=yield.update'),('      if (!fallback_ok)','      if (entry<0')]:
 assert old[old.index(a):old.index(b)]==s[s.index(a):s.index(b)]
c.write_text(s)
(r/'runner.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True))))
