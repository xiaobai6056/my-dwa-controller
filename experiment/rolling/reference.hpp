#pragma once
#include <optional>
#include "rolling_model.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
enum class ReferencePhase {Waiting,Locked,Done};
struct ReferenceGuide
{
  double extra_clearance=.10,forward_offset=.10;
  ReferencePhase phase=ReferencePhase::Waiting;
  std::optional<Pose2D> proposal,target;
  int votes=0,side=0;
  int64_t last_stamp=0,lock_stamp=0;
  bool fresh=false,valid=false,aborted=false,released=false;
  void observe(const ScanSnapshot *,const Exclusion &,const Pose2D &,const std::vector<Box> &,int64_t);
  bool approve(bool stopped,bool admitted,bool useful,bool fallback_safe);
  bool complete(const Pose2D &,const Exclusion &,bool ordinary_admitted,bool recovers_path);
  void abort();
  void riskReleasedWhileWaiting();
  double desiredYaw(const Pose2D &) const;
};
my_dwa_controller::ScoreResult referenceScore(
  const my_dwa_controller::ScoreResult &,const Trajectory &,const Pose2D &,const Pose2D &,double heading_multiplier=1.);
void fillCurrentRays(ScanSnapshot &,const Pose2D &,const std::vector<Box> &);
bool bypassActionAdmitted(int selected_rank,const Velocity2D & command);
