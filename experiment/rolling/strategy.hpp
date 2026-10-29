#pragma once
#include <string>
#include <stdexcept>
#include "my_dwa_controller/trajectory_scorer.hpp"
inline my_dwa_controller::ScoreWeights strategy(const std::string & scheme)
{
  my_dwa_controller::ScoreWeights w;
  if (scheme=="A") {return w;}
  w.heading=0;
  if (scheme=="B") {return w;}
  w.path=0;
  if (scheme=="C") {return w;}
  w.speed=.2;
  if (scheme=="D") {return w;}
  if (scheme=="E") {w.speed=.4;return w;}
  if (scheme=="F") {w.goal=.3;return w;}
  if (scheme=="G") {w.goal=0;return w;}
  throw std::invalid_argument("Unknown A-G ablation scheme");
}
