#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include "rolling_model.hpp"
#include "my_dwa_controller/motion_predictor.hpp"
using Json=nlohmann::json;
int main(int argc,char ** argv)
{
  if (argc!=2) {return 2;}
  const std::string root=argv[1];Json trials;std::ifstream(root+"/results/summary.json")>>trials;
  MotionConfig config;config.angular_acceleration=3.2;MotionPredictor predictor(config);
  Json result=Json::array();
  for (const auto & trial:trials) {
    const double start=trial["start"];std::ostringstream name;name<<std::fixed<<std::setprecision(2)<<start;
    std::ifstream file(root+"/results/start_"+name.str()+".jsonl");std::string line;Json first=nullptr;
    while (std::getline(file,line)) {
      const auto r=Json::parse(line);auto p=point(r["pose"][0],r["pose"][1]);p.theta=r["pose"][2];
      const Velocity2D measured{r["measured"][0],r["measured"][1]},selected{r["selected"][0],r["selected"][1]};
      const auto trajectory=predictor.predict(p,measured,selected,1.7).trajectory;
      const auto z=exclusion(r["scan_t"]);
      const double reserve=std::max(std::abs(measured.v),std::abs(selected.v))*.05;
      bool reaches=false;
      for (const auto & q:trajectory.poses) {
        if (q.x>=z.lo && q.x<=z.hi && q.y<z.rear) {reaches=true;break;}
      }
      if (r["selected_rank"].get<int>()>=0 && reaches && z.clear(trajectory,reserve)) {
        first={{"t",r["t"]},{"selected",r["selected"]},{"pose",r["pose"]},{"score",r["score"]}};break;
      }
    }
    result.push_back({{"start",start},{"first_selected_rear_reaching_nominal",first}});
  }
  std::ofstream(root+"/selected_candidate_audit.json")<<result.dump(2);std::cout<<result.dump(2)<<'\n';
}
