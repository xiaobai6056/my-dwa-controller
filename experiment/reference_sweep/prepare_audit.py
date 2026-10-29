"""Parameterize the existing read-only score replay; no rolling execution."""
import pathlib
root=pathlib.Path('/home/user/ros2_experiments/reference_sweep_20261029')
p=root/'source/experiment/rolling/audit_reference.cpp';s=p.read_text()
s=s.replace('int main()','int main(int argc,char ** argv)')
s=s.replace('  const std::string root="/home/user/ros2_experiments/locked_reference_20261029";\n  Json input;std::ifstream(root+"/input.json")>>input;',
'''  if (argc!=4) {return 2;}
  Json input;std::ifstream(argv[1])>>input;
  const double heading_multiplier=input["reference_parameters"].value("heading_multiplier",1.);''')
s=s.replace('plant.time=4;','plant.time=4;')
s=s.replace('std::ifstream file(root+"/results/phase1/B_t4/start_4.00.jsonl");','std::ifstream file(argv[2]);')
s=s.replace('    const auto r=Json::parse(line);const double t=r["t"],source=r["scan_t"];',
'''    const auto r=Json::parse(line);const double t=r["t"],source=r["scan_t"];
    if (result.empty() && last_scan<0) {plant.time=t;}''')
s=s.replace('for (double key:{5.,6.,8.,8.4,8.6})','for (double key:{5.,6.,7.,8.,8.4,8.6,8.8,9.})')
s=s.replace('    if (chosen) {','    if (chosen && r["guided_scoring"].get<bool>()) {')
s=s.replace('      const auto target=point(r["reference"]["target"][0],r["reference"]["target"][1]);',
'''      const auto ref=r["reference"]["target"].is_null()?r["reference"]["proposal"]:r["reference"]["target"];
      const auto target=point(ref[0],ref[1]);''')
s=s.replace('risk.available && risk.conflicts.empty();','risk.available && risk.conflicts.empty() && !filter.observedGeometryCollision(n.trajectory,prepared) && !filter.observedGeometryCollision(b.trajectory,prepared);')
s=s.replace('plant.pose,target);','plant.pose,target,heading_multiplier);')
s=s.replace('choices["selected"]=examine(r["selected"]);',
'''choices["selected_proposal"]=examine(r["proposed_candidate"]);
      choices["actual_command"]=r["selected"];choices["held"]=r["command_held"];''')
s=s.replace('choices["selected"]["guided_terms"]','choices["selected_proposal"]["guided_terms"]')
s=s.replace('std::ofstream(root+"/score_audit.json")','std::ofstream(argv[3])')
p.write_text(s)
print('Read-only replay accepts trial input/trace/output and verifies effective Heading score parity.')
