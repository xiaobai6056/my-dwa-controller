import pathlib,shutil
r=pathlib.Path('/home/user/ros2_experiments/polyline_bypass_20261029');p=r/'source/experiment/rolling'
h=p/'reference.hpp';s=h.read_text().replace('  double extra_clearance=', '  bool polyline=false;\n  std::optional<Pose2D> proposal_entry,entry,anchor,proposal_anchor;\n  std::vector<Pose2D> line() const {return {};}\n  double extra_clearance=')
s=s.replace('bool ordinary_admitted,bool recovers_path);','bool ordinary_admitted,bool recovers_path,bool reliably_clear=false);');h.write_text(s)
c=p/'reference.cpp';s=c.read_text().replace('bool admitted,bool recovers_path)\n','bool admitted,bool recovers_path,bool reliably_clear)\n');c.write_text(s)
shutil.copy2('/path/to/local-experiment-inputs/polyline_bypass_20261029/test_polyline.cpp',p/'test_polyline.cpp')
c=p/'CMakeLists.txt';c.write_text(c.read_text()+'''\nadd_executable(test_polyline test_polyline.cpp)
target_link_libraries(test_polyline reference_model GTest::gtest_main)
add_test(NAME polyline COMMAND test_polyline)
''')
