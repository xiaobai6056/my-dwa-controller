import pathlib,shutil
root=pathlib.Path('/home/user/ros2_experiments/reference_sweep_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/reference_sweep_20261029')
folder=root/'source/experiment/rolling'
# API stub: new settings exist but are intentionally unused until red is observed.
p=folder/'reference.hpp';s=p.read_text().replace('  ReferencePhase phase=','  double extra_clearance=.10,forward_offset=.10;\n  ReferencePhase phase=')
s=s.replace('const Trajectory &,const Pose2D &,const Pose2D &);','const Trajectory &,const Pose2D &,const Pose2D &,double heading_multiplier=1.);')
p.write_text(s)
p=folder/'reference.cpp';s=p.read_text().replace('const Pose2D & target)\n','const Pose2D & target,double heading_multiplier)\n');p.write_text(s)
shutil.copy2(local/'test_reference_tuning.cpp',folder/'test_reference_tuning.cpp')
p=folder/'CMakeLists.txt';p.write_text(p.read_text()+'''\nadd_executable(test_reference_tuning test_reference_tuning.cpp)
target_link_libraries(test_reference_tuning reference_model GTest::gtest_main)
add_test(NAME reference_tuning COMMAND test_reference_tuning)
''')
shutil.copy2(local/'build.sh',root/'build.sh')
