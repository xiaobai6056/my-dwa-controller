import pathlib,shutil
r=pathlib.Path('/home/user/ros2_experiments/polyline_bypass_20261029');p=r/'source/experiment/rolling'
shutil.copy2('/path/to/local-experiment-inputs/polyline_bypass_20261029/failure_check.cpp',p/'failure_check.cpp')
c=p/'CMakeLists.txt';c.write_text(c.read_text()+'''\nadd_executable(failure_check failure_check.cpp)
target_link_libraries(failure_check reference_model my_dwa_controller::motion_predictor my_dwa_controller::velocity_sampler my_dwa_controller::trajectory_scorer my_dwa_controller::obstacle_evaluator my_dwa_controller::dynamic_trajectory_evaluator nlohmann_json::nlohmann_json)
''')
