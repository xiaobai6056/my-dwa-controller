"""Add the offline reference unit target without modifying production modules."""
import pathlib,shutil
root=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/locked_reference_20261029')
folder=root/'source/experiment/rolling'
for n in ['reference.hpp','reference.cpp','test_reference.cpp']:shutil.copy2(local/n,folder/n)
p=folder/'CMakeLists.txt';s=p.read_text()
s+='''
find_library(CONTROLLER_LIBRARY NAMES my_dwa_controller
  PATHS "${my_dwa_controller_DIR}/../../../lib" REQUIRED NO_DEFAULT_PATH)
add_library(reference_model reference.cpp)
target_link_libraries(reference_model rolling_model ${CONTROLLER_LIBRARY} my_dwa_controller::trajectory_scorer)
add_executable(test_reference test_reference.cpp)
target_link_libraries(test_reference reference_model GTest::gtest_main)
add_test(NAME reference_lifecycle COMMAND test_reference)
'''
p.write_text(s)
