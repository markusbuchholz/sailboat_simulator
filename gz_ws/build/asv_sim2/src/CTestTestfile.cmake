# CMake generated Testfile for 
# Source directory: /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src
# Build directory: /home/blueboat_sitl/gz_ws/build/asv_sim2/src
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(UNIT_LiftDragModel_TEST "/home/blueboat_sitl/gz_ws/build/asv_sim2/bin/UNIT_LiftDragModel_TEST" "--gtest_output=xml:/home/blueboat_sitl/gz_ws/build/asv_sim2/test_results/UNIT_LiftDragModel_TEST.xml")
set_tests_properties(UNIT_LiftDragModel_TEST PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/usr/share/cmake/gz-cmake3/cmake3/GzBuildTests.cmake;137;add_test;/home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src/CMakeLists.txt;44;gz_build_tests;/home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src/CMakeLists.txt;0;")
add_test(check_UNIT_LiftDragModel_TEST "/usr/bin/python3.10" "/usr/share/gz/gz-cmake3/tools/check_test_ran.py" "/home/blueboat_sitl/gz_ws/build/asv_sim2/test_results/UNIT_LiftDragModel_TEST.xml")
set_tests_properties(check_UNIT_LiftDragModel_TEST PROPERTIES  _BACKTRACE_TRIPLES "/usr/share/cmake/gz-cmake3/cmake3/GzBuildTests.cmake;157;add_test;/home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src/CMakeLists.txt;44;gz_build_tests;/home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src/CMakeLists.txt;0;")
subdirs("systems")
