# boat_pid_controller

error on Dynamic reconfigure

####
#### Running command: "make -j6 -l6" in "/home/renan/catkin_ws/build"
####
[  0%] Built target _kingfisher_msgs_generate_messages_check_deps_Helm
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_UpdateStatus
[  0%] Built target _kingfisher_msgs_generate_messages_check_deps_Sense
[  0%] Built target _kingfisher_msgs_generate_messages_check_deps_Course
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_CalibrateArmEnable
[  0%] Built target _kingfisher_msgs_generate_messages_check_deps_Drive
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_TareEnable
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_UpdateSource
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_UpdateSources
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_CalibrateArmData
[  0%] Built target _baxter_maintenance_msgs_generate_messages_check_deps_TareData
[  0%] Built target std_msgs_generate_messages_nodejs
[  0%] Built target std_msgs_generate_messages_lisp
Scanning dependencies of target _kingfisher_control_generate_messages_check_deps_Course
[  0%] Built target std_msgs_generate_messages_eus
Scanning dependencies of target _kingfisher_control_generate_messages_check_deps_Drive
[  0%] Generating dynamic reconfigure files from cfg/YawDynamic.cfg: /home/renan/catkin_ws/devel/include/kingfisher_control/YawDynamicConfig.h /home/renan/catkin_ws/devel/lib/python2.7/dist-packages/kingfisher_control/cfg/YawDynamicConfig.py
/home/renan/catkin_ws/build/boat_pid_controller/setup_custom_pythonpath.sh: 5: exec: /home/renan/catkin_ws/src/boat_pid_controller/cfg/YawDynamic.cfg: Permission denied
boat_pid_controller/CMakeFiles/kingfisher_control_gencfg.dir/build.make:65: recipe for target '/home/renan/catkin_ws/devel/include/kingfisher_control/YawDynamicConfig.h' failed
make[2]: *** [/home/renan/catkin_ws/devel/include/kingfisher_control/YawDynamicConfig.h] Error 126
[  0%] Built target _vesc_msgs_generate_messages_check_deps_VescStateStamped
CMakeFiles/Makefile2:3434: recipe for target 'boat_pid_controller/CMakeFiles/kingfisher_control_gencfg.dir/all' failed
make[1]: *** [boat_pid_controller/CMakeFiles/kingfisher_control_gencfg.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
[  0%] Built target _vesc_msgs_generate_messages_check_deps_VescState
[  0%] Built target _kingfisher_control_generate_messages_check_deps_Course
[  0%] Built target _kingfisher_control_generate_messages_check_deps_Drive
[  0%] Built target _kingfisher_control_generate_messages_check_deps_PidDiagnose
Makefile:138: recipe for target 'all' failed
make: *** [all] Error 2
Invoking "make -j6 -l6" failed
