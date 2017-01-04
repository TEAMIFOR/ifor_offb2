# ifor_offb2

px4 sim environment

T1> cd catkin_ws/src/Firmware/ 
T1> make posix_sitl_default gazebo

T2> cd catkin_ifor_ws
T2> source devel/setup.bash
T2> roslaunch ifor_offb2 ifor_offb2_launch.launch

//off record extern setmode and arming, symlink error fix(=>TODO)
T3> rosrun mavros mavsis mode -c OFFBOARD
T3> rosrun mavros mavsafety arm