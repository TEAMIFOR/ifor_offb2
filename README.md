# OFFBOARD DRONE CONTROL (exec w/o ros_control)

<h3>COMMENTS:</h3>
Sim for offboard control without controller node (no PID).

Mission location is being set by a 3 coord mission_pos_arr, change it to alter mission, deafult set to form "Z" shape

Ros event check is changed to thread processing because event received from ros is not processed when entering console (singular nature of code execution) 

<h3>INSTRUCTIONS:</h3>
"make posix_sitl_default gazebo" then build and run this package

