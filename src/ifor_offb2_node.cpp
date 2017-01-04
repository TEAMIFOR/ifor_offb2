#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mansoo_offboard_node");

    ros::Rate rate_50(50.0);

    rate_50.sleep();

    //land();
    mavros_msgs::CommandTOL land_cmd;
    
    land_cmd.request.altitude = 20;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;
    if(land_client_.call(land_cmd)){
        ROS_INFO("land_cmd send ok %d", land_cmd.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }

    return 0;
}