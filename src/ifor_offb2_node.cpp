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
    ros::Rate rate_20(20.0);

    ros::Publisher local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher pub_att_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
    ros::Publisher pub_thr_ = nh_.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10); 
    ros::ServiceClient arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient disarming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/disarming");
    ros::ServiceClient set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    
    while(ros::ok() && current_state_.connected){
        ros::spinOnce();
        rate_20.sleep();
    }


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