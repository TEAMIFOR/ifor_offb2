#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state_;
void cbState(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifor_offb2_node");

	
	ros::Rate rate_50(50.0);
    ros::Rate rate_20(20.0);
        


    ros::NodeHandle n;
    ros::ServiceClient arming_client_ = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient disarming_client_ = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/disarming");
    ros::ServiceClient set_mode_client_ = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Subscriber state_sub_ = n.subscribe<mavros_msgs::State>("mavros/state", 10, cbState);
	
	const std_msgs::String::ConstPtr& msg1;
	
    ros::Subscriber move_x_sub_ = n.subscribe("ifor_drone/x_pos", 10, msg1 );
    std::string::size_type sz;   // alias of size_t
    int pos_x = std::stoi (msg1->data, &sz);
	targetPose_.pose.position.x = pos_x;

    const std_msgs::String::ConstPtr& msg2;
    
    ros::Subscriber move_y_sub_ = n.subscribe("ifor_drone/y_pos", 10, msg2 );
    targetPose_.pose.position.y = pos_y;
    int pos_y = std::stoi (msg2->data, &sz);

    const std_msgs::String::ConstPtr& msg3;

    ros::Subscriber move_z_sub_ = n.subscribe("ifor_drone/z_pos", 10, msg3 );
    int pos_z = std::stoi (msg3->data, &sz);
    targetPose_.pose.position.z = pos_z;

    ros::Publisher local_pos_pub_ = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    // wait for FCU connection
    while(ros::ok() && current_state_.connected){
        ros::spinOnce();
        rate_20.sleep();
    }     


    geometry_msgs::PoseStamped mPose[5];
    mPose[0].pose.position.x = 3;
    mPose[0].pose.position.y = 3;
    mPose[0].pose.position.z = 5;
    mPose[1].pose.position.x = -3;
    mPose[1].pose.position.y = 3;
    mPose[1].pose.position.z = 5;
    mPose[2].pose.position.x = 3;
    mPose[2].pose.position.y = -3;
    mPose[2].pose.position.z = 5;
    mPose[3].pose.position.x = -3;
    mPose[3].pose.position.y = -3;
    mPose[3].pose.position.z = 5;
    mPose[4].pose.position.x = 0;
    mPose[4].pose.position.y = 0;
    mPose[4].pose.position.z = 5;
    //  geometry_msgs::PoseStamped pose;
    for (int count = 0; count < 1 && ros::ok(); count++)
    {
        drone.targetPose_.pose.position.x = mPose[0].pose.position.x;
        drone.targetPose_.pose.position.y = mPose[0].pose.position.y;
        drone.targetPose_.pose.position.z = mPose[0].pose.position.z;
        int nPosIndex = 0;
        while (ros::ok())
        {
            if (!(abs(drone.targetPose_.pose.position.x - drone.localPose_.pose.position.x) < 0.03 &&
                  abs(drone.targetPose_.pose.position.y - drone.localPose_.pose.position.y) < 0.03 &&
                  abs(drone.targetPose_.pose.position.z - drone.localPose_.pose.position.z) < 0.03) )
            {


                drone.moveToPos();
            


            }else{
                    if (++nPosIndex == 5)
                    break;
                    drone.targetPose_.pose.position.x = mPose[nPosIndex].pose.position.x;
                    drone.targetPose_.pose.position.y = mPose[nPosIndex].pose.position.y;
                    drone.targetPose_.pose.position.z = mPose[nPosIndex].pose.position.z;
            }
            rate_50.sleep();
        }
    }

    rate_50.sleep();

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