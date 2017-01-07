#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


void state_callb (const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

class ifor_drone
{

public:
    ifor_drone() {}
    ~ifor_drone() {}

public:
    
    ros::NodeHandle     			n;
    ros::Subscriber     			state_sub;
    ros::Subscriber     			location_sub;
    ros::Subscriber     			x_coord_sub;
    ros::Subscriber     			y_coord_sub;
    ros::Subscriber     			z_coord_sub;
    ros::Publisher      			local_pos_pub;
    ros::ServiceClient 				land_cl;
    mavros_msgs::State  			current_state;
    geometry_msgs::PoseStamped      target_loc;
    geometry_msgs::PoseStamped      local_loc;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ifor_offb2_node");

	
	ros::Rate rate_50(50.0);
    ros::Rate rate_20(20.0);
        
    ifor_drone ifor_drone_obj; 


    ros::ServiceClient arming_cl = ifor_drone_obj.n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient disarming_cl = ifor_drone_obj.n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/disarming");
    ros::ServiceClient set_mode_cl = ifor_drone_obj.n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ifor_drone_obj.state_sub = ifor_drone_obj.n.subscribe<mavros_msgs::State>("mavros/state", 10, state_callb);
	ifor_drone_obj.local_pos_pub = ifor_drone_obj.n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ifor_drone_obj.land_cl = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

	const std_msgs::String::ConstPtr& msg1;
	
    ros::Subscriber ifor_drone_obj.x_coord_sub = n.subscribe("ifor_drone/x_pos", 10, msg1 );
    std::string::size_type sz;   // alias of size_t
    int pos_x = std::stoi (msg1->data, &sz);
	ifor_drone_obj.target_loc.pose.position.x = pos_x;

    const std_msgs::String::ConstPtr& msg2;
    
    ros::Subscriber ifor_drone_obj.y_coord_sub = n.subscribe("ifor_drone/y_pos", 10, msg2 );
    ifor_drone_obj.target_loc.pose.position.y = pos_y;
    int pos_y = std::stoi (msg2->data, &sz);

    const std_msgs::String::ConstPtr& msg3;

    ros::Subscriber ifor_drone_obj.z_coord_sub = n.subscribe("ifor_drone/z_pos", 10, msg3 );
    int pos_z = std::stoi (msg3->data, &sz);
    ifor_drone_obj.target_loc.pose.position.z = pos_z;

    

    // wait for FCU connection
    while(ros::ok() && ifor_drone_obj.current_state.connected){
        ros::spinOnce();
        rate_20.sleep();
    }     


    geometry_msgs::PoseStamped mission_loc_arr[5];
    mission_loc_arr[0].pose.position.x = 3;
    mission_loc_arr[0].pose.position.y = 3;
    mission_loc_arr[0].pose.position.z = 5;
    mission_loc_arr[1].pose.position.x = -3;
    mission_loc_arr[1].pose.position.y = 3;
    mission_loc_arr[1].pose.position.z = 5;
    mission_loc_arr[2].pose.position.x = 3;
    mission_loc_arr[2].pose.position.y = -3;
    mission_loc_arr[2].pose.position.z = 5;
    mission_loc_arr[3].pose.position.x = -3;
    mission_loc_arr[3].pose.position.y = -3;
    mission_loc_arr[3].pose.position.z = 5;
    mission_loc_arr[4].pose.position.x = 0;
    mission_loc_arr[4].pose.position.y = 0;
    mission_loc_arr[4].pose.position.z = 5;
    //  geometry_msgs::PoseStamped pose;
    for (int count = 0; count < 1 && ros::ok(); count++)
    {
        ifor_drone_obj.target_loc.pose.position.x = mission_loc_arr[0].pose.position.x;
        ifor_drone_obj.target_loc.pose.position.y = mission_loc_arr[0].pose.position.y;
        ifor_drone_obj.target_loc.pose.position.z = mission_loc_arr[0].pose.position.z;
        int nPosIndex = 0;
        while (ros::ok())
        {
            if (!(abs(ifor_drone_obj.target_loc.pose.position.x - ifor_drone_obj.local_loc.pose.position.x) < 0.03 &&
                  abs(ifor_drone_obj.target_loc.pose.position.y - ifor_drone_obj.local_loc.pose.position.y) < 0.03 &&
                  abs(ifor_drone_obj.target_loc.pose.position.z - ifor_drone_obj.local_loc.pose.position.z) < 0.03) )
            {


                mavros_msgs::SetMode set_offb_mode;
			    mavros_msgs::CommandBool arm_cmd;
			    
			    set_offb_mode.request.custom_mode = "OFFBOARD";
			 
			    arm_cmd.request.value = true;

			    ros::Time last_request = ros::Time::now();
			    
			    while (ros::ok()) {
			        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
			        {
			            if( set_mode_cl.call(set_offb_mode) && set_offb_mode.response.success)
			            {
			                ROS_INFO("Offboard mode enabled");
			            }
			            last_request = ros::Time::now();
			        
			        }else{

			            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			            {
			                if( arming_cl.call(arm_cmd) &&
			                    arm_cmd.response.success)
			                {
			                    ROS_INFO("Drone armed");
			                }

			                last_request = ros::Time::now();
			            }
			        }
			       
			        if ( abs(ifor_drone_obj.target_loc.pose.position.x - ifor_drone_obj.local_loc.pose.position.x) < 0.03 &&
			             abs(ifor_drone_obj.target_loc.pose.position.y - ifor_drone_obj.local_loc.pose.position.y) < 0.03 &&
			             abs(ifor_drone_obj.target_loc.pose.position.z - ifor_drone_obj.local_loc.pose.position.z) < 0.03)
			            return;
			        else
			            ifor_drone_obj.local_pos_pub.publish(target_loc);

			        std::this_thread::sleep_for(std::chrono::milliseconds(100));
			    }
            


            }else{
                    if (++nPosIndex == 5)
                    break;
                    ifor_drone_obj.target_loc.pose.position.x = mission_loc_arr[nPosIndex].pose.position.x;
                    ifor_drone_obj.target_loc.pose.position.y = mission_loc_arr[nPosIndex].pose.position.y;
                    ifor_drone_obj.target_loc.pose.position.z = mission_loc_arr[nPosIndex].pose.position.z;
            }
            rate_50.sleep();
        }
    }

    rate_50.sleep();

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 20;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;
    
    if(land_cl.call(land_cmd)){
        ROS_INFO("land_cmd send ok %d", land_cmd.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }

    return 0;
}