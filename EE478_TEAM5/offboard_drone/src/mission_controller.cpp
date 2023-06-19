#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include <iostream>

geometry_msgs::PoseStamped cur_pose;
geometry_msgs::TwistStamped setpoint_vel;
mavros_msgs::State cur_state;
std::vector<float> data;

ros::Publisher local_vel_pub;
ros::ServiceClient take_picture_client;

float home_x = 0.0;
float home_y = 0.0;
float home_z = 1.0;

float cur_image_x = 0.5;
float cur_image_y = 0.5;
float cur_image_height = 0.3;

bool mission_complete = true;
int mission_mode = 0;
// 0 = default, 1 = move to target, 2 = move to home, 3 = take picture, 4: draw heart

bool is_takeoff = false;
bool is_taken = false;

void cur_Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cur_pose = *msg;
}

void mavros_State_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    cur_state = *msg;
}

void gesture_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // mesaage to data
    data = msg->data;


}

void run()
{
    int num_person = data.size()/4;

    float cur_x, cur_y, cur_z;

    for (int i = 0; i < num_person; ++i) {
        if (data[4*i] == 1.0) // left hand = 1
        {
            if (mission_complete){
                mission_mode = 1;
                mission_complete = false;
            }
            
            break;
        }
        else if (data[4*i] == 2.0) // right hand = 2
        {
            cur_image_x = data[4*i+1];
            cur_image_y = data[4*i+2];
            cur_image_height = data[4*i+3];

            if (mission_complete){
                mission_mode = 2;
                mission_complete = false;
            }

            break;
        }
        else if (data[4*i] == 3) // both hand = 3
        {
            if (mission_complete) {
                mission_mode = 3;
                mission_complete = false;
                is_taken = false;
            }
            
            break;
        }

        else if (data[4*i] == 4) // 
        {
            if (mission_complete) {
                mission_mode = 4;
                misssion_complete = false;
            }
            
            break;
        }
        else continue;
    }

    
    float velocity_x, velocity_y, velocity_z;
    
    if (mission_mode == 0)
    {
        setpoint_vel.twist.linear.x = 0;
	    setpoint_vel.twist.linear.y = 0;
	    setpoint_vel.twist.linear.z = 0;
	    local_vel_pub.publish(setpoint_vel);
    }

    else if (mission_mode == 1)
    {
        home_x = 0.0;
        home_y = 0.0;
        home_z = 1.0;

        cur_x = cur_pose.pose.position.x;
        cur_y = cur_pose.pose.position.y;
        cur_z = cur_pose.pose.position.z;

        if ((std::abs(home_x - cur_x) < 0.2) && (std::abs(home_y - cur_y) < 0.2) && (std::abs(home_z - cur_z) < 0.2)){
            mission_complete = true;
            mission_mode = 0;

            setpoint_vel.twist.linear.x = 0;
            setpoint_vel.twist.linear.y = 0;
            setpoint_vel.twist.linear.z = 0;
            local_vel_pub.publish(setpoint_vel);
        }

        else {
            velocity_x = 7.5* (home_x - cur_x);
            velocity_y = 4* (home_y - cur_y);
            velocity_z = 1.5* (home_z - cur_z);
    
            if(velocity_x > 0.3){
                velocity_x = 0.3;
            }
            if(velocity_x < -0.3){
                velocity_x = -0.3; 
            }
        
            if(velocity_y > 0.3){
                velocity_y = 0.3;
            }
            if(velocity_y < -0.3){
                velocity_y = -0.3;
            }

            if(velocity_z > 0.3){
                velocity_z = 0.3;
            }
            if(velocity_z < -0.3){
                velocity_z = -0.3;
            }


            ROS_INFO("mission mode 2.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", velocity_x, velocity_y, velocity_z);
    
            setpoint_vel.twist.linear.x = velocity_x;
            setpoint_vel.twist.linear.y = velocity_y;
            setpoint_vel.twist.linear.z = velocity_z;

            local_vel_pub.publish(setpoint_vel);
        }
    }

    else if (mission_mode == 2)
    {

        float target_x = 0.5;
        float target_y = 0.5;
        float target_height = 0.3;

        if ((std::abs(target_x - cur_image_x) < 0.05) && (std::abs(target_y - cur_image_y) < 0.05) && (std::abs(target_height - cur_image_height) < 0.05)){
            mission_complete = true;
            mission_mode = 0;

            setpoint_vel.twist.linear.x = 0;
            setpoint_vel.twist.linear.y = 0;
            setpoint_vel.twist.linear.z = 0;
            local_vel_pub.publish(setpoint_vel);
        }

        else{
            velocity_x = -7.5* (cur_image_height - target_height);
            velocity_y = 4* (cur_image_x - target_x);
            velocity_z = -1.5* (cur_image_y - target_y);

            if(velocity_x > 0.3){
                velocity_x = 0.3;
            }
            if(velocity_x < -0.3){
                velocity_x = -0.3; 
            }
        
            if(velocity_y > 0.3){
                velocity_y = 0.3;
            }
            if(velocity_y < -0.3){
                velocity_y = -0.3;
            }

            if(velocity_z > 0.3){
                velocity_z = 0.3;
            }
            if(velocity_z < -0.3){
                velocity_z = -0.3;
            }
        
            // velocity control
            ROS_INFO("mission mode 1.");
            ROS_INFO("v_x: %f , v_y: %f, v_z: %f .", velocity_x, velocity_y, velocity_z);

            setpoint_vel.twist.linear.x = velocity_x;
            setpoint_vel.twist.linear.y = velocity_y;
            setpoint_vel.twist.linear.z = velocity_z;

            local_vel_pub.publish(setpoint_vel);
        }
    }

    else if (mission_mode == 3)
    {
        // take a picture
        setpoint_vel.twist.linear.x = 0;
	    setpoint_vel.twist.linear.y = 0;
	    setpoint_vel.twist.linear.z = 0;

	    local_vel_pub.publish(setpoint_vel);
	
        ROS_INFO("mission mode 3.");

	    if(!is_taken){
            std_srvs::SetBool hands_up;
            hands_up.request.data = true;
            take_picture_client.call(hands_up);
            is_taken = true;
            misssion_complete = true;
            mission_mode = 0;
	    }
    }

    else if (mission_mode == 4)
    {
        
    }

    else ROS_INFO("mission mode 0.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gesture_out_subscriber");
    ros::NodeHandle nh;

    take_picture_client = nh.serviceClient<std_srvs::SetBool>("service/take_picture");

    ros::Subscriber gesture_sub = nh.subscribe("/posenet/gesture_out", 10, gesture_Callback);
    ros::Subscriber cur_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, cur_Pose_Callback);
    ros::Subscriber mavros_state_sub = nh.subscribe("/mavros/state", 1, mavros_State_Callback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
/*
  
  while(ros::ok() && !cur_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
*/
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

/*
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

   ros::Time last_request = ros::Time::now();
*/
    while(ros::ok()){
	/*
        if( cur_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
	else if( !cur_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        }
        else
	{
		if(!is_takeoff)
		{
						
			double dist = sqrt(pow(home_x - cur_pose.pose.position.x, 2) + pow(home_y - cur_pose.pose.position.y, 2) + pow(home_z - cur_pose.pose.position.z, 2));

			setpoint_vel.twist.linear.x = 0.1 * (home_x - cur_pose.pose.position.x);
			setpoint_vel.twist.linear.y = 0.1 * (home_y - cur_pose.pose.position.y);
			setpoint_vel.twist.linear.z = 0.1 * (home_z - cur_pose.pose.position.z);
			local_vel_pub.publish(setpoint_vel);
			
			if(dist < 0.3 && cur_pose.pose.position.z > 1.0)
			{
				is_takeoff = true;
			}
			else 
				run();
		}

	}

*/
	run();
        ros::spinOnce();
        rate.sleep();
    }
       

    return 0;
}
