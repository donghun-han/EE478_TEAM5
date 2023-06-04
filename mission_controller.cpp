# include <ros/ros.h>
# include <std_msgs/Float32MultiArray.h>

int mission_mode = 0 // 0 = default, 1 = move to target, 
                     // 2 = move to home, 3 = take picture



void gesture_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // mesaage to data
    std::vector<std::vector<float>> data;
    std::size_t numRows = msg->layout.dim[0].size;
    std::size_t numCols = msg->layout.dim[1].size;

    // Reshape the 1D vector into a 2D vector
    for (std::size_t i = 0; i < numRows; ++i) {
        std::vector<float> row;
        for (std::size_t j = 0; j < numCols; ++j) {
            row.push_back(msg->data[i * numCols + j]);
        }
        data.push_back(row);
    }
    
    int index = -1;

    // check gesture
    for (int i = 0; i < numRows; ++i) {
        if (data[i][0] == 1) // left hand = 1
        {
            mission_mode = 2;
            break;
        }
        else if (data[i][0] == 2) // right hand = 2
        {
            mission_mode = 1;
            index = i;
            break;
        }
        else if (data[i][0] == 3) // both hand = 3
        {
            mission_mode = 3;
            break;
        }
        else continue;
    }

    if (mission_mode == 1)
    {
        std::vector<float> target = data[index];

        float current_x, current_y, current_height;
        float target_x = 0.5;
        float target_y = 0.5;
        float target_height = 0.2;
        float velocity_x, velocity_y, velocity_z;

        current_x = target[1];
        current_y = target[2];
        current_height = target[3];

        velocity_x = 0.001* (current_x - target_x);
        velocity_y = 0.001* (current_y - target_y);
        velocity_z = 0.001* (current_height - target_height);

        // velocity control
        LogInfo("mission mode 1.");
    }
    else if (mission_mode == 2)
    {
        // home 가라 명령
        // subscribe pose.localposition form orb_slam --> 0,0,0 으로 velocity control
        LogInfo("mission mode 2.");
    }
    else if (mission_mode == 3)
    {
        // take a picture
        LogInfo("mission mode 3.");
    }
    else LogInfo("mission mode 0.");

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gesture_out_subscriber");
    ros::NodeHandle nh;

    // Create a subscriber for the 'posnet' topic
    ros::Subscriber gesture_sub = nh.subcribe("gesture_out", 10, gesture_Callback);


    // Enter the ROS event loop
    ros::spin();

    

    return 0;
}