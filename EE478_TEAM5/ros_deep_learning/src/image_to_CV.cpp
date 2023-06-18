/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_srvs/SetBool.h>

std::string topic_name;

sensor_msgs::ImageConstPtr raw_image;


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{

	raw_image = input;	

}
/*
void img_to_CV(sensor_msgs::ImageConstPtr image_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
    	{
      		cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
    	}
    	
	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}

	cv::imwrite("/home/team5/test.jpg", cv_ptr);


}
*/

bool img_to_CV(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
    	{
      		cv_ptr = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
    	}
    	
	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return false;
    	}

	cv::imwrite("/home/usrg/test"+ std::to_string(ros::Time::now().toSec()) +".jpg", cv_ptr->image);
	return true;

}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ros::init(argc, argv, "image_to_CV_node");
	ros::NodeHandle nh;

	int loop_rate_hz = 15; // ROS rate


	/*
	 * subscribe to image topic
	 */
	// auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "/camera/color/image_raw/", 5, img_callback);
	ros::Subscriber img_sub = nh.subscribe("/camera/color/image_raw", 5, img_callback);
	//topic_name = ROS_SUBSCRIBER_TOPIC(img_sub);
	ros::ServiceServer server = nh.advertiseService("service/take_picture", img_to_CV);
	

	/*
	 * start ros spin & publishing data
	 */
	// rate
	ros::Rate loop_rate(loop_rate_hz);

	ROS_INFO("video_output node initialized, waiting for messages");
	while (ros::ok()) {
		// main process
		ros::spinOnce();
		loop_rate.sleep();
	}	
	//ROS_INFO("video_output node initialized, waiting for messages");
	//ROS_SPIN();



	return 0;
}

