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

#include "ros_compat.h"
#include "image_converter.h"
#include <std_msgs/Float32MultiArray.h>

#include <jetson-inference/poseNet.h>
#include <jetson-utils/videoOutput.h>


// globals
poseNet* net = NULL;
videoOutput* stream = NULL;
imageConverter* image_cvt = NULL;
uint32_t overlay_flags = poseNet::OVERLAY_DEFAULT;

std::string topic_name;

// publisher
Publisher<std_msgs::Float32MultiArray> detected_gesture_pub = NULL;

// helper function for sort
// - soft estimated poses w.r.t. bounding box position (Left is the most left-side pixel x-value)
bool order_compare(poseNet::ObjectPose pose1, poseNet::ObjectPose pose2) { return pose1.Left < pose2.Left; }

// helper function to parse pose output & determine the gesture
float estimate_gesture(poseNet::ObjectPose pose, int img_width, int img_height)
{
	// Example skeleton code for gesture estimation
	// - Parsing keypoint data based on 'keypoint.ID'

	/*
	 * - Bounding box position (Left/Right/Top/Bottom)
	 * - Keypoint list
	 * -- 0: nose
	 * -- 1: left_eye
	 * -- 2: right_eye
	 * -- 3: left_ear
	 * -- 4: right_ear
	 * -- 5: left_sholder
	 * -- 6: right_sholder
	 * -- 7: left_elbow
	 * -- 8: right_elbow
	 * -- 9: left_wrist
	 * -- 10: right_wrist
	 * -- 11: left_hip
	 * -- 12: right_hip
	 * -- 13: left_knee
	 * -- 14: right_knee
	 * -- 15: left_ankle
	 * -- 16: right_ankle
	 * -- 17: neck
	 
	 */
	int right_shoulder_id = 6;
	int left_wrist_id = 9;
	int right_wrist_id = 10;
	int right_ankle_id = 16;
	int left_knee_id = 13;

	double right_shoulder_pixel_y = -1;
	double left_wrist_pixel_y = -1;
	double right_wrist_pixel_y = -1;
	double right_ankle_pixel_y = -1;
	double left_knee_pixel_y = -1;

	for (auto keypoint : pose.Keypoints) {
		// neck keypoint
		if (keypoint.ID == right_shoulder_id) {
			// LogInfo("Keypoint (Neck) x: %i y: %i in image", keypoint.x, keypoint.y);
			right_shoulder_pixel_y = keypoint.y / img_height;
		}
		// left_wrist keypoint
		if (keypoint.ID == left_wrist_id) {
			// LogInfo("Keypoint (Left Wrist) x: %i y: %i in image", keypoint.x, keypoint.y);
			left_wrist_pixel_y = keypoint.y / img_height;
		}
		// right_wrist keypoint
		if (keypoint.ID == right_wrist_id) {
			// LogInfo("Keypoint (Right Wrist) x: %i y: %i in image", keypoint.x, keypoint.y);
			right_wrist_pixel_y = keypoint.y / img_height;
		}
		// right_ankle keypoint
		if (keypoint.ID == right_ankle_id) {
			// LogInfo("Keypoint (Right Ankle) x: %i y: %i in image", keypoint.x, keypoint.y);
			right_ankle_pixel_y = keypoint.y / img_height;
		}
		// left_knee keypoint
		if (keypoint.ID == left_knee_id) {
			// LogInfo("Keypoint (Left knee) x: %i y: %i in image", keypoint.x, keypoint.y);
			left_knee_pixel_y = keypoint.y / img_height;
		}
	}

	// Check keypoint data condition
	bool is_got_all_data = (right_shoulder_pixel_y != -1) && (left_wrist_pixel_y != -1) && (right_wrist_pixel_y != -1) && (right_ankle_pixel_y != -1) && (left_knee_pixel_y != -1);
	// Determine the gesture
	// 0: nothing
	// 1: left hand up
	// 2: right hand up
	// 3: both hand up
	// 4: right leg up
	float gesture_id = 0; // nothing

	if (is_got_all_data) {
		if ((left_wrist_pixel_y < right_shoulder_pixel_y) && (right_wrist_pixel_y > right_shoulder_pixel_y)) {
			gesture_id = 1; // left hand up
		}

		if ((left_wrist_pixel_y > right_shoulder_pixel_y) && (right_wrist_pixel_y < right_shoulder_pixel_y)) {
			gesture_id = 2; //right hand up
		}

		if ((left_wrist_pixel_y < right_shoulder_pixel_y) && (right_wrist_pixel_y < right_shoulder_pixel_y)) {
			gesture_id = 3; //both hand up
		}

		if (right_ankle_pixel_y < left_knee_pixel_y) {
			gesture_id = 4; // right leg up
		}
	}

	return gesture_id;
}

float* boundingbox_position(poseNet::ObjectPose pose, int img_width, int img_height)
{
	float BB_left_pixel = pose.Left / img_width;
	float BB_right_pixel = pose.Right / img_width;
	float BB_Top_pixel = pose.Top / img_height;
	float BB_Bottom_pixel = pose.Bottom / img_height;

	static float BB[2];

	BB[0] = (BB_left_pixel + BB_right_pixel)/2;
	BB[1] = (BB_Top_pixel + BB_Bottom_pixel)/2;

	return BB;
}

float get_body_length(poseNet::ObjectPose pose, int img_width, int img_height)
{
	int left_shoulder_id = 5;
	int left_hip_id = 11;

	float left_shoulder_pixel_y = -1;
	float left_hip_pixel_y = -1;

	for (auto keypoint : pose.Keypoints) {
		// left shoulder keypoint
		if (keypoint.ID == left_shoulder_id) {
			// LogInfo("Keypoint (Left Shoulder) x: %i y: %i in image", keypoint.x, keypoint.y);
			left_shoulder_pixel_y = keypoint.y / img_height;
		}

		// left hip keypoint
		if (keypoint.ID == left_hip_id) {
			// LogInfo("Keypoint (Left Hip) x: %i y: %i in image", keypoint.x, keypoint.y);
			left_hip_pixel_y = keypoint.y / img_height;
		}
	}

	// Check keypoint data condition
	bool is_got_all_data = (left_shoulder_pixel_y != -1) && (left_hip_pixel_y != -1);

	float body_length = 0.3;

	if (is_got_all_data) {
		body_length = left_hip_pixel_y - left_shoulder_pixel_y;
	}

	return body_length;

}


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{
	// convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		LogInfo("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}
        else {
                LogInfo("converted image %ux%u", input->width, input->height);
        }

	// run pose estimation
	std::vector<poseNet::ObjectPose> poses;
        bool isDetectPose = net->Process(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight(), poses, poseNet::OVERLAY_DEFAULT);
	
	if( !isDetectPose )
	{
		LogError("posenet: failed to process frame\n");
		return;
	}
	
	LogInfo("posenet: detected %zu %s(s)\n", poses.size(), net->GetCategory());
	
	// Sort detected poses
	std::sort(poses.begin(), poses.end(), order_compare); // more left (less pixel x), closer to 0 index. 
        
	// Parsing keypoint data
	std_msgs::Float32MultiArray detectedGestures;
	std::vector<float> pose_info;

	detectedGestures.layout.dim.push_back(std_msgs::MultiArrayDimension());
	detectedGestures.layout.dim.push_back(std_msgs::MultiArrayDimension());
	detectedGestures.layout.dim[0].size = 0;
	detectedGestures.layout.dim[1].size = 4;
	detectedGestures.layout.dim[0].stride = 4;
	detectedGestures.layout.dim[1].stride = 1;
	detectedGestures.layout.dim[0].label = "rows";
	detectedGestures.layout.dim[1].label = "columns";

	if (poses.size() > 0) {
		for (auto pose : poses) {
			float output_gesture = estimate_gesture(pose, image_cvt->GetWidth(), image_cvt->GetHeight());
			float* output_boundingbox = boundingbox_position(pose, image_cvt->GetWidth(), image_cvt->GetHeight());
			float output_bodylenght = get_body_length(pose, image_cvt->GetWidth(), image_cvt->GetHeight());

			pose_info.push_back(output_gesture);
			pose_info.push_back(output_boundingbox[0]);
			pose_info.push_back(output_boundingbox[1]);
			pose_info.push_back(output_bodylenght);


		}
		detectedGestures.data.insert(detectedGestures.data.end(), pose_info.begin(), pose_info.end());
	}
	
	// Publish as Float32MultiArray
	detected_gesture_pub->publish(detectedGestures);

	// render the image with pose output
	stream->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());

	// update status bar
	char str[256];
	sprintf(str, "%s (%ux%u) | %.1f FPS", topic_name.c_str(), image_cvt->GetWidth(), image_cvt->GetHeight(), stream->GetFrameRate());
	stream->SetStatus(str);	

	// check for EOS
	if( !stream->IsStreaming() )
		ROS_SHUTDOWN();
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("pose_net_node");
	

	/*
	 * Define publisher
	 */ 
	
	ROS_CREATE_PUBLISHER(std_msgs::Float32MultiArray, "gesture_out", 2, detected_gesture_pub);


	/*
	 * declare parameters
	 */
	videoOptions video_options;

	std::string resource_str;
	std::string codec_str;
	std::string overlay_str = "links,keypoints";
	int video_bitrate = video_options.bitRate;
	int loop_rate_hz = 15; // ROS rate

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("bitrate", video_bitrate);
	ROS_DECLARE_PARAMETER("loop_rate_hz", loop_rate_hz);
	

	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("bitrate", video_bitrate);
	ROS_GET_PARAMETER("loop_rate_hz", loop_rate_hz);


	/*
	 * set pretrained model parameters
	 */
	const char* modelName = "resnet18-body";
	float threshold = 0.15;
	uint32_t maxBatchSize = 1;
	// set inference parameters
	overlay_flags = poseNet::OverlayFlagsFromStr(overlay_str.c_str());


	/*
	 * create poseNet from pretrained model
	 */
	net = poseNet::Create(modelName, threshold, maxBatchSize);
	
	if( !net )
	{
		LogError("posenet: failed to initialize poseNet\n");
		return 1;
	}
        ROS_INFO("model is loaded!");

	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	video_options.bitRate = video_bitrate;

	ROS_INFO("opening video output: %s", resource_str.c_str());


	/*
	 * create stream
	 */
	stream = videoOutput::Create(resource_str.c_str(), video_options); 
	
	if( !stream )
	{
		ROS_ERROR("failed to open video output");
		return 0;
	}


	/*
	 * create image converter
	 */
	image_cvt = new imageConverter();

	if( !image_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * subscribe to image topic
	 */
	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
	topic_name = ROS_SUBSCRIBER_TOPIC(img_sub);

	/*
	 * start streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


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


	/*
	 * free resources
	 */
	delete stream;
	delete image_cvt;

	return 0;
}

