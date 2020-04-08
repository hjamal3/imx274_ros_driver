#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "mipi_cameras/camera.h"

// HJ (3/28/2020): service for taking pictures from the mipi cameras. saves to file
// request service on "camera" and node stores current image

cv::Mat frame0_global, frame1_global;
cv::VideoCapture cap0;
cv::VideoCapture cap1;
int counter = 0;

bool service_callback(mipi_cameras::camera::Request  &req,
         mipi_cameras::camera::Response &res)
{
	res.res = 1;
	std::cout << "mipi_service node: taking image #"<< counter << std::endl;
	//double secs = ros::Time::now().toSec();
	cv::imwrite(cv::format( "/home/powerranger-leopard/Desktop/images/%d_left_%d.jpg", counter), frame0_global );
	cv::imwrite(cv::format( "/home/powerranger-leopard/Desktop/images/%d_right_%d.jpg", counter), frame1_global);
	// save the images
	counter++;
	return true;
}

void timer_callback(const ros::TimerEvent& event)
{
    	cv::Mat frame0, frame1;
	cap0.read(frame0);
	if(!frame0.empty()) 
	{
		frame0_global = frame0;
	}
	cap1.read(frame1);
	if(!frame1.empty()) 
	{
		frame1_global = frame1;
	}
}

int main(int argc, char** argv) {

	std::cout << "Starting MIPI camera service node." << std::endl;
	ros::init(argc, argv, "mipi_service_node");
	ros::NodeHandle nh;


	/* Service to process photo requests */
  	ros::ServiceServer service = nh.advertiseService("camera", service_callback);

	/* Camera grabs latest data periodically. 100 hz. */
	ros::Timer timer = nh.createTimer(ros::Duration(0.01), timer_callback);

	int width, height;
	nh.param("width", width, 1280);
	nh.param("height", height, 720);

	std::string pipeline0 = "nvarguscamerasrc sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=30/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::string pipeline1 = "nvarguscamerasrc sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=30/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::cout << "pipeline0: " << pipeline0 << std::endl;
	std::cout << "pipeline1: " << pipeline1 << std::endl;

	cap0.open(pipeline0, cv::CAP_GSTREAMER);
	cap1.open(pipeline1, cv::CAP_GSTREAMER);

	if (!cap0.isOpened() || !cap1.isOpened() ) {
        	std::cout << "VideoCapture or VideoWriter not opened" << std::endl;
        	exit(-1);
    	}
	ros::spin();
	return 0;
}

