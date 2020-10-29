#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "mipi_cameras/camera.h"
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

cv::VideoCapture cap0;
cv::VideoCapture cap1;
int counter = 0;
ros::Publisher lens_pub0;
ros::Publisher lens_pub1;
ros::Publisher info_pub0;
ros::Publisher info_pub1;

cv_bridge::CvImage out_msg0;
cv_bridge::CvImage out_msg1;


sensor_msgs::CameraInfo cam_info0;
sensor_msgs::CameraInfo cam_info1;
bool success0(false),success1(false);
ros::Time time0,time1;

void timer_callback(const ros::TimerEvent& event)
{
    static cv::Mat frame0, frame1;
    
	// grab right image and store time of grab
	if (success0 and success1)
	{
		// retrieve image data
		cap0.retrieve(frame0);
		cap1.retrieve(frame1);
	}	

	if(frame0.empty() || frame1.empty()) 
	{
        if (frame0.empty()) std::cerr << "frame0 empty. Ignoring..." << std::endl;
        if (frame1.empty()) std::cerr << "frame1 empty. Ignoring..." << std::endl;
	    return;
	}

	// store latest
	out_msg0.image = frame0;
	out_msg0.header.stamp = time0;
	out_msg1.image = frame1;
	out_msg1.header.stamp = time0;//time1;
	cam_info0.header.stamp = time0;
	cam_info1.header.stamp = time0; //time1;

	lens_pub0.publish(out_msg0.toImageMsg());
	lens_pub1.publish(out_msg1.toImageMsg());
	info_pub0.publish(cam_info0);
	info_pub1.publish(cam_info1);


}

int main(int argc, char** argv) {

	std::cout << "Starting MIPI camera service node." << std::endl;
	ros::init(argc, argv, "mipi_service_node");
	ros::NodeHandle nh;

	int width, height, rate;
	nh.param("width", width, 640);
	nh.param("height", height, 480);
	nh.param("rate", rate, 2);

	const std::string cname0="mipi_left";
	const std::string cname1="mipi_right";

	std::string url0,url1;
	nh.param<std::string>("left_calib",url0,"package://mipi_cameras/calib/left.yaml");
	nh.param<std::string>("right_calib",url1,"package://mipi_cameras/calib/right.yaml");

	info_pub0 = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 0);
	info_pub1 = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 0);

	camera_info_manager::CameraInfoManager cinfo0(nh,cname0,url0);
	camera_info_manager::CameraInfoManager cinfo1(nh,cname1,url1);

	cam_info0=cinfo0.getCameraInfo();
	cam_info1=cinfo1.getCameraInfo();


	// initialize some settings
	out_msg0.encoding = sensor_msgs::image_encodings::BGR8;
	out_msg1.encoding = sensor_msgs::image_encodings::BGR8;
	out_msg0.header.frame_id = "left/image_raw";
	out_msg1.header.frame_id = "right/image_raw";
	cam_info0.header.frame_id = "map";
	cam_info1.header.frame_id = "map";

	/* Camera grabs latest data periodically. 100 hz. */
	ros::Timer timer = nh.createTimer(ros::Duration(1./rate), timer_callback);

	std::string pipeline0 = "nvarguscamerasrc gainrange='1.00 10.00' ispdigitalgainrange='1.00 1.00' "
    "sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::string pipeline1 = "nvarguscamerasrc gainrange='1.00 10.00' ispdigitalgainrange='1.00 1.00' "
    "sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	/*std::string pipeline0 = "nvarguscamerasrc "
    "sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=30/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	std::string pipeline1 = "nvarguscamerasrc "
    "sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=30/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";*/

	std::cout << "pipeline0: " << pipeline0 << std::endl;
	std::cout << "pipeline1: " << pipeline1 << std::endl;

	cap0.open(pipeline0, cv::CAP_GSTREAMER);
	cap1.open(pipeline1, cv::CAP_GSTREAMER);

	if (!cap0.isOpened() || !cap1.isOpened() ) {
        	std::cout << "VideoCapture or VideoWriter not opened" << std::endl;
        	exit(-1);
    	}
    	
	lens_pub0 = nh.advertise<sensor_msgs::Image>("left/image_raw", 0);
	lens_pub1 = nh.advertise<sensor_msgs::Image>("right/image_raw", 0);

	if (!cap0.isOpened() || !cap1.isOpened() ) {
        	std::cout << "VideoCapture or VideoWriter not opened" << std::endl;
        	exit(-1);
    	}
	while(ros::ok()) {
	  time0 = ros::Time::now();
	  success0 = cap0.grab();
	  time1 = ros::Time::now();
	  success1 = cap1.grab();
	  ros::spinOnce();
	}
	
	cap0.release();
	cap1.release();
	return 0;
}