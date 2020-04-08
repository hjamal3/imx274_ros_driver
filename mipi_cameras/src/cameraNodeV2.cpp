#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// HJ (12/18/2019): Publishes camera data using gstreamer via opencv

int main(int argc, char** argv) {

	std::cout << "Starting MIPI camera node..." << std::endl;
	ros::init(argc, argv, "mipi_cam_node_fast");
	ros::NodeHandle nh("~");
	int width, height;
	nh.param("width", width, 1280);
	nh.param("height", height, 720);

	std::string pipeline0 = "nvarguscamerasrc sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::string pipeline1 = "nvarguscamerasrc sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::cout << "pipeline0: " << pipeline0 << std::endl;
	std::cout << "pipeline1: " << pipeline1 << std::endl;

	cv::VideoCapture cap0(pipeline0, cv::CAP_GSTREAMER);
	cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

	if (!cap0.isOpened() || !cap1.isOpened() ) {
        	std::cout << "VideoCapture or VideoWriter not opened" << std::endl;
        	exit(-1);
    	}

    	cv::Mat frame0, frame1;
	ros::Publisher lens_pub0 = nh.advertise<sensor_msgs::Image>("/mipi/cam0", 1);
	ros::Publisher lens_pub1 = nh.advertise<sensor_msgs::Image>("/mipi/cam1", 1);
	bool success0 = false;
	bool success1 = true;

    	while(ros::ok()) {
		success0 = cap0.grab();
		success1 = cap1.grab();
		if (success0 and success1)
		{
			cap0.retrieve(frame0);
			cap1.retrieve(frame1);
			cv_bridge::CvImage out_msg;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			out_msg.image = frame0;
			lens_pub0.publish(out_msg.toImageMsg());
			out_msg.image = frame1;
			lens_pub1.publish(out_msg.toImageMsg());
		}
	}
	ros::spin();
	return 0;
}

