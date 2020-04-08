#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// HJ (12/18/2019): Publishes camera data using gstreamer via opencv

int main(int argc, char** argv) {

	std::cout << "Starting MIPI camera node..." << std::endl;
	ros::init(argc, argv, "mipi_cam_node");
	ros::NodeHandle nh("~");
	int width, height;
	nh.param("width", width, 1280);
	nh.param("height", height, 720);

	std::string pipeline0 = "nvarguscamerasrc sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=6/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

	std::string pipeline1 = "nvarguscamerasrc sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=6/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

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
    	while(ros::ok()) {
		cap0.read(frame0);
		if(!frame0.empty()) 
		{
            		//cv::Mat gray0;
            		//cv::cvtColor(frame0, gray0, CV_BGR2GRAY);
			cv_bridge::CvImage out_msg;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			//out_msg.image = gray0;
			out_msg.image = frame0;
			out_msg.header.frame_id = "cam_left";
			out_msg.header.stamp = ros::Time::now();
			//cv::imshow("frame", frame);
			//cv::waitKey(2);
			lens_pub0.publish(out_msg.toImageMsg());
		}
		cap1.read(frame1);
		if(!frame1.empty()) {
			//cv::Mat gray1;
		    	//cv::cvtColor(frame1, gray1, CV_BGR2GRAY);
			cv_bridge::CvImage out_msg;
			out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			//out_msg.image = gray1;
			out_msg.image = frame1;
			out_msg.header.frame_id = "cam_right";
			out_msg.header.stamp = ros::Time::now();
			//cv::imshow("frame", frame);
			//cv::waitKey(2);

			lens_pub1.publish(out_msg.toImageMsg());
			}
	}
	ros::spin();
	return 0;
}

