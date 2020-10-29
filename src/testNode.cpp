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
	nh.param("width", width, 640);
	nh.param("height", height, 480);

	std::string pipeline0 = "nvarguscamerasrc sensor-id=" + std::to_string(0) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True";

	std::string pipeline1 = "nvarguscamerasrc sensor-id=" + std::to_string(1) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" + std::to_string(height) + ", framerate=60/1" + " ! nvvidconv flip-method=0 ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True";

	std::cout << "pipeline0: " << pipeline0 << std::endl;
	std::cout << "pipeline1: " << pipeline1 << std::endl;

	cv::VideoCapture cap0(pipeline0, cv::CAP_GSTREAMER);
	cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);

	if (!cap0.isOpened() || !cap1.isOpened() ) {
        	std::cout << "VideoCapture or VideoWriter not opened" << std::endl;
        	exit(-1);
    	}
    	
	cv::Mat frame0, frame1;
	ros::Publisher lens_pub0 = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 0);
	ros::Publisher lens_pub1 = nh.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 0);
	bool success0 = false;
	bool success1 = false;

    	while(ros::ok()) {

    		// grab left image and store time of grab
			success0 = cap0.grab();
			ros::Time time0 = ros::Time::now();

    		// grab right image and store time of grab
			success1 = cap1.grab();
			ros::Time time1 = ros::Time::now();
			if (success0 and success1)
			{
				// retrieve image data
				cap0.retrieve(frame0);
				cap1.retrieve(frame1);

				// store in cv_bridge message
				cv_bridge::CvImage out_msg0;
				out_msg0.encoding = sensor_msgs::image_encodings::BGR8;
				out_msg0.image = frame0;
				out_msg0.header.frame_id = "left_cam";
				out_msg0.header.stamp = time0;
				cv_bridge::CvImage out_msg1;
				out_msg1.encoding = sensor_msgs::image_encodings::BGR8;
				out_msg1.image = frame1;
				out_msg1.header.frame_id = "right_cam";
				out_msg1.header.stamp = time1;

				// convert to ROS and publish
				lens_pub0.publish(out_msg0.toImageMsg());
				lens_pub1.publish(out_msg1.toImageMsg());
			}
	}
	
	cap0.release();
	cap1.release();
	return 0;
}

