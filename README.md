# IMX274-ROS-driver
Currently setup with CTI Spacely carrier board and IMX274 from Leopard Imaging. 2 Camera setup. Can remove one camera in code.  
Copy mipi_cameras into catkin_ws/src  
Type catkin_make  
Type source devel/setup.bash  

For stereo type cameras: rosrun mipi_cameras mipi_cam_node_fast  
For regular cameras: rosrun mipi_cameras mipi_cam_node  

