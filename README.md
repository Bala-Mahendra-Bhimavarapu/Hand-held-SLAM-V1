
# Hand-held-SLAM-V1

Hand held SLAM running on a pi 5 with an Arducam tof camera, Pi AI cam, and a MPU 6050 IMU. 

Version 1: ToF camera depth and distance mapping with color view and point cloud, Object detection along with landmark creation in 3d space and UDP streaming on Pi AI camera, with IMU for orientation and movement.


Arducam ToF Camera:  
  - Arducam ToF camera works to find distance of objects and helps to map distance for landmarks.  
  - Arducam ToF camera also creates a point cloud that is viewable through rviz2 and adding pointcloud through creating a visualization.  
  - Note: The Arducam ToF camera requires 5V 5Amps to work at optimal performance and you may need to create a venv to install ArducamDepthCamera  


Pi AI Camera: 
  - Pi AI camera detects objects in its frame and recognizes them as either objects or landmarks based on whether the class of the object is found in a certain list of objects that are designated as landmark objects. If the object is recognized as a landmark it is logged and then run through a code to map where it is on a cloudpoint and create a 3d visualization of where it is along with coordinates. 
  - The Pi AI camera also runs a UDP stream for live viewing.


MPU6050 IMU:
  - As of now in this project the IMU just runs in the background and isn't really working and I plan to integrate for path planning and mapping in V2.


Issues:
  - One issue i have been having for visualization on the cloudpoint is that the cloudpoint faces directly up instead of where the camera is supposed to face. I have set paremeters for where each camera and sensor module is in 3d space with transformation and setup so all of them face the same direction, yet cloudpoint continues to face up. I will try to debug in either V1.5 or V2.
  - Another issue is that each time the landmarks update the points on the cloudpoint showing where the landmarks are flicker and needs to remain smooth while updating.
  - I have yet to check if the landmarks are saved in 3d space for mapping since i dont have a 3d mapping system ready yet for V1 nor a path planning system and plan to integrate for V2 of the Handheld SLAM project.


Special Notes:
  - When building ROS2 Humble make sure empy is uninstalled otherwise it will cause errors in the build process.
  - To check if ROS2 Humble is build properly instead of using `ros2 --version` use `echo $ROS_DISTRO` and check if it says humble.
  - Running rviz2 requires x11 compatibility which isn't naturally found in Raspberry pi OS and you can force x11 compatibility using this command `export QT_QPA_PLATFORM=xcb` you can also add this to ~/.bashrc
  - To automatically enable ros2 in system and files along with x11 compatibility you can add `source ~/ros2_humble/install/setup.bash`,  and `export QT_QPA_PLATFORM=xcb`.
  - The whole system runs at a decent speed but can improve for object detection, streaming, and tof camera, optimization will take place after the whole system rover system is ready and working or atleast one rover is ready and working.
  - Also there are excess files in the project right now as i failed to do some of the things i wanted to and had to scale back a bit and the main code working right know is the sensor only code and it can be run by activating ros2 and then running `ros2 launch slam_bringup sensors_only.launch.py`.
