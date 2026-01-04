# Hand-held-SLAM-V1
Hand held SLAM running on a pi 5 with an Arducam tof camera, Pi AI cam, and a MPU 6050 IMU. Version 1: ToF camera depth and distance mapping with color view and point cloud, Object detection along with landmark creation in 3d space and UDP streaming on Pi AI camera, with IMU for orientation and movement.

Arducam ToF Camera:
  Arducam ToF camera works to find distance of objects and helps to map distance for landmarks.
  Arducam ToF camera also creates a point cloud that is viewable through rviz2 and adding pointcloud through creating a visualization.
  Note: The Arducam ToF camera requires 5V 5Amps to work at optimal performance and you may need to create a venv to install ArducamDepthCamera 
