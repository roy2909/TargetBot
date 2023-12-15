# YOLOv8 Package

This packages uses the computer vision model of Yolo8 that is trained on the bowling pins of various colors (red, blue, green, yellow) to find the bowling pins and their coordinates with respect to the base of the arm and displays them as colored markers in Rviz2. 

## Quickstart
1. Use `ros2 run image_yolo yolo` to start the yolo node
    - This node has a service call that detect pins that is called while the gun is scanning for targets. 
    - It is unable to detect without the realsense camera node running.