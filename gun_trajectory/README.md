# Gun Trajectory Package

This packages interacts with the moveit group in order to motion plan the arm to the necessary locations for the demo. The package uses cartesian planning and IK planning to move the arm. 

## Quickstart
1. Use `ros2 run gun_trajectory shoot` to start the node
    - This node contains services that the control node calls so it does not move the arm without the control node.