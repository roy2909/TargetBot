# Control Package

This packages controls the pipeline for the demonstration by calling the necessary services to other nodes. The pipeline is asking user for input, scanning for targets, scanning for guns, picking up the gun, aiming and shooting at each of the pins, placing the gun and then repeating. 

## Quickstart
1. Use `ros2 run control control` to start the control node
    - This node will wait for most of the other nodes to be up and running before continuing. The exception is the april tag node and the realsense.
