# Trigger

This ROS2 package interacts with an Arduino via serial communication, allowing the system to interact with I/O.

Specifically, the package exposes a service which upon being called sends a string via serial to the Arduino. The program running on the Arduino reads the incoming data and switches the outputs as needed.

## Quickstart

To test the package, an Arduino needs to be connected, and the correct serial number has to be configured in the node.

1. Run the package by issuing the command `ros2 run trigger trigger`.
2. Call the service by doing `ros2 service call /fire trigger_interfaces/srv/Fire "{gun_id: 0}"`

This should make the Arduino operate its I/O, and on the console of the running node, an output confirming that the Arduino received the message should print.