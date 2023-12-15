# Audio Input Package

This packages uses the pyaudio package with the Google Web Speech API that listens to an user's choice of target and transcibes
that to a string that can be used in aiming at the chosen target. The inputs are set to only red, blue, green, and yellow. 

## Quickstart
1. Use `ros2 service call /place trajectory_interfaces/srv/UserInput ` to start the listener which will return the answer string
    - Once the listener has heard 5 different inputs, it will automatically exit out. 

## Citations
1. https://realpython.com/python-speech-recognition/