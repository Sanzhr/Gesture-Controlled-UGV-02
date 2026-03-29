# Gesture-Controlled UGV-02 (ROS2)

## Description
This project presents a gesture-controlled mobile robot system based on UGV-02 and ROS2.

The robot was operated in a ROS2 environment, while a Python script running on a laptop used the webcam to track hand position in real time. Based on hand distance and direction relative to the camera, movement commands were generated and used to control the robot.

The project combines computer vision, gesture recognition, and mobile robot control.

## Features
- Real-time hand tracking using laptop webcam
- Gesture-based control of robot motion
- Forward/backward movement based on hand distance
- Left/right turning based on hand position or finger direction
- Robot operation in ROS2 environment
- Real-time visual feedback using OpenCV

## Technologies Used
- ROS2
- Python
- MediaPipe
- OpenCV

## Hardware
- UGV-02 mobile robot
- Laptop webcam
- Linux-based control environment
- Wired connection to robot

## Control Logic
The Python script detects the hand through the webcam and estimates its position relative to the camera:

- hand far from camera -> move forward
- hand close to camera -> move backward
- hand shifted left/right -> turn left or right
- no hand detected -> stop

## Project Goal
The goal of this project was to explore human-robot interaction and gesture-based control of a mobile robot using computer vision in a ROS2 environment.

## Notes
The main implemented part in this repository is the Python-based gesture recognition and control logic.  
Robot-side launch and execution were performed through ROS2 commands in Linux.
