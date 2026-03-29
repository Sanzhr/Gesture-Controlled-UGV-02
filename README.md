# Gesture-Controlled UGV-02 (ROS2)

## Description
This project presents a gesture-controlled mobile robot system based on the UGV-02 platform and ROS2.

The robot operates in a ROS2 environment, while a Python script running on a laptop uses the webcam to track hand position in real time. Based on the hand distance and direction relative to the camera, movement commands are generated and used to control the robot.

The project combines computer vision, gesture recognition, and mobile robot control.

## Features
- Real-time hand tracking using a laptop webcam
- Gesture-based control of robot motion
- Forward and backward movement based on hand distance
- Left and right turning based on hand position or finger direction
- Robot operation in a ROS2 environment
- Real-time visual feedback using OpenCV

## Technologies Used
- ROS2
- Python
- MediaPipe
- OpenCV
- Linux

## Hardware
- UGV-02 mobile robot platform
- Laptop webcam
- Linux-based control environment
- Wired connection between laptop and robot

## Control Logic
The Python script detects the hand through the webcam and estimates its position relative to the camera.

Control behavior:
- Hand far from the camera -> move forward
- Hand close to the camera -> move backward
- Hand shifted left or right -> turn left or right
- No hand detected -> stop

## Project Goal
The goal of this project was to explore human-robot interaction and gesture-based control of a mobile robot using computer vision in a ROS2 environment.

## Notes
The main implemented part in this repository is the Python-based gesture recognition and control logic.
The robot platform itself was launched and operated through ROS2 tools in Linux.

## Video Demonstration
A demonstration of the robot response to hand gestures is available here:  
https://github.com/user-attachments/assets/40b3c5e1-a53e-4245-87dd-fa5b8c8275ad
