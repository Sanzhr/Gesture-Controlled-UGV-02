# Gesture-Controlled-UGV-02
Implementation of gesture controll into UGV-02

This project implements a gesture-controlled system for a UGV-02 robot, using MediaPipe for hand tracking and gesture recognition, and serial communication to control robot movement in real time. Hand proximity controls forward/backward motion, and index finger tilt controls left/right turning.

Features:
Real-time hand tracking using webcam (via MediaPipe)
Gesture detection: turn left or right using index finger direction
Depth-based control: move forward or backward depending on hand distance
Sends commands over serial to control UGV robot wheels
Visual feedback via OpenCV

Gesture	Action:
Hand far (>60)	Move Forward
Hand close (<40)	Move Backward
Index finger right	Turn Right
Index finger left	Turn Left
No hand / neutral	Stop

Hardware Used:
UGV-02 robot platform
Webcam (default system camera used)
PC running the Python script
Serial connection (USB) to UGV (e.g., via ESP32/Arduino/etc.)

How to Run
Connect your UGV robot to your PC via USB (default: COM6)
Clone this repository and open the Python file.
Run the script:

Acknowledgements:
MediaPipe for robust hand tracking
OpenCV for real-time camera interface
UGV-02 robot hardware platform

