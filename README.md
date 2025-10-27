# ArUco-Course-Correction

How do we drive without depth sensing LiDAR in our eyes? 

We have a pinhole stereo camera (2 eyes) that perceives depth with trigonometry. We also know where the lane divider, curb, and other landmarks should be when driving straight, turning left across an intersection, etc.

We then use these two to correct our throttle and steering.

In short, I'm developing an image based visual servoing PID controller to adhere to a spontaneous path from a LiDAR-based motion planner, or follow somebody.

I programmed this with:
- ROS1 Noetic
- C++
- Python

for an RC car equipped with:
- ESP32
- Jetson
- ZED Mini stereo camera
- RoboSense Airy LiDAR. 

I'm using ArUco/QR codes to standardize landmarks, and the OpenCV open source computer vision library to detect corners of the markers.




# Technical Background

We make a linear approximation that the change in the visual feed, in a small timestep, can be described with a linear transformation.

This allows us to use, in essence, the Jacobian matrix to relate the motion of the coordinates of the Aruco corners in the pixel frame with the 'Twist' (v_x, v_y, v_z, w_x, w_y, w_z) vector of the vehicle.

The main challenge is the fact I don't have a drone with 6 degrees of freedom. I have an Ackerman steering based car, and I'm trying to use nullspace projection to effectively course correct with only 2 degrees of freedom (throttle and steering)
