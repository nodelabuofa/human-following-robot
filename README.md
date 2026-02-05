# Problem Statement

Following a human is a useful skill for robots, but implementing this on small, low-cost platforms has 4 challenges:

1. Finding the person; human appearance changes with orientation, distance and lighting.
2. Locking onto them as they walk behind things and reappear
3. Estimating position without depth cameras or LiDAR is mathematically ambiguous.
4. Trajectory planning and environment mapping with constraints on compute

# Solution

1. Use QR/ArUco codes to find and track people instead
3. **Use 'where they are' - 'where they should be' as the error, no mapping or depth needed!**

![Visual Servoing Diagram](images/visual-servoing.png)

The PID feedback controller relates motion of QR code's corners in the video feed to the car's motion in real life. 

![Interaction Matrix](images/motion.png)

The derivation is clever, I've explained it simply, assuming no background; feel free to see my notes in ./Notes.


# Results

![PID IBVS Controller](images/output.gif)


### Key Challenges

Works well when QR code radially oriented, but struggles in scenarios needing maneouvring as controller **assumes car can 'drive sideways'**, and also **doesn't plan motion.**



