# Trajectory Planning Controller

**Goal**: shadow human, face them at fixed distance (0.2m) when they stop

**Nutshell**: Plans a trajectory over multiple time steps, aware of motion constraints.

**Significance**: Intelligent, behaviour tunable, robust.

**Progress**: Developing for deployment


# The Upgrade

Visual error controller thought the car could jump left/right and go at Mach speed. This **predictive planner and controller** is actually aware of the car's kinematic model and control constraints.

# Problem Statement  

Looking under the hood, we're minimizing control effort and tracking error. Penalty tuning needed to balance lazy response with aggressive overshooting.

More specifically, this is a Linear Model Predictive Controller. I linearize to simplify how sine/cosine/tangent vary nonlinearly.

sequenceDiagram
    participant T as Time (t)
    participant P as Planner (Optimizer)
    participant R as Robot (Hardware)
    
    Note over T,R: The Receding Horizon Loop
    
    T->>P: 1. Current State (x, y, v)
    P->>P: 2. Predict next N steps<br/>(Minimize Cost J)
    P->>R: 3. Execute ONLY 1st Step (u0)
    R->>R: 4. Move (Physics happens)
    R->>T: 5. New State (Error/Noise added)
    
    Note over T,R: Repeat loop at t+1




This project implements an autonomous human-following system for a car-like robot. The core of this branch is the transition from a reactive **Visual Servoing** approach to a **Linear Time-Varying Model Predictive Controller (LTV-MPC)**, allowing for smoother, more natural motion that respects the physical constraints of the robot.

## 🎯 The Goal
The robot must follow a human at a fixed distance (0.2m) and orientation, effectively "shadowing" their movements.

---

## 🔄 The Technical Evolution

### From Visual Servoing (Reactive)
Our previous method used **Image-Based Visual Servoing (IBVS)**. It calculated an "error" based on how the ArUco marker appeared in the camera frame and tried to minimize it instantly.
*   **The Problem:** It assumed the robot could move sideways (holonomic) and lacked a "memory" of the robot's momentum or future path.
*   **See [Problem Statement.pdf](docs/Problem%20Statement.pdf)** for the original challenges.

### To LTV-MPC (Predictive)
The new **LTV-MPC** approach looks ahead 1 second into the future (10 steps of 0.1s). Every time the robot gets a new position from the camera, it solves an optimization problem to find the best sequence of steering and speed.
*   **Why LTV?** Since the robot's motion is non-linear (it turns in curves), we **linearize** the model around its current speed and heading every step. This makes the math simple enough to solve in real-time.
*   **The Result:** The robot "anticipates" curves and slows down or speeds up gracefully.

---

## 🛠 System Architecture

### 1. Perception & State Estimation
Using a ZED Mini camera, we detect ArUco markers to determine the human's relative position $[x, y, \theta]$.
*   **Reference:** [ArUco Marker Tracking](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)

### 2. Kinematic Bicycle Model
We treat the robot like a bicycle. This ensures the controller never asks the wheels to turn sharper than they physically can or "slide" sideways.
*   **Key Constraints:** Steering angle ($\delta$) is limited to $\pm 25^\circ$.
*   **Further Reading:** [The Kinematic Bicycle Model](https://www.shuffleai.blog/blog/Kinematic_Bicycle_Model.html)

### 3. Optimization with CasADi
We use the **CasADi** framework to translate our control problem into math that the **IPOPT** solver can understand. It minimizes a "cost function" that penalizes being too far from the human and using too much "jerk" in the steering.
*   **Tool:** [CasADi Optimization Suite](https://web.casadi.org/)

---

## 📂 Documentation & Math
For the deep-dives into the derivations and logic, refer to the `docs/` folder:
- **[Derivation.pdf](docs/Derivation.pdf)**: The full LTV state-space derivation and linearization process.
- **[Pseudocode.pdf](docs/Pseudocode.pdf)**: A step-by-step breakdown of the control loop.
- **[Problem Statement.pdf](docs/Problem%20Statement.pdf)**: Detailed breakdown of the 4 challenges of human following.

---

## 🚀 Quick Start
```bash
# 1. Start the ROS environment and Vision
roslaunch human_following_robot aruco.launch

# 2. Launch the MPC Controller
python3 scripts/trajectory_planning_controller.py
```

*Note: This project requires ROS Noetic and the CasADi python library.*
