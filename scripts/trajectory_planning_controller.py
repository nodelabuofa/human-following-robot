#!/usr/bin/env python3

"""
Takes end goal (face human from 0.2m away),
finding optimal trajectory for next 5-15 timesteps, and outputting

State  : x = [x_pos, y_pos, theta]   (relative to ArUco/human)
Control: u = [v, delta]              (velocity [m/s], steering angle [rad])

Discretization then linearization gives state-space model:
    x(k+1) = A_d(k) * x(k) + B_d(k) * u(k) + d_d(k)

Refer to docs/'MPC Derivation.pdf' for full derivation
"""

import rospy
import casadi as ca
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# GLOBAL/CONFIG

N          = 10       # prediction horizon (number of time steps to look ahead)
DT         = 0.1      # fixed time step [s]
WHEELBASE = 0.167    # rear wheel-to-wheel distance
VEHICLE_LENGTH = 0.174 # rear wheel to front wheel distance

V_MAX      =  1.0     # max forward velocity [m/s]
V_MIN      = -1.0  
DELTA_MAX  =  0.45    # max steering angle [rad]  (~25 degrees)
DELTA_MIN  = -0.45

### BEHAVIOURAL PARAMETERS - balance smooth driving (lazy response) with adhering to trajectory
# just coefficients scaling tracking error penalty for state/[x, y, theta]
Q   = np.diag([10.0, 10.0, 1.0])

# penalizes error at end of horizaon
Q_f = np.diag([100.0, 100.0, 10.0])

# control effort penalty, doesn't like large [v, delta] inputs
R   = np.diag([1.0, 1.0])

# constant target terminal state
DEFAULT_TARGET = np.array([0.0, 0.2, 0.0])

# GLOBAL STATE (so callback functions can access)
current_control = np.array([0.01, 0.0]) # last applied [v, delta] — CHECK WITH DR. HASHEMI IF THIS CONSIDERED OPERATING POINT
prev_X_sol = None   # shape (3, N+1)
prev_U_sol = None   # shape (2, N)
mpc_template = None 
drive_pub = None 

def main():
    global drive_pub, mpc_template
    
    rospy.init_node('mpc_node', anonymous=False)
    rospy.loginfo("[MPC] Initializing node...")
    
    # functional programming
    # compile boilerplate 'optimization-problem'-object for CasADi solver
    mpc_template = make_mpc_template()

    drive_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # unpack actual throttle/steering from encoders, but currently it just uses previous control input to simplify things
    # callbakcs = 'dont bother me until you got mail'
    rospy.Subscriber('/current_vel_steer', Float32MultiArray, parse_prev_control_callback)
    
    # when new human position comes, compute new trajectory
    rospy.Subscriber('/aruco_pose_topic', Float32MultiArray, trajectory_planner_callback)

    rospy.loginfo("[MPC] Ready. Listening on /aruco_pose_topic")
    rospy.spin()

def make_mpc_template():
    # BUILDS BOILERPLATE, LIKE COMPILING A REGEX PATTERN. DONE ONCE AT STARTUP
    opti = ca.Opti()

    X_var = opti.variable(3, N + 1)   # state at k = 0, 1, ..., N, state has one more because we use current state to get 'next control'
    U_var = opti.variable(2, N)       # control inputs at k = 0, 1, ..., N-1

    p_x0   = opti.parameter(3)        # operating point state (ie. x(t-1))
    p_Ad   = opti.parameter(3, 3)     # A_d, gotten by inputing operating point state and control
    p_Bd   = opti.parameter(3, 2)     # B_d same
    p_dd   = opti.parameter(3) 
    p_xref = opti.parameter(3) 

    # current observed state always first state
    opti.subject_to(X_var[:, 0] == p_x0)

    # constrain to linearized/discretized state-space model
    for k in range(N):
        x_next = p_Ad @ X_var[:, k] + p_Bd @ U_var[:, k] + p_dd
        opti.subject_to(X_var[:, k + 1] == x_next)

    # control action limits
    opti.subject_to(opti.bounded(V_MIN,     U_var[0, :], V_MAX))
    opti.subject_to(opti.bounded(DELTA_MIN, U_var[1, :], DELTA_MAX))

    cost = 0
    for k in range(N):
        err = X_var[:, k] - p_xref
        u_k = U_var[:, k]
        # functionally the same as LQR, ie. quadratic penalty
        cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([u_k.T, R, u_k]) 

    # penalty for not facing QR code and being 0.2m from human at end of trajectory
    err_f = X_var[:, N] - p_xref
    cost += ca.mtimes([err_f.T, Q_f, err_f])

    opti.minimize(cost)

    # using IPOPT solver from CasADi
    solver_opts = {
        'ipopt.print_level': 0,  
        'ipopt.max_iter':    300,
        'ipopt.tol':         1e-6,
        'print_time':        0,
        'ipopt.sb':          'yes'
    }
    opti.solver('ipopt', solver_opts)
    
    rospy.loginfo("[MPC] Problem built. Horizon N=%d, dt=%.2f s, L=%.3f m", N, DT, L)

    # bundle template into dictionary to return
    mpc_template = {
        'opti': opti,
        'X_var': X_var,
        'U_var': U_var,
        'p_x0': p_x0,
        'p_Ad': p_Ad,
        'p_Bd': p_Bd,
        'p_dd': p_dd,
        'p_xref': p_xref
    }
    return mpc_template

def parse_prev_control_callback(msg):
    # unpack actual throttle/steer motor encoder speed/positions from Float32Multiarray
    global current_control
    if len(msg.data) >= 2:
        current_control = np.array([msg.data[0], msg.data[1]])
    return current_control

def get_linearized_matrices(state_op, control_op, dt, L):
    
    v_o = control_op[0]
    delta_o = control_op[1]
    theta_o = state_op[2] 

    # avoid divide by zero blowup at zero velocity
    if abs(v_o) < 1e-3:
        v_o = 1e-3

    s_th = np.sin(theta_o)
    c_th = np.cos(theta_o)
    t_d  = np.tan(delta_o)
    sec2 = 1.0 / (np.cos(delta_o) ** 2 )  # sec^2(delta_o) = 1 / cos^2(delta_o)

    #   [ 1   0   -v_o * sin(theta_o) * dt ]
    #   [ 0   1    v_o * cos(theta_o) * dt ]
    #   [ 0   0    1                       ]
    A_d = np.array([
        [1.0,  0.0,  -v_o * s_th * dt],
        [0.0,  1.0,   v_o * c_th * dt],
        [0.0,  0.0,     1.0          ]
    ])

    #   dt * [ cos(theta_o),        0                     ]
    #        [ sin(theta_o),        0                     ]
    #        [ tan(delta_o)/L,      v_o * sec^2(delta_o)/L ]
    B1 = dt * np.array([
        [c_th,   0.0                ],
        [s_th,   0.0                ],
        [t_d / L,   v_o * sec2 / L  ]
    ])

    #   0.5 * dt^2 * (v_o/L) *  [ -sin(theta_o)*tan(delta_o),  -v_o*sin(theta_o)*sec^2(delta_o) ]
    #                           [  cos(theta_o)*tan(delta_o),   v_o*cos(theta_o)*sec^2(delta_o) ]
    #                           [  0,                           0                               ]
    B2 = 0.5 * dt**2 * (v_o / L) * np.array([
        [-s_th * t_d,   -v_o * s_th * sec2],
        [ c_th * t_d,    v_o * c_th * sec2],
        [ 0.0,           0.0              ]
    ])

    B_d = B1 + B2

    #   dt * [  theta_o * v_o * sin(theta_o)          ]
    #        [ -theta_o * v_o * cos(theta_o)          ]
    #        [ -delta_o * v_o * sec^2(delta_o) / L    ]
    d1 = dt * np.array([
         theta_o * v_o * s_th,
        -theta_o * v_o * c_th,
        -delta_o * v_o * sec2 / L
    ])

    #   0.5 * dt^2 * delta_o * v_o^2 * sec^2(delta_o) / L * [  sin(theta_o) ]
    #                                                         [ -cos(theta_o) ]
    #                                                         [  0            ]
    d2 = 0.5 * dt**2 * delta_o * v_o**2 * sec2 / L * np.array([
         s_th,
        -c_th,
         0.0
    ])

    d_d = d1 + d2

    linearized_matrices = (A_d, B_d, d_d)
    return linearized_matrices
    
def trajectory_planner_callback(aruco_pose_msg):
    # when new human position comes, compute new trajectory
    # WHEN NEW HUMAN POSITION COMES, COMPUTE NEW TRAJECTORY
    global current_control
    
    if len(aruco_pose_msg.data) < 3:
        rospy.logwarn("[MPC] pose_callback: incomplete data (got %d values, need 3).", len(aruco_pose_msg.data))
        return

    # extract state from Float32MultiArray message
    x_lat  = float(aruco_pose_msg.data[0])
    y_long = float(aruco_pose_msg.data[1])
    theta  = np.radians(float(aruco_pose_msg.data[2]))   # degrees -> radians
    state_op = np.array([x_lat, y_long, theta])
    
    # linearize matrices, inputting t-1 state and control, ie. operating point
    linearized_matrices = get_linearized_matrices(state_op, current_control, DT, L)
    
    # plug into casadi's input and get solution
    trajectory = solve_mpc(mpc_template, linearized_matrices, state_op, current_control)
    
    if trajectory is not None:
        next_action = trajectory
        # turns numpy array into Twist() and publishes to car
        pack_publish_control(next_action)
        return next_action
    else:
        return None

def solve_mpc(mpc_template, linearized_matrices, state_op, control_op):
    global prev_X_sol, prev_U_sol, current_control
    
    # unpack template
    opti   = mpc_template['opti']
    X_var  = mpc_template['X_var']
    U_var  = mpc_template['U_var']
    p_x0   = mpc_template['p_x0']
    p_Ad   = mpc_template['p_Ad']
    p_Bd   = mpc_template['p_Bd']
    p_dd   = mpc_template['p_dd']
    p_xref = mpc_template['p_xref']
    
    # unpack matrices
    A_d, B_d, d_d = linearized_matrices

    opti.set_value(p_x0,   state_op)
    opti.set_value(p_xref, DEFAULT_TARGET) # USING CONSTANT TARGET
    opti.set_value(p_Ad,   A_d)
    opti.set_value(p_Bd,   B_d)
    opti.set_value(p_dd,   d_d)

    # feed previous trajectory shifted one step to give good initial guess
    if prev_X_sol is not None and prev_U_sol is not None:
        # drop first column
        X_warm = np.hstack([prev_X_sol[:, 1:], prev_X_sol[:, -1:]])
        # drop first column
        U_warm = np.hstack([prev_U_sol[:, 1:], prev_U_sol[:, -1:]])
        opti.set_initial(X_var, X_warm)
        opti.set_initial(U_var, U_warm)

    try:
        sol = opti.solve()

        # store full trajectory
        prev_X_sol = sol.value(X_var)   
        prev_U_sol = sol.value(U_var)   

        # optimal next control action
        u_opt = prev_U_sol[:, 0]   # [v, delta]
        current_control = u_opt    # store for next linearization (optional, as we have callback)

        return u_opt # TRAJECTORY (NEXT ACTION)

    except RuntimeError:
        rospy.logwarn("[MPC] Solver failed (infeasible or max iterations). Sending STOP.")
        # publish zero velocity if doesn't work
        # pack_publish_control(np.zeros(2)) # OPTIONALLY STOP HERE
        # reset so next attempt starts fresh
        prev_X_sol = None
        prev_U_sol = None
        return None

def pack_publish_control(next_action):
    # turns numpy array back into Twist vector to publish to car
    v = next_action[0]
    delta = next_action[1]
    
    updated_twist_vector = differential_wheel_speed(v, delta)
    
    drive_pub.publish(updated_twist_vector)
    
def differential_wheel_speed(velocity, steer):
    global VEHICLE_LENGTH, WHEELBASE
    vehicle_length = VEHICLE_LENGTH
    wheelbase = WHEELBASE
    updated_twist_vector = Twist()

    if abs(velocity) < 1e-2: # prevents division by zero and huge steering values
        updated_twist_vector.angular.y = 0.0 # steering

    # linear.x is left wheel, linear.y is right wheel
    # left is positive radian max, right is negative radian max
    # 0.1778 is vehicle length (wheel center to center), 0.1667 wheelbase (rear tires, tread center to center)
    if steer == 0:
        # z axis portrudes perpendicular from lens, so z from computed twist is desired linear velocity of end effector/camera
        updated_twist_vector.linear.x = float(velocity)
        updated_twist_vector.linear.y = float(velocity)
    else:
        turning_radius = vehicle_length / np.tan(steer) # ackerman steering equation
        ackerman_differential_ratio = (abs(turning_radius) - (wheelbase/2)) / (abs(turning_radius) + (wheelbase/2))
        
        if turning_radius > 0: # turning left
            updated_twist_vector.linear.y = float(velocity) # outer (right) wheel has max velocity
            updated_twist_vector.linear.x = float(float(velocity) * ackerman_differential_ratio)
        else: # turning right
            updated_twist_vector.linear.x = float(velocity) # outer (left) wheel has max velocity
            updated_twist_vector.linear.y = float(float(velocity) * ackerman_differential_ratio)

    return updated_twist_vector

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
