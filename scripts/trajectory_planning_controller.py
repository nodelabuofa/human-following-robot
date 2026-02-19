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
L          = 0.167    # wheel-to-wheel-distance

V_MAX      =  1.0     # max forward velocity [m/s]
V_MIN      = -1.0     # min velocity (negative = reverse)
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


current_state   = np.zeros(3)            # latest observed [x, y, theta]
current_control = np.array([0.01, 0.0]) # last applied [v, delta] — CHECK WITH DR. HASHEMI IF THIS CONSIDERED OPERATING POINT
target_state    = DEFAULT_TARGET.copy()

# previous optimal trajectories shifted forward each update
prev_X_sol = None   # shape (3, N+1)
prev_U_sol = None   # shape (2, N)

drive_pub = None    # assign globally so helper functions can access

# CasADi boilerplate built with these once in build_mpc(), reused every loop
opti    = None
X_var   = None   # Decision variable: state trajectory   [3 × (N+1)]
U_var   = None   # Decision variable: control trajectory [2 × N]
p_x0    = None   # Parameter: initial state              [3]
p_Ad    = None   # Parameter: A_d matrix                 [3×3]
p_Bd    = None   # Parameter: B_d matrix                 [3×2]
p_dd    = None   # Parameter: d_d affine offset          [3]
p_xref  = None   # Parameter: reference/target state     [3]


### HELPER FUNCTIONS

# compute first order linearized (Jacobian) matrices
def get_linearized_matrices(v_o, theta_o, delta_o):
    # avoid divide by zero blowup at zero velocity
    if abs(v_o) < 1e-3:
        v_o = 1e-3

    s_th = np.sin(theta_o)
    c_th = np.cos(theta_o)
    t_d  = np.tan(delta_o)
    sec2 = 1.0 / np.cos(delta_o) ** 2    # sec^2(delta_o) = 1 / cos^2(delta_o)

    #   [ 1   0   -v_o * sin(theta_o) * dt ]
    #   [ 0   1    v_o * cos(theta_o) * dt ]
    #   [ 0   0    1                       ]
    A_d = np.array([
        [1.0,  0.0,  -v_o * s_th * DT],
        [0.0,  1.0,   v_o * c_th * DT],
        [0.0,  0.0,   1.0            ]
    ])

    #   dt * [ cos(theta_o),        0                     ]
    #        [ sin(theta_o),        0                     ]
    #        [ tan(delta_o)/L,      v_o * sec^2(delta_o)/L ]
    B1 = DT * np.array([
        [c_th,      0.0              ],
        [s_th,      0.0              ],
        [t_d / L,   v_o * sec2 / L  ]
    ])

    #   0.5 * dt^2 * (v_o/L) *  [ -sin(theta_o)*tan(delta_o),  -v_o*sin(theta_o)*sec^2(delta_o) ]
    #                           [  cos(theta_o)*tan(delta_o),   v_o*cos(theta_o)*sec^2(delta_o) ]
    #                           [  0,                           0                               ]
    B2 = 0.5 * DT**2 * (v_o / L) * np.array([
        [-s_th * t_d,   -v_o * s_th * sec2],
        [ c_th * t_d,    v_o * c_th * sec2],
        [ 0.0,           0.0              ]
    ])

    B_d = B1 + B2

    #   dt * [  theta_o * v_o * sin(theta_o)          ]
    #        [ -theta_o * v_o * cos(theta_o)          ]
    #        [ -delta_o * v_o * sec^2(delta_o) / L    ]
    d1 = DT * np.array([
         theta_o * v_o * s_th,
        -theta_o * v_o * c_th,
        -delta_o * v_o * sec2 / L
    ])

    #   0.5 * dt^2 * delta_o * v_o^2 * sec^2(delta_o) / L * [  sin(theta_o) ]
    #                                                         [ -cos(theta_o) ]
    #                                                         [  0            ]
    d2 = 0.5 * DT**2 * delta_o * v_o**2 * sec2 / L * np.array([
         s_th,
        -c_th,
         0.0
    ])

    d_d = d1 + d2

    return A_d, B_d, d_d


# builds boilerplate, like compiling a regex pattern. Done once at startup
def build_mpc():
    global opti, X_var, U_var, p_x0, p_Ad, p_Bd, p_dd, p_xref

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


# called every iteration, solves trajectory
def solve_mpc():
    # takes linearized matrices, state and control, spits out only next control action, publishing to ROS
    global current_control, prev_X_sol, prev_U_sol

    # operating point
    v_op     = current_control[0]
    delta_op = current_control[1]
    theta_op = current_state[2]

    if abs(v_op) < 1e-3:
        v_op = 1e-3   # avoids divide by zero blowup, I forget where though lol

    A_d, B_d, d_d = get_linearized_matrices(v_op, theta_op, delta_op)

    opti.set_value(p_x0,   current_state)
    opti.set_value(p_xref, target_state)
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
        u_opt          = prev_U_sol[:, 0]   # [v, delta]
        current_control = u_opt             # store for next linearization

        # publish to ESP32 on car
        twist = Twist()
        twist.linear.x  = float(u_opt[0])   # velocity [m/s]
        twist.angular.z = float(u_opt[1])   # steering angle [rad]
        drive_pub.publish(twist)

        rospy.loginfo_throttle(
            0.5,
            "[MPC] v=%.3f m/s | delta=%.2f deg | x_err=%.3f m | y_err=%.3f m" % (
                u_opt[0],
                np.degrees(u_opt[1]),
                current_state[0] - target_state[0],
                current_state[1] - target_state[1]
            )
        )

    except RuntimeError:
        rospy.logwarn("[MPC] Solver failed (infeasible or max iterations). Sending STOP.")
        # publish zero velocity if doesn't work
        drive_pub.publish(Twist())
        # reset so next attempt starts fresh
        prev_X_sol = None
        prev_U_sol = None



###ROS CALLBACKS
def pose_callback(msg):
    # gets ArUco pose, updates and triggers new MPC solve
    global current_state

    if len(msg.data) < 3:
        rospy.logwarn("[MPC] pose_callback: incomplete data (got %d values, need 3).", len(msg.data))
        return

    x_lat  = float(msg.data[0])
    y_long = float(msg.data[1])
    theta  = np.radians(float(msg.data[2]))   # degrees → radians

    current_state = np.array([x_lat, y_long, theta])
    solve_mpc()


###MAIN FUNCTINOA
def main():
    global drive_pub

    rospy.init_node('mpc_node', anonymous=False)
    rospy.loginfo("[MPC] Initializing node...")

    # build CasADi MPC solver boilerplate once to avoid reinitializing every iteration
    build_mpc()

    # ROS publisher and subscriber
    drive_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/aruco_pose_topic', Float32MultiArray, pose_callback)

    rospy.loginfo("[MPC] Ready. Listening on /aruco_pose_topic")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
