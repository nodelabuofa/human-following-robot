#!/usr/bin/env python3

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import csv
import os

# --- CONFIGURATION / PARAMETERS ---
N = 14             # Prediction horizon (5 seconds)
DT = 0.15             # Time step [s]
VEHICLE_LENGTH = 0.174 
L = VEHICLE_LENGTH   # Wheelbase
V_MAX = 1.0
V_MIN = -1.0
DELTA_MAX = 0.45
DELTA_MIN = -0.45

# Cost weights (User updated)
Q = np.diag([3.0, 3.0, 3.0])
Q_f = np.diag([25.0, 25.0, 45])
R = np.diag([2.0, 2.0])

DEFAULT_TARGET = np.array([0.0, 0.0, 0.0])

def get_linearized_matrices(state_op, control_op, dt, L):
    v_o = control_op[0]
    delta_o = control_op[1]
    theta_o = state_op[2] 

    if abs(v_o) < 1e-3:
        v_o = 1e-3

    s_th = np.sin(theta_o)
    c_th = np.cos(theta_o)
    t_d  = np.tan(delta_o)
    sec2 = 1.0 / (np.cos(delta_o) ** 2)

    A_d = np.array([
        [1.0,  0.0,  -v_o * s_th * dt],
        [0.0,  1.0,   v_o * c_th * dt],
        [0.0,  0.0,     1.0          ]
    ])

    B1 = dt * np.array([
        [c_th,   0.0                ],
        [s_th,   0.0                ],
        [t_d / L,   v_o * sec2 / L  ]
    ])

    B2 = 0.5 * dt**2 * (v_o / L) * np.array([
        [-s_th * t_d,   -v_o * s_th * sec2],
        [ c_th * t_d,    v_o * c_th * sec2],
        [ 0.0,           0.0              ]
    ])

    B_d = B1 + B2

    d1 = dt * np.array([
         theta_o * v_o * s_th,
        -theta_o * v_o * c_th,
        -delta_o * v_o * sec2 / L
    ])

    d2 = 0.5 * dt**2 * delta_o * v_o**2 * sec2 / L * np.array([
         s_th,
        -c_th,
         0.0
    ])

    d_d = d1 + d2
    return A_d, B_d, d_d

def make_mpc_template():
    opti = ca.Opti()

    X_var = opti.variable(3, N + 1)
    U_var = opti.variable(2, N)

    p_x0   = opti.parameter(3)
    p_xref = opti.parameter(3) 
    
    # PARAMETERS FOR EVERY STEP (Time-Varying)
    p_Ad_list = [opti.parameter(3, 3) for _ in range(N)]
    p_Bd_list = [opti.parameter(3, 2) for _ in range(N)]
    p_dd_list = [opti.parameter(3) for _ in range(N)]

    opti.subject_to(X_var[:, 0] == p_x0)

    for k in range(N):
        # Apply unique linearized model at each step
        x_next = p_Ad_list[k] @ X_var[:, k] + p_Bd_list[k] @ U_var[:, k] + p_dd_list[k]
        opti.subject_to(X_var[:, k + 1] == x_next)

    opti.subject_to(opti.bounded(V_MIN,     U_var[0, :], V_MAX))
    opti.subject_to(opti.bounded(DELTA_MIN, U_var[1, :], DELTA_MAX))

    cost = 0
    for k in range(N):
        err = X_var[:, k] - p_xref
        u_k = U_var[:, k]
        cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([u_k.T, R, u_k]) 

    err_f = X_var[:, N] - p_xref
    cost += ca.mtimes([err_f.T, Q_f, err_f])

    opti.minimize(cost)

    solver_opts = {
        'ipopt.print_level': 0,  
        'ipopt.max_iter':    300,
        'ipopt.tol':         1e-6,
        'print_time':        0,
        'ipopt.sb':          'yes'
    }
    opti.solver('ipopt', solver_opts)
    
    return {
        'opti': opti, 'X_var': X_var, 'U_var': U_var,
        'p_x0': p_x0, 'p_xref': p_xref,
        'p_Ad_list': p_Ad_list, 'p_Bd_list': p_Bd_list, 'p_dd_list': p_dd_list
    }

def solve_mpc(mpc_template, state_op):
    opti = mpc_template['opti']
    X_var = mpc_template['X_var']
    U_var = mpc_template['U_var']
    
    # 1. INITIAL GUESS TRAJECTORY (Assume robot moves forward briefly then stops)
    X_guess = np.tile(state_op, (N + 1, 1)).T
    U_guess = np.zeros((2, N))
    U_guess[0, :] = 0.5 # Assume 0.5 m/s for linearization guess

    # 2. SUCCESSIVE LINEARIZATION (2 iterations usually enough for convergence)
    for iteration in range(2):
        for k in range(N):
            # RE-LINEARIZE AROUND EACH WAYPOINT in the current guess
            A_d, B_d, d_d = get_linearized_matrices(X_guess[:, k], U_guess[:, k], DT, L)
            opti.set_value(mpc_template['p_Ad_list'][k], A_d)
            opti.set_value(mpc_template['p_Bd_list'][k], B_d)
            opti.set_value(mpc_template['p_dd_list'][k], d_d)

        opti.set_value(mpc_template['p_x0'], state_op)
        opti.set_value(mpc_template['p_xref'], DEFAULT_TARGET)
        
        # Seed the solver with the guess
        opti.set_initial(X_var, X_guess)
        opti.set_initial(U_var, U_guess)

        try:
            sol = opti.solve()
            X_guess = sol.value(X_var)
            U_guess = sol.value(U_var)
        except RuntimeError:
            print(f"MPC Solver failed at iteration {iteration}")
            return None, None

    return X_guess, U_guess

def save_to_csv(trajectory, filename="trajectory.csv"):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['step', 'x', 'y', 'theta'])
        for i in range(trajectory.shape[1]):
            writer.writerow([i, trajectory[0, i], trajectory[1, i], trajectory[2, i]])
    print(f"Trajectory saved to {filename}")

def plot_trajectory(trajectory, target):
    plt.figure(figsize=(8, 6))
    plt.plot(trajectory[0, :], trajectory[1, :], color='gray', linewidth=1, label='Predicted Trajectory')
    plt.plot(target[0], target[1], 'rx', markersize=10, label='Target')
    
    # Draw orientation arrows at each waypoint
    for i in range(trajectory.shape[1]):
        # Swapped sin/cos and negated to match longitudinal heading definition
        plt.arrow(trajectory[0, i], trajectory[1, i], 
                  -0.05 * np.sin(trajectory[2, i]), -0.05 * np.cos(trajectory[2, i]),
                  head_width=0.02, head_length=0.03, fc='green', ec='green')

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('LTV-MPC Trajectory Plot (Linearized at Every Waypoint)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def main():
    # Dummy initial position: x=1.0, y=1.0, theta=pi/4
    initial_state = np.array([1.0, 1.0, np.pi/4])
    
    print(f"Starting LTV-MPC from {initial_state} towards {DEFAULT_TARGET}")
    
    mpc_template = make_mpc_template()
    X_sol, U_sol = solve_mpc(mpc_template, initial_state)

    if X_sol is not None:
        save_to_csv(X_sol)
        plot_trajectory(X_sol, DEFAULT_TARGET)
    else:
        print("Failed to compute trajectory.")

if __name__ == "__main__":
    main()
