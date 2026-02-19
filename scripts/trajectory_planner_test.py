#!/usr/bin/env python3
"""
Standalone MPC Trajectory Planner Test (Non-ROS)
Tests the MPC logic from trajectory_planning_controller.py with a dummy initial state.
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION (from trajectory_planning_controller.py) ---
N          = 15       # prediction horizon
DT         = 0.1      # time step [s]
L          = 0.167    # wheel-to-wheel-distance
V_MAX      = 0.5      # slower for testing
V_MIN      = -0.5
DELTA_MAX  = 0.45
DELTA_MIN  = -0.45

Q   = np.diag([20.0, 20.0, 5.0])
Q_f = np.diag([100.0, 100.0, 50.0])
R   = np.diag([1.0, 1.0])

# Human Frame: Origin [0,0] is the Human.
# y is distance, x is lateral offset. 
# theta = 0 means robot is facing the human.
TARGET_STATE = np.array([0.0, 0.2, 0.0])
INITIAL_STATE = np.array([1.5, 3.0, 0.2]) # [x, y, theta]

def get_linearized_matrices(v_o, theta_o, delta_o):
    """
    Inverted Kinematics (Human Frame):
    x_dot = -v * sin(theta)
    y_dot = -v * cos(theta)
    th_dot = (v/L) * tan(delta)
    """
    if abs(v_o) < 1e-3:
        v_o = 1e-3

    s_th = np.sin(theta_o)
    c_th = np.cos(theta_o)
    t_d  = np.tan(delta_o)
    sec2 = 1.0 / np.cos(delta_o) ** 2

    # A_d = I + Ac * dt
    # Ac = [0, 0, -v*cos(th)]
    #      [0, 0,  v*sin(th)]
    #      [0, 0,  0        ]
    A_d = np.array([
        [1.0,  0.0,  -v_o * c_th * DT],
        [0.0,  1.0,   v_o * s_th * DT],
        [0.0,  0.0,   1.0            ]
    ])

    # B1 = Bc * dt
    # Bc = [-sin(th), 0]
    #      [-cos(th), 0]
    #      [tan(d)/L, v*sec2(d)/L]
    B1 = DT * np.array([
        [-s_th,      0.0              ],
        [-c_th,      0.0              ],
        [t_d / L,    v_o * sec2 / L  ]
    ])

    # B2 = 0.5 * dt^2 * Ac * Bc
    B2 = 0.5 * DT**2 * (v_o / L) * np.array([
        [-c_th * t_d,   -v_o * c_th * sec2],
        [ s_th * t_d,    v_o * s_th * sec2],
        [ 0.0,           0.0              ]
    ])
    B_d = B1 + B2

    # d1 = dt * (f(xo,uo) - Ac*xo - Bc*uo)
    d1 = DT * np.array([
         theta_o * v_o * c_th,
        -theta_o * v_o * s_th,
        -delta_o * v_o * sec2 / L
    ])

    # d2 = 0.5 * dt^2 * Ac * dc
    d2 = 0.5 * DT**2 * delta_o * v_o**2 * sec2 / L * np.array([
         c_th,
        -s_th,
         0.0
    ])
    d_d = d1 + d2

    return A_d, B_d, d_d

class MPCPlanner:
    def __init__(self):
        self.opti = ca.Opti()
        self.X_var = self.opti.variable(3, N + 1)
        self.U_var = self.opti.variable(2, N)
        
        self.p_x0   = self.opti.parameter(3)
        self.p_Ad   = self.opti.parameter(3, 3)
        self.p_Bd   = self.opti.parameter(3, 2)
        self.p_dd   = self.opti.parameter(3)
        self.p_xref = self.opti.parameter(3)

        self.opti.subject_to(self.X_var[:, 0] == self.p_x0)

        for k in range(N):
            x_next = self.p_Ad @ self.X_var[:, k] + self.p_Bd @ self.U_var[:, k] + self.p_dd
            self.opti.subject_to(self.X_var[:, k + 1] == x_next)

        self.opti.subject_to(self.opti.bounded(V_MIN,     self.U_var[0, :], V_MAX))
        self.opti.subject_to(self.opti.bounded(DELTA_MIN, self.U_var[1, :], DELTA_MAX))

        cost = 0
        for k in range(N):
            err = self.X_var[:, k] - self.p_xref
            u_k = self.U_var[:, k]
            cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([u_k.T, R, u_k])

        err_f = self.X_var[:, N] - self.p_xref
        cost += ca.mtimes([err_f.T, Q_f, err_f])

        self.opti.minimize(cost)

        solver_opts = {
            'ipopt.print_level': 0,
            'ipopt.max_iter':    300,
            'ipopt.tol':         1e-6,
            'print_time':        0,
            'ipopt.sb':          'yes'
        }
        self.opti.solver('ipopt', solver_opts)

        self.prev_X_sol = None
        self.prev_U_sol = None

    def solve(self, current_state, target_state, current_control):
        v_op     = current_control[0]
        delta_op = current_control[1]
        theta_op = current_state[2]

        A_d, B_d, d_d = get_linearized_matrices(v_op, theta_op, delta_op)

        self.opti.set_value(self.p_x0,   current_state)
        self.opti.set_value(self.p_xref, target_state)
        self.opti.set_value(self.p_Ad,   A_d)
        self.opti.set_value(self.p_Bd,   B_d)
        self.opti.set_value(self.p_dd,   d_d)

        if self.prev_X_sol is not None and self.prev_U_sol is not None:
            X_warm = np.hstack([self.prev_X_sol[:, 1:], self.prev_X_sol[:, -1:]])
            U_warm = np.hstack([self.prev_U_sol[:, 1:], self.prev_U_sol[:, -1:]])
            self.opti.set_initial(self.X_var, X_warm)
            self.opti.set_initial(self.U_var, U_warm)

        sol = self.opti.solve()
        self.prev_X_sol = sol.value(self.X_var)
        self.prev_U_sol = sol.value(self.U_var)
        
        return self.prev_X_sol, self.prev_U_sol

def simulate_kinematics(x, u):
    """Inverted non-linear kinematics for simulation"""
    x_pos, y_pos, theta = x
    v, delta = u
    
    x_next = x_pos - v * np.sin(theta) * DT
    y_next = y_pos - v * np.cos(theta) * DT
    theta_next = theta + (v / L) * np.tan(delta) * DT
    
    return np.array([x_next, y_next, theta_next])

def main():
    planner = MPCPlanner()
    
    current_state = INITIAL_STATE.copy()
    current_control = np.array([0.1, 0.0])
    
    sim_steps = 100
    state_history = [current_state]
    control_history = []
    
    print(f"Starting simulation from {INITIAL_STATE} to {TARGET_STATE}...")
    
    for i in range(sim_steps):
        try:
            X_pred, U_pred = planner.solve(current_state, TARGET_STATE, current_control)
            
            # Apply the first control action
            u_apply = U_pred[:, 0]
            control_history.append(u_apply)
            
            # Update state using non-linear kinematics (the "real" world)
            current_state = simulate_kinematics(current_state, u_apply)
            state_history.append(current_state)
            
            # Update current control for next linearization
            current_control = u_apply
            
            dist = np.linalg.norm(current_state[:2] - TARGET_STATE[:2])
            if dist < 0.05:
                print(f"Goal reached in {i} steps!")
                break
                
        except Exception as e:
            print(f"Solver failed at step {i}: {e}")
            break
            
    state_history = np.array(state_history)
    control_history = np.array(control_history)
    
    # --- PLOTTING ---
    plt.figure(figsize=(12, 5))
    
    # Trajectory Plot
    plt.subplot(1, 2, 1)
    plt.plot(state_history[:, 0], state_history[:, 1], 'b-o', label='Trajectory', markersize=3)
    plt.plot(TARGET_STATE[0], TARGET_STATE[1], 'rx', label='Target', markersize=10)
    plt.plot(INITIAL_STATE[0], INITIAL_STATE[1], 'go', label='Start')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('MPC Trajectory Simulation')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Controls Plot
    plt.subplot(1, 2, 2)
    plt.plot(control_history[:, 0], label='Velocity [m/s]')
    plt.plot(np.degrees(control_history[:, 1]), label='Steering [deg]')
    plt.xlabel('Step')
    plt.title('Control Inputs')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    print("Plotting results. Close the window to exit.")
    plt.show()

if __name__ == '__main__':
    main()
