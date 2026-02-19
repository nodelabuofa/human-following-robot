# mpc_param_env.py
from isaaclab.envs import DirectRLEnv
import torch

class AckermannMPCParamEnv(DirectRLEnv):
    cfg: AckermannMPCParamEnvCfg

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        # spawn N parallel copies, get waypoint targets, etc.

    def _get_observations(self) -> dict:
        pose_error = self._compute_pose_error()   # from ArUco sim analogue
        velocity    = self.robot.data.root_lin_vel_b
        curvature   = self._lookahead_curvature()
        nominal_params = self.current_mpc_params   # track current baseline
        obs = torch.cat([pose_error, velocity, curvature, nominal_params], dim=-1)
        return {"policy": obs}

    def _apply_action(self):
        # RL outputs parameter DELTAS, not direct commands
        param_deltas = self.actions * self.cfg.action_scale
        adjusted_params = self.nominal_mpc_params + param_deltas
        adjusted_params = torch.clamp(adjusted_params, self.cfg.param_min, self.cfg.param_max)

        # Call your MPC here — implement as a Python module wrapping your C++ solver
        steering, throttle = self.mpc_solver.solve(
            self.robot.data.root_state_w, adjusted_params
        )
        self.robot.set_joint_position_target(steering, joint_ids=self.steer_ids)
        self.robot.set_joint_velocity_target(throttle, joint_ids=self.drive_ids)

    def _get_rewards(self) -> torch.Tensor:
        tracking_reward  = -torch.norm(self._compute_pose_error(), dim=-1)
        smoothness       = -torch.norm(self.actions - self.prev_actions, dim=-1)
        return tracking_reward + 0.1 * smoothness
