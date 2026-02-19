# scripts/train.py
import gymnasium as gym
from skrl.agents.torch.sac import SAC, SAC_DEFAULT_CONFIG
from skrl.envs.wrappers.torch import wrap_env

env = gym.make("AckermannMPCParam-v0", num_envs=512, headless=True)
env = wrap_env(env)

cfg = SAC_DEFAULT_CONFIG.copy()
cfg["gradient_steps"] = 1
cfg["batch_size"] = 4096
cfg["learn_entropy"] = True  # adaptive temperature — good for param spaces

agent = SAC(models={"policy": ..., "critic_1": ..., "critic_2": ...},
            memory=..., cfg=cfg, observation_space=env.observation_space,
            action_space=env.action_space)
agent.init()
agent.train(timesteps=500_000)
