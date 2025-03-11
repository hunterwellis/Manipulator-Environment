#!/usr/bin/env python3
import gym
import gym_arm

env = gym.make("ArmEnv-v0")  # Uses the registered environment
obs = env.reset()

for _ in range(100):
    action = env.action_space.sample()  # Take a random action
    obs, reward, done, _ = env.step(action)
    if done:
        env.reset()
