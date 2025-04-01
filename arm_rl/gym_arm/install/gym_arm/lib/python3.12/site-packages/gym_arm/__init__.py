from gym.envs.registration import register

register(
    id='ArmEnv-v0',
    entry_point='gym_arm.arm_env:ArmEnv',
    max_episode_steps=100,
)
