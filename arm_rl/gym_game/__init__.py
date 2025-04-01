from gym.envs.registration import register

register(
    id='ArmEnv-v0',
    entry_point='gym_game.envs:ArmEnv',
    max_episode_steps=100,
)
