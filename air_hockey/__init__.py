from gym.envs.registration import register

register(id='AirHockey-v0',
         entry_point='air_hockey.envs:AirHockeyEnv',
         max_episode_steps=1000000)