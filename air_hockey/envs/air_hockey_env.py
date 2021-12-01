import gym
from gym import spaces
import numpy as np
from air_hockey.envs.airhockeygame import AirHockeyGame

"""This file contains the environment class defined by OpenAIGym, for the AirHockey game."""

class AirHockeyEnv(gym.env):
    """This is an environment class, which is a wrapper for an Air Hockey game."""

    def __init__(self):
        """This method creates the environment, and a new Air Hockey game."""
        self.pygame = AirHockeyGame()
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Discrete(4)

    
    def reset(self):
        """This method resets the game."""
        del self.pygame
        self.pygame = AirHockeyGame()
        obs = self.pygame.observe()
        return obs


    def step(self, action):
        """This method steps through the game."""
        self.pygame.action(action)
        obs = self.pygame.observe()
        reward = self.pygame.evaluate()
        done = self.pygame.is_done()
        return obs, reward, done, {}


    def render(self):
        """This method renders the game."""
        self.pygame.view()