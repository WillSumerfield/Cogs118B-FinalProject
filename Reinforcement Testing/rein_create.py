"""This code is partially copied from https://github.com/keras-rl/keras-rl/blob/master/examples/naf_pendulum.py
Credit for the model topology goes to Raphael Meudec"""

import gym
from keras import backend, Model, Input
from keras.activations import sigmoid, linear
from rl.policy import BoltzmannQPolicy
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from rl.agents import DQNAgent
from rl.memory import SequentialMemory
from gym.envs.registration import register
from numpy import random, array
from rl.core import Processor


# Register our environment
register(id='AirHockeyGoal-v0',
         entry_point='air_hockey.envs:AirHockeyGoalEnv',
         max_episode_steps=200)


class GoalProcessor(Processor):

    def process_reward(self, reward):
        # The magnitude of the reward can be important. Since each step yields a relatively
        # high reward, we reduce the magnitude by two orders.
        return reward / 100.


# A custom activation function to range between min and max values
def scaled_activation(x, target_min=0, target_max=1):

    # Put x in between 0 and 2 in a tanh
    sig_x = backend.tanh(x) + 1

    # Scale the output between the two values
    return (sig_x * ((target_max-target_min)/2)) + target_min


def create_env():
    env = gym.make("AirHockeyGoal-v0")
    random.seed(123)
    env.seed(123)
    states = env.observation_space.shape[0]
    actions = env.action_space.n

    return env, states, actions


def build_model(states, actions):

    model = Sequential()

    # Input layer
    model.add(Flatten(input_shape=(1, states)))

    # Hidden Layers
    model.add(Dense(24, activation=sigmoid))
    model.add(Dense(24, activation=sigmoid))

    # Output layer
    model.add(Dense(actions, activation=linear))

    return model


def test_env(episodes, env):
    for episode in range(1, episodes + 1):
        env.reset()
        done = False
        score = 0

        while not done:
            env.render()
            action = random.rand()*5
            print(action)
            n_state, reward, done, info = env.step(action)
            score += reward

        print("Episode:{} Score:{}".format(episode, score))


def build_agent(model, actions):
    policy = BoltzmannQPolicy()
    memory = SequentialMemory(limit=10000, window_length=1)
    dqn = DQNAgent(model=model, memory=memory, policy=policy, nb_actions=actions, nb_steps_warmup=100,
                   target_model_update=1e-2)
    return dqn
