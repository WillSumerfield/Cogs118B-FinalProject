import gym
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from rl.agents import NAFAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
from gym.envs.registration import register
from numpy import random

# Register our environment
register(id='AirHockeyGoal-v0',
         entry_point='air_hockey.envs:AirHockeyGoalEnv',
         max_episode_steps=200)


def create_env():
    env = gym.make("AirHockeyGoal-v0")
    states = env.observation_space.shape[0]
    actions = env.action_space.shape[0]

    return env, states, actions


def build_model(states, actions):
    model = Sequential()
    model.add(Flatten(input_shape=(1, states)))
    model.add(Dense(24, activation='relu'))
    model.add(Dense(24, activation='relu'))
    model.add(Dense(actions, activation='linear'))
    return model


def test_env(episodes, env):
    for episode in range(1, episodes + 1):
        env.reset()
        done = False
        score = 0

        while not done:
            env.render()
            action = random.rand(3)
            n_state, reward, done, info = env.step(action)
            score += reward
            print("Episode:{} Score:{}".format(episode, score))


def build_agent(model, actions):
    policy = BoltzmannQPolicy()
    memory = SequentialMemory(limit=50000, window_length=1)
    agent = NAFAgent(V_model=model, memory=memory, policy=policy,
                     nb_actions=actions, nb_steps_warmup=1000, target_model_update=1e-2)
    return agent
