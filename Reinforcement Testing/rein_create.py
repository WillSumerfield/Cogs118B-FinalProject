import gym
from Game.gamestate import Gamestate
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

def create_env():
    env = Gamestate(None, None)
    states = 5
    actions = env.action_space.n
    
    return env, states, actions

def build_model(states, actions):
    model = Sequential()
    model.add(Flatten(input_shape=(1,states)))
    model.add(Dense(3, activation='relu'))
    model.add(Dense(actions, activation='linear'))
    return model

def test_env(episodes, env):
    for episode in range(1, episodes+1):
        env.reset()
        done = False
        score = 0

        while not done:
            env.render()
            action = 1
            n_state, reward, done, info = env.step(action)
            score += reward
            print("Episode:{} Score:{}".format(episode, score))

def build_agent(model, actions):
    policy = BoltzmannQPolicy()
    memory = SequentialMemory(limit=50000, window_length=1)
    dnq = DQNAgent(model=model, memory=memory, policy=policy, 
                   nb_actions=actions, nb_steps_warmup=1000, target_model_update=1e-2)
    return dnq
