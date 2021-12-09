from rein_create import *
from tensorflow.keras.optimizers import Adam
import numpy as np

# Create the environment
env, states, actions = create_env()

# Create the model
model = build_model(states, actions)

# Create the agent
dnq = build_agent(model, actions)

# Compile the agent
dnq.compile(Adam(lr=1e-3), metrics=['mae'])

# Load the saved weights
dnq.load_weights("..Models/dnq_weights.h5f")

# Test the saved model
scores = dnq.test(env, nb_episodes=5, visualize=True)
print("Mean Score: " + str(np.mean(scores.history['episode_reward'])))
