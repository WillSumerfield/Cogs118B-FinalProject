import numpy as np
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from rein_create import *

# Create the gym environment
env, states, actions = create_env()

# Build the model
model = build_model(states, actions)

# Build the agent
dnq = build_agent(model, actions)

# Test the agent
#test_env(5, env)

# Train the model
dnq.compile(Adam(lr=1e-3), metrics=['mae'])
dnq.fit(env, nb_steps=50000, visualize=False)

# Test the model
scores = dnq.test(env, nb_episodes=20, visualize=True)
print(np.mean(scores.history['episode_reward']))

# Save the model
dnq.save_weights('Reinforcement Testing/dnq_weights.h5f', overwrite=True)
