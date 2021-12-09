import numpy as np
from rein_create import *
from keras.optimizer_v1 import Adam
Adam._name = None

# Create the gym environment
env, states, actions = create_env()

# Build the model
models = build_model(states, actions)

# Build the agent
dnq = build_agent(models, actions)

# Test the agent
test_env(5, env)

# Train the model
dnq.compile(Adam(lr=1e-3), metrics=['mae'])

# Train the model
while True:
    # Fit the model
    dnq.fit(env, nb_steps=10000, visualize=True)
    # Save the model
    dnq.save_weights('Reinforcement Testing/dnq_weights.h5f', overwrite=True)
