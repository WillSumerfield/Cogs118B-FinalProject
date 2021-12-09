from rein_create import *
from tensorflow.keras.optimizers import Adam

# Create the environment
env, states, actions = create_env()

# Create the model
model = build_model(states, actions)

# Create the agent
dnq = build_agent(model, actions)

# Compile the agent
dnq.compile(Adam(lr=1e-3), metrics=['mae'])

# Load the saved weights
dnq.load_weights("../Models/dnq_weights.h5f")

# Resume training
dnq.fit(env, nb_steps=50000, visualize=False)

# Save the model
dnq.save_weights('../Models/dnq_weights.h5f', overwrite=True)
