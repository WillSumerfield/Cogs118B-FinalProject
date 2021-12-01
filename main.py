"""This file is used to run the game."""

from Game.gamestate import Gamestate
from Game.visuals import GameWindow
import numpy as np
import sched
import time

# The inverse of the number of frames per second
FPS = 1/36

# Create a new game
game = Gamestate(None, None)

# Create a new game window
window = GameWindow(game)

# Testing
game.left_striker.velocity = np.array([0.0,0.0])
game.right_striker.velocity = np.array([-50.0,8.0])

# Define how to run the game
def run(s):
    game.tick(None, None)
    window.set_gamestate(game)
    window.window.update()
    scheduler.enter(FPS, 1, run, (s,))

# Run the game at regular intervals
scheduler = sched.scheduler(time.time, time.sleep)
scheduler.enter(FPS, 1, run, (scheduler,))
scheduler.run()
