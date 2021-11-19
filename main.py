"""This file is used to run the game."""

from Game.gamestate import Gamestate
from Game.visuals import GameWindow
import numpy as np
import sched
import time

# Create a new game
game = Gamestate(None, None)

# Create a new game window
window = GameWindow(game)

# Testing
#game.ball.velocity = np.array([0, 0])
game.ball.position = np.array([300.0, 0.0])
game.left_striker.velocity = np.array([1, 0])


# Define how to run the game
def run(s):
    game.tick(None, None)
    window.set_gamestate(game)
    window.window.update()
    scheduler.enter(1/60, 1, run, (s,))


# Run the game at regular intervals
scheduler = sched.scheduler(time.time, time.sleep)

scheduler.enter(1/60, 1, run, (scheduler,))
scheduler.run()
