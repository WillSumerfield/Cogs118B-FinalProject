"""This file contains the GameWindow class, which is used to draw the game to the screen."""

from tkinter import *
from Game.gamestate import Gamestate
from typing import Union


class GameWindow:
    """This class defines a window used to draw the game. It must be updated manually - each time you set the gamestate,
    it is like setting a picture, not a movie."""

    #region Constants

    GOAL_WIDTH = 50

    #endregion Constants

    def __init__(self, game: Union[Gamestate, None]):
        """Creates a new window with the given gamestate currently displayed

        :param game - The gamestate to display. This value can also be None, for an empty screen.
        """

        # Create the window
        self.window = Tk()

        # Title the Window
        self.window.title("Game Window")

        # Get the width and height of the screen
        width = Gamestate.BOARD_WIDTH + (self.GOAL_WIDTH * 2)
        height = Gamestate.BOARD_HEIGHT

        # Set the window size
        self.window.configure(width=width, height=height)

        # Stop the window from being resizeable
        self.window.resizable(False, False)

        # Center the window
        x_pos = int(self.window.winfo_screenwidth() / 2 - Gamestate.BOARD_WIDTH / 2)
        y_pos = int(self.window.winfo_screenheight() / 2 - Gamestate.BOARD_HEIGHT / 2)
        self.window.geometry("+{}+{}".format(x_pos, y_pos))

        # Create a canvas
        self.canvas = Canvas(self.window, bg='light gray', width=width, height=height)

        # Set the current game
        if game is not None: self.set_gamestate(game)

    def set_gamestate(self, game: Gamestate):
        """Draws a new gamestate to the window

        :param game - The gamestate to display"""

        # Clear the current canvas
        self.canvas.delete('all')

        # Draw the Goals
        self.canvas.create_rectangle(0, game.GOAL_MIN_Y, self.GOAL_WIDTH, game.GOAL_MAX_Y, fill="green")
        self.canvas.create_rectangle(Gamestate.BOARD_WIDTH + (self.GOAL_WIDTH * 2), game.GOAL_MIN_Y,
                                     Gamestate.BOARD_WIDTH + (self.GOAL_WIDTH * 2) - self.GOAL_WIDTH,
                                     game.GOAL_MAX_Y, fill="green")

        # Draw board perimeter
        self.canvas.create_rectangle(self.GOAL_WIDTH, 3,
                                     Gamestate.BOARD_WIDTH + (self.GOAL_WIDTH * 2) - self.GOAL_WIDTH, game.BOARD_HEIGHT,
                                     outline="black", width=2)

        # Draw center line
        self.canvas.create_line((Gamestate.BOARD_WIDTH/2) + self.GOAL_WIDTH, 0,
                                (Gamestate.BOARD_WIDTH/2) + self.GOAL_WIDTH, Gamestate.BOARD_HEIGHT)

        # Draw the Ball
        self.canvas.create_oval(self.GOAL_WIDTH + game.ball.position[0] + game.ball.RADIUS,
                                game.ball.position[1] + game.ball.RADIUS,
                                self.GOAL_WIDTH + game.ball.position[0] - game.ball.RADIUS,
                                game.ball.position[1] - game.ball.RADIUS,
                                fill="orange")

        # Draw the Strikers
        self.canvas.create_oval(self.GOAL_WIDTH + game.left_striker.position[0] + game.left_striker.RADIUS,
                                game.left_striker.position[1] + game.left_striker.RADIUS,
                                self.GOAL_WIDTH + game.left_striker.position[0] - game.left_striker.RADIUS,
                                game.left_striker.position[1] - game.left_striker.RADIUS,
                                fill="red")
        self.canvas.create_oval(self.GOAL_WIDTH + game.right_striker.position[0] + game.right_striker.RADIUS,
                                game.right_striker.position[1] + game.right_striker.RADIUS,
                                self.GOAL_WIDTH + game.right_striker.position[0] - game.right_striker.RADIUS,
                                game.right_striker.position[1] - game.right_striker.RADIUS,
                                fill="red")

        # Refresh the canvas
        self.canvas.pack()

