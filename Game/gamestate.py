import typing

import numpy
import numpy as np


class Gamestate:
    """
    This class contains the current state of a game.
    It contains the subclasses Striker and Ball, which represent the balls and strikers on the field.
    """

    # region Environment Positions

    BOARD_WIDTH = 600
    BOARD_HEIGHT = 200
    BOARD_CENTER_X = BOARD_WIDTH / 2
    BOARD_CENTER_Y = BOARD_HEIGHT / 2

    GOAL_SIZE = 120
    GOAL_Y = BOARD_HEIGHT - (GOAL_SIZE / 2)

    STRIKER_CENTER_OFFSET = 200
    LEFT_STRIKER_STARTING_X = BOARD_CENTER_X - STRIKER_CENTER_OFFSET
    LEFT_STRIKER_STARTING_Y = BOARD_CENTER_Y - STRIKER_CENTER_OFFSET
    RIGHT_STRIKER_STARTING_X = BOARD_CENTER_X + STRIKER_CENTER_OFFSET
    RIGHT_STRIKER_STARTING_Y = BOARD_CENTER_Y + STRIKER_CENTER_OFFSET

    LEFT, RIGHT = False, True

    # endregion Environment Positions

    def __init__(self, left_agent, right_agent):

        # Assign the agents
        self.left_agent = left_agent
        self.right_agent = right_agent

        # Create the strikers
        self.left_striker = self.Striker(self.LEFT)
        self.right_striker = self.Striker(self.RIGHT)

        # Create the Ball
        self.ball = self.Ball()

    class Striker:
        """
        This class describes the strikers used by the agents to hit the game ball.

        The striker is effectively a circle which can collide and rebound against the walls, center of the field,
        and the game ball.
        Agents can give the striker a velocity. The velocity of strikers decays over time. Once a striker reaches a
        velocity of 0, a cooldown timer begins. Until the cooldown timer is complete, the striker cannot move.
        """

        # The radius of the striker circle
        RADIUS = 20

        # The cooldown in frames after the striker has stopped moving
        MAX_COOLDOWN = 30

        def __init__(self, side):
            """Creates an unmoving striker of the given side"""

            self.side = side
            self._velocity = np.zeros(2)
            self._cooldown = self.MAX_COOLDOWN

            # Check Starting Side
            if side == Gamestate.LEFT:
                self.position = np.array([[Gamestate.LEFT_STRIKER_STARTING_X], [Gamestate.LEFT_STRIKER_STARTING_Y]])
            else:
                self.position = np.array([[Gamestate.RIGHT_STRIKER_STARTING_X], [Gamestate.RIGHT_STRIKER_STARTING_Y]])

        def set_velocity(self, velocity: np.array) -> bool:
            """Sets the velocity, and checks if the striker can move

            :param velocity  -- the left agent's input, in the form [velocity (0-1), want_to_move (True-False)]

            :returns True/False if the striker could/couldn't move
            """

            if self._cooldown == self.MAX_COOLDOWN:
                self._velocity = velocity
                return True
            else:
                return False


    class Ball:
        """This class describes the game ball used to score points"""

        # The radius of the ball circle
        RADIUS = 20

        def __init__(self):
            """Creates an unmoving ball at the center of the field"""
            self.position = np.zeros((2, 1))
            self.velocity = np.zeros((2, 1))

    def tick(self, left_agent_input: numpy.array, right_agent_input: np.array) -> np.array:
        """Steps through a single frame of the game

        :param left_agent_input  -- the left agent's input, in the form [velocity (0-1), want_to_move (True-False)]
        :param right_agent_input -- the right agent's input, in the form [velocity (0-1), want_to_move (True-False)]

        :returns A np column with the form [left_striker_x, left_striker_y, left_striker_x_velocity, 
                                            left_striker_y_velocity, left_striker_cooldown, left_striker_can_move,
                                            right_striker_x, right_striker_y, right_striker_x_velocity, 
                                            right_striker_y_velocity, right_striker_cooldown, right_striker_can_move,
                                            ball_x, ball_y, ball_x_velocity, ball_y_velocity]
        """

        # Update the Strikers

        # Update the Ball

        # Check for Goals

        return np.array([])

    def wall_collision(self, instance: typing.Union[Striker, Ball], wall_x, wall_y):
        pass

    def ball_collision(self, striker: Striker):
        """"""

    def game_end(self):
        """Ends the game and rewards the agents."""

        pass
