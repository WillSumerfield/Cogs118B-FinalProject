"""This file contains the all the code used to run the airhockey game"""

import math
import typing
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
    GOAL_MIN_Y = BOARD_HEIGHT - (GOAL_SIZE / 2)
    GOAL_MAX_Y = GOAL_MIN_Y + GOAL_SIZE

    STRIKER_CENTER_OFFSET = 200
    LEFT_STRIKER_STARTING_X = BOARD_CENTER_X - STRIKER_CENTER_OFFSET
    LEFT_STRIKER_STARTING_Y = 0
    RIGHT_STRIKER_STARTING_X = BOARD_CENTER_X + STRIKER_CENTER_OFFSET
    RIGHT_STRIKER_STARTING_Y = 0

    LEFT, RIGHT = False, True
    BOTTOM, TOP = False, True

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

        # The percentage of energy transferred between a striker and a ball
        ENERGY_TRANSFER = 0.8

        def __init__(self, side):
            """Creates an unmoving striker of the given side"""

            self.side = side
            self._velocity = np.zeros(2)
            self._cooldown = self.MAX_COOLDOWN

            # Set the starting position
            if side == Gamestate.LEFT:
                self._position = np.array([Gamestate.LEFT_STRIKER_STARTING_X], [Gamestate.LEFT_STRIKER_STARTING_Y])
            else:
                self._position = np.array([Gamestate.RIGHT_STRIKER_STARTING_X], [Gamestate.RIGHT_STRIKER_STARTING_Y])

        def set_velocity(self, velocity: np.array) -> bool:
            """Sets the velocity, and checks if the striker can move

            :param velocity  -- the left agent's input, in the form [velocity (0-1), want_to_move (True-False)]

            :returns True/False if the striker could/couldn't move
            """

            if (self._cooldown == self.MAX_COOLDOWN) and :
                self._velocity = velocity
                return True
            else:
                return False

        def find_closeness(self, ball):
            """Returns the striker's closeness to the ball and walls in their direction, in terms of ticks until
            contact.

            :returns (ball_distance, horizontal_wall_distance, horizontal_wall_type - LEFT or RIGHT,
                      vertical_wall_distance, vertical_wall_type - TOP or BOTTOM)
            """

            ball_distance = math.sqrt(((ball._position[0] - self._position[0]) ** 2
                                       + (ball._position[1] - self._position[1]) ** 2
                                       - (Gamestate.Ball.RADIUS - self.RADIUS) ** 2)
                                      / ((ball._velocity[0] - self._velocity[0])**2
                                         + (ball._velocity[1] - self._velocity[1])**2))

            # Get Horizontal Wall Distance
            if self._velocity[0] > 0:
                horizontal_wall_distance =
                horizontal_wall_type = Gamestate.RIGHT
            elif self._velocity[0] < 0:
                horizontal_wall_distance = 1
                horizontal_wall_type = Gamestate.LEFT
            else:
                horizontal_wall_distance = None
                horizontal_wall_type = None

            # Get Vertical Wall Distance
            if self._velocity[1] > 0:
                vertical_wall_distance = 1
                vertical_wall_type = Gamestate.TOP
            elif self._velocity[1] < 0:
                vertical_wall_distance = 1
                vertical_wall_type = Gamestate.BOTTOM
            else:
                vertical_wall_distance = None
                vertical_wall_type = None

            return (ball_distance, horizontal_wall_distance, horizontal_wall_type,
                    vertical_wall_distance, vertical_wall_type)

    class Ball:
        """This class describes the game ball used to score points"""

        # The radius of the ball circle
        RADIUS = 20

        def __init__(self):
            """Creates an unmoving ball at the center of the field"""
            self._position = np.zeros(2)
            self._velocity = np.zeros(2)

        def in_goal(self) -> typing.Union[(bool, (bool, bool))]:
            """Returns whether the ball has scored a goal, and which goal was scored on
            Ex: (True, RIGHT) if the ball went into the right goal

            :returns False if not, (True, LEFT or RIGHT) if true
            """

            # Check if y is within the correct bounds
            if (self._position[1] >= Gamestate.GOAL_MIN_Y) and (self._position[1] <= Gamestate.GOAL_MAX_Y):
                # Check if x is within the correct bounds
                if self._position[0] < self.RADIUS:
                    return True, Gamestate.LEFT
                elif self._position[0] > Gamestate.BOARD_WIDTH - self.RADIUS:
                    return True, Gamestate.RIGHT
                # Not within goal bounds
                else:
                    return False,

    def tick(self, left_agent_input: np.array, right_agent_input: np.array) -> typing.Union[np.array, bool]:
        """Steps through a single frame of the game

        :param left_agent_input  -- the left agent's input, in the form [velocity (0-1), want_to_move (True-False)]
        :param right_agent_input -- the right agent's input, in the form [velocity (0-1), want_to_move (True-False)]

        :returns A np array with the form [left_striker_x, left_striker_y, left_striker_x_velocity,
                                            left_striker_y_velocity, left_striker_cooldown, left_striker_can_move,
                                            right_striker_x, right_striker_y, right_striker_x_velocity, 
                                            right_striker_y_velocity, right_striker_cooldown, right_striker_can_move,
                                            ball_x, ball_y, ball_x_velocity, ball_y_velocity]
                or LEFT or RIGHT for a goal score
        """

        # Calculate how close everything will be to one another

        # Update the Strikers

        # Update the Ball

        # Check for Goals
        goal = self.ball.in_goal()
        if goal[0]: return goal[1]

        return np.array([])

    def wall_collision(self, instance: typing.Union[Striker, Ball], wall_x: float, wall_y: float):
        pass

    def ball_collision(self, striker: Striker):
        """"""

    def game_end(self, winning_side: bool) -> np.array():
        """Ends the game and rewards the agents

        :returns A np array with the form [left_agent_reward, right_agent_reward]
        """

        return np.array([1, 0]) if winning_side == Gamestate.LEFT else np.array([0, 1])
