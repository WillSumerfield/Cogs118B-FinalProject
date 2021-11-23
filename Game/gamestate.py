"""This file contains the all the code used to run the Air Hockey game"""

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
    BOARD_HEIGHT = 400
    BOARD_CENTER_X = BOARD_WIDTH / 2
    BOARD_CENTER_Y = BOARD_HEIGHT / 2

    GOAL_SIZE = 120
    GOAL_MIN_Y = BOARD_CENTER_Y - (GOAL_SIZE / 2)
    GOAL_MAX_Y = GOAL_MIN_Y + GOAL_SIZE

    STRIKER_CENTER_OFFSET = 200
    LEFT_STRIKER_STARTING_X = BOARD_CENTER_X - STRIKER_CENTER_OFFSET
    LEFT_STRIKER_STARTING_Y = BOARD_CENTER_Y
    RIGHT_STRIKER_STARTING_X = BOARD_CENTER_X + STRIKER_CENTER_OFFSET
    RIGHT_STRIKER_STARTING_Y = BOARD_CENTER_Y

    LEFT, RIGHT, BOTTOM, TOP = 1, 2, 3, 4

    COLLISION = {"BALL_LEFT_STRIKER": 0, "BALL_RIGHT_STRIKER": 1, "BALL_HORIZONTAL_WALL": 2, "BALL_VERTICAL_WALL": 3,
                 "LEFT_STRIKER_HORIZONTAL_WALL": 4, "LEFT_STRIKER_VERTICAL_WALL": 5,
                 "RIGHT_STRIKER_HORIZONTAL_WALL": 6, "RIGHT_STRIKER_VERTICAL_WALL": 7}

    ALL_COLLISIONS = [True] * 8

    # A value used to represent non-colliding object's distances
    NOT_CLOSE = 1000

    # The magnitude of speed lost per frame 
    SLOWDOWN = 0.995

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
        RADIUS = 10

        # The cooldown in frames after the striker has stopped moving
        MAX_COOLDOWN = 30

        # The percentage of energy transferred between a striker and a ball
        ENERGY_TRANSFER = 0.8

        def __init__(self, side: int):
            """Creates an unmoving striker of the given side"""

            self.side = side
            self.velocity = np.zeros(2)
            self._cooldown = self.MAX_COOLDOWN

            # Set the starting position
            if side == Gamestate.LEFT:
                self.position = np.array([Gamestate.LEFT_STRIKER_STARTING_X, Gamestate.LEFT_STRIKER_STARTING_Y])
            else:
                self.position = np.array([Gamestate.RIGHT_STRIKER_STARTING_X, Gamestate.RIGHT_STRIKER_STARTING_Y])

        def set_velocity(self, velocity: np.array) -> bool:
            """Sets the velocity, and checks if the striker can move
            The striker can move if it is off cooldown and if it is not moving

            :param velocity  -- the left agent's input, in the form [velocity (0-1), want_to_move (True-False)]

            :returns True/False if the striker could/couldn't move
            """

            # Set velocity if the cooldown is at max and if the velocity is at 0
            if (self._cooldown == self.MAX_COOLDOWN) and (self.velocity == np.zeros(2)):
                self.velocity = velocity
                self._cooldown = self.MAX_COOLDOWN
                return True
            else:
                return False

        def find_wall_closeness(self) -> (float, int, float, int):
            """Returns the striker's closeness to the horizontal and vertical wall in it's direction,
            in terms of ticks until contact.

            :returns (horizontal_wall_distance, horizontal_wall_type - LEFT or RIGHT,
                      vertical_wall_distance, vertical_wall_type - TOP or BOTTOM)
            """

            # Get Horizontal Wall Distance
            if self.velocity[0] > 0:
                if self.side == Gamestate.RIGHT:
                    horizontal_wall_distance = (Gamestate.BOARD_WIDTH - self.position[0] - self.RADIUS) \
                                               / self.velocity[0]
                else:
                    horizontal_wall_distance = (Gamestate.BOARD_CENTER_X - self.position[0] - self.RADIUS) \
                                               / self.velocity[0]
                horizontal_wall_type = Gamestate.RIGHT
            elif self.velocity[0] < 0:
                if self.side == Gamestate.RIGHT:
                    horizontal_wall_distance = (self.position[0] - Gamestate.BOARD_CENTER_X - self.RADIUS) / -self.velocity[0]
                else:
                    horizontal_wall_distance = (self.position[0] - self.RADIUS) \
                                               / -self.velocity[0]
                horizontal_wall_type = Gamestate.LEFT
            else:
                horizontal_wall_distance = Gamestate.NOT_CLOSE
                horizontal_wall_type = Gamestate.NOT_CLOSE

            # Get Vertical Wall Distance
            if self.velocity[1] > 0:
                vertical_wall_distance = (Gamestate.BOARD_HEIGHT - self.position[1] - self.RADIUS) / self.velocity[1]
                vertical_wall_type = Gamestate.TOP
            elif self.velocity[1] < 0:
                vertical_wall_distance = (self.position[1] - self.RADIUS) / -self.velocity[1]
                vertical_wall_type = Gamestate.BOTTOM
            else:
                vertical_wall_distance = Gamestate.NOT_CLOSE
                vertical_wall_type = Gamestate.NOT_CLOSE

            return horizontal_wall_distance, horizontal_wall_type, vertical_wall_distance, vertical_wall_type

    class Ball:
        """This class describes the game ball used to score points"""

        # The radius of the ball circle
        RADIUS = 20

        def __init__(self):
            """Creates an unmoving ball at the center of the field"""
            self.position = np.array([Gamestate.BOARD_WIDTH / 2, Gamestate.BOARD_HEIGHT / 2])
            self.velocity = np.zeros(2)

        def find_wall_closeness(self) -> (float, int, float, int):
            """Returns the striker's closeness to the horizontal and vertical wall in it's direction,
            in terms of ticks until contact.

            :returns (horizontal_wall_distance, horizontal_wall_type - LEFT or RIGHT,
                      vertical_wall_distance, vertical_wall_type - TOP or BOTTOM)
            """

            # Get Horizontal Wall Distance
            if self.velocity[0] > 0:
                horizontal_wall_distance = (Gamestate.BOARD_WIDTH - self.position[0] - self.RADIUS) / self.velocity[0]
                horizontal_wall_type = Gamestate.RIGHT
            elif self.velocity[0] < 0:
                horizontal_wall_distance = (self.position[0] - self.RADIUS) / -self.velocity[0]
                horizontal_wall_type = Gamestate.LEFT
            else:
                horizontal_wall_distance = Gamestate.NOT_CLOSE
                horizontal_wall_type = Gamestate.NOT_CLOSE

            # Get Vertical Wall Distance
            if self.velocity[1] > 0:
                vertical_wall_distance = (Gamestate.BOARD_HEIGHT - self.position[1] - self.RADIUS) / self.velocity[1]
                vertical_wall_type = Gamestate.TOP
            elif self.velocity[1] < 0:
                vertical_wall_distance = (self.position[1] - self.RADIUS) / -self.velocity[1]
                vertical_wall_type = Gamestate.BOTTOM
            else:
                vertical_wall_distance = Gamestate.NOT_CLOSE
                vertical_wall_type = Gamestate.NOT_CLOSE

            return horizontal_wall_distance, horizontal_wall_type, vertical_wall_distance, vertical_wall_type

    def ball_distance_to_striker(self, striker: Striker) -> float:
        """Returns the distance in frames from the ball to a given striker

        :param striker - The striker to the ball's distance from
        """

        # Find the differences in velocity and position
        pos_diff = self.ball.position - striker.position
        vel_diff = self.ball.velocity - striker.velocity

        # Check the overall velocity
        b = (pos_diff[0] * vel_diff[0]) + (pos_diff[1] * vel_diff[1])

        # If the striker is not nearing the ball, exit now
        if b >= 0: return Gamestate.NOT_CLOSE

        # Find components of quadratic
        a = (vel_diff[0] ** 2) + (vel_diff[1] ** 2)
        b = 2 * b
        c = (pos_diff[0] ** 2) + (pos_diff[1] ** 2) - ((self.ball.RADIUS + striker.RADIUS) ** 2)

        # Check if the ball and striker will not collide
        if b ** 2 < 4 * a * c: return Gamestate.NOT_CLOSE

        # Find both of the potential times
        contact_1 = (-b + math.sqrt((b ** 2) - (4 * a * c))) / (2 * a)
        contact_2 = (-b - math.sqrt((b ** 2) - (4 * a * c))) / (2 * a)

        # Return the first time the circles contact
        return min(contact_1, contact_2)

    def wall_collision(self, instance: typing.Union[Striker, Ball], wall: int):
        """Calculates a collision with the given striker. Returns the remaining velocity.

        :param instance - The instance the calculate collisions with
        :param wall - The wall collided with
        """

        # Check the given wall
        if (wall == Gamestate.TOP) or (wall == Gamestate.BOTTOM):
            # If it is a horizontal wall

            # Remove small errors in y
            instance.position[1] = round(instance.position[1])

            # Invert the y velocity
            instance.velocity[1] = -instance.velocity[1]

        else:
            # If it is a vertical wall

            # Remove small errors in x
            instance.position[0] = round(instance.position[0])

            # Update the x velocity
            instance.velocity[0] = -instance.velocity[0]

    def ball_collision(self, striker: Striker):
        """Calculates the collision between a given striker and the ball.

        :param striker - The striker the calculate collisions with
        """

        # region Update Velocities

        # Calculate the difference in positions
        pos_diff = self.ball.position - striker.position
        pos_diff_mag = pos_diff[0]**2 + pos_diff[1]**2

        # Store a copy of the ball's old velocity
        ball_old_velocity = np.copy(self.ball.velocity)

        # Find the velocity of the ball
        self.ball.velocity = self.ball.velocity \
                             - (((self.ball.velocity - striker.velocity).dot(pos_diff) /
                                 pos_diff_mag) * pos_diff)

        # Find the velocity of the striker
        striker.velocity = striker.velocity \
                           - (((striker.velocity - ball_old_velocity).dot(-pos_diff) /
                               pos_diff_mag) * -pos_diff)

        # endregion Update Velocities

    def increment_positions(self, frames: float):
        """Increments the positions of the ball and strikers by their velocity, given a span of time (frames).

        :param frames - The number of frames to increment the positions by
        """

        self.ball.position += self.ball.velocity * frames
        self.left_striker.position += self.left_striker.velocity * frames
        self.right_striker.position += self.right_striker.velocity * frames

    def calculate_collisions(self, frames_remaining: float, previous_collision: int):
        """Calculates the number of frames before each collision. Run the closest one.
        This function will be recalled by collisions, to check for subsequent collisions, which previously were not
        detected.
        """

        # A list of collisions to check
        collision_list = [self.NOT_CLOSE, self.NOT_CLOSE, self.NOT_CLOSE, self.NOT_CLOSE, self.NOT_CLOSE,
                          self.NOT_CLOSE, self.NOT_CLOSE, self.NOT_CLOSE]

        # Calculate how close everything will be to one another, in terms of frames

        # Check for ball-striker collisions
        if previous_collision != self.COLLISION["BALL_LEFT_STRIKER"]:
            collision_list[self.COLLISION["BALL_LEFT_STRIKER"]] = \
                self.ball_distance_to_striker(self.left_striker)
        if previous_collision != self.COLLISION["BALL_RIGHT_STRIKER"]: 
            collision_list[self.COLLISION["BALL_RIGHT_STRIKER"]] = \
                self.ball_distance_to_striker(self.right_striker)

        # Check for ball-wall collisions
        ball_closeness = self.ball.find_wall_closeness()
        if previous_collision != self.COLLISION["BALL_HORIZONTAL_WALL"]:
            collision_list[self.COLLISION["BALL_HORIZONTAL_WALL"]] = ball_closeness[0]
        if previous_collision != self.COLLISION["BALL_VERTICAL_WALL"]:
            collision_list[self.COLLISION["BALL_VERTICAL_WALL"]] = ball_closeness[2]

        # Check for left_striker-wall collisions
        left_striker_closeness = self.left_striker.find_wall_closeness()
        if previous_collision != self.COLLISION["LEFT_STRIKER_HORIZONTAL_WALL"]:
            collision_list[self.COLLISION["LEFT_STRIKER_HORIZONTAL_WALL"]] = left_striker_closeness[0]
        if previous_collision != self.COLLISION["LEFT_STRIKER_VERTICAL_WALL"]:
            collision_list[self.COLLISION["LEFT_STRIKER_VERTICAL_WALL"]] = left_striker_closeness[2]
        
        # Check for right_striker-wall collisions
        right_striker_closeness = self.right_striker.find_wall_closeness()
        if previous_collision != self.COLLISION["RIGHT_STRIKER_HORIZONTAL_WALL"]:
            collision_list[self.COLLISION["RIGHT_STRIKER_HORIZONTAL_WALL"]] = right_striker_closeness[0]
        if previous_collision != self.COLLISION["RIGHT_STRIKER_VERTICAL_WALL"]:
            collision_list[self.COLLISION["RIGHT_STRIKER_VERTICAL_WALL"]] = right_striker_closeness[2]

        # Sort the smallest distance
        shortest_distance = min(collision_list)

        # Check if any collisions occur this frame
        if shortest_distance <= frames_remaining:
            # If there is a collision this frame

            # Increment all positions
            self.increment_positions(shortest_distance)

            # Remove the remaining frames
            frames_remaining -= shortest_distance

            # Tracks which collision occurs
            collision = None

            # Run the collision for the shortest distance
            if shortest_distance == collision_list[self.COLLISION["BALL_LEFT_STRIKER"]]:
                # If the first collision is the ball and left striker
                self.ball_collision(self.left_striker)
                collision = self.COLLISION["BALL_LEFT_STRIKER"]

            elif shortest_distance == collision_list[self.COLLISION["BALL_RIGHT_STRIKER"]]:
                # If the first collision is the ball and right striker
                self.ball_collision(self.right_striker)
                collision = self.COLLISION["BALL_RIGHT_STRIKER"]

            elif shortest_distance == collision_list[self.COLLISION["BALL_HORIZONTAL_WALL"]]:
                # If the first collision is the ball and a horizontal wall
                self.wall_collision(self.ball, ball_closeness[1])
                collision = self.COLLISION["BALL_HORIZONTAL_WALL"]

            elif shortest_distance == collision_list[self.COLLISION["BALL_VERTICAL_WALL"]]:
                # If the first collision is the ball and a vertical wall
                self.wall_collision(self.ball, ball_closeness[3])
                collision = self.COLLISION["BALL_VERTICAL_WALL"]

            elif shortest_distance == collision_list[self.COLLISION["LEFT_STRIKER_HORIZONTAL_WALL"]]:
                # If the first collision is the left striker and a horizontal wall
                self.wall_collision(self.left_striker, left_striker_closeness[1])
                collision = self.COLLISION["LEFT_STRIKER_HORIZONTAL_WALL"]

            elif shortest_distance == collision_list[self.COLLISION["LEFT_STRIKER_VERTICAL_WALL"]]:
                # If the first collision is the left striker and a vertical wall
                self.wall_collision(self.left_striker, left_striker_closeness[3])
                collision = self.COLLISION["LEFT_STRIKER_VERTICAL_WALL"]

            elif shortest_distance == collision_list[self.COLLISION["RIGHT_STRIKER_HORIZONTAL_WALL"]]:
                # If the first collision is the right striker and a horizontal wall
                self.wall_collision(self.right_striker, right_striker_closeness[1])
                collision = self.COLLISION["RIGHT_STRIKER_HORIZONTAL_WALL"]

            else:
                # If the first collision is the right striker and a vertical wall
                self.wall_collision(self.right_striker, right_striker_closeness[3])
                collision = self.COLLISION["RIGHT_STRIKER_VERTICAL_WALL"]

            # Recalculate Collisions in case there are any new collisions due to previous ones
            if collision != None: self.calculate_collisions(frames_remaining, collision)
        else:
            # If there were no collisions in this check, increment the positions the rest of the way
            self.increment_positions(frames_remaining)

    def tick(self, left_agent_input: np.array, right_agent_input: np.array) -> typing.Union[np.array, int]:
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

        # Calculate the collisions in order
        self.calculate_collisions(1, None)

        # Decay the speeds
        self.ball.velocity *= self.SLOWDOWN
        self.left_striker.velocity *= self.SLOWDOWN
        self.right_striker.velocity *= self.SLOWDOWN

        # Return the current state
        return np.array([])
