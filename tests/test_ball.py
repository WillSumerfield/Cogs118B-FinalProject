"""This file is used to test whether the gamestate is working as intended"""
import math

import pytest
from Game.gamestate import Gamestate
import numpy as np


# region Fixtures

@pytest.fixture()
def game():
    return Gamestate(None, None)


@pytest.fixture()
def ball_bounced_off_left():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([-game.BOARD_CENTER_X - game.Ball.RADIUS, 0])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_right():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([game.BOARD_CENTER_X + game.Ball.RADIUS, 0])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_top():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([0, game.BOARD_CENTER_Y + game.Ball.RADIUS])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_bottom():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([0, -game.BOARD_CENTER_Y - game.Ball.RADIUS])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_left_clockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([-game.BOARD_CENTER_X - game.Ball.RADIUS, 10])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_right_clockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([game.BOARD_CENTER_Y + game.Ball.RADIUS, -10])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_top_clockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([10, game.BOARD_CENTER_Y + game.Ball.RADIUS])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_bottom_clockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([-10, -game.BOARD_CENTER_Y - game.Ball.RADIUS])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_left_anticlockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([-game.BOARD_CENTER_X - game.Ball.RADIUS, -10])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_right_anticlockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([game.BOARD_CENTER_Y + game.Ball.RADIUS, 10])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_top_anticlockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([-10, game.BOARD_CENTER_Y + game.Ball.RADIUS])
    game.ball._tick()
    return game


@pytest.fixture()
def ball_bounced_off_bottom_anticlockwise():
    game = Gamestate(None, None)
    game.ball._velocity = np.array([10, -game.BOARD_CENTER_Y - game.Ball.RADIUS])
    game.ball._tick()
    return game


# Dynamic Fixture
# Sets the left striker at the center of the field, with a ball moving towards and bouncing off of it
def centered_left_striker_and_ball_at_position(x, y):
    game = Gamestate(None, None)
    game.left_striker._position = np.array([0, 0])
    game.ball._position = np.array([x, y])
    game.ball._velocity = np.array([-x, -y])
    game.tick()
    return game


# Dynamic Fixture
# Sets the right striker at the center of the field, with a ball moving towards and bouncing off of it
def centered_right_striker_and_ball_at_position(x, y):
    game = Gamestate(None, None)
    game.right_striker._position = np.array([0, 0])
    game.ball._position = np.array([x, y])
    game.ball._velocity = np.array([-x, -y])
    game.tick()
    return game


# endregion Fixtures

def test_position(game, ball_bounced_off_left, ball_bounced_off_right, ball_bounced_off_top, ball_bounced_off_bottom,
                  ball_bounced_off_left_clockwise, ball_bounced_off_right_clockwise, ball_bounced_off_top_clockwise,
                  ball_bounced_off_bottom_clockwise, ball_bounced_off_left_anticlockwise,
                  ball_bounced_off_right_anticlockwise, ball_bounced_off_top_anticlockwise,
                  ball_bounced_off_bottom_anticlockwise):

    # Check for wall perfectly horizontal wall bounces
    assert np.equal(ball_bounced_off_left.ball.position, 
                    np.array([game.ball.RADIUS * 2, game.BOARD_CENTER_Y]))
    assert np.equal(ball_bounced_off_right.ball.position, 
                    np.array([game.BOARD_WIDTH - game.ball.RADIUS * 2, game.BOARD_CENTER_Y]))

    # Check for wall perfectly vertical wall bounces
    assert np.equal(ball_bounced_off_top.ball.position, 
                    np.array([game.BOARD_CENTER_Y, game.BOARD_HEIGHT - game.ball.RADIUS * 2]))
    assert np.equal(ball_bounced_off_bottom.ball.position, 
                    np.array([game.BOARD_CENTER_Y, game.ball.RADIUS * 2]))

    # Check for clockwise bounces on all walls
    assert np.equal(ball_bounced_off_left_clockwise.ball.position, 
                    np.array([game.ball.RADIUS * 2, game.BOARD_CENTER_X+10]))
    assert np.equal(ball_bounced_off_right_clockwise.ball.position,
                    np.array([game.BOARD_WIDTH - game.ball.RADIUS * 2, game.BOARD_CENTER_X-10]))
    assert np.equal(ball_bounced_off_top_clockwise.ball.position,
                    np.array([game.BOARD_CENTER_Y+10, game.BOARD_HEIGHT - game.ball.RADIUS * 2]))
    assert np.equal(ball_bounced_off_bottom_clockwise.ball.position,
                    np.array([game.BOARD_CENTER_Y-10, game.ball.RADIUS * 2]))

    # Check for anticlockwise bounces on all walls
    assert np.equal(ball_bounced_off_left_anticlockwise.ball.position,
                    np.array([game.ball.RADIUS * 2, game.BOARD_CENTER_X - 10]))
    assert np.equal(ball_bounced_off_right_anticlockwise.ball.position,
                    np.array([game.BOARD_WIDTH - game.ball.RADIUS * 2, game.BOARD_CENTER_X + 10]))
    assert np.equal(ball_bounced_off_top_anticlockwise.ball.position,
                    np.array([game.BOARD_CENTER_Y - 10, game.BOARD_HEIGHT - game.ball.RADIUS * 2]))
    assert np.equal(ball_bounced_off_bottom_anticlockwise.ball.position,
                    np.array([game.BOARD_CENTER_Y + 10, game.ball.RADIUS * 2]))

    # Check for many collisions with each striker
    for i in range(16):
        degree = (i * 2 * math.pi) / 16
        left = centered_left_striker_and_ball_at_position(10 * math.cos(degree), 10 * math.sin(degree))
        right = centered_right_striker_and_ball_at_position(10 * math.cos(degree), 10 * math.sin(degree))
        assert np.equals(left.ball._position, np.array([game.BOARD_CENTER_X - (2 * math.cos(degree)),
                                                        game.BOARD_CENTER_Y - (2 * math.sin(degree))]))
        assert np.equals(right.ball._position, np.array([game.BOARD_CENTER_X - (2 * math.cos(degree)),
                                                        game.BOARD_CENTER_Y - (2 * math.sin(degree))]))


def test_velocity():

    # Check for many collisions with each striker
    for i in range(16):
        degree = (i * 2 * math.pi) / 16
        left = centered_left_striker_and_ball_at_position(10 * math.cos(degree), 10 * math.sin(degree))
        right = centered_right_striker_and_ball_at_position(10 * math.cos(degree), 10 * math.sin(degree))
        assert np.equals(left.ball._position, np.array([8 * math.cos(degree), 8 * math.sin(degree)]))
        assert np.equals(right.ball._position, np.array([8 * math.cos(degree), 8 * math.sin(degree)]))

