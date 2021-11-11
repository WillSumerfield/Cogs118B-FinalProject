import pytest
import numpy as np
from Game.gamestate import Gamestate


@pytest.fixture()
def create_game():
    return Gamestate(None, None)


def test_gamestate(gamestate: Gamestate):

    # Check for correct instantiation
    assert isinstance(gamestate, Gamestate)
    assert isinstance(gamestate.left_striker, gamestate.Striker)
    assert isinstance(gamestate.right_striker, gamestate.Striker)
    assert isinstance(gamestate.ball, gamestate.Ball)

    # Check collisions

    # Check tick

    # Check game end
    assert gamestate.game_end(Gamestate.LEFT) == np.array([1, 0])
    assert gamestate.game_end(Gamestate.RIGHT) == np.array([0, 1])
