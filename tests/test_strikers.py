def test_striker(gamestate: Gamestate):

    # Check position
    assert gamestate.left_striker.position == np.array([100, 100])
    assert gamestate.right_striker.position == np.array([500, 100])

    # Check velocity
    assert gamestate.left_striker._velocity[0] == 0 and gamestate.left_striker._velocity[1] == 0
    assert gamestate.right_striker._velocity[0] == 0 and gamestate.right_striker._velocity[1] == 0

    # Check cooldown
    assert gamestate.left_striker._cooldown == gamestate.left_striker.MAX_COOLDOWN
    assert gamestate.right_striker._cooldown == gamestate.right_striker.MAX_COOLDOWN

    # Check wall collisions

    # Check ball collisions
