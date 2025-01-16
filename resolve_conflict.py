# -*- coding utf-8 -*-

# Trial positions for parking the alpha axis
park_angles = [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0]

def resolve_conflict(a, b, ax):
    """
    Resove confict between two positioners

    assume we have already tried to move a and hit b:
    assume we are done if a gets to position, leave b wherever we have to to do
    this unless it was already in position. park is [0,0] so we can try an
    array of alternate park positions

    Arguments
    ---------
    a : positioner
        positioner a
    b : positioner
        positioner b
    ax : mathplotlib axes or None
        axes to plot positioner on
    """
    if b.target:
        if not b.in_position:
            # try to move it. If it goes to its destination safely this should
            # be the best outcome
            if b.move_to_target(ax):
                if a.move_to_target(ax):
                    return True
                else:

                    # moving b to position didn't help
                    if a.blocker == b:

                        # put b back
                        b.reverse_last_move(ax)
                        for testpos in park_angles:
                            b.alpha_motor.set_path(testpos)
                            b.beta_motor.set_path(0.0)

                            # try turning it to each park in turn
                            if b.move(ax):

                                # if we fail here we go back
                                if a.move_to_target(ax):
                                    return True
                                else:
                                    c = a.blocker
                                    # then we hit something else this time,
                                    # so our position for b is good
                                    if c != b:
                                        print('Starting resolution of clash between ',
                                              a.id, ' and ', c.id)

                                        # down the rabbit hole!
                                        return resolve_conflict(a, c, ax)
                                    else:
                                        # could not resolve
                                        return False

                        # We failed to find a pseudo park position for b that helps a
                        return False

            else:

                # we could not deploy b for some reason, so better make
                # sure that it's not hitting a
                if b.blocker == a:

                    # try to ask the question in reverse
                    return resolve_conflict(b, a, ax)
                else:
                    c=b.blocker
                    print('Starting resolution of clash between ', b.id,
                          ' and ', c.id)

                    # can we fix this one
                    if resolve_conflict(b, c, ax):
                        if a.move_to_target(ax):
                            return True
                        else:
                            c = a.blocker
                            if c != b:
                                print ('Starting resolution of clash between ',
                                       a.id,' and ', c.id)
                                return resolve_conflict(a, c, ax)
                            else:
                                return False
        else:

            # b is already deployed. can we put b back to park
            if b.reverse_last_move(ax):

                # try different starting positions for a
                for testpos in park_angles:
                    b.alpha_motor.set_path(testpos)
                    b.beta_motor.set_path(0.0)
                    if a.move(ax):
                        if b.move_to_target(ax):
                            if a.move_to_target(ax):

                                # Fixed it
                                return True
                            else:
                                b.reverse_last_move(ax)

                # Still here, so that didn't help
                if a.move_to_target(ax):

                    # This cleared the problem for a
                    if b.move_to_target(ax):

                        # sorted
                        return True
                    else:
                        c = b.blocker
                        if c == a:

                            # we can move a into position, but now we can't
                            # move b back
                            print('Seem to be stuck here.. parked ', b.id,
                                  ' moved ', a.id,' but ', b.id, ' is now stuck.')
                            return False
    else:

        # b is just parked and has no target
        for testpos in park_angles:
            b.alpha_motor.set_path(testpos)
            b.beta_motor.set_path(0.0)
            if b.move(ax):
                if a.move_to_target(ax):
                    return True
        print('Could not find a safe alternate park position for ', b.id)
