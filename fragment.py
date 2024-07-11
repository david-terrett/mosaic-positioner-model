from matplotlib import pyplot as plt
from focal_plane import focal_plane
from math import pi

fp=focal_plane()
plt.interactive(True)
fp.live_view=True
fp.create_random_targets(density=6.8)
fp.live_view=True
fp.simple_allocator()
fp._park_all()
fp.plot()
fp.live_view=True
ps=fp.positioners
parks = [[-pi,-2*pi],[-3*pi/4,-7*pi/4],[-pi/2,-3*pi/2],[-pi/4,-5*pi/4],[0,pi],[pi/4,5*pi/4],[pi/2,3*pi/2],[3*pi/4,7*pi/4],[pi,2*pi]]


def resolve_conflict(a,b,fig,ax):
    # assume we have already tried to move a and hit b:
    # assume we are done if a gets to position, leave b wherever we have to to do this unless it was already in position
    # park is [0,pi] so we can try an array of alternate park positions
    if (b.target):
        if (not b.in_position):
            # try to move it. If it goes to its destination safely this should be the best outcome
            if b.move_to_position(fig,ax):
                if a.move_to_position(fig,ax):
                    return True # all good
                else:
                    if a.collision_list[0] == b:  # moving b to position didn't help
                        b.reverse_last_move(fig,ax) # put b back
                        for testpose in parks:
                            if b.move_to_pose(fig,ax,testpose): # try turning it to each pose in turn
                                if a.move_to_position(fig,ax): # if we fail here we go back
                                    return True
                                else:
                                    c=a.collision_list[0]
                                    if (c != b):  # then we hit something else this time, so our position for b is good
                                        print ('Starting resolution of clash between ',a.id,' and ',c.id)
                                        return resolve_conflict(a,c,fig,ax) # down the rabbit hole!
                                    else:
                                        return False # could not resolve
                        return False # We failed to find a pseudo park position for b that helps a
            else: # we could not deploy b for some reason, so better make sure that it's not hitting a
                if b.collision_list[0] == a:
                    return resolve_conflict(b,a,fig,ax) # try to ask the question in reverse
                else:
                    c=b.collision_list[0]
                    print ('Starting resolution of clash between ',b.id,' and ',c.id)
                    if resolve_conflict(b,c,fig,ax): # can we fix this one
                        if a.move_to_position(fig,ax):
                            return True
                        else:
                            c=a.collision_list[0]
                            if (c != b):
                                print ('Starting resolution of clash between ',a.id,' and ',c.id)
                                return resolve_conflict(a,c,fig,ax)
                            else:
                                return False
        else:  # b is already deployed
            if b.reverse_last_move(fig,ax): # can we put b back to park
                for testpose in parks:      # try different starting positions for a
                    if a.move_to_pose(fig,ax,testpose):
                        if b.move_to_position(fig,ax):
                            if a.move_to_position(fig,ax):
                                return True    # Fixed it
                            else:
                                b.reverse_last_move(fig,ax)
                # Still here, so that didn't help
                if a.move_to_position(fig,ax): # This cleared the problem for a
                    if b.move_to_position(fig,ax):
                        return True   # sorted
                    else:
                        c=b.collision_list[0]
                        if c==a:  # we can move a into position, but now we can't move b back
                            print ('Seem to be stuck here.. parked ',b.id,' moved ',a.id,' but ',b.id,' is now stuck.')
                            return False
    else:   # b is just parked and has no target
        for testpose in parks:
            if b.move_to_pose(fig,ax,testpose):
                if a.move_to_position(fig,ax):
                    return True
        print ('Could not find a safe alternate park position for ',b.id)


def rerun(fp, istart):
    for i in fp.positioners:
        if (i.id < istart):
            next
        if i.target and not i.in_position:
            print ('moving ',i.id)
            if i.move_to_position(fp.figure,fp.axes):
                inp=input('Next ?')
                if inp == "n":
                    break
                else:
                    continue
            else:
                b = i.collision_list[0]
                if (not b.in_position):
                    if b.move_to_position(fp.figure,fp.axes):
                        if i.move_to_position(fp.figure,fp.axes):
                            inp=input('Next ?')
                            if inp == "n":
                                break
                        else:
                            c = i.collision_list[0]
                            if (c == b):
                                print ("Stuck at ",b.id)
                                break
                    else:
                        c=b.collision_list[0]
                        if (c.id > b.id):
                            if c.move_to_postitoin(fp.figure,fp.axes):
                                if b.move_to_position(fp.figure,fp.axes):
                                    if i.move_to_position(fp.figure,fp.axes):
                                        inp=input('Next ?')
                                        if inp == "n":
                                            break
                                    else:
                                        d=i.collision_list[0]
                                        if (d == b):
                                            print ("Stuck at ",b.id)
                                else:
                                    d=b.collision_list[0]
                                    if (d == c):
                                        print ("Stuck at ",c.id)
                else:
                    if b.reverse_last_move(fp.figure,fp.axes):
                        if i.move_to_position(fp.figure,fp.axes):
                            b.move_to_position(fp.figure,fp.axes)
                            inp=input('Next ?')
                            if inp == "n":
                                break
                        else:
                         c=i.collision_list[0]
                         if c == b:
                             print ("Reversed ",b.id," and still stuck")
                             break
                    else:
                        print ("Stuck at ",b.id)
                        break


for i in fp.positioners:
    if i.target and not i.in_position:
        i.trajectory_from_here_simultaneous(i.tpose)
rerun(0)
