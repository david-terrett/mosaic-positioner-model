# -*- coding utf-8 -*-

def rerun(fp, istart, pause=True):
    for i in fp.positioners:
        if i.id < istart:
            continue
        if i.target and not i.in_position:
            print ('moving ',i.id)
            i.zoom_to(fp.figure)
            if i.move_to_target(fp.axes):
                if not _next(pause):
                    break
                else:
                    continue
            else:
                b = i.collision_list[0]
                if not b.in_position:
                    if b.move_to_target(fp.axes):
                        if i.move_to_target(fp.axes):
                            if not _next(pause):
                                break
                        else:
                            c = i.collision_list[0]
                            if c == b:
                                print("Stuck at ", b.id)
                    else:
                        c=b.collision_list[0]
                        if c.id > b.id:
                            if c.move_to_target(fp.axes):
                                if b.move_to_target(fp.axes):
                                    if i.move_to_target(fp.axes):
                                        if not _next(pause):
                                            break
                                    else:
                                        d = i.collision_list[0]
                                        if d == b:
                                            print("Stuck at ", b.id)
                                else:
                                    d = b.collision_list[0]
                                    if d == c:
                                        print("Stuck at ", c.id)
                else:
                    print("Reversing ", b.id)
                    if b.reverse_last_move(fp.axes):
                        if i.move_to_target(fp.axes):
                            b.move_to_target(fp.axes)
                            if not _next(pause):
                                break
                        else:
                            c = i.collision_list[0]
                            if c == b:
                                print("Reversed ", b.id, " and still stuck")
                                break
                    else:
                        print("Stuck at ", b.id)
                        break

def _next(pause):
    if pause:
        inp  = input('Next ?')
        if inp == "n":
            return False
    return True
