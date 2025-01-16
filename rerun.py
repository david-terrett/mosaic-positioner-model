# -*- coding utf-8 -*-

def rerun(fp, istart, pause=True, log=True):
    for i in fp.positioners:
        if i.id < istart:
            continue
        if i.target and not i.in_position:
            print ('moving ',i.id)
            i.zoom_to(fp.figure)
            if i.move_to_target(fp.axes, log=log):
                if not _next(pause):
                    break
                else:
                    continue
            else:
                b = i.blocker
                if not b.in_position:
                    if b.move_to_target(fp.axes):
                        if i.move_to_target(fp.axes, log=log):
                            if not _next(pause):
                                break
                        else:
                            c = i.blocker
                            if c == b:
                                print("Stuck at ", b.id)
                    else:
                        c=b.blocker
                        if c.id > b.id:
                            if c.move_to_target(fp.axes, log=log):
                                if b.move_to_target(fp.axes, log=log):
                                    if i.move_to_target(fp.axes, log=log):
                                        if not _next(pause):
                                            break
                                    else:
                                        d = i.blocker
                                        if d == b:
                                            print("Stuck at ", b.id)
                                else:
                                    d = b.blocker
                                    if d == c:
                                        print("Stuck at ", c.id)
                else:
                    print("Reversing ", b.id)
                    if b.reverse_last_move(fp.axes, log=log):
                        if i.move_to_target(fp.axes, log=log):
                            b.move_to_target(fp.axes, log=log)
                            if not _next(pause):
                                break
                        else:
                            c = i.blocker
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
