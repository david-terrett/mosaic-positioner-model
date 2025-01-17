# -*- coding utf-8 -*-

def rerun(fp, istart, pause=True):
    for i in fp.positioners:
        if i.id < istart:
            continue
        if i.target and not i.on_target:
            print ('moving ',i.id)
            i.zoom_to(fp.figure)
            if i.move_to_target(fp.axes):
                print(i.id, ': Moved to final position')
                if not _next(pause):
                    break
                else:
                    continue
            else:
                print(i.id, ': Blocked by ', i.blocker.id)
                b = i.blocker
                if not b.on_target:
                    if b.move_to_target(fp.axes):
                        print(b.id, ': Moved to final position')
                        if i.move_to_target(fp.axes):
                            print(i.id, ': Moved to final position')
                            if not _next(pause):
                                break
                        else:
                            c = i.blocker
                            if c == b:
                                print("Stuck at ", b.id)
                    else:
                        c=b.blocker
                        if c.id > b.id:
                            if c.move_to_target(fp.axes):
                                if b.move_to_target(fp.axes):
                                    print(b.id, ': Moved to final position')
                                    if i.move_to_target(fp.axes):
                                        print(i.id, ': Moved to final position')
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
                    if b.reverse_last_move(fp.axes):
                        if i.move_to_target(fp.axes):
                            print(i.id, ': Moved to final position')
                            if b.move_to_target(fp.axes):
                                print(b.id, ': Moved to final position')
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
