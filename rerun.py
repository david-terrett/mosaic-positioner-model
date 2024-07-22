# -*- coding utf-8 -*-

def rerun(fp, istart):
    for i in fp.positioners:
        if i.id < istart:
            next
        if i.target and not i.in_position:
            print ('moving ',i.id)
            if i.move_to_position(fp.figure, fp.axes):
                inp=input('Next ?')
                if inp == "n":
                    break
                else:
                    continue
            else:
                b = i.collision_list[0]
                if not b.in_position:
                    if b.move_to_position(fp.figure, fp.axes):
                        if i.move_to_position(fp.figure, fp.axes):
                            inp=input('Next ?')
                            if inp == "n":
                                break
                        else:
                            c = i.collision_list[0]
                            if c == b:
                                print("Stuck at ", b.id)
                                break
                    else:
                        c=b.collision_list[0]
                        if c.id > b.id:
                            if c.move_to_position(fp.figure, fp.axes):
                                if b.move_to_position(fp.figure, fp.axes):
                                    if i.move_to_position(fp.figure, fp.axes):
                                        inp = input('Next ?')
                                        if inp == "n":
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
                    if b.reverse_last_move(fp.figure, fp.axes):
                        if i.move_to_position(fp.figure, fp.axes):
                            b.move_to_position(fp.figure, fp.axes)
                            inp  = input('Next ?')
                            if inp == "n":
                                break
                        else:
                            c = i.collision_list[0]
                            if c == b:
                                print("Reversed ", b.id, " and still stuck")
                                break
                    else:
                        print("Stuck at ", b.id)
                        break
