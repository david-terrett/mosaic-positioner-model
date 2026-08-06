# -*- coding utf-8 -*-
from matplotlib import pyplot as plt
import numpy as np
from positioner_model.positioner import pose

def simple_allocator(fp):
    """
    Simple target to positioner assignment algorithm

    Parameters
    ---------
    fp : focal_plane
        focal_plane
    """
    fp.clear_all_target_assignments()

    # sort the list of positioners so that we   allocate the ones
    # with the fewest targets first
    positioners = sorted(fp.positioners, key=lambda p: len(p.targets))

    for pos in positioners:

        # If this positioner doesn't have a target
        if not pos.target:

            # For each of this positioner's targets
            for t in pos.targets:
                if not t.positioner:
                    if pos.try_assigning_target(t, alt=False,
                                                ignore_no_target=True):
                        pos.set_pose_to_target()
                        break
                    else:
                        if pos.try_assigning_target(t, alt=True,
                                                    ignore_no_target=True):
                            pos.set_pose_to_target()
                            break

    
    # Look for places for the unallocated positioners
    
    for pos in fp.positioners:
        if not pos.target:
            try:
                pos.uncollide()
            except RuntimeError:
                print("Warning: no position found for positioner", pos.id)

               
        # ---------------- ADDED RESCUE PASS ----------------
        # ---------------- ADDED RESCUE PASS ----------------
    # Try to recover unallocated positioners by stealing a reachable target
    # from an already allocated positioner, then reassigning that displaced one.

     

    def sync_motors_to_pose(pos):
        m = pos.get_motors_from_pose(pose(pos.theta_1, pos.theta_2))
        pos.alpha_motor.set(m.alpha)
        pos.beta_motor.set(m.beta)

    def snapshot():
        return [
            (
                p,
                p.target,
                getattr(p, "alt", False),
                getattr(p, "target_pose", None),
                p.theta_1,
                p.theta_2,
                p.alpha_motor.position,
                p.beta_motor.position,
                p.blocker,
                p.on_target,
            )
            for p in fp.positioners
        ]

    def restore(state):
        for p, *_ in state:
            p.assign_target(None)

        for p, target, alt, target_pose, theta_1, theta_2, alpha, beta, blocker, on_target in state:
            if target is not None:
                p.assign_target(target, alt)
                p.target_pose = target_pose
                p.set_pose_to_target()
                sync_motors_to_pose(p)
            else:
                p.set_pose(pose(theta_1, theta_2))
                p.alpha_motor.set(alpha)
                p.beta_motor.set(beta)
                p.set_pose_from_motors()

            p.blocker = blocker
            p.on_target = on_target

    def try_chain(pos, depth=3, seen=None):
        if pos.target:
            return True

        if depth <= 0:
            return False

        if seen is None:
            seen = set()

        if pos in seen:
            return False

        seen.add(pos)

        for t in pos.targets:
            owner = t.positioner

            if owner is pos:
                return True

            for alt in (False, True):
                state = snapshot()

                if owner is not None:
                    owner.assign_target(None)
                    owner.on_target = False
                    owner.blocker = None

                if pos.try_assigning_target(t, alt=alt, ignore_no_target=True):
                    pos.set_pose_to_target()
                    sync_motors_to_pose(pos)

                    if owner is None:
                        return True

                    if try_chain(owner, depth=depth - 1, seen=seen):
                        return True

                restore(state)

        return False

    for pos in positioners:
        if not pos.target:
            try_chain(pos, depth=4)

    # ---------------- PLACE NO-TARGET POSITIONERS ----------------
    # Every no-target positioner still needs a real non-colliding physical pose.
    # Prefer folded beta=0. If impossible, use any safe alpha/beta pose.

    def place_one_no_target(pos):
        alpha0 = pos.alpha_motor.position
        beta0 = pos.beta_motor.position

        alpha_trials = [0.0, alpha0 % 360.0]
        alpha_trials += list(range(0, 361, 5))

        seen = set()

        for alpha in alpha_trials:
            alpha = float(alpha) % 360.0
            key = ("folded", round(alpha, 9))

            if key in seen:
                continue

            seen.add(key)

            pos.alpha_motor.set(alpha)
            pos.beta_motor.set(0.0)
            pos.set_pose_from_motors()
            pos.on_target = False
            pos.blocker = None

            if pos.has_collision(ignore_no_target=False) is None:
                return True

        alpha_lo = int(max(0.0, pos.alpha_motor.low_limit))
        alpha_hi = int(min(360.0, pos.alpha_motor.high_limit))
        beta_lo = int(max(0.0, pos.beta_motor.low_limit))
        beta_hi = int(min(360.0, pos.beta_motor.high_limit))

        for alpha in range(alpha_lo, alpha_hi + 1, 10):
            for beta in range(beta_lo, beta_hi + 1, 10):
                pos.alpha_motor.set(float(alpha))
                pos.beta_motor.set(float(beta))
                pos.set_pose_from_motors()
                pos.on_target = False
                pos.blocker = None

                if pos.has_collision(ignore_no_target=False) is None:
                    return True

        pos.alpha_motor.set(alpha0)
        pos.beta_motor.set(beta0)
        pos.set_pose_from_motors()
        pos.on_target = False
        pos.blocker = None
        return False

    def place_no_target_positioners():
        failed = []

        no_target = [
            p for p in fp.positioners
            if p.exists() and p.target is None
        ]

        no_target.sort(key=lambda p: len(p.neighbours), reverse=True)

        for pos in no_target:
            if not place_one_no_target(pos):
                failed.append(pos.id)

        return failed

    failed = place_no_target_positioners()

    if failed:
        print("[ALLOC] warning: could not place no-target safely:", failed)

    # ---------------- FINAL SAFETY CLEANUP ----------------
    # Sacrifice only one side of each remaining collision. If a sacrificed
    # no-target positioner still cannot be placed, the next pass sacrifices
    # the other side of that collision.

    def sacrifice_score(pos):
        target_cost = 0 if pos.target is None else 1000
        reachable_count = len(pos.targets)

        return (
            target_cost,
            -reachable_count,
            pos.id,
        )

    def collision_victims(already_sacrificed):
        victims = set()

        for p in fp.positioners:
            if not p.exists():
                continue

            blocker = p.has_collision(ignore_no_target=False)

            if blocker is None:
                continue

            pair = (p, blocker)
            fresh = [q for q in pair if q.id not in already_sacrificed]

            if fresh:
                victim = min(fresh, key=sacrifice_score)
            else:
                victim = min(pair, key=sacrifice_score)

            victims.add(victim.id)

        return victims

    sacrificed = set()
    max_cleanup_passes = 10

    for cleanup_pass in range(max_cleanup_passes):
        victims = collision_victims(sacrificed)

        if not victims:
            break

        new_victims = victims - sacrificed
        sacrificed.update(victims)

        print(
            f"[ALLOC] cleanup pass {cleanup_pass + 1}: "
            f"sacrificing {sorted(victims)}"
        )

        for pos in fp.positioners:
            if pos.id in victims:
                pos.assign_target(None)
                pos.on_target = False
                pos.blocker = None

        failed = place_no_target_positioners()

        if failed:
            print("[ALLOC] warning: could not place no-target safely:", failed)

        bad_now = [
            p.id for p in fp.positioners
            if p.exists() and p.has_collision(ignore_no_target=False) is not None
        ]

        if not bad_now:
            break

        if not new_victims:
            print("[ALLOC] cleanup stuck, remaining collisions:", bad_now)
            break

    bad = []

    for pos in fp.positioners:
        if not pos.exists():
            continue

        if pos.has_collision(ignore_no_target=False) is not None:
            bad.append(pos.id)

    print(f"[ALLOC] positioners still colliding: {len(bad)}")

    if bad:
        print("[ALLOC] ids:", bad)

    
