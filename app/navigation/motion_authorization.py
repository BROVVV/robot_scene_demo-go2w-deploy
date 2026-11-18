"""Motion admission helpers shared by bounded tests and Web execution."""


def motion_preflight_blockers(state, *, now, max_age_s=.5):
    """Return every missing, stale or rejecting live gate before reservation.

    The bridge remains the final authority.  This earlier read-only check keeps
    a one-use ledger from being reserved merely because a new subscriber has
    not received the low-rate sport state yet.
    """
    blockers=[]
    sport=state.get('sport')
    if not isinstance(sport,tuple) or len(sport)!=3:
        blockers.append('sport_state_missing')
    else:
        mode,error,at=sport
        if not isinstance(at,(int,float)) or not 0<=now-at<=max_age_s:
            blockers.append('sport_state_stale')
        elif mode!=1:
            blockers.append('robot_not_in_sport_mode')
        elif error!=0:
            blockers.append('robot_error_nonzero')
    expected={
        'lease_alive':True,
        'lidar_fresh':True,
        'localization_valid':True,
        'robot_error_zero':True,
        'emergency_stop':False,
        'remote_override':False,
        'software_arm':False,
    }
    for key,wanted in expected.items():
        sample=state.get(key)
        if not isinstance(sample,tuple) or len(sample)!=2:
            blockers.append(key+'_missing')
            continue
        value,at=sample
        if not isinstance(at,(int,float)) or not 0<=now-at<=max_age_s:
            blockers.append(key+'_stale')
        elif value is not wanted:
            blockers.append(key+'_rejecting')
    return blockers


def arm_with_arbiter_reset(*,arm,is_armed,reset,disarm):
    success=False
    try:
        arm()
        success=bool(is_armed() and reset())
        return success
    except Exception:
        return False
    finally:
        if not success and is_armed():
            try:disarm()
            except Exception:pass
