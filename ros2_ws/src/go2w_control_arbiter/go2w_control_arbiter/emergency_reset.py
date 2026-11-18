"""Explicit reset admission; a false emergency level never clears the latch."""
import math


def reset_blocker(samples, now):
    for name,expected in [('emergency_stop',False),('software_arm',True),('remote_override',False)]:
        sample=samples.get(name)
        if not sample or sample[0] is not expected:
            return name+' not ready'
        age=now-sample[1]
        if not math.isfinite(age) or not 0<=age<=.3:
            return name+' stale'
    return ''
