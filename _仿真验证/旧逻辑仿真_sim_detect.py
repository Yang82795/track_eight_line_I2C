# Simulate the CURRENT Avoid_detect() ALL_ON state machine (avoid_obstacle.cpp)
TRIG_CM = 35.0
CONFIRM = 2
TICK_MS = 50

def sim(d_left, d_right, ticks=40):
    left_cnt = 0
    right_cnt = 0
    turn = 0
    last = 0
    now = 1000
    trace = []
    for _ in range(ticks):
        now += TICK_MS
        if now - last < TICK_MS:
            continue
        last = now
        if turn == 0:
            turn = 1
            d = d_left
            if 2.0 < d <= TRIG_CM: left_cnt += 1
            else: left_cnt = 0
        else:
            turn = 0
            d = d_right
            if 2.0 < d <= TRIG_CM: right_cnt += 1
            else: right_cnt = 0
        trace.append((now, turn, round(d,1), left_cnt, right_cnt))
        if left_cnt >= CONFIRM and right_cnt >= CONFIRM:
            return "OBS_BOTH", trace
        elif left_cnt >= CONFIRM:
            return "OBS_LEFT", trace
        elif right_cnt >= CONFIRM:
            return "OBS_RIGHT", trace
    return "OBS_NONE(never triggers)", trace

for label, dl, dr in [
    ("obstacle only in LEFT cone  (30cm)", 30, 999),
    ("obstacle only in RIGHT cone (30cm)", 999, 30),
    ("obstacle dead centre        (30cm both)", 30, 30),
    ("obstacle far               (50cm left)", 50, 999),
]:
    res, tr = sim(dl, dr)
    print(f"{label:38s} -> {res:26s} (ticks={len(tr)})")
    if len(tr) <= 6:
        for t in tr: print("      ", t)
    print()