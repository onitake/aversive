#!/usr/bin/python

import sys, math

RPS = 10.
LASER_RADIUS = 25. # mm

MIN = 200.
MAX = 3500.
NBITS = 9
STEPS = (1 << 9)
k = math.pow(MAX/MIN, 1./STEPS)

# t is in us, result is 9 bits
def time_to_frame(t):
    # process angle from t
    a = (t / (1000000./RPS)) * 2. * math.pi

    # process d from a (between 20cm and 350cm)
    d = LASER_RADIUS / math.sin(a/2)

    frame = math.log(d/MIN)/math.log(k)
    if frame >= 512:
        frame = 511
    else:
        frame = int(frame)
    print frame
    return frame

# frame is integer 9 bits, result is distance
def frame_to_distance(frame):
    d = MIN*(math.pow(k, frame))
    print d
    return d

x = time_to_frame(float(sys.argv[1]))
frame_to_distance(x)
