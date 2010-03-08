#!/usr/bin/python

import sys, math

if 1:
    RPS = 10.
    TIMER_FREQ = 2000000.
else:
    RPS = 40.
    TIMER_FREQ = 16000000.

LASER_RADIUS = 25. # mm

MIN = 200.
MAX = 3500.
NBITS = 9
STEPS = (1 << 9)
k = math.pow(MAX/MIN, 1./STEPS)

def mm_to_framedist(mm):
    d = mm
    d -= MIN
    d /= (MAX-MIN)
    d *= 512
    return d

def framedist_to_mm(d):
    d /= 512.
    d *= (MAX-MIN)
    d += MIN
    return d

# t is in us, result is 9 bits
def time_to_frame(t):
    # process angle from t
    a = (t / (1000000./RPS)) * 2. * math.pi

    # process d from a (between 20cm and 350cm)
    d = LASER_RADIUS / math.sin(a/2)
    frame = int(mm_to_framedist(d))
    return frame

# frame is integer 9 bits, result is laserdiff time
def frame_to_time(frame):
    d = framedist_to_mm(frame)
    a = 2 * math.asin(LASER_RADIUS/d)
    t = (a * (TIMER_FREQ/RPS)) / (2. * math.pi)
    return t

def sample_to_offset(samples, table):
    offsets = samples[:]
    for i in range(len(offsets)):
        o = offsets[i]
        framedist = mm_to_framedist(o[0])
        off = o[1] - table[int(framedist)]
        offsets[i] = framedist, off
    return offsets

def linear_interpolation(offsets, framedist, time):
    if framedist <= offsets[0][0]:
        return time + offsets[0][1]
    if framedist >= offsets[-1][0]:
        return time + offsets[-1][1]

    #print (offsets, framedist, time)
    o_prev = offsets[0]
    for o in offsets[1:]:
        if framedist > o[0]:
            o_prev = o
            continue
        x = (framedist - o_prev[0]) / (o[0] - o_prev[0])
        return time + o_prev[1] + (x * (o[1] - o_prev[1]))
    return None

#x = time_to_frame(float(sys.argv[1]))
#frame_to_distance(x)
#frame_to_time(int(sys.argv[1]))


table = [0] * 512
for i in range(512):
    table[i] = frame_to_time(i)

# linear correction: distance_mm, time
samples = [
    (250., 7600.),
    (500., 3000.),
    (3000., 400.),
    ]

offsets = sample_to_offset(samples, table)
print "#include <aversive.h>"
print "#include <aversive/pgmspace.h>"
print "prog_uint16_t framedist_table[] = {"
for i in range(512):
    if (i % 8) == 0:
        print "	",
    print "%d,"%(int(linear_interpolation(offsets, i, table[i]))),
    if (i % 8 == 7):
        print
print "};"
