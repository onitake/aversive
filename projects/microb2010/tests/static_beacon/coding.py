#!/usr/bin/python

import sys, math
import matplotlib.pyplot as plt


#RPS = 10.
RPS = 20.
#RPS = 40.
#TIMER_FREQ = 2000000.
TIMER_FREQ = 16000000.

LASER_RADIUS = 20. # mm

MIN = 200.
MAX = 3500.
NBITS = 9
STEPS = (1 << 9)
k = math.pow(MAX/MIN, 1./STEPS)

def mm_to_frame(mm):
    d = mm
    d -= MIN
    d /= (MAX-MIN)
    d *= 512
    return d

def frame_to_mm(d):
    d /= 512.
    d *= (MAX-MIN)
    d += MIN
    return d

# t is in us, result is 9 bits
def us_to_frame(t):
    # process angle from t
    a = (t / (1000000./RPS)) * 2. * math.pi

    # process d from a (between 20cm and 350cm)
    d = LASER_RADIUS / math.sin(a/2)
    frame = int(mm_to_frame(d))
    return frame

# frame is integer 9 bits, result is laserdiff time in us
def frame_to_us(frame):
    d = frame_to_mm(frame)
    a = 2 * math.asin(LASER_RADIUS/d)
    t = (a * (1000000./RPS)) / (2. * math.pi)
    return t

# theorical: laser timediff to robot distance
def us_to_mm(us):
    return frame_to_mm(us_to_frame(us))

# theorical: robot distance to laserdiff
def mm_to_us(mm):
    return frame_to_us(mm_to_frame(mm))

def time_us_to_tick(us):
    return (us / 1000000.) * TIMER_FREQ

def time_tick_to_us(t):
    return (t * 1000000.) / TIMER_FREQ


##################

# linear correction: distance_mm, time_us
# must be ordered
samples = [
    (330.,  15681./16),
    (778.,  6437./16),
    (1180., 4351./16),
    (1608., 3221./16),
    (2045., 2583./16),
    (2487., 2167./16),
    ]

dist_mm = map(frame_to_mm, range(512))

# theorical curve
theorical = [0] * 512
for i in range(512):
    theorical[i] = frame_to_us(i)

# find offset and update theorical curve
off = samples[-1][1] - mm_to_us(3000.)
#print "offset=%f"%(off)
theo_off = [0] * 512
for i in range(512):
    mm = frame_to_mm(i)
    theo_off[i] = mm_to_us(mm) + off

final = [0] * 512
for i in range(512):
    mm = frame_to_mm(i)

    # find between which samples we are
    smp = 0
    while smp < (len(samples) - 2):
        if samples[smp+1][0] >= mm:
            break
        smp += 1

    mm_start = us_to_mm(samples[smp][1] - off)
    mm_end = us_to_mm(samples[smp+1][1] - off)

    # interpolation
    ratio = (mm - samples[smp][0]) / (samples[smp+1][0] - samples[smp][0])
    mm_new = mm_start + ratio * (mm_end - mm_start)

    if mm_new < 0:
        mm_new = 1.
    final[i] = mm_to_us(mm_new) + off


plt.plot(
    dist_mm, theorical, "r-",
    dist_mm, theo_off, "b-",
    dist_mm, final, "g-",
    map(lambda x:x[0], samples), map(lambda x:x[1], samples), "g^",
    )
plt.show()

print "#include <aversive.h>"
print "#include <aversive/pgmspace.h>"
print "prog_uint16_t framedist_table[] = {"
for i in range(512):
    if (i % 8) == 0:
        print "	",
    print "%d,"%(int(time_us_to_tick(final[i]))),
    if (i % 8 == 7):
        print
print "};"
