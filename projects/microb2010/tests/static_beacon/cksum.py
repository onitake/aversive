#!/usr/bin/python

import random

# val is 12 bits. Return the 16 bits value that includes the 4 bits
# cksum in MSB.
def do_cksum(val):
    x = (val & 0xfff)
    # add the three 4-bits blocks of x together
    cksum = x & 0xf
    x = x >> 4
    cksum += x & 0xf
    cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4)
    x = x >> 4
    cksum += x & 0xf
    cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4)
    cksum = (~cksum) & 0xf
    return (cksum << 12) + (val & 0xfff)

# val is 16 bits, including 4 bits-cksum in MSB, return 0xFFFF is cksum
# is wrong, or the 12 bits value on success.
def verify_cksum(val):
    x = (val & 0xfff)
    # add the four 4-bits blocks of val together
    cksum = val & 0xf
    val = val >> 4
    cksum += val & 0xf
    cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4)
    val = val >> 4
    cksum += val & 0xf
    cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4)
    val = val >> 4
    cksum += val & 0xf
    cksum = (cksum & 0xf) + ((cksum & 0xf0) >> 4)
    if cksum == 0xf:
        return x
    return 0xffff # wrong value



err = 0
total = 0
for i in range(200):
    x = random.randint(0,(1<<12)-1)
    y = do_cksum(x)
    y = random.randint(0,(1<<16)-1)
    z = verify_cksum(y)
    if z == 0xffff:
        err += 1
    total += 1
print "%d errors / %d"%(err, total)
