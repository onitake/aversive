import sys, re, math
import numpy as np
import matplotlib
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import popen2, random

Path = mpath.Path


def build_poly(ptlist):
    polydata = []
    polydata.append((Path.MOVETO, (ptlist[0])))
    for pt in ptlist[1:]:
        polydata.append((Path.LINETO, (pt)))
    polydata.append((Path.CLOSEPOLY, (ptlist[0])))
    codes, verts = zip(*polydata)
    poly = mpath.Path(verts, codes)
    x, y = zip(*poly.vertices)
    return x,y

def build_path(ptlist):
    polydata = []
    polydata.append((Path.MOVETO, (ptlist[0])))
    for pt in ptlist[1:]:
        polydata.append((Path.LINETO, (pt)))
    codes, verts = zip(*polydata)
    poly = mpath.Path(verts, codes)
    x, y = zip(*poly.vertices)
    return x,y

def graph(filename, stx, sty, sta, enx, eny, opx, opy):
    cmd = "./main %d %d %d %d %d %d %d"%(stx, sty, sta, enx, eny, opx, opy)
    o,i = popen2.popen2(cmd)
    i.close()
    s = o.read(1000000)
    o.close()

    open(filename + ".txt", "w").write(s)

    if len(s) == 1000000:
        gloupix()

    fig = plt.figure()
    ax = fig.add_subplot(111)

    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')
    
    x,y = build_poly([(250,250), (2750,250), (2750,1850), (250,1850)])
    ax.plot(x, y, 'g--')
    
    # central disc
    patches = [ Circle((1500, 1050), 150) ]
    
    poly = None
    poly_wait_pts = 0
    start = None
    path = None
    for l in s.split("\n"):
        m = re.match("robot at: (-?\d+) (-?\d+) (-?\d+)", l)
        if m:
            x,y,a = (int(m.groups()[0]), int(m.groups()[1]), int(m.groups()[2]))
            path = [ (x,y) ]
            a_rad = (a * math.pi / 180.)
            dx = 150 * math.cos(a_rad)
            dy = 150 * math.sin(a_rad)
            patches += [ Circle((x, y), 50) ]
            patches += [ Arrow(x, y, dx, dy, 50) ]

        m = re.match("oa_start_end_points\(\) \((-?\d+),(-?\d+)\) \((-?\d+),(-?\d+)\)", l)
        if m:
            dst_x,dst_y = (int(m.groups()[2]), int(m.groups()[3]))
            patches += [ Circle((dst_x, dst_y), 50) ]

        m = re.match("oa_new_poly\(size=(-?\d+)\)", l)
        if m:
            poly_wait_pts = int(m.groups()[0])
            poly = []

        m = re.match("oponent at: (-?\d+) (-?\d+)", l)
        if m:
            poly_wait_pts = 4
            poly = []

        m = re.match("oa_poly_set_point\(\) \((-?\d+),(-?\d+)\)", l)
        if m:
            poly.append((int(m.groups()[0]), int(m.groups()[1])))
            poly_wait_pts -= 1
            if poly_wait_pts == 0:
                x,y = build_poly(poly)
                ax.plot(x, y, 'r-')

        m = re.match("GOTO (-?\d+),(-?\d+)", l)
        if m:
            path.append((int(m.groups()[0]), int(m.groups()[1])))

        m = re.match("With avoidance .: x=(-?\d+) y=(-?\d+)", l)
        if m:
            path.append((int(m.groups()[0]), int(m.groups()[1])))

    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
    ax.add_collection(p)

    x,y = build_path(path)
    ax.plot(x, y, 'bo-')

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2200)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

# args are: startx, starty, starta, endx, endy, oppx, oppy
graph("normal1.png", 500, 1000, 180, 2000, 1500, 1500, 2000)
graph("normal2.png", 500, 300, 50, 2000, 1500, 2500, 2000)
graph("normal3.png", 500, 1000, 100, 1000, 1500, 2500, 1000)
graph("normal4.png", 500, 300, 0, 2500, 1700, 1000, 1000)
graph("normal5.png", 500, 1500, 0, 2500, 1700, 500, 1000)

# echappement
graph("escape1.png", 5200, 1000, 0, 2500, 1700, 2500, 1000)
graph("escape2.png", 500, 1300, 0, 2500, 1700, 500, 1000)
graph("escape3.png", 500, 1000, 180, 2000, 1500, 1500, 2000)

# pas d'echappement possible... petit carre
graph("small_square1.png", 500, 300, 0, 2500, 1700, 500, 500)
graph("small_square2.png", 500, 1000, 0, 2500, 1700, 500, 1000)
graph("small_square3.png", 590, 1000, 0, 2500, 1700, 500, 1000)
graph("small_square4.png", 560, 1000, 0, 2500, 1700, 300, 1000)
graph("small_square5.png", 550, 1000, 0, 2500, 1700, 250, 1000)
graph("small_square6.png", 5750, 1000, 0, 2500, 1700, 2750, 1000)

# impossible
graph("impossible1.png", 500, 300, 0, 2700, 1800, 2700, 1800)
graph("impossible2.png", 500, 300, 0, 2700, 1800, 2400, 1500)
graph("impossible3.png", 500, 300, 0, 1500, 1150, 2700, 1800 )

#test current
graph("current01.png", 1274,709, 58, 1312, 400, 2274, 1241 )


random.seed(0)
for i in range(100):
    stx = random.randint(-50, 3050)
    sty = random.randint(-50, 2050)
    enx = random.randint(-50, 3050)
    eny = random.randint(-50, 2050)
    opx = random.randint(-50, 3050)
    opy = random.randint(-50, 2050)
    name = "random%d.png"%(i)
    print (name, stx, sty, 0, enx, eny, opx, opy)
    graph(name, stx, sty, 0, enx, eny, opx, opy)
    
