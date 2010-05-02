import sys, re, math
import numpy as np
import matplotlib
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import popen2, random

OFFSET_CORN_X=150
OFFSET_CORN_Y=222
STEP_CORN_X=225
STEP_CORN_Y=250

WAYPOINTS_NBX = 13
WAYPOINTS_NBY = 8

TYPE_WAYPOINT=0
TYPE_DANGEROUS=1
TYPE_WHITE_CORN=2
TYPE_BLACK_CORN=3
TYPE_OBSTACLE=4
TYPE_NEIGH=5

col = [TYPE_WAYPOINT] * WAYPOINTS_NBY
waypoints = [col[:] for i in range(WAYPOINTS_NBX)]
corn_table = [TYPE_WHITE_CORN]*18

corn_side_confs = [
    [ 1, 4 ],
    [ 0, 4 ],
    [ 2, 4 ],
    [ 2, 3 ],
    [ 0, 3 ],
    [ 1, 3 ],
    [ 1, 6 ],
    [ 0, 6 ],
    [ 2, 6 ],
    ]
corn_center_confs = [
    [ 5, 8 ],
    [ 7, 8 ],
    [ 5, 9 ],
    [ 7, 8 ],
    ]

Path = mpath.Path

def mark_one_neigh(i, j):
    global waypoints
    if i < 0 or j < 0 or i >= WAYPOINTS_NBX or j >= WAYPOINTS_NBY:
        return
    if waypoints[i][j] != TYPE_WAYPOINT:
        return
    waypoints[i][j] = TYPE_NEIGH

def mark_all_neigh(i, j):
    global waypoints
    if waypoints[i][j] != TYPE_WAYPOINT:
        return
    waypoints[i][j] = TYPE_NEIGH
    mark_one_neigh(i,j+1)
    mark_one_neigh(i,j-1)
    mark_one_neigh(i+1,j)
    mark_one_neigh(i-1,j)
    if i & 1 == 0:
        mark_one_neigh(i+1,j-1)
        mark_one_neigh(i-1,j-1)
    else:
        mark_one_neigh(i+1,j+1)
        mark_one_neigh(i-1,j+1)

def pt2corn(i,j):
    if i == 0 and j == 2: return 0
    if i == 0 and j == 4: return 1
    if i == 0 and j == 6: return 2
    if i == 2 and j == 3: return 3
    if i == 2 and j == 5: return 4
    if i == 2 and j == 7: return 5
    if i == 4 and j == 4: return 6
    if i == 4 and j == 6: return 7
    if i == 6 and j == 5: return 8
    if i == 6 and j == 7: return 9
    if i == 8 and j == 4: return 10
    if i == 8 and j == 6: return 11
    if i == 10 and j == 3: return 12
    if i == 10 and j == 5: return 13
    if i == 10 and j == 7: return 14
    if i == 12 and j == 2: return 15
    if i == 12 and j == 4: return 16
    if i == 12 and j == 6: return 17
    return -1

def corn_get_sym(i):
    sym = [15, 16, 17, 12, 13, 14, 10, 11, 8, 9, 6, 7, 3, 4, 5, 0, 1, 2]
    return sym[i]

def init_corn_table(conf_side, conf_center):
    global corn_table, corn_side_confs, corn_center_confs
    print "confs = %d, %d"%(conf_side, conf_center)
    for i in range(18):
        if i in corn_side_confs[conf_side]:
            corn_table[i] = TYPE_BLACK_CORN
            continue
        if corn_get_sym(i) in corn_side_confs[conf_side]:
            corn_table[i] = TYPE_BLACK_CORN
            continue
        if i in corn_center_confs[conf_center]:
            corn_table[i] = TYPE_BLACK_CORN
            continue
        if corn_get_sym(i) in corn_center_confs[conf_center]:
            corn_table[i] = TYPE_BLACK_CORN
            continue
        corn_table[i] = TYPE_WHITE_CORN
            
def init_waypoints():
    global waypoints, corn_table

    for i in range(WAYPOINTS_NBX):
        for j in range(WAYPOINTS_NBY):
            # corn
            c = pt2corn(i, j)
            if c >= 0:
                waypoints[i][j] = corn_table[c]
                continue
            # too close of border
            if (i & 1) == 1 and j == WAYPOINTS_NBY -1:
                waypoints[i][j] = TYPE_OBSTACLE
                continue
            # hill
            if i >= 2 and i < WAYPOINTS_NBX - 2 and j < 2:
                waypoints[i][j] = TYPE_OBSTACLE
                continue
            # dangerous points
            if i == 0 or i == WAYPOINTS_NBX-1:
                waypoints[i][j] = TYPE_DANGEROUS
                continue
            if (i&1) == 0 and j == WAYPOINTS_NBY-1:
                waypoints[i][j] = TYPE_DANGEROUS
                continue

            waypoints[i][j] = TYPE_WAYPOINT
            
        print i, waypoints[i]
    return waypoints

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

def nearest_corn(x, y):
    OFFX=150
    OFFY=222
    STEPX=450
    STEPY=500

    x -= OFFX
    x += STEPX/2
    x /= STEPX

    y -= OFFY
    y += STEPY/2
    if (x & 1) == 1:
        y -= STEPY/2
    y /= STEPY

    i = (x * 2)
    j = (y * 2) + (x & 1)
    if i >= WAYPOINTS_NBX:
        return None
    if j >= WAYPOINTS_NBY:
        return None

    if (i & 1) == 0:
        y = OFFSET_CORN_Y
    else:
        y = OFFSET_CORN_Y + STEP_CORN_Y/2
    y += (j * STEP_CORN_Y)

    x = OFFSET_CORN_X + (i * STEP_CORN_X)

    return x, y


def build_area(ax):
    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')

    x,y = build_poly([(0,0), (0,500), (500,500), (500,0)])
    ax.plot(x, y, 'y-')

    x,y = build_poly([(3000,0), (3000,500), (2500,500), (2500,0)])
    ax.plot(x, y, 'b-')

    x,y = build_poly([(740,0), (740,500), (2260,500), (2260,0)])
    ax.plot(x, y, 'g--')

    """
    # courbes a 2 balles
    x,y = build_path([(375,0), (375,2050)])
    ax.plot(x, y, 'r-')

    x,y = build_path([(150,1972), (2850,472)])
    ax.plot(x, y, 'r-')

    x,y = build_path([(0,1722), (3000,1722)])
    ax.plot(x, y, 'r-')

    x,y = build_path([(150,972), (1950,1972)])
    ax.plot(x, y, 'r-')

    acoef = (2850-150)/5.
    bcoef = (472-1972)/5.
    x,y = build_path([(600,1972), (600+bcoef, 1972-acoef)])
    ax.plot(x, y, 'r-')

    p = PatchCollection([Circle((2400,972), 225)],
                        cmap=matplotlib.cm.jet,
                        alpha=0.5, facecolor=(1.,0.,0.))
    ax.add_collection(p)

    """

    """
    #courbes a 2 balles 50
    for xx in range(0, 3000, 100):
        print xx
        for yy in range(0, 2100, 100):
            pouet = nearest_corn(xx, yy)
            if pouet == None:
                continue
            xxx, yyy = pouet
            x,y = build_path([(xx,yy), (xxx,yyy)])
            ax.plot(x, y, 'r-')
    """

   # limit
    #x,y = build_poly([(250,250), (2750,250), (2750,1850), (250,1850)])
    #ax.plot(x, y, 'g--')

    xtick = [i * STEP_CORN_X + OFFSET_CORN_X for i in range(WAYPOINTS_NBX)]
    ax.set_xticks(xtick)
    ytick = [(i-1) * STEP_CORN_Y/2 + OFFSET_CORN_Y for i in range(WAYPOINTS_NBY*2)]
    ax.set_yticks(ytick)

    init_corn_table(random.randint(0,8), random.randint(0,3))

    waypoints = init_waypoints()
    mark_all_neigh(5,6)

    wcorn = []
    bcorn = []
    points = []
    dpoints = []
    neighs = []

    i = 0
    j = 0
    x = OFFSET_CORN_X
    while x < 3000:
        if (i & 1) == 0:
            y = OFFSET_CORN_Y
        else:
            y = OFFSET_CORN_Y + STEP_CORN_Y/2
        j = 0
        while y < 2100:

            if waypoints[i][j] == TYPE_WHITE_CORN:
                wcorn.append(Circle((x, y), 25))
            if waypoints[i][j] == TYPE_BLACK_CORN:
                bcorn.append(Circle((x, y), 25))
            elif waypoints[i][j] == TYPE_WAYPOINT:
                points.append(Circle((x, y), 10))
            elif waypoints[i][j] == TYPE_DANGEROUS:
                dpoints.append(Circle((x, y), 10))
            elif waypoints[i][j] == TYPE_NEIGH:
                neighs.append(Circle((x, y), 10))

            j += 1
            y += STEP_CORN_Y
        i += 1
        x += STEP_CORN_X

    p = PatchCollection(wcorn, cmap=matplotlib.cm.jet,
                        alpha=0.5, facecolor=(.5,.5,1))
    ax.add_collection(p)

    p = PatchCollection(bcorn, cmap=matplotlib.cm.jet,
                        alpha=0.7, facecolor=(0,0,0))
    ax.add_collection(p)

    p = PatchCollection(points, cmap=matplotlib.cm.jet,
                        alpha=0.3, facecolor=(1,1,0))
    ax.add_collection(p)

    p = PatchCollection(dpoints, cmap=matplotlib.cm.jet,
                        alpha=0.5, facecolor=(1,0,1))
    ax.add_collection(p)

    p = PatchCollection(neighs, cmap=matplotlib.cm.jet,
                        alpha=1., facecolor=(1,0,0),
                        edgecolor=(1,0,0))
    ax.add_collection(p)


def graph(filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')

    build_area(ax)

    ax.grid(color = (0.3, 0.3, 0.3))
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2200)
    #ax.set_xlim(0, 825)
    #ax.set_ylim(1472, 1972)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

graph("test.png")
