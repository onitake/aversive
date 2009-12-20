import sys, re, math
import numpy as np
import matplotlib
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
from numpy.random import randn
import pylab
import popen2, random

Path = mpath.Path
FLOAT = "([-+]?[0-9]*\.?[0-9]+)"
INT = "([-+]?[0-9][0-9]*)"
RANDOM_ERROR = 0.3 # deg
beacons = [ (0.0, 1050.0), (3000.0, 0.0), (3000.0, 2100.0) ]

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

def get_angle(ref, b):
    """get angle from robot point of view (ref) of beacon 'b'"""
    a = math.atan2(b[1]-ref[1], b[0]-ref[0])
    ea = (math.pi/180.) * RANDOM_ERROR * random.random()
    ea = random.choice([ea, -ea])
    return a + ea, ea

    alpha = math.atan2(a[1]-ref[1], a[0]-ref[0])
    beta = math.atan2(b[1]-ref[1], b[0]-ref[0])
    gamma = beta-alpha
    if gamma < 0:
        gamma = gamma + 2*math.pi
    return gamma + error, error

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def graph(filename, real_x, real_y):

    real_pt = (real_x, real_y)

    # display beacons
    patches = []
    for b in beacons:
        patches += [ Circle((b[0], b[1]), 40, alpha=0.4) ]

    patches += [ Circle((real_x, real_y), 20, alpha=0.4, facecolor="red") ]

    # process angles from robot position
    a0,ea0 = get_angle((real_x, real_y), beacons[0])
    a1,ea1 = get_angle((real_x, real_y), beacons[1])
    a2,ea2 = get_angle((real_x, real_y), beacons[2])
    text  = "a0 = %2.2f (%+2.2f deg)\n"%(a0, ea0*(180./math.pi))
    text += "a1 = %2.2f (%+2.2f deg)\n"%(a1, ea1*(180./math.pi))
    text += "a2 = %2.2f (%+2.2f deg)\n"%(a2, ea2*(180./math.pi))

    a01 = a1-a0
    if a01 < 0:
        a01 += 2*math.pi
    a12 = a2-a1
    if a12 < 0:
        a12 += 2*math.pi
    a20 = a0-a2
    if a20 < 0:
        a20 += 2*math.pi

    cmd = "./main angle2pos %f %f %f"%(a01, a12, a20)
    o,i = popen2.popen2(cmd)
    i.close()
    s = o.read(1000000)
    o.close()

    open(filename + ".txt", "w").write(s)

    if len(s) == 1000000:
        gloupix()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title("Erreur de position en mm lorsqu'on ajoute une erreur de mesure\n"
                 "d'angle aleatoire comprise entre - %1.1f et %1.1f deg"%(RANDOM_ERROR,
                                                                          RANDOM_ERROR))

    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')

    for l in s.split("\n"):
        m = re.match("circle: x=%s y=%s r=%s"%(FLOAT, FLOAT, FLOAT), l)
        if m:
            x,y,r = (float(m.groups()[0]), float(m.groups()[1]), float(m.groups()[2]))
            print x,y,r
            patches += [ Circle((x, y), r, facecolor="none") ]
        m = re.match("p%s: x=%s y=%s"%(INT, FLOAT, FLOAT), l)
        if m:
            n,x,y = (float(m.groups()[0]), float(m.groups()[1]), float(m.groups()[2]))
            if (n == 0):
                patches += [ Circle((x, y), 20, alpha=0.4, facecolor="yellow") ]
                result_pt = (x, y)
            text += l + "\n"

    p = PatchCollection(patches, cmap=matplotlib.cm.jet, match_original=True)

    # text area, far from the point
    l = [(800., 1800.), (800., 500.), (1500., 1800.), (1500., 500.),
         (2200., 1800.), (2200., 500.)]
    l.sort(cmp=lambda p1,p2: (dist(p1,real_pt)<dist(p2,real_pt)) and 1 or -1)
    x,y = l[0]
    text += "real_pt: x=%2.2f, y=%2.2f\n"%(real_x, real_y)
    text += "error = %2.2f mm"%(dist(real_pt, result_pt))
    matplotlib.pyplot.text(x, y, text, size=8,
             ha="center", va="center",
             bbox = dict(boxstyle="round",
                         ec=(1., 0.5, 0.5),
                         fc=(1., 0.8, 0.8),
                         alpha=0.6,
                         ),
             alpha=0.8
             )
    ax.add_collection(p)

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2200)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

def do_random_test():
    random.seed(0)
    for i in range(100):
        x = random.randint(0, 3000)
        y = random.randint(0, 2100)
        graph("test%d.png"%i, x, y)

def do_graph_2d(data, filename, title):
    # Make plot with vertical (default) colorbar
    fig = plt.figure()
    ax = fig.add_subplot(111)

    cax = ax.imshow(data)
    ax.set_title(title)

    # Add colorbar, make sure to specify tick locations to match desired ticklabels
    cbar = fig.colorbar(cax, ticks=[0, 50])
    cbar.ax.set_yticklabels(['0', '50 et plus'])# vertically oriented colorbar
    fig.savefig(filename)

def get_data(cmd, sat=0):
    data = np.array([[0.]*210]*300)
    oo,ii = popen2.popen2(cmd)
    ii.close()
    while True:
        l = oo.readline()
        if l == "":
            break
        try:
            x,y,e = l[:-1].split(" ")
        except:
            print "Fail: %s"%(l)
            continue
        x = int(x)
        y = int(y)
        e = float(e)
        if sat:
            if e < sat:
                e = 0
            else:
                e = sat
        data[x,y] = e
    oo.close()
    return data

def do_graph_2d_simple_error():
    for i in range(4):
        for j in ["0.1", "0.5", "1.0"]:
            data = get_data("./main simple_error %d %s"%(i,j))
            if i != 3:
                title  = 'Erreur de position en mm, pour une erreur\n'
                title += 'de mesure de %s deg sur la balise %d'%(j,i)
            else:
                title  = 'Erreur de position en mm, pour une erreur\n'
                title += 'de mesure de %s deg sur les 3 balises'%(j)
            do_graph_2d(data, "error_a%d_%s.png"%(i,j), title)

def do_graph_2d_move_error():
    i = 0
    for period in [ 20, 40 ]:
        for speed in [ 0.3, 0.7, 1. ]:
            angle_deg = 0
            while angle_deg < 360:
                angle_rad = angle_deg * (math.pi/180.)
                data = get_data("./main move_error %f %f %f"%(speed, period, angle_rad))
                do_graph_2d(data, "error_move_error_%d.png"%(i),
                            'Erreur de mesure si le robot se deplace a %2.2f m/s\n'
                            'vers %d deg (periode tourelle = %d ms)'%(speed, angle_deg, period))
                angle_deg += 45
                i += 1
    period = 20
    speed = 1.
    angle_deg = 45
    angle_rad = angle_deg * (math.pi/180.)
    data = get_data("./main move_error %f %f %f"%(speed, period, angle_rad), sat=20)
    do_graph_2d(data, "error_move_error_%d.png"%(i),
                "En rouge, l'erreur de mesure est > 2cm (pour un deplacement\n"
                'a %2.2f m/s vers %d deg et une periode tourelle = %d ms)'%(speed, angle_deg, period))

#do_random_test()
#do_graph_2d_simple_error()
do_graph_2d_move_error()
