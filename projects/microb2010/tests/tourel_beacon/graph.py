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
RANDOM_ERROR_A = 0.5 # deg
RANDOM_ERROR_D = 0.5 # percent
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
    ea = (math.pi/180.) * RANDOM_ERROR_A * random.random()
    ea = random.choice([ea, -ea])
    return a + ea, ea

def get_distance(ref, b):
    """get distance between robot (ref) and beacon 'b'"""
    d = math.sqrt((b[1]-ref[1])**2 + (b[0]-ref[0])**2)
    ed = RANDOM_ERROR_D * random.random()
    ed = random.choice([ed, -ed])
    return d + (d * ed / 100.), ed

def dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

# graph position from distance + angle
def graph_da(filename, real_x, real_y, real_a):
    pcol = []

    print "real_pt = %s"%(str((real_x, real_y, real_a)))
    real_pt = (real_x, real_y)

    # display beacons
    patches = []
    for b in beacons:
        patches += [ Circle((b[0], b[1]), 40, alpha=0.4) ]
    pcol.append(PatchCollection(patches, facecolor="yellow", alpha = 1))

    patches = [ Circle((real_x, real_y), 20, alpha=0.4, facecolor="red") ]
    pcol.append(PatchCollection(patches, facecolor="red", alpha = 1))

    # process angles from robot position
    a0,ea0 = get_angle((real_x, real_y), beacons[0])
    a1,ea1 = get_angle((real_x, real_y), beacons[1])
    a0 -=  real_a
    a1 -=  real_a
    text  = "a0 = %2.2f (%+2.2f deg)\n"%(a0, ea0*(180./math.pi))
    text += "a1 = %2.2f (%+2.2f deg)\n"%(a1, ea1*(180./math.pi))
    d0,ed0 = get_distance((real_x, real_y), beacons[0])
    d1,ed1 = get_distance((real_x, real_y), beacons[1])
    text += "d0 = %2.2f (%+2.2f %%)\n"%(d0, ed0)
    text += "d1 = %2.2f (%+2.2f %%)\n"%(d1, ed1)

    cmd = "./main ad2pos %f %f %f %f"%(a0, a1, d0, d1)
    print cmd
    o,i = popen2.popen2(cmd)
    i.close()
    s = o.read(1000000)
    o.close()

    open(filename + ".txt", "w").write(s)

    if len(s) == 1000000:
        gloupix()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    title = "Erreur de position en mm, qd l'erreur "
    title += "d'angle aleatoire est comprise\n"
    title += "erreur -%1.1f et %1.1f deg "%(RANDOM_ERROR_A, RANDOM_ERROR_A)
    title += "et de distance de +/- %2.2f %%"%(RANDOM_ERROR_D)
    ax.set_title(title)

    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')

    result_pt = (-1, -1)
    patches = []
    for l in s.split("\n"):
        m = re.match("circle: x=%s y=%s r=%s"%(FLOAT, FLOAT, FLOAT), l)
        if m:
            x,y,r = (float(m.groups()[0]), float(m.groups()[1]), float(m.groups()[2]))
            print x,y,r
            patches += [ Circle((x, y), r, facecolor="none") ]
        m = re.match("p%s: x=%s y=%s a=%s"%(INT, FLOAT, FLOAT, FLOAT), l)
        if m:
            n,x,y,a = (float(m.groups()[0]), float(m.groups()[1]),
                        float(m.groups()[2]), float(m.groups()[3]))
            if (n == 0):
                patches += [ Circle((x, y), 20, alpha=0.4, facecolor="yellow") ]
                result_pt = (x, y)
            text += l + "\n"

    pcol.append(PatchCollection(patches, facecolor="none", alpha = 0.6))
    pcol.append(PatchCollection(patches, facecolor="blue", alpha = 0.2))

    patches = [ Circle(result_pt, 20, alpha=0.4, facecolor="green") ]
    pcol.append(PatchCollection(patches, cmap=matplotlib.cm.jet, alpha = 1))

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
    for p in pcol:
        ax.add_collection(p)

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2200)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

# graph position from angles
def graph(filename, real_x, real_y, real_a):
    pcol = []
    print "real_pt = %s"%(str((real_x, real_y, real_a)))
    real_pt = (real_x, real_y)

    # display beacons
    patches = []
    for b in beacons:
        patches += [ Circle((b[0], b[1]), 40, alpha=0.4) ]
    pcol.append(PatchCollection(patches, facecolor="yellow", alpha = 1))

    patches = [ Circle((real_x, real_y), 20, alpha=0.4, facecolor="red") ]
    pcol.append(PatchCollection(patches, facecolor="red", alpha = 1))

    # process angles from robot position
    a0,ea0 = get_angle((real_x, real_y), beacons[0])
    a1,ea1 = get_angle((real_x, real_y), beacons[1])
    a2,ea2 = get_angle((real_x, real_y), beacons[2])
    a0 -=  real_a
    a1 -=  real_a
    a2 -=  real_a
    text  = "a0 = %2.2f (%+2.2f deg)\n"%(a0, ea0*(180./math.pi))
    text += "a1 = %2.2f (%+2.2f deg)\n"%(a1, ea1*(180./math.pi))
    text += "a2 = %2.2f (%+2.2f deg)\n"%(a2, ea2*(180./math.pi))

    cmd = "./main angle2pos %f %f %f"%(a0, a1, a2)
    print cmd
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
                 "d'angle aleatoire comprise entre - %1.1f et %1.1f deg"%(RANDOM_ERROR_A,
                                                                          RANDOM_ERROR_A))

    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')

    result_pt = (-1, -1)
    patches = []
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
                result_pt = (x, y)
            text += l + "\n"

    pcol.append(PatchCollection(patches, facecolor="none", alpha = 0.6))
    pcol.append(PatchCollection(patches, facecolor="blue", alpha = 0.2))

    patches = [ Circle(result_pt, 20, alpha=0.4, facecolor="green") ]
    pcol.append(PatchCollection(patches, cmap=matplotlib.cm.jet, alpha = 1))

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
    for p in pcol:
        ax.add_collection(p)

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2200)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

def do_random_test():
    random.seed(0)
    for i in range(21):
        print "---- random %d"%i
        x = random.randint(0, 3000)
        y = random.randint(0, 2100)
        a = random.random()*2*math.pi
        graph("test%d.png"%i, x, y, a)
        graph_da("test_da%d.png"%i, x, y, a)

def do_graph_2d(data, filename, title):
    # Make plot with vertical (default) colorbar
    fig = plt.figure()
    ax = fig.add_subplot(111)

    cax = ax.imshow(data)
    ax.set_title(title)

    # Add colorbar, make sure to specify tick locations to match desired ticklabels
    cbar = fig.colorbar(cax, ticks=[0, 50])
    cbar.ax.set_yticklabels(['0', '50 and more'])# vertically oriented colorbar
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
            print "do_graph_2d_simple_error %s %s"%(i, j)
            data = get_data("./main simple_error %d %s"%(i,j))
            if i != 3:
                title  = 'Erreur de position en mm, pour une erreur\n'
                title += 'de mesure de %s deg sur la balise %d'%(j,i)
            else:
                title  = 'Erreur de position en mm, pour une erreur\n'
                title += 'de mesure de %s deg sur les 3 balises'%(j)
            do_graph_2d(data, "error_a%d_%s.png"%(i,j), title)

def do_graph_2d_ad_error():
    for d in ["0.0", "0.1", "0.5"]:
        for a in ["0.0", "0.1", "0.5"]:
            for i in ["0", "1", "2"]:
                print "do_graph_2d_ad_error %s %s %s"%(i, d, a)
                data = get_data("./main da_error %s %s %s"%(i, d, a))
                title  = 'Erreur de position en mm, pour une erreur\n'
                title += "d'angle de %s deg et dist de %s %% (algo %s)"%(a, d, i)
                do_graph_2d(data, "error_da_%s_%s_%s.png"%(i, d, a), title)

def do_graph_2d_move_error():
    i = 0
    for period in [ 20, 40 ]:
        for speed in [ 0.3, 0.7, 1. ]:
            angle_deg = 0
            print "do_graph_2d_move_error %s %s"%(period, speed)
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

do_random_test()
do_graph_2d_simple_error()
do_graph_2d_move_error()
do_graph_2d_ad_error()
