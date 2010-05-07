import math, sys, time, os, random, re
from visual import *

FLOAT = "([-+]?[0-9]*\.?[0-9]+)"
INT = "([-+]?[0-9][0-9]*)"

AREA_X = 3000.
AREA_Y = 2100.

ROBOT_HEIGHT=5 # 350
CORN_HEIGHT=5  # 150
BALL_CYLINDER=1 # 0

ROBOT_WIDTH=320
ROBOT_LENGTH=250

area = [ (0.0, 0.0, -0.2), (3000.0, 2100.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=areasize, color=(0.0, 1.0, 0.0))

scene.autoscale=0

# all positions of robot every 5ms
save_pos = []

robot = box(color=(0.4, 0.4, 0.4))
lspickle = box(color=(0.4, 0.4, 0.4))
rspickle = box(color=(0.4, 0.4, 0.4))

opp = box(color=(0.7, 0.2, 0.2))

last_pos = (0.,0.,0.)
hcenter_line = curve()
hcenter_line.pos = [(-AREA_X/2, 0., 0.3), (AREA_X/2, 0., 0.3)]
vcenter_line = curve()
vcenter_line.pos = [(0., -AREA_Y/2, 0.3), (0., AREA_Y/2, 0.3)]

yellowarea = [ (0.0, 0.0, -0.5), (500.0, 500.0, 0.5) ]
yellowareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , yellowarea)
yellowarea_box = box(pos=(-AREA_X/2+250,-AREA_Y/2+250,0), size=yellowareasize, color=(1.0, 1.0, 0.0))

bluearea = [ (0.0, 0.0, -0.5), (500.0, 500.0, 0.5) ]
blueareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , bluearea)
bluearea_box = box(pos=(AREA_X/2-250,-AREA_Y/2+250,0), size=blueareasize, color=(0.0, 0.0, 1.0))

greyarea = [ (0.0, 0.0, -0.5), (1520.0, 500.0, 0.5) ]
greyareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , greyarea)
greyarea_box = box(pos=(0,-AREA_Y/2+250,0), size=greyareasize, color=(0.3, 0.6, 0.3))

YELLOW = 0
BLUE = 1
color = YELLOW

def square(sz):
    sq = curve()
    sq.pos = [(-sz, -sz, 0.3),
              (-sz, sz, 0.3),
              (sz, sz, 0.3),
              (sz, -sz, 0.3),
              (-sz, -sz, 0.3),]
    return sq

sq1 = square(250)
sq2 = square(500)

robot_x = 0.
robot_y = 0.
robot_a = 0.
robot_lspickle_deployed = 0
robot_rspickle_deployed = 0
robot_lspickle_autoharvest = 0
robot_rspickle_autoharvest = 0
robot_trail = curve()
robot_trail_list = []
max_trail = 500

area_objects = []

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
TYPE_BALL=5
TYPE_NEIGH=6

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

            # balls
            if (i & 1) == 0 and j > 3 and \
                    (not (i == 0 and j == WAYPOINTS_NBY-1)) and \
                    (not (i == WAYPOINTS_NBX-1 and j == WAYPOINTS_NBY-1)):
                waypoints[i][j] = TYPE_BALL
                continue
            if (i == 0 or i == WAYPOINTS_NBX-1) and j > 2 and \
                    (not (i == 0 and j == WAYPOINTS_NBY-1)) and \
                    (not (i == WAYPOINTS_NBX-1 and j == WAYPOINTS_NBY-1)):
                waypoints[i][j] = TYPE_BALL
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


def toggle_obj_disp():
    global area_objects

    """
    if area_objects == []:
        c = sphere(radius=5, color=(0., 0.,1.),
                   pos=(1238.-AREA_X/2, 1313.-AREA_Y/2, 5))
        area_objects.append(c)
        c = sphere(radius=5, color=(0., 0.,1.),
                   pos=(1364.-AREA_X/2, 1097.-AREA_Y/2, 5))
        area_objects.append(c)
        c = sphere(radius=5, color=(0., 0.,1.),
                   pos=(1453.-AREA_X/2, 1176.-AREA_Y/2, 5))
        area_objects.append(c)
        c = sphere(radius=5, color=(0., 0.,1.),
                   pos=(1109.-AREA_X/2, 1050.-AREA_Y/2, 5))
        area_objects.append(c)
"""
    if area_objects == []:
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
                print x,y
                if waypoints[i][j] == TYPE_WHITE_CORN:
                    c = cylinder(axis=(0,0,1), length=CORN_HEIGHT,
                                 radius=25, color=(0.8,0.8,0.8),
                                 pos=(x-AREA_X/2,y-AREA_Y/2,CORN_HEIGHT/2))
                    area_objects.append(c)
                elif waypoints[i][j] == TYPE_BLACK_CORN:
                    c = cylinder(axis=(0,0,1), length=CORN_HEIGHT,
                                 radius=25, color=(0.2,0.2,0.2),
                                 pos=(x-AREA_X/2,y-AREA_Y/2,CORN_HEIGHT/2))
                    area_objects.append(c)
                elif waypoints[i][j] == TYPE_BALL:
                    if BALL_CYLINDER == 1:
                        c = cylinder(axis=(0,0,1), radius=50,
                                     length=CORN_HEIGHT,
                                     color=(1., 0.,0.),
                                     pos=(x-AREA_X/2,y-AREA_Y/2,CORN_HEIGHT/2))
                    else:
                        c = sphere(radius=50, color=(1., 0.,0.),
                                   pos=(x-AREA_X/2,y-AREA_Y/2,50))

                    area_objects.append(c)
                else:
                    c = sphere(radius=5, color=(0., 0.,1.),
                               pos=(x-AREA_X/2,y-AREA_Y/2,5))
                    area_objects.append(c)
                j += 1
                y += STEP_CORN_Y
            i += 1
            x += STEP_CORN_X
    else:
        for o in area_objects:
            if o.visible:
                o.visible = 0
            else:
                o.visible = 1

def toggle_color():
    global color
    global BLUE, YELLOW
    if color == YELLOW:
        color = BLUE
    else:
        color = YELLOW

def set_opp(x, y):
    opp.size = (300, 300, ROBOT_HEIGHT)
    opp.pos = (x, y, ROBOT_HEIGHT/2)

def set_robot():
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos, robot_x, robot_y, robot_a

    if color == YELLOW:
        tmp_x = robot_x - AREA_X/2
        tmp_y = robot_y - AREA_Y/2
        tmp_a = robot_a
    else:
        tmp_x = -robot_x + AREA_X/2
        tmp_y = -robot_y + AREA_Y/2
        tmp_a = robot_a

    robot.pos = (tmp_x, tmp_y, ROBOT_HEIGHT/2)
    axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot.axis = axis
    robot.size = (ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT)

    lspickle.pos = (tmp_x + (robot_lspickle_deployed*60) * math.cos((tmp_a+90)*math.pi/180),
                    tmp_y + (robot_lspickle_deployed*60) * math.sin((tmp_a+90)*math.pi/180),
                    ROBOT_HEIGHT/2)
    lspickle.axis = axis
    lspickle.size = (20, ROBOT_WIDTH, 5)
    if robot_lspickle_autoharvest:
        lspickle.color = (1, 0, 0)
    else:
        lspickle.color = (0.4, 0.4, 0.4)

    rspickle.pos = (tmp_x + (robot_rspickle_deployed*60) * math.cos((tmp_a-90)*math.pi/180),
                    tmp_y + (robot_rspickle_deployed*60) * math.sin((tmp_a-90)*math.pi/180),
                    ROBOT_HEIGHT/2)
    rspickle.axis = axis
    rspickle.size = (20, ROBOT_WIDTH, 5)
    if robot_rspickle_autoharvest:
        rspickle.color = (1, 0, 0)
    else:
        rspickle.color = (0.4, 0.4, 0.4)

    # save position
    save_pos.append((robot.pos.x, robot.pos.y, tmp_a))

    pos = robot.pos.x, robot.pos.y, 0.3
    if pos != last_pos:
        robot_trail_list.append(pos)
        last_pos = pos
    robot_trail_l = len(robot_trail_list)
    if robot_trail_l > max_trail:
        robot_trail_list = robot_trail_list[robot_trail_l - max_trail:]
    robot_trail.pos = robot_trail_list

def graph():
    pass

def save():
    f = open("/tmp/robot_save", "w")
    for p in save_pos:
        f.write("%f %f %f\n"%(p[0], p[1], p[2]))
    f.close()

def silent_mkfifo(f):
    try:
        os.mkfifo(f)
    except:
        pass

#init_corn_table(random.randint(0,8), random.randint(0,3))
init_corn_table(0, 0)
waypoints = init_waypoints()
toggle_obj_disp()

while True:
    silent_mkfifo("/tmp/.robot_sim2dis")
    silent_mkfifo("/tmp/.robot_dis2sim")
    while True:
        fr = open("/tmp/.robot_sim2dis", "r")
        fw = open("/tmp/.robot_dis2sim", "w", 0)
        while True:
            m = None
            l = fr.readline()

            # parse position
            if not m:
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l)
                if m:
                    robot_x = int(m.groups()[0])
                    robot_y = int(m.groups()[1])
                    robot_a = int(m.groups()[2])
                    set_robot()

            # parse ballboard
            if not m:
                m = re.match("ballboard=%s"%(INT), l)
                if m:
                    print "ballboard: %d"%(int(m.groups()[0]))

            # parse cobboard
            if not m:
                m = re.match("cobboard=%s,%s"%(INT,INT), l)
                if m:
                    #print "cobboard: %x,%x"%(int(m.groups()[0]),int(m.groups()[1]))
                    side = int(m.groups()[0])
                    flags = int(m.groups()[1])
                    if side == 0:
                        robot_lspickle_deployed = ((flags & 1) * 2)
                        robot_lspickle_autoharvest = ((flags & 2) != 0)
                    else:
                        robot_rspickle_deployed = ((flags & 1) * 2)
                        robot_rspickle_autoharvest = ((flags & 2) != 0)

            if scene.mouse.events != 0:
                oppx, oppy, oppz = scene.mouse.getevent().project(normal=(0,0,1))
                set_opp(oppx, oppy)
                try:
                    if color == YELLOW:
                        fw.write("opp %d %d"%(int(oppx + 1500), int(oppy + 1050)))
                    else:
                        fw.write("opp %d %d"%(int(1500 - oppx), int(1050 - oppy)))
                except:
                    print "not connected"

            if scene.kb.keys == 0:
                continue

            k = scene.kb.getkey()
            x,y,z = scene.center
            if k == "left":
                scene.center = x-10,y,z
            elif k == "right":
                scene.center = x+10,y,z
            elif k == "up":
                scene.center = x,y+10,z
            elif k == "down":
                scene.center = x,y-10,z
            elif k == "l":
                fw.write("l")
            elif k == "r":
                fw.write("r")
            elif k == "c":
                robot_trail_list = []
            elif k == "x":
                save_pos = []
            elif k == "g":
                graph()
            elif k == "s":
                save()
            elif k == "h":
                toggle_obj_disp()
            elif k == "i":
                toggle_color()
            else:
                print k

            # EOF
            if l == "":
                break

        fr.close()
        fw.close()

