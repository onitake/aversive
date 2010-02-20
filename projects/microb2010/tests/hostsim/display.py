import math, sys, time, os, random
from visual import *

AREA_X = 3000.
AREA_Y = 2100.

area = [ (0.0, 0.0, -0.2), (3000.0, 2100.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=areasize, color=(0.0, 1.0, 0.0))

scene.autoscale=0

# all positions of robot every 5ms
save_pos = []

robot = box(pos = (0, 0, 150),
            size = (250,320,350),
            color = (1, 0, 0) )

last_pos = robot.pos.x, robot.pos.y, robot.pos.z
hcenter_line = curve()
hcenter_line.pos = [(-AREA_X/2, 0., 0.3), (AREA_X/2, 0., 0.3)]
vcenter_line = curve()
vcenter_line.pos = [(0., -AREA_Y/2, 0.3), (0., AREA_Y/2, 0.3)]

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


def toggle_obj_disp():
    global area_objects

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
                    c = cylinder(axis=(0,0,1), length=150,
                                 radius=25, color=(0.8,0.8,0.8),
                                 pos=(x-AREA_X/2,y-AREA_Y/2,75))
                    area_objects.append(c)
                elif waypoints[i][j] == TYPE_BLACK_CORN:
                    c = cylinder(axis=(0,0,1), length=150,
                                 radius=25, color=(0.2,0.2,0.2),
                                 pos=(x-AREA_X/2,y-AREA_Y/2,75))
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


def set_robot(x, y, a):
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos

    robot.pos = (x - AREA_X/2, y - AREA_Y/2, 150)
    robot.axis = (math.cos(a*math.pi/180),
                  math.sin(a*math.pi/180),
                  0)
    robot.size = (250, 320, 350)

    # save position
    save_pos.append((robot.pos.x, robot.pos, a))

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

init_corn_table(random.randint(0,8), random.randint(0,3))
waypoints = init_waypoints()
toggle_obj_disp()

while True:
    silent_mkfifo("/tmp/.robot_sim2dis")
    silent_mkfifo("/tmp/.robot_dis2sim")
    while True:
        fr = open("/tmp/.robot_sim2dis", "r")
        fw = open("/tmp/.robot_dis2sim", "w", 0)
        while True:
            l = fr.readline()
            try:
                x,y,a = map(lambda x:int(x), l[:-1].split(" "))
                set_robot(x,y,a)
            except ValueError:
                pass

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

            # EOF
            if l == "":
                break

        fr.close()
        fw.close()

