import math, sys, time, os
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
            size = (300,300,300),
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

def set_robot(x, y, a):
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos

    robot.pos = (x - AREA_X/2, y - AREA_Y/2, 150)
    robot.axis = (math.cos(a*math.pi/180) * 300,
                  math.sin(a*math.pi/180) * 300,
                  0)

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

while True:
    silent_mkfifo("/tmp/.robot_sim2dis")
    silent_mkfifo("/tmp/.robot_dis2sim")
    while True:
        fr = open("/tmp/.robot_sim2dis", "r")
        fw = open("/tmp/.robot_dis2sim", "w", 0)
        while True:
            l = fr.readline()
            if l == "":
                break
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

        fr.close()
        fw.close()

