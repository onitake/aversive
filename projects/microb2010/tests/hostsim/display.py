import math, sys, time, os
from visual import *

AREA_X = 3000.
AREA_Y = 2100.

area = [ (0.0, 0.0, -0.2), (3000.0, 2100.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=areasize, color=(0.0, 1.0, 0.0))

scene.autoscale=0

robot = box(pos = (0, 0, 150),
            size = (300,300,300),
            color = (1, 0, 0) )

last_pos = robot.pos.x, robot.pos.y, robot.pos.z
robot_trail = curve()
max_trail = 500

def set_robot(x, y, a):
    global robot, last_pos
    robot.pos = (x - AREA_X/2, y - AREA_Y/2, 150)
    robot.axis = (math.cos(a*math.pi/180) * 300,
                  math.sin(a*math.pi/180) * 300,
                  0)

    pos = robot.pos.x, robot.pos.y, robot.pos.z
    if pos != last_pos:
        robot_trail.append(pos)
        last_pos = pos
    robot_trail_l = len(robot_trail.pos)
    if robot_trail_l> max_trail:
        robot_trail.pos = robot_trail.pos[robot_trail_l - max_trail:]

while True:
    try:
        os.mkfifo("/tmp/.robot")
    except:
        pass
    while True:
        f = open("/tmp/.robot")
        while True:
            l = f.readline()
            if l == "":
                break
            x,y,a = map(lambda x:int(x), l[:-1].split(" "))
            set_robot(x,y,a)
        f.close()


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
    elif k == "k":
        robot_trail.pos = robot_trail.pos[0:0]
