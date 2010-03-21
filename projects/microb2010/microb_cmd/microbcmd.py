#! /usr/bin/env python

import os,sys,termios,atexit
import serial
from select import select
import cmd
#import pylab
from  matplotlib import pylab 
from math import *

import numpy 
import shlex
import time
import math
import warnings
warnings.filterwarnings("ignore","tempnam",RuntimeWarning, __name__)

import logging
log = logging.getLogger("MicrobShell")
_handler = logging.StreamHandler()
_handler.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
log.addHandler(_handler)
log.setLevel(1)

MICROB_PATH=os.path.dirname(sys.argv[0])

SPM_PAGE_SIZE = 256

def crc_ccitt_update (crc, data):
    """crc argument is the previous value of 16 bits crc (the initial
    value is 0xffff). 'data' is the 8 bits value added to crc. The
    function returns the new crc value."""

    data ^= (crc & 0xff)
    data ^= (data << 4)
    data &= 0xff
    
    ret = (data << 8) & 0xffff
    ret |= ((crc >> 8) & 0xff)
    ret ^= ((data >> 4) & 0xff)
    ret ^= ((data << 3) & 0xffff)
    return ret

def do_crc(buf):
    i = 0
    crc = 0xffff
    sum = 0
    while i < len(buf):
        crc = crc_ccitt_update(crc, ord(buf[i]))
        sum +=  ord(buf[i])
        i += 1
    return (crc << 16) + (sum & 0xffff)

def prog_page(ser, addr, buf):
    """program a page from buf at addr"""
    
    # switch in program mode 
    ser.flushInput()
    ser.write('p')

    # send address
    s = ser.readline()
    if not s.endswith("addr?\r\n"):
        print "failed (don't match addr)"
        return -1
    ser.write("%x\n"%addr)
    s = ser.readline()
    if not s.startswith("ok"):
        print "failed"
        return -1

    # fill page with buf data
    page = [ '\xff' ] * SPM_PAGE_SIZE
    i = 0
    while i < SPM_PAGE_SIZE and i < len(buf):
        page[i] = buf[i]
        i += 1
    
    # send data
    i = 0
    while i < SPM_PAGE_SIZE:
        c = page[i]
        ser.write(c)
        i += 1

    sys.stdout.write(".")
    sys.stdout.flush()

    # compare crc
    avr_crc = int(ser.readline()[0:8], 16)

    crc = do_crc(page)
    if crc != avr_crc:
        print "failed: bad crc %x %x"%(crc, avr_crc)
        ser.write('n')
        return -1

    ser.write('y')
    s = ser.readline()
    if not s.startswith("OK"):
        print "failed"
        return -1
    return 0

def read32(ser, addr):
    """read a 32 bits value at addr"""
    
    # switch in program mode 
    ser.flushInput()
    ser.write('d')

    # send address
    s = ser.readline()
    if not s.endswith("addr?\r\n"):
        print "failed (don't match addr)"
        return -1
    ser.write("%x\n"%addr)
    s = ser.readline()
    return int(s)

def check_crc(ser, buf, offset, size):
    """Process the crc of buf, ask for a crc of the flash, and check
    that value is correct"""
    if size <= 0:
        return 0

    # go in CRC mode
    ser.flushInput()
    ser.write('c')

    # send addr
    s = ser.readline()
    if not s.endswith("addr?\r\n"):
        print "failed <%s>"%s
        return -1
    ser.write("%x\n"%offset)

    # send size
    s = ser.readline()
    if not s.startswith("size?"):
        print "failed"
        return -1
    ser.write("%x\n"%size)

    # compare CRC
    crc = do_crc(buf[offset:offset+size])
    avr_crc = int(ser.readline()[0:8], 16)
    if crc != avr_crc:
        return -1
    return 0

class SerialLogger:
    def __init__(self, ser, filein, fileout=None):
        self.ser = ser
        self.filein = filein
        self.fin = open(filein, "a", 0)
        if fileout:
            self.fileout = fileout
            self.fout = open(fileout, "a", 0)
        else:
            self.fileout = filein
            self.fout = self.fin
    def fileno(self):
        return self.ser.fileno()
    def read(self, *args):
        res = self.ser.read(*args)
        self.fin.write(res)
        return res
    def write(self, s):
        self.fout.write(s)
        self.ser.write(s)





"""
fig = figure()

ax = subplot(111)


X = 45.
Y = -10.
l1 = 9.
l2 = 21.13
l3 = 47.14

l_mirror = 249.
h_mirror = 13.


def ang2_a_mirror(b):
    x2 = X+l1*math.cos(b)
    y2 = Y+l1*math.sin(b)

    A = (l3**2+x2**2+y2**2-l2**2)/(2*l3)

    DELTA = -(A**2-x2**2-y2**2)
    B = +math.sqrt(DELTA)

    D = x2**2+y2**2

    c_a = (x2*A+y2*B)/D
    s_a = -(x2*B-y2*A)/D

    a = math.atan2(s_a, c_a)
    return x2, y2, c_a, s_a, a


def ang2_H_L(l_telemetre, c_a, s_a, a):
    d = h_mirror*c_a/s_a
    H = (l_telemetre - l_mirror - d)*math.sin(2*a)
    L = l_mirror + d + H/math.tan(2*a)
    return H, L

all_p = []
for b in xrange(0, 360, 20):
    b = b*2*math.pi / 360.

    x2, y2, c_a, s_a, a = ang2_a_mirror(b)
    x1 = l3*c_a
    y1 = l3*s_a
    
    px = [0, x1, x2, X]
    py = [0, y1, y2, Y]

    all_p+=[px, py]

    print math.sqrt((x2-x1)**2+(y2-y1)**2)

    H, L = ang2_H_L(400., c_a, s_a, a)
    print H, L
    
ax.plot(*all_p)

show()

"""
         



class Interp(cmd.Cmd):
    prompt = "Microb> "
    def __init__(self, tty, baudrate=57600):
        cmd.Cmd.__init__(self)
        self.ser = serial.Serial(tty,baudrate=baudrate)
        self.escape  = "\x01" # C-a
        self.quitraw = "\x02" # C-b
        self.serial_logging = False
        self.default_in_log_file = "/tmp/microb.in.log"
        self.default_out_log_file = "/tmp/microb.out.log"

    def do_quit(self, args):
        return True

    def do_log(self, args):
        """Activate serial logs.
        log <filename>           logs input and output to <filename>
        log <filein> <fileout>   logs input to <filein> and output to <fileout>
        log                      logs to /tmp/microb.log or the last used file"""

        if self.serial_logging:
            log.error("Already logging to %s and %s" % (self.ser.filein, 
                                                        self.ser.fileout))
        else:
            self.serial_logging = True
            files = [os.path.expanduser(x) for x in args.split()]
            if len(files) == 0:
                files = [self.default_in_log_file, self.default_out_log_file]
            elif len(files) == 1:
                self.default_in_log_file = files[0]
                self.default_out_log_file = None
            elif len(files) == 2:
                self.default_in_log_file = files[0]
                self.default_out_log_file = files[1]
            else:
                print "Can't parse arguments"

            self.ser = SerialLogger(self.ser, *files)
            log.info("Starting serial logging to %s and %s" % (self.ser.filein, 
                                                               self.ser.fileout))


    def do_unlog(self, args):
        if self.serial_logging:
            log.info("Stopping serial logging to %s and %s" % (self.ser.filein, 
                                                               self.ser.fileout))
            self.ser = self.ser.ser
            self.serial_logging = False
        else:
            log.error("No log to stop")
        

    def do_raw(self, args):
        "Switch to RAW mode"
        stdin = os.open("/dev/stdin",os.O_RDONLY)
        stdout = os.open("/dev/stdout",os.O_WRONLY)

        stdin_termios = termios.tcgetattr(stdin)
        raw_termios = stdin_termios[:]
        
        try:
            log.info("Switching to RAW mode")

            # iflag
            raw_termios[0] &= ~(termios.IGNBRK | termios.BRKINT | 
                                termios.PARMRK | termios.ISTRIP | 
                                termios.INLCR | termios.IGNCR | 
                                termios.ICRNL | termios.IXON)
            # oflag
            raw_termios[1] &= ~termios.OPOST;
            # cflag
            raw_termios[2] &= ~(termios.CSIZE | termios.PARENB);
            raw_termios[2] |= termios.CS8;
            # lflag
            raw_termios[3] &= ~(termios.ECHO | termios.ECHONL | 
                                termios.ICANON | termios.ISIG | 
                                termios.IEXTEN);

            termios.tcsetattr(stdin, termios.TCSADRAIN, raw_termios)

            mode = "normal"
            while True:
                ins,outs,errs=select([stdin,self.ser],[],[])
                for x in ins:
                    if x == stdin:
                        c = os.read(stdin,1)
                        if mode  == "escape":
                            mode =="normal"
                            if c == self.escape:
                                self.ser.write(self.escape)
                            elif c == self.quitraw:
                                return
                            else:
                                self.ser.write(self.escape)
                                self.ser.write(c)
                        else:
                            if c == self.escape:
                                mode = "escape"
                            else:
                                self.ser.write(c)
                    elif x == self.ser:
                        os.write(stdout,self.ser.read())
        finally:
            termios.tcsetattr(stdin, termios.TCSADRAIN, stdin_termios)
            log.info("Back to normal mode")
            

    def do_arm_x(self, args):
        fsdf
        my_h = 100
        my_r = 220
        my_ang = 90

        self.ser.write("armxy %d %d %d\n"%(my_h, -my_r, my_ang))
        time.sleep(1)
        
        for i in xrange(-my_r, my_r, 25):
            self.ser.write("armxy %d %d %d\n"%(my_h, i, my_ang))
            self.ser.flushInput()
            
            time.sleep(0.03)

    def do_arm_y(self, args):
        my_x = 80
        my_r = 145
        my_ang = 0
        self.ser.write("armxy %d %d %d\n"%(-my_r, my_x, my_ang))
        time.sleep(1)
        
        for i in xrange(-my_r, my_r, 25):
            self.ser.write("armxy %d %d %d\n"%(i, my_x, my_ang))
            self.ser.flushInput()
            
            time.sleep(0.03)

    def do_arm_circ(self, args):
        add_h = 120
        add_d = 120
        l = 70
        for i in xrange(0, 360, 10):
            x = l*math.cos(i*math.pi/180)
            y = l*math.sin(i*math.pi/180)
            
            
            self.ser.write("armxy %d %d 90\n"%(x+add_h, y+add_d))
            self.ser.flushInput()
            
            time.sleep(0.05)

    def do_arm_init(self, args):
        self.arm_h = 130
        self.arm_v = 130
        self.mov_max = 20
        
        self.ser.write("armxy %d %d\n"%(self.arm_h, self.arm_v))        

    def arm_py_goto(self, h, v, a):
        """
        dh, dv = h-self.arm_h, v-self.arm_v
        d = math.sqrt(dh**2 + dv**2)

        old_h = self.arm_h
        old_v = self.arm_v
        
        mov_todo = int(d/self.mov_max)
        for i in xrange(1, mov_todo):
            p_h = dh*i/mov_todo
            p_v = dv*i/mov_todo

            new_h = old_h+p_h
            new_v = old_v+p_v
            
            self.ser.write("armxy %d %d %d\n"%(new_h, new_v, a))
            self.ser.flushInput()
            self.arm_h = new_h
            self.arm_v = new_v

            time.sleep(0.04)
            
        self.ser.write("armxy %d %d %d\n"%(h, v, a))
        self.ser.flushInput()
        """

        self.ser.write("armxy %d %d %d\n"%(h, v, a))
        self.ser.flushInput()

        time.sleep(0.2)
    
            
                
    def do_arm_tt(self, args):
        for i in xrange(2):
            self.arm_py_goto(80, 80, 200)
            self.arm_py_goto(80, 200, 200)
            self.arm_py_goto(200, 200, 200)
            self.arm_py_goto(200, 80, 200)

    def do_arm_harve(self, args):
        angl1 = 1
        angl2 = 100
        my_time = 0.03
        self.arm_py_goto(130,130,angl1)
        self.arm_py_goto(-150,60,angl1)
        time.sleep(0.1)
        
        self.ser.write("pwm 1B -3000\n")
        self.ser.flushInput()
        time.sleep(0.2)

        self.arm_py_goto(-120,60,angl1)
        time.sleep(2)
        self.arm_py_goto(-120,60,angl2)
        time.sleep(2)
        self.arm_py_goto(-150,60,angl2)
        self.ser.write("pwm 3C -3000\n")
        self.ser.flushInput()
        time.sleep(0.2)
        self.arm_py_goto(-130,60,angl2)
        self.arm_py_goto(0,160,angl2)

        #middle point
        self.arm_py_goto(-40,200,angl2)

        h = -150
        d = 210
        
        self.arm_py_goto(h,d,angl2)
        time.sleep(.3)
        self.ser.write("pwm 3C 3000\n")
        time.sleep(0.1)
        self.arm_py_goto(h+60,d,angl2)
        time.sleep(0.1)

        self.arm_py_goto(h+60,d,angl1)
        time.sleep(0.3)
        self.arm_py_goto(h+40,d,angl1)
        time.sleep(0.3)
        self.arm_py_goto(h+30,d,angl1)
        time.sleep(0.3)
        self.ser.write("pwm 1B 3000\n")
        time.sleep(0.1)
        self.arm_py_goto(h+70,d,angl1)

        self.ser.write("pwm 1B 0\n")
        self.ser.write("pwm 3C 0\n")

        self.arm_py_goto(130,130,angl2)

        
        
    def update_graph(self, val):
        freq = self.sfreq.val
        self.theta_max = freq*math.pi*2.0
        self.theta = pylab.arange(0.0, self.theta_max, self.theta_max/len(self.r))
        self.theta = self.theta[:len(self.r)]

        self.myplot.set_xdata(self.theta)
        draw()
    """    
    def do_graph(self, args):
        self.ser.write("pwm 1A 2000\n")
        time.sleep(0.5)
        print "sampling..."
        self.ser.write("sample start\n")
        while True:
            l = self.ser.readline()
            if "dump end" in l:
                break
        #time.sleep(2)
        self.ser.write("pwm 1A 0\n")
        l = self.ser.readline()
        l = self.ser.readline()

        print "dumping..."
        self.ser.write("sample dump\n")
        vals = []
        while True:
            l = self.ser.readline()
            if l[0] in ['s', 'c', 'a']:
                continue
            if l[0] in ['e']:
                break
            tokens = [x for x in shlex.shlex(l)]
            v = int(tokens[0])
            #v = min(v, 150)
            vals.append(v)
        vals.reverse()
        print "total vals:", len(vals)

        pylab.subplot(111, polar = True)
        self.r = vals
        valinit = 5.38
        #theta_max = 4.8*2.3*pi
        self.theta_max =valinit*pylab.pi
        self.theta = pylab.arange(0.0, self.theta_max, self.theta_max/len(self.r))
        
        self.myplot, = pylab.plot(self.theta, self.r)

        #slide bar
        axfreq = pylab.axes([0.25, 0.1, 0.65, 0.03])
        self.sfreq = pylab.Slider(axfreq, "Freq", 1, 20, valinit = valinit)
        self.sfreq.on_changed(self.update_graph)

        pylab.show()
    """


    def do_dump(self, args):

        t = [x for x in shlex.shlex(args)]
        
        t.reverse()
        do_img = False

        #send speed,debug=off
        #self.ser.write("scan_params 500 0\n")
        #send algo 1 wrkazone 1 cx 15 cy 15
        self.ser.write("scan_img 1 1 15 15\n")
        
        print t
        while len(t):
            x = t.pop()
            if x == 'img':
                do_img = True

        print "dumping..."
        self.ser.write("sample dump 0 0 400 0\n")



        while True:
            l = self.ser.readline()

            if "start dumping" in l:
                tokens = [x for x in shlex.shlex(l)]
                num_rows = int(tokens[-1])
                print "num row: ", num_rows
                break
            print l.strip()
        #scan_stop = time.time()
        #print "total time:", scan_stop-scan_start


        vals = []
        while True:
            l = self.ser.readline()

            if l[0] in ['s', 'c', 'a']:
                continue
            if l[0] in ['e']:
                break
            tokens = [x for x in shlex.shlex(l)]
            v = int(tokens[0])
            #v = min(v, 150)
            vals.append(v)


        #vals.reverse()
        print "total vals:", len(vals)
        valinit = 5

        #num_rows = int(600/valinit)
        #num_cols = int(valinit)
        num_rows_orig = num_rows
        num_rows *= 1
        num_cols = len(vals)/num_rows

        data = []
        pt_num = 0
        my_min = None
        my_max = None

        print "dim", num_rows, num_cols
        print "sav img to pgm"
        fimg = open("dump.pgm", "wb")
        fimg.write("P5\n#toto\n%d %d\n255\n"%(num_rows, num_cols))
        for i in xrange(num_cols):
            data.append([])
            #data[-1].append(0.0)
            
            for j in xrange(num_rows):
                if vals[pt_num]>0x10:
                    p = 0
                else:
                    p=vals[pt_num] * 0x20
                if (p>0xFF):
                    p = 0xFF
                   
                fimg.write(chr(p))
                if my_min == None or my_min>p:
                    my_min = p
                if  p!=255 and (my_max == None or my_max<p):
                    my_max = p
                if p >= 205:
                    p = 0
                p/=1.


                
                data[-1].append(p)
                pt_num+=1
            #data[-1].append(1.)
        fimg.close()
        print my_min, my_max
        #print data
        data = numpy.array(data)

        if do_img:
            ax = pylab.subplot(111)
            ax.imshow(data)
        
        
        #pylab.subplot(111, polar = True)
        self.r = vals
        #theta_max = 4.8*2.3*pi
        self.theta_max =valinit*pylab.pi
        self.theta = pylab.arange(0.0, self.theta_max, self.theta_max/len(self.r))

        """
        tmp = []
        for x in data:
            tmp+=list(x)
        self.myplot, = pylab.plot(tmp)
        

        """
        if not do_img :
            tmpx = []
            tmpy = []
            for x in data:
                tmpy+=list(x)
                tmpx+=range(len(x))
            self.myplot, = pylab.plot(tmpx, tmpy)
        

        #slide bar
        #axfreq = pylab.axes([0.25, 0.1, 0.65, 0.03])
        #self.sfreq = pylab.Slider(axfreq, "Freq", 1, 20, valinit = valinit)
        #self.sfreq.on_changed(self.update_graph)

        pylab.show()


    def do_scan_params(self, args):
        t = [x for x in shlex.shlex(args)]
        
        if len(t)!=2:
            return
        t = [int(x) for x in t]
        self.ser.write("scan_params %d %d\n"%tuple(t))

    def do_graph(self, args):
        t = [x for x in shlex.shlex(args)]
        
        t.reverse()
        do_img = False

        #send speed,debug=off
        #self.ser.write("scan_params 500 0\n")
        #send algo 1 wrkazone 1 cx 15 cy 15
        self.ser.write("scan_img 1 1 15 15\n")
        
        print t
        while len(t):
            x = t.pop()
            if x == 'img':
                do_img = True
                
            
        scan_start = time.time()
        print "sampling..."

        self.ser.write("scan_do\n")

        flog = open('log.txt', 'w')

        while True:
            l = self.ser.readline()
            flog.write(l)

            if "dump end" in l:
                break
        flog.close()

        #time.sleep(2)
        #self.ser.write("pwm 1A 0\n")
        #l = self.ser.readline()
        #l = self.ser.readline()


        print "dumping..."
        self.ser.write("sample dump 0 0 400 0\n")



        while True:
            l = self.ser.readline()

            if "start dumping" in l:
                tokens = [x for x in shlex.shlex(l)]
                num_rows = int(tokens[-1])
                print "num row: ", num_rows
                break
            print l.strip()
        scan_stop = time.time()
        print "total time:", scan_stop-scan_start


        vals = []
        while True:
            l = self.ser.readline()

            if l[0] in ['s', 'c', 'a']:
                continue
            if l[0] in ['e']:
                break
            tokens = [x for x in shlex.shlex(l)]
            v = int(tokens[0])
            #v = min(v, 150)
            vals.append(v)


        #vals.reverse()
        print "total vals:", len(vals)
        valinit = 5

        #num_rows = int(600/valinit)
        #num_cols = int(valinit)
        num_rows_orig = num_rows
        num_rows *= 1
        num_cols = len(vals)/num_rows

        data = []
        pt_num = 0
        my_min = None
        my_max = None

        print "dim", num_rows, num_cols
        print "sav img to pgm"
        fimg = open("dump.pgm", "wb")
        fimg.write("P5\n#toto\n%d %d\n255\n"%(num_rows, num_cols))
        for i in xrange(num_cols):
            data.append([])
            #data[-1].append(0.0)
            
            for j in xrange(num_rows):
                if vals[pt_num]>0x10:
                    p = 0
                else:
                    p=vals[pt_num] * 0x20
                if (p>0xFF):
                    p = 0xFF
                   
                fimg.write(chr(p))
                if my_min == None or my_min>p:
                    my_min = p
                if  p!=255 and (my_max == None or my_max<p):
                    my_max = p
                if p >= 205:
                    p = 0
                p/=1.


                
                data[-1].append(p)
                pt_num+=1
            #data[-1].append(1.)
        fimg.close()
        print my_min, my_max
        #print data
        data = numpy.array(data)

        if do_img:
            ax = pylab.subplot(111)
            ax.imshow(data)
        
        
        #pylab.subplot(111, polar = True)
        self.r = vals
        #theta_max = 4.8*2.3*pi
        self.theta_max =valinit*pylab.pi
        self.theta = pylab.arange(0.0, self.theta_max, self.theta_max/len(self.r))

        """
        tmp = []
        for x in data:
            tmp+=list(x)
        self.myplot, = pylab.plot(tmp)
        

        """
        if not do_img :
            tmpx = []
            tmpy = []
            for x in data:
                tmpy+=list(x)
                tmpx+=range(len(x))
            self.myplot, = pylab.plot(tmpx, tmpy)
        

        #slide bar
        #axfreq = pylab.axes([0.25, 0.1, 0.65, 0.03])
        #self.sfreq = pylab.Slider(axfreq, "Freq", 1, 20, valinit = valinit)
        #self.sfreq.on_changed(self.update_graph)

        pylab.show()

        
    def bootloader(self, filename, boardnum):
        self.ser.write("\n")
        time.sleep(0.4)
        self.ser.write("bootloader\n")
        time.sleep(0.4)
        self.ser.write("\n")

        print "start programming"
        self.ser.flushInput()
        f = open(filename)
        buf = f.read()
        addr = 0
        while addr < len(buf):
            #time.sleep(0.1)
            if check_crc(self.ser, buf, addr, SPM_PAGE_SIZE) == 0:
                sys.stdout.write("*")
                sys.stdout.flush()
            elif prog_page(self.ser, addr, 
                         buf[addr:addr+SPM_PAGE_SIZE]) != 0:
                return
            addr += SPM_PAGE_SIZE
        if check_crc(self.ser, buf, 0, len(buf)):
            print "crc failed"
            return
        print "Done."
        self.ser.write("x")
        self.do_raw("")
        
    def do_bootloader(self, args):
        self.bootloader(args, 0)

    def do_mainboard(self, args):
        filename = os.path.join(MICROB_PATH, "../mainboard/main.bin")
        self.bootloader(filename, 1)

    def do_mechboard(self, args):
        filename = os.path.join(MICROB_PATH, "../mechboard/main.bin")
        self.bootloader(filename, 2)

    def do_sensorboard(self, args):
        filename = os.path.join(MICROB_PATH, "../sensorboard/main.bin")
        self.bootloader(filename, 3)

    def do_toto(self, args):
        for i in range(10):
            time.sleep(1)
            self.ser.write("pwm s3(3C) 200\n")
            time.sleep(1)
            self.ser.write("pwm s3(3C) 250\n")

if __name__ == "__main__":
    try:
        import readline,atexit
    except ImportError:
        pass
    else:
        histfile = os.path.join(os.environ["HOME"], ".microb_history")
        atexit.register(readline.write_history_file, histfile)
        try:
            readline.read_history_file(histfile)
        except IOError:
            pass

    device = "/dev/ttyS0"
    if len(sys.argv) > 1:
        device = sys.argv[1]
    interp = Interp(device)
    while 1:
        try:
            interp.cmdloop()
        except KeyboardInterrupt:
            print
        except Exception,e:
            l = str(e).strip()
            if l:
                log.exception("%s" % l.splitlines()[-1])
            continue
        break
