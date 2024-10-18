
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from collections import deque
import time


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

CELLROWS=7
CELLCOLS=14

#### MY VARIABLES ####

m_speed = 0.15
num_mem_sell = 6

front_sens_m = deque(maxlen=num_mem_sell)
left_sens_m = deque(maxlen=num_mem_sell)
right_sens_m = deque(maxlen=num_mem_sell)
back_sens_m = deque(maxlen=num_mem_sell)

compas = deque(maxlen=3)
gps = deque(maxlen=3)
calib_gps = [float(0), float(0), False]

timer1 = [int(0), False]
time_spd = int(0)

move = {"x":0, "y":0, "state":'Stop'}
point_f = False

rows = 27   # Number of rows
cols = 55   # Number of columns

# Create a matrix of 3D vectors [0, 0, None]
MATRIX = [[ None for _ in range(cols)] for _ in range(rows)]
MATRIX[13][27] = 'I'    # Start
#### END MY VARIABELS ####



class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.myrobot()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.myrobot() 
    
    def myrobot(self):
        time_spd = time_setings()
        gps_now = gps_setings(self)

        if self.measures.irSensor[0] < 1 and move['state'] == 'Stop':
            move['x'] = move['x'] + 1
            move['state'] = 'Moving'
        
        if int(gps_now[0]) != 0 or int(gps_now[1]) !=0:
            
            if self.measures.irSensor[0] > 2.5:
                MATRIX[13 + int(gps_now[1])][28 + int(gps_now[0])] = '|'    # Wall
            else:
                MATRIX[13 + int(gps_now[1])][27 + int(gps_now[0])] = 'X'    # Line

        diff_in_x = move['x'] - gps_now[0]
        diff_in_y = move['y'] - gps_now[1]

        m_speed = set_m_speed(diff_in_x, diff_in_y)

        if diff_in_x > 0:
            left_m_s = m_speed
            right_m_s = m_speed
            mess = '\tFORWARD'
        elif diff_in_x < 0:
            left_m_s = -m_speed
            right_m_s = -m_speed
            mess = '\tBACKWARD'
        else:
            left_m_s = 0
            right_m_s = 0
            mess = '\tSTOP'
            move['state'] = 'Stop'
        
        print(f"- time: {time_spd}s    {left_m_s} {right_m_s}  [x, y]={gps_now} front={self.measures.irSensor[0]}")
        print(mess, move)

        mapping_out()

        self.driveMotors(left_m_s, right_m_s)



## my func ##
def mapping_out():
    file_content = ''
    i = 1

    for row in MATRIX:
        file_content = file_content + f'{i}\t'
        i += 1
        for col in row:
            if col == None:
                file_content = file_content + '_'
            else:
                file_content = file_content + col
        file_content = file_content + '\n'

    # Path to save the file
    file_path = '/home/ma/Aveiro/RMI/ciberRatoTools/agent/' + 'mapping.out'

    # Writing content to the file
    with open(file_path, "w") as file:
        file.write(file_content)

def set_m_speed(diff_in_x, diff_in_y):
    # m_speed (0 -- min; 0.15 -- max)
    m_speed = 0

    diff_in_x = abs(diff_in_x)
    diff_in_y = abs(diff_in_y)

    distans = diff_in_x if diff_in_x > diff_in_y else diff_in_y

    if distans <= 0.5:
        m_speed = 0.05
    else:
        m_speed = 0.15

    return m_speed

def gps_setings(self):
    if not calib_gps[2]:
        calib_gps[0] = self.measures.x
        calib_gps[1] = self.measures.y
        calib_gps[2] = True
        
    gps.append([self.measures.x, self.measures.y])
    gps_now = gps.popleft()
    gps_now[0] = gps_now[0] - calib_gps[0]
    gps_now[1] = gps_now[1] - calib_gps[1]

    return gps_now

def time_setings():
    if timer1[1]:
        timer1[0] = 0
        timer1[1] = False
    if timer1[0] == 0:
        timer1[0] = int(time.time())

    time_spd = int(time.time()) - timer1[0]

    return time_spd

def update_memo(self):
    # Append the new sensor readings
    front_sens_m.append(self.measures.irSensor[0])
    left_sens_m.append(self.measures.irSensor[1])
    right_sens_m.append(self.measures.irSensor[2])
    back_sens_m.append(self.measures.irSensor[3])

    compas.append(self.measures.compass)
    gps.append([self.measures.x, self.measures.y])
    

    print_sensdist()

def side_sens_diff(self):
    
    if average(list(front_sens_m)) > 1.2:   ## avr of front sensor distance measures
        # corners
        dist = 0.2
        speed_mod = 0.4
    else:
        # straight 
        dist = 1
        speed_mod = 0.3

    side_sens_diff = average(list(right_sens_m)) - average(list(left_sens_m))
    if side_sens_diff > +dist:
        # move left
        self.driveMotors(-motors_speed * speed_mod, motors_speed * speed_mod)
    elif side_sens_diff < -dist:
        # move right
        self.driveMotors(motors_speed * speed_mod ,-motors_speed * speed_mod)
    else:
        # move straight
        self.driveMotors(+ motors_speed,+ motors_speed)

def emergency_stop(self):
    if     self.measures.irSensor[0]   > 2 \
        or self.measures.irSensor[1]   > 4 \
        or self.measures.irSensor[2]   > 4 :
        return True
    else:
        return False

def print_sensdist():
    # print("\n")
    # print("front = ", list(front_sens_m), " back = ", list(back_sens_m), "\n")
    # print("left = ", list(left_sens_m), "   right = ", list(right_sens_m), "\n")
    print('gps: ', list(gps))
    return

def average(lst):
        return sum(lst) / len(lst) if lst else 0



class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
