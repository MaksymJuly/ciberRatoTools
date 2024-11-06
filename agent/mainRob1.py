
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from collections import deque

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

CELLROWS=7
CELLCOLS=14

## MY VARIABLES ##
motors_speed = 0.15
num_mem_sell = 6

front_sens_m = deque(maxlen=num_mem_sell) # Initialize deques
left_sens_m = deque(maxlen=num_mem_sell)
right_sens_m = deque(maxlen=num_mem_sell)
back_sens_m = deque(maxlen=num_mem_sell)


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
        update_memo(self)

        if emergency_stop(self):
            for i in range(1000):
                if self.measures.irSensor[3] > 1:
                    break
                self.driveMotors(-motors_speed,-motors_speed)
        else:
            # wander right and left
            side_sens_diff(self)

## my func ##
def update_memo(self):
    # Append the new sensor readings
    front_sens_m.append(self.measures.irSensor[0])
    left_sens_m.append(self.measures.irSensor[1])
    right_sens_m.append(self.measures.irSensor[2])
    back_sens_m.append(self.measures.irSensor[3])

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
    print("\n")
    print("front = ", list(front_sens_m), " back = ", list(back_sens_m), "\n")
    print("left = ", list(left_sens_m), "   right = ", list(right_sens_m), "\n")
    return

def average(lst):
        return sum(lst) / len(lst) if lst else 0

## end my func ##

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
