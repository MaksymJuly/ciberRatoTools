
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

from collections import deque

CELLROWS=7
CELLCOLS=14

motors_speed = 0.15

# Initialize deques with a maximum length of 4
front_sens_m = deque(maxlen=4)
left_sens_m = deque(maxlen=4)
right_sens_m = deque(maxlen=4)
back_sens_m = deque(maxlen=4)

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
    # sens_id = {'center':0, 'left':1, 'right':2, 'back':3}
    
    update_memo(self)

    if emergency_stop(self):
        # Stop
        self.driveMotors(0,0)
        for i in range(1500):
            if self.measures.irSensor[3] > 2.5:
                print("cia fignia")
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
    a = average(list(right_sens_m)) - average(list(left_sens_m))

    
    if average(list(front_sens_m)) > 1.2:
        # corners
        dist = 0.2
        speed_mod = 0.6
    else:
        # straight 
        dist = 1
        speed_mod = 0.3

    if a > +dist:
        # move left
        self.driveMotors(-motors_speed * speed_mod, motors_speed * speed_mod)
    elif a < -dist:
        # move right
        self.driveMotors(motors_speed * speed_mod ,-motors_speed * speed_mod)
    else:
        # move straight
        self.driveMotors(+ motors_speed,+ motors_speed)

# def side_sens_diff(self):
#     # Calculate difference between right and left sensors
#     a = average(list(right_sens_m)) - average(list(left_sens_m))

#     # Set a base speed and dynamic factor for speed adjustment
#     base_speed = motors_speed
#     dynamic_speed_factor = 1.0

#     # Adjust speed based on front sensor distance
#     front_distance = average(list(front_sens_m))
    
#     if front_distance < 0.8:
#         # Increase speed if there's a lot of space ahead
#         dynamic_speed_factor = 3
#     elif front_distance > 1.3:
#         # Moderate speed for moderate space
#         dynamic_speed_factor = 1.2
#     elif front_distance > 3.0:
#         # Slow down as the front gets closer
#         dynamic_speed_factor = 0.8
#     else:
#         # Near an obstacle, slow down further
#         dynamic_speed_factor = 0.5

#     # Further reduce speed in corners, but allow more aggressive turns when space permits
#     if a > +0.5:
#         # If the robot is turning left
#         self.driveMotors(-base_speed * dynamic_speed_factor, base_speed * dynamic_speed_factor)
#     elif a < -0.5:
#         # If the robot is turning right
#         self.driveMotors(base_speed * dynamic_speed_factor, -base_speed * dynamic_speed_factor * 0.8)
#     else:
#         # Moving straight
#         self.driveMotors(base_speed * dynamic_speed_factor, base_speed * dynamic_speed_factor)

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

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

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
