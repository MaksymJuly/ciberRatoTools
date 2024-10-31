
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
calib_gps = [float(0), float(0), False]

# Map matrix
rows, cols = 27, 55   # Number of rows, cols
center_row = rows // 2
center_col = cols // 2
MATRIX = [[ '▣' for _ in range(cols)] for _ in range(rows)] # Create a matrix 

# Path to save the file
file_path = '/home/ma/Aveiro/RMI/ciberRatoTools/agent/' + 'mapping.out'
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
    
    def get_gps(self):
        # Check if GPS calibration is done
        if not calib_gps[2]:
            # Ensure we have valid GPS readings before calibration
            if self.measures.x is None or self.measures.y is None:
                print("Error: GPS readings are None, cannot calibrate.")
                return {'x': None, 'y': None}

            calib_gps[0] = self.measures.x  # Store initial x calibration
            calib_gps[1] = self.measures.y  # Store initial y calibration
            calib_gps[2] = True  # Calibration done

        # Ensure the GPS readings are valid after calibration
        if self.measures.x is None or self.measures.y is None:
            print("Error: GPS readings are None after calibration.")
            return {'x': None, 'y': None}

        # Return calibrated GPS coordinates
        gps = {'x': self.measures.x - calib_gps[0] + center_col, 'y': self.measures.y - calib_gps[1] + center_row}
        return gps

    def mapping_out(self, file_path):
        file_content, i = '', 1
        MATRIX[13][27] = '○'    # Start

        for row in MATRIX:
            file_content = file_content + f'{i}\t'
            i += 1
            for col in row:
                file_content = file_content + col
            file_content = file_content + f' {i}\n'

        # Writing content to the file
        with open(file_path, "w") as file:
            file.write(file_content)
        print('\n - MapOut')

    def update_wall_matrix(self):
        pass

    def plan_next_move(self):
        gps = self.get_gps()
        start = (int(gps['y']), int(gps['x']))  # Starting position in (row, column) format
        goal = self.find_nearest_unvisited_cell(start)  # Define the goal as the nearest unvisited cell

        if goal is None:
            return None, None  # No valid move found

        return goal[0], goal[1]

    def find_nearest_unvisited_cell(self, start):
        # Get the robot's current GPS position
        current_x, current_y = start[1], start[0]

        nearest_cell = None

        up_sell = (current_x, current_y-1)
        dw_sell = (current_x, current_y+1)
        le_sell = (current_x-1, current_y)
        ri_sell = (current_x+1, current_y)

        # Iterate over the MATRIX to find unvisited cells
        for row in range(rows):
            for col in range(cols):
                if MATRIX[row][col] == '▣':  # '▣' represents unvisited cells
                    if (row, col) == ri_sell:
                        nearest_cell = ri_sell
                        return nearest_cell
                    elif (row, col) == le_sell:
                        nearest_cell = le_sell
                        return nearest_cell
                    else:
                        if (row, col) == dw_sell:
                            nearest_cell = dw_sell
                            return nearest_cell
                        elif (row, col) == up_sell:
                            nearest_cell = up_sell
                            return nearest_cell
                
        # non_visited = 0
        # if non_visited <= 1538:
        #     print('- mapping_out(file_path)')
        #     self.mapping_out(file_path)
        #     quit()

        return nearest_cell if nearest_cell else None  # Return the nearest cell or None if no unvisited cells are found

    def go_to(self, x, y):
        # Get current position from GPS
        gps = self.get_gps()
        current_x, current_y = gps['x'], gps['y']

        # Calculate the angle to the target
        angle_to_target = atan2(y - current_y, x - current_x)
        
        # Calculate distance to target
        distance_to_target = sqrt((x - current_x) ** 2 + (y - current_y) ** 2)

        # Calculate the angle difference to the target
        angle_difference = angle_to_target - radians(self.measures.compass)  # Convert robot's angle to radians
        angle_difference = (angle_difference + pi) % (2 * pi) - pi  # Normalize to range [-π, π]

        # Adjust motor speeds based on angle difference and distance
        if abs(angle_difference) > 0.1:  # If the angle difference is significant
            # Rotate towards the target
            if angle_difference > 0:
                self.driveMotors(0, 0.03)  # Turn left
            else:
                self.driveMotors(0, -0.03)  # Turn right
        elif distance_to_target > 1:  # Move forward if close enough to the target
            self.driveMotors(0.03, 0.03)  # Move forward
        else:
            self.driveMotors(0, 0)  # Stop

        # Optionally, print debug information
        print(  f"Current Position: ({current_x:.2f}, {current_y:.2f}),\n"
                f"Target Position: ({x:.2f}, {y:.2f}),\n"
                f"Distance to Target: {distance_to_target:.2f},\n"
                f"Angle to Target: {degrees(angle_to_target):.2f}°,\n"
                f"Angle Difference: {degrees(angle_difference):.2f}°\n\n")

    def myrobot(self):
        # Initialize target variables
        if not hasattr(self, 'target_x') or not hasattr(self, 'target_y'):
            self.target_x, self.target_y = None, None

        # Check if we need to plan a new move
        if self.target_x is None and self.target_y is None:
            self.target_x, self.target_y = self.plan_next_move()  # Find next target

        # Move towards the current target
        self.go_to(self.target_x, self.target_y)

        # Get current position from GPS
        gps = self.get_gps()
        current_x, current_y = gps['x'], gps['y']

        # Check if we have reached the target
        distance_to_target = sqrt((self.target_x - current_x) ** 2 + (self.target_y - current_y) ** 2)

        if distance_to_target < 0.1:  # If close enough to the target
            # Mark the cell as visited in the matrix
            self.update_wall_matrix()  # Update walls based on sensor readings
            self.target_x, self.target_y = None, None  # Clear the target for the next move


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
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()