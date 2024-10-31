
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

#### MY VARIABLES ####
calib_gps = [float(0), float(0), False]

rows = 26   # Number of rows
cols = 54   # Number of columns
MATRIX = [[ '▣' for _ in range(cols)] for _ in range(rows)]
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
        gps = {'x': self.measures.x - calib_gps[0], 'y': self.measures.y - calib_gps[1]}
        return gps

    def go_to(self, x, y):
        # Get current position from GPS
        gps = self.get_gps()
        current_x, current_y = gps['x'], gps['y']

        # Calculate the angle to the target
        angle_to_target = atan2(y - current_y, x - current_x)
        
        # Calculate distance to target
        distance_to_target = sqrt((x - current_x) ** 2 + (y - current_y) ** 2)

        # Calculate the angle difference to the target
        angle_difference = angle_to_target - radians(self.measures.angle)  # Convert robot's angle to radians
        angle_difference = (angle_difference + pi) % (2 * pi) - pi  # Normalize to range [-π, π]

        # Adjust motor speeds based on angle difference and distance
        if abs(angle_difference) > 0.1:  # If the angle difference is significant
            # Rotate towards the target
            if angle_difference > 0:
                self.driveMotors(0, 0.15)  # Turn left
            else:
                self.driveMotors(0, -0.15)  # Turn right
        elif distance_to_target > 1:  # Move forward if close enough to the target
            self.driveMotors(0.15, 0.15)  # Move forward
        else:
            self.driveMotors(0, 0)  # Stop

        # Optionally, print debug information
        print(f"Current Position: ({current_x:.2f}, {current_y:.2f}), "
            f"Target Position: ({x:.2f}, {y:.2f}), "
            f"Distance to Target: {distance_to_target:.2f}, "
            f"Angle to Target: {degrees(angle_to_target):.2f}°, "
            f"Angle Difference: {degrees(angle_difference):.2f}°")

    def update_wall_matrix(self):
        # Get current GPS coordinates and convert to grid position
        gps = self.get_gps()
        current_x, current_y = gps['x'], gps['y']
        row = int(current_y // 1)  # Assuming each cell is of size 1x1
        col = int(current_x // 1)

        # Check for walls using ultrasonic sensors
        if self.measures.irSensor[0] < 1.6:  # Front sensor (wall close)
            MATRIX[row][col + 1] = 'W'  # Mark wall on the right side
        if self.measures.irSensor[1] < 1.6:  # Left sensor
            MATRIX[row + 1][col] = 'W'  # Mark wall at the top (left of robot)
        if self.measures.irSensor[2] < 1.6:  # Right sensor
            MATRIX[row - 1][col] = 'W'  # Mark wall at the bottom (right of robot)

        # Mark the current cell as visited
        MATRIX[row][col] = 'V'

    def plan_next_move(self):
        # Implementing A* algorithm to find the next move
        gps = self.get_gps()
        start = (int(gps['y']), int(gps['x']))  # Starting position in (row, column) format
        goal = self.find_nearest_unvisited_cell()  # Define the goal as the nearest unvisited cell

        if goal is None:
            return None, None  # No valid move found

        # A* algorithm setup
        open_set = deque([start])  # Cells to explore
        came_from = {}  # Track path
        g_score = {start: 0}  # Cost from start to current cell
        f_score = {start: self.heuristic(start, goal)}  # Estimated cost from start to goal

        while open_set:
            current = min(open_set, key=lambda cell: f_score.get(cell, float('inf')))

            if current == goal:
                return self.reconstruct_path(came_from, current)  # Return the path from start to goal

            open_set.remove(current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1  # Cost from start to neighbor
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # This path to neighbor is better than any previous one
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return None, None  # No path found

    def get_neighbors(self, cell):
        row, col = cell
        neighbors = []

        # Check four possible directions (up, down, left, right)
        if row > 0 and MATRIX[row - 1][col] != 'W':  # Up
            neighbors.append((row - 1, col))
        if row < rows - 1 and MATRIX[row + 1][col] != 'W':  # Down
            neighbors.append((row + 1, col))
        if col > 0 and MATRIX[row][col - 1] != 'W':  # Left
            neighbors.append((row, col - 1))
        if col < cols - 1 and MATRIX[row][col + 1] != 'W':  # Right
            neighbors.append((row, col + 1))

        return neighbors

    def heuristic(self, a, b):
        # Use Manhattan distance as heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        # Reconstruct the path from start to goal
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()  # Return path from start to goal
        if path:
            target_cell = path[0]  # Get the next target from the path
            return target_cell[1] + 0.5, target_cell[0] + 0.5  # Return as (x, y) coordinates
        return None, None

    def find_nearest_unvisited_cell(self):
        # Get the robot's current GPS position
        gps = self.get_gps()
        current_x, current_y = int(gps['x']), int(gps['y'])

        nearest_cell = None
        min_distance = float('inf')  # Initialize with a very large distance

        # Iterate over the MATRIX to find unvisited cells
        for row in range(rows):
            for col in range(cols):
                if MATRIX[row][col] == '▣':  # Assuming '▣' represents unvisited cells
                    # Calculate the distance to this unvisited cell
                    distance = sqrt((row - current_y) ** 2 + (col - current_x) ** 2)

                    # If this cell is closer than the previous closest one, update
                    if distance < min_distance:
                        min_distance = distance
                        nearest_cell = (row, col)

        return nearest_cell if nearest_cell else None  # Return the nearest cell or None if no unvisited cells are found

    def myrobot(self):
        # Initialize target variables
        if not hasattr(self, 'target_x') or not hasattr(self, 'target_y'):
            self.target_x, self.target_y = None, None

        # Check if we need to plan a new move
        if self.target_x is None and self.target_y is None:
            self.target_x, self.target_y = self.plan_next_move()  # Use A* to find next target

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