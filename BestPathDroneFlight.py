from djitellopy import tello
from matplotlib import pyplot as plt
from matplotlib import patches as patches
import cv2
import numpy as np
import pandas as pd
from heapq import heappush, heappop
from tello_sim import Simulator


global visited
global redzone #contains all points in the red zones
global earth

earth = np.zeros((100,100,3), dtype='uint8')
visited = set()

#zones that are to be avoided can be added here to be mapped onto the grid by which the drone will use to fly
def red_zones():
    #change path based on location of csv file
    zones = pd.read_csv(r"C:/Users/abdul/OneDrive/Documents/REU Summer 2022/Drone Path Mapping/Drone Path Mapping/Best Path/Red Zones.csv")
    for index, row in zones.iterrows():
       map_zone(row['x1'],row['y1'],row['x2'],row['y2'])

#helper function to red_zones() 
def map_zone(x1,y1,x2,y2):
    #p1 = (x1,y1), p2 = (x1,y2), p3 = (x2,y2), p4 = (x2,y1)
    y = y1
    for t in range(x1,x2+1):
        visited.add((t,y))
        for t2 in range(y1,y2+1):
            visited.add((t,t2))
        y = y + 1

#plots red zones onto the graph with the desired path
def plot_red_zones():
    zones = pd.read_csv(r"C:/Users/abdul/OneDrive/Documents/REU Summer 2022/Drone Path Mapping/Drone Path Mapping/Best Path/Red Zones.csv")

    #prints out all the red zones into a graph
    for index, row in zones.iterrows():
        y = row['y1']
        for t in range(row['x1'],row['x2']+1):
            plt.scatter(t,y)
            for t2 in range(row['y1'],row['y2']+1):
                plt.scatter(t,t2)
            y = y + 1
    plt.show()

def a_star_graph_search(start, goal_function, successor_function, heuristic):
    came_from= dict() #where the robot came from
    distance = {start: 0} #distance from start
    frontier = PriorityQueue() #priorityqueue where everything is stored
    frontier.add(start)
    while frontier:
        node = frontier.pop()
        if node in visited:
            continue
        if goal_function==node:
            return reconstruct_path(came_from, start, node)
        visited.add(node)
        for successor in successor_function(node):
            frontier.add(successor, priority = distance[node] + 1 + heuristic(successor))
            if (successor not in distance or distance[node] + 1 < distance[successor]):
                distance[successor] = distance[node] +1
                came_from[successor] = node
    return None

def reconstruct_path(came_from, start, end):
    """
    >>> came_from = {'b': 'a', 'c': 'a', 'd': 'c', 'e': 'd', 'f': 'd'}
    >>> reconstruct_path(came_from, 'a', 'e')
    ['a', 'c', 'd', 'e']
    """
    reverse_path = [end]
    while end != start:
        end = came_from[end]
        reverse_path.append(end)
    return list(reversed(reverse_path))

#goal function sees whether we have reached the final goal of the path
def get_goal_function(dest):
    """
    >>> f = get_goal_function([[0, 0], [0, 0]])
    >>> f((0, 0))
    False
    >>> f((0, 1))
    False
    >>> f((1, 1))
    True
    """
    #M = len(grid)
    #N = len(grid[0])
    #def is_bottom_right(cell):
        #return cell == (M-1, N-1)
    #return is_bottom_right
    return dest

#sucessor function
#function is to find the cells adjacent to the current cell
def get_successor_function(grid):
    """
    >>> f = get_successor_function([[0, 0, 0], [0, 1, 0], [1, 0, 0]])
    >>> sorted(f((1, 2)))
    [(0, 1), (0, 2), (2, 1), (2, 2)]
    >>> sorted(f((2, 1)))
    [(1, 0), (1, 2), (2, 2)]
    """
    def get_clear_adjacent_cells(cell):
        i,j = cell
        return (
            (i + a, j + b)
            for a in (-1, 0, 1)
            for b in (-1, 0, 1)
            if a != 0 or b != 0
            if 0 <= i + a < len(grid)
            if 0 <= j + b < len(grid[0])
            if grid[i+a][j + b] == 0
        )
    return get_clear_adjacent_cells

#heuristic
#the goal of the heuristic is to find the distance to the goal in a clear grid of the same size
def get_heuristic(grid, dest):
    """
    >>> f = get_heuristic([[0, 0], [0, 0]])
    >>> f((0, 0))
    1
    >>> f((0, 1))
    1
    >>> f((1, 1))
    0
    """
    #M, N = len(grid)
    (a, b) = goal_cell = (dest[0],dest[1])
    def get_clear_path_distance_from_goal(cell):
        (i, j) = cell
        return max(abs(a - i), abs(b-j))
    return get_clear_path_distance_from_goal

#priority queue
class PriorityQueue:

    def __init__(self, iterable=[]):
        self.heap = []
        for value in iterable:
            heappush(self.heap, (0, value))

    def add(self, value, priority=0):
        heappush(self.heap, (priority, value))

    def pop(self):
        priority, value = heappop(self.heap)
        return value

    def __len__(self):
        return len(self.heap)

#best path plotting
def plotallpoints(points):
    for point in points:
        print(point)
        plt.scatter(point[0],point[1])
        path=plt.imshow
    plt.show()

def addsone(grid):
    i=0
    d=0
    return grid

#flies the drone based off the path created in the A* pathing algorithm
def drone_flight(shortest_path, origin):
    prev_point = origin
    bearing = 0
    desiredangle = 0
    
    #handles connection and takeoff of Tello drone
    drone = Simulator()
    #drone.connect() #when using djitello library
    drone.takeoff()

    for point in shortest_path:
        #initializing if on first point, drone doesn't move
        if prev_point == point:
            continue

        #up and down movements
        if point[0] == prev_point[0]:
            if point[1] > prev_point[1]:
                desiredangle = -bearing
            elif point[1] < prev_point[1]:
                down = 180 - bearing
                desiredangle = down

        #right and left movements
        if point[1] == prev_point[1]:
            if point[0] > prev_point[0]:
                desiredangle = 270 - bearing
            elif point[0] < prev_point[0]:
                desiredangle = 90 - bearing

        #right diagonal movements
        if point[0] > prev_point[0]:
            if point[1] > prev_point[1]:
                desiredangle = 315 - bearing
            elif point[1] < prev_point[1]:
                if bearing != 225:
                    desiredangle = 225 - bearing
                else:
                    desiredangle = 225
        #left diagonal movements
        elif point[0] < prev_point[0]:
            if point[1] > prev_point[1]:
                desiredangle = 45 - bearing
            elif point[1] < prev_point[1]:
                desiredangle = 135 - bearing
        
        #if the new drone movement has same bearing as previous, move forward
        if bearing == desiredangle:
            drone.forward(10)
            desiredangle = 0
            prev_point = point
            continue

        #general drone movement
        drone.ccw(desiredangle)
        bearing += desiredangle 
        drone.forward(10)

        #resets bearing to within 0-360 range
        if bearing >= 360:
            bearing -= 360

        desiredangle = 0
        prev_point = point
    
    drone.land()

def main():
    #coordinates for origin and destination
    origin=(30,35)
    dest=(50, 70)

    w, h = 100, 100
    grid = [[0 for x in range(w)] for y in range(h)] 
    grid=addsone(grid)

    #maps all red zones
    red_zones()
    #plot_red_zones()

    shortest_path=a_star_graph_search(start=origin, goal_function= get_goal_function(dest), successor_function=get_successor_function(grid), heuristic= get_heuristic(grid, dest))
    if shortest_path is None or grid[0][0] == 1:
        print("A path could not be found.")
        return -1
    else:
        #regular plot of shortest path
        plotallpoints(shortest_path)

    drone_flight(shortest_path, origin)

if __name__ == "__main__":
    main()