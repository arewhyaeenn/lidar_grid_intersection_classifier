import numpy as np
import math
from math import sqrt
from heapq import *

class AI:
    directions = {'NORTH': 1.0, 'SOUTH': -1.0, "EAST": 1.0, "WEST": -1.0}
    sub_goal_state = (-1, -1)
    goal_state = (-1, -1)
    current_goal_state = (-1,-1)
    current_direction = "EAST"
    current_state = (-1, -1)
    visited = []
    cost = None
    path = None
    turning = False
    path = None





    # {Key: Midpoint, Neighbors}
    graph = {
        (0, 0): [(95, 95), [(0, 1), (1, 0)]],
        (1, 0): [(95, 185), [(0, 0), (2, 0)]],
        (2, 0): [(95, 275), [(1, 0), (3, 0), (2, 1)]],
        (3, 0): [(95, 365), [(2, 0), (4, 0)]],
        (4, 0): [(95, 455), [(3, 0), (5, 0), (4, 1)]],
        (5, 0): [(95, 545), [(4, 0), (6, 0)]],
        (6, 0): [(95, 635), [(5, 0), (6, 1)]],
        (0, 1): [(185, 95), [(0, 0), (0, 2)]],
        (1, 1): [(185, 185), [(2, 1), (1, 2)]],
        (2, 1): [(185, 275), [(1, 1), (2, 0)]],
        (3, 1): [(185, 365), [(3, 2)]],
        (4, 1): [(185, 455), [(4, 0), (4, 2)]],
        (5, 1): [(185, 545), [(5, 2)]],
        (6, 1): [(185, 635), [(6, 0), (6, 2)]],
        (0, 2): [(275, 95), [(0, 1), (0, 3)]],
        (1, 2): [(275, 185), [(1, 1)]],
        (2, 2): [(275, 275), [(3, 2), (2, 3)]],
        (3, 2): [(275, 365), [(3, 1), (2, 2)]],
        (4, 2): [(275, 455), [(4, 1), (5, 2)]],
        (5, 2): [(275, 545), [(5, 1), (4, 2)]],
        (6, 2): [(275, 635), [(6, 1), (6, 3)]],
        (0, 3): [(365, 95), [(0, 2), (1, 3)]],
        (1, 3): [(365, 185), [(0, 3), (2, 3)]],
        (2, 3): [(365, 275), [(2, 2), (1, 3)]],
        (3, 3): [(365, 365), [(3, 4)]],
        (4, 3): [(365, 455), [(4, 4), (5, 3)]],
        (5, 3): [(365, 545), [(4, 3), (6, 3)]],
        (6, 3): [(365, 635), [(6, 2), (5, 3), (6, 4)]],
        (0, 4): [(455, 95), [(1, 4), (0, 5)]],
        (1, 4): [(455, 185), [(0, 4), (2, 4)]],
        (2, 4): [(455, 275), [(1, 4), (2, 4), (3, 4)]],
        (3, 4): [(455, 365), [(2, 4), (3, 3)]],
        (4, 4): [(455, 455), [(4, 3), (4, 5)]],
        (5, 4): [(455, 545), [(5, 5)]],
        (6, 4): [(455, 635), [(6, 3), (6, 5)]],
        (0, 5): [(545, 95), [(0, 4), (0, 6)]],
        (1, 5): [(545, 185), [(1, 6)]],
        (2, 5): [(545, 275), [(2, 4), (2, 6)]],
        (3, 5): [(545, 365), [(4, 5)]],
        (4, 5): [(545, 455), [(3, 5), (4, 4)]],
        (5, 5): [(545, 545), [(5, 4), (5, 6)]],
        (6, 5): [(545, 635), [(6, 4), (6, 6)]],
        (0, 6): [(635, 95), [(0, 5), (1, 6)]],
        (1, 6): [(635, 185), [(0, 6), (1, 5)]],
        (2, 6): [(635, 275), [(2, 5), (3, 6)]],
        (3, 6): [(635, 365), [(2, 6), (4, 6)]],
        (4, 6): [(635, 455), [(3, 6), (5, 6)]],
        (5, 6): [(635, 545), [(4, 6), (5, 5), (6, 6)]],
        (6, 6): [(635, 635), [(5, 6), (6, 5)]],

    }




    def __init__(self, goal, sim):
        self.goal_state = goal
        self.sim = sim
        pass

    def decide(self, image, dir, position):
        """
        Takes a list of distances to nearby obstacles and computes move parameters.
        :param image: np.array of distances from surrounding obstacles as measured by a 360-degree LIDAR
        :returns angle: represents the clockwise angle (radians) to turn at that time
        :returns speed: scales the bot's velocity
        :returns message: message to show as a label by the bot
        """
        # message = ''
        # angle = 0
        #
        # # choose left or right, whatever has most distant obstacles
        # mid = len(image) // 2
        # l = image[:mid]  # left side of the bot
        # r = image[mid:]  # right side of the bot
        # fl = image[mid // 2:mid]  # front left quarter
        # fr = image[mid:mid + mid // 2]  # front right quarter
        # bl = image[:mid // 2]  # back left quarter
        # br = image[mid + mid // 2:]  # back right quarter
        #
        #
        # # If in the goal coordinate
        # # Get the next direction that you need to go
        #
        # self.current_state = self.convert_to_coordinate(position)
        # if self.current_state == self.goal_state:
        #
        #     return {'angle': 0, 'speed': 0, 'quote': 'Done'}
        # elif self.current_state == self.current_goal_state:
        #     if self.sim.path:
        #         self.current_goal_state = self.sim.path.pop()
        #         print("Current goal state", self.current_goal_state)
        #         self.current_direction = self.get_new_direction(self.current_goal_state)
        #     # Get Next off queue
        # elif self.current_goal_state == (-1, -1):
        #     #Get next off queue
        #     self.sim.path.pop()
        #     self.current_goal_state = self.sim.path.pop()
        #     self.current_direction = self.get_new_direction(self.current_goal_state)
        #
        #
        #
        # if self.current_direction == 'WEST' or self.current_direction == 'EAST':
        #     this_check = abs(dir[1] - self.directions[self.current_direction])
        #     other_check = abs(dir[1] + self.directions[self.current_direction])
        #     if abs(this_check - other_check) < .05:
        #         angle = 0
        #         message += 'straight'
        #         self.turning = False
        #     elif this_check <= other_check:
        #         angle = -1
        #         self.turning = True
        #         message += 'left'
        #     else:
        #         angle = 1
        #         message += 'right'
        #         self.turning = True
        # else:
        #     this_check = abs(dir[0] - self.directions[self.current_direction])
        #     other_check = abs(dir[0] + self.directions[self.current_direction])
        #     if abs(this_check - other_check) < .05:
        #         self.turning = False
        #         angle = 0
        #         message += 'straight'
        #     elif this_check <= other_check:
        #         angle = -1
        #         message += 'left'
        #         self.turning = True
        #     else:
        #         angle = 1
        #         message += 'right'
        #         self.turning = True
        #
        # # determine "safe" speed
        # # first, calculate the "front" quarter of the LIDAR image
        # front = image[3 * mid // 4: 5 * mid // 4]
        # # consider the closest obstacle in the front as the guideline for the speed
        # speed = pow(min(front) / np.average(image), 2)
        # if self.turning:
        #     speed *= 0.8
        # message += '\nspeed:{:2.0f}'.format(speed * 100)

        return {'angle': 0, 'speed': 0, 'quote': 'Training'}

    def a_star_search(self, start, goal):
        fringe = []
        heappush(fringe, (0, start))  # heap, priority, node
        predecessors = {start: None}
        cost = {start: 0}

        while fringe:
            current = heappop(fringe)[1]  # get the second element of the pair; first is the current priority

            if current == goal:
                break

            for next in self.graph[current][1]:
                new_cost = cost[current] + 10  # g
                if next not in cost or new_cost < cost[next]:
                    cost[next] = new_cost
                    priority = new_cost + self.heuristic(next, 0)  # d = g + h
                    heappush(fringe, (priority, next))  # reinsert with the new priority
                    predecessors[next] = current
        self.cost = cost
        self.path = self.construct_path(start, self.goal_state, predecessors)

    def convert_to_coordinate(self, position):
        x, y = position
        x = (x - 50) / 90
        y = (y - 50) / 90
        return tuple([int(y), int(x)])

    def check_goal(self, position):
        x, y = position
        x = (x - 80) / 90
        y = (y - 80) / 90
        tup = tuple([round(y), round(x)])
        return tup

    def heuristic(self, next, type):
        result = -1
        if type == 1:
            #Euclidian
            result = self.euclidian(next)
        else:
            #Manhatten
            result = self.manhatten(next)

        return result



    def euclidian(self, next):
        return math.floor(math.sqrt(pow((next[0] - self.goal_state[0]), 2) + pow((next[1] - self.goal_state[1]), 2)))

    def manhatten(self, next):

        return math.floor(abs(next[0] - self.goal_state[0]) - abs(next[1] - self.goal_state[1]))

    def construct_path(self, start, goal, predecessors):
        path = []
        cost = 0
        current = goal
        while current != start:
            path.append(current)
            current = predecessors[current]

        print(path)
        return path


    def get_new_direction(self, current_goal_state):
        direction = ''
        current_state = self.current_state
        print(current_state)
        print(current_goal_state)
        if current_state[0] + 1 == current_goal_state[0]:
            direction = "SOUTH"
        if current_state[0] - 1 == current_goal_state[0]:
            direction = "NORTH"
        if current_state[1] + 1 == current_goal_state[1]:
            direction = "EAST"
        if current_state[1] - 1 == current_goal_state[1]:
            direction = "WEST"

        return direction






