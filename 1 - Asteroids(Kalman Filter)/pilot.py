from collections import deque

import asteroid
from matrix import matrix

import numpy as np  # type: ignore
from numpy.linalg import inv  # type: ignore
import math

class Pilot():
    def __init__(self, minimum_threshold, in_bounds):
        self.minimum_threshold = minimum_threshold
        self.last_angles = deque(maxlen=100)
        #self.last_angle = []
        self.observed = dict()  # this is the dictionary of MEASUREMENTS for each asteroid
        self.curr_state = dict()  # this is a dictionary of the estimates for each asteroid
        self.curr_P = dict()  # this is a dictionary of the uncertainties for each asteroid
        self.future =  []

    def observe_asteroids(self, asteroid_locations):
        """
           asteroid_locations - a list of asteroid observations. Each
           observation is a tuple (i,x,y) where i is the unique ID for
           the asteroid, and x,y are the x,y locations (with noise) of the
           current observation of the asteroid for this timestep.
           Only asteroids that are currently 'in-bounds' will appear
           in this list.  The list may change in size as asteroids move
           out-of-bounds or new asteroids appear in-bounds.

           Return Values:
                    None
        """

        self.observed.clear()
        for i in asteroid_locations:
            self.observed[i[0]] = [i[1], i[2]]

    def estimate_asteroid_locs(self):
        """ Should return an iterable (list or tuple for example) that
            contains data in the format (i,x,y), consisting of estimates
            for all in-bound asteroids. """

        #based on the observed list of asteroids, add new asteroids that crop up and remove old ones that leave
        #initialize new asteroids to [0, 0, 0, 0] for their state before the first measurement
        for i in self.observed:
            if (i not in self.curr_state.keys()):
                self.curr_state[i] = [0, 0, 0, 0]
                self.curr_P[i] = ([[15.0, 0.0, 0.0, 0.0],
                                [0.0, 15.0, 0.0, 0.0],
                                [0.0, 0.0, 15.0, 0.0],
                                [0.0, 0.0, 0.0, 15.0]])
        for i in self.curr_state.copy():
            if (i not in self.observed.keys()):
                self.curr_state.pop(i)

        #iterate through the current state asteroids and perform a single step kalman filter on each
        for i in self.curr_state:
            F = np.array([[1.0, 0.0, 1.0, 0.0],
                          [0.0, 1.0, 0.0, 1.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]])
            x = self.curr_state[i]
            P = self.curr_P[i]
            z = self.observed[i]
            R = np.array([[0.3, 0.0],
                        [0.0, 0.3]])
            H = np.array([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]])
            z = np.array(z)
            y = z - np.matmul(H, x)
            S = np.matmul(H, np.matmul(P, H.transpose())) + R
            K = np.matmul(P, np.matmul(H.transpose(), inv(S)))
            x = x + np.matmul(K, y)
            P = np.matmul(np.identity(4) - (np.matmul(K, H)), P)
            self.curr_state[i] = np.matmul(F, x)
            self.curr_P[i] = np.matmul(F, np.matmul(P, F.transpose()))








        list = []
        for i in self.curr_state:
            list.append((i, self.curr_state[i][0], self.curr_state[i][1]))
            self.future.append((i, self.curr_state[i][0], self.curr_state[i][1]))
        return list

    def next_move(self, craft_state):
        """
            craft_state - implemented as CraftState in craft.py.

            return values:
              angle change: the craft may take the following actions:
                                turn left: 1
                                turn right: -1
                                go straight: 0
                            Turns adjust the craft's heading by
                             angle_increment.
              speed change: the craft may:
                                accelerate: 1,
                                decelerate: -1
                                continue at its current velocity: 0
                            Speed changes adjust the craft's velocity by
                            speed_increment, maxing out at max_speed.
         """
        val = sum(self.last_angles)
        if (val < 0):
            best_angle = 1
        elif (val > 0):
            best_angle = -1
        else:
            best_angle = 0
        speed = 1

        min_distance = float('inf')
        possible_angles = [1, -1, 0]
        distance = -1000
        min = 100
        # Iterate over possible angles to find the best move
        for angle in possible_angles:
            # Predict the future position based on the angle change
            future_position = craft_state.steer(0, 1)
            self.estimate_asteroid_locs()
            distdict = dict()
            for i in self.curr_state:
                distdict[i] = math.sqrt(
                    (self.curr_state[i][0] - future_position.x) ** 2 + (self.curr_state[i][1] - future_position.y) ** 2)
            sortedList = []
            for i in distdict:
                sortedList.append(distdict[i])
            sortedList.sort()
            # Check if this is the shortest distance found

            if sortedList[0] < 0.089 and sortedList[0] < min:
                speed = 1
                if sortedList[0] < 0.06:
                    speed = 0
                min = sortedList[0]

                angleLeft = dict()
                future_position = craft_state.steer(-1, 1)
                for x in self.curr_state:
                    angleLeft[x] = math.sqrt(
                        (self.curr_state[x][0] - future_position.x) ** 2 + (self.curr_state[x][1] - future_position.y) ** 2)
                sortedList2 = []
                for i in angleLeft:
                    sortedList2.append(angleLeft[i])
                sortedList2.sort()
                leftMin = sortedList2[0]

                angleRight = dict()
                future_position = craft_state.steer(1, 1)
                for y in self.curr_state:
                    angleRight[y] = math.sqrt(
                        (self.curr_state[y][0] - future_position.x) ** 2 + (
                                    self.curr_state[y][1] - future_position.y) ** 2)
                sortedList3 = []
                for i in angleRight:
                    sortedList3.append(angleRight[i])
                sortedList3.sort()
                rightMin = sortedList3[0]

                if (leftMin > rightMin):
                    best_angle = -1
                else:
                    best_angle = 1

        # Return the best angle change and speed change
        self.last_angles.appendleft(best_angle)
        return best_angle, speed
