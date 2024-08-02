import numpy as np  # type: ignore
from numpy.linalg import inv  # type: ignore
from numpy import linalg as la  # type: ignore
import random
import math

#returns the direction of angle change necessary to move curr heading toward heading

def angleDirection(currHeading, heading):
    if currHeading > heading + 0.3:
        print(currHeading, heading)
        currHeading -= heading
        if (currHeading < math.pi):
            return -1
        else:
            return 1
    elif currHeading < heading - 0.3:
        heading -= currHeading
        if (heading < math.pi):
            return 1
        else:
            return -1
    else:
        return 0

def motionUpdate(x, F):
    return np.matmul(F, x)
def motionUncertaintyUpdate(F, P):
    return np.matmul(F, np.matmul(P, F.transpose()))

def kalman(x, P, z):
    F = np.array([[1.0, 0.0, 1.0, 0.0],
                 [0.0, 1.0, 0.0, 1.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]])
    R = np.array([[0.3, 0.0],
                  [0.0, 0.3]])
    H = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]])
    z = np.array(z)
    y = z - np.matmul(H, x)
    S = np.matmul(H, np.matmul(P, H.transpose())) + R
    S = inv(S)
    K = np.matmul(P, np.matmul(H.transpose(), S))
    x = x + np.matmul(K, y)
    P = np.matmul(np.identity(4) - (np.matmul(K, H)), P)

    return (x, P)

class Pilot():

    def __init__(self, minimum_threshold, in_bounds):
        self.minimum_threshold = minimum_threshold
        self.in_bounds = in_bounds
        self.curr_observed_state = dict() #this is the dictionary of MEASUREMENTS for each asteroid
        self.curr_state = dict() #this is a dictionary of the estimates for each asteroid
        self.curr_P = dict() #this is a dictionary of the uncertainties for each asteroid

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

        self.curr_observed_state.clear()
        for i in asteroid_locations:
            self.curr_observed_state[i[0]] = [i[1], i[2]]
    
    def estimate_asteroid_locs(self):
        """ Should return an iterable (list or tuple for example) that
            contains data in the format (i,x,y), consisting of estimates
            for all in-bound asteroids. """

        #based on the observed list of asteroids, add new asteroids that crop up and remove old ones that leave
        #initialize new asteroids to [0, 0, 0, 0] for their state before the first measurement
        for i in self.curr_observed_state:
            if (i not in self.curr_state.keys()):
                self.curr_state[i] = [0, 0, 0, 0]
                self.curr_P[i] = ([[10.0, 0.0, 0.0, 0.0],
                                [0.0, 10.0, 0.0, 0.0],
                                [0.0, 0.0, 10.0, 0.0],
                                [0.0, 0.0, 0.0, 10.0]])
        for i in self.curr_state.copy():
            if (i not in self.curr_observed_state.keys()):
                self.curr_state.pop(i)

        #iterate through the current state asteroids and perform a single step kalman filter on each
        for i in self.curr_state:
            F = np.array([[1.0, 0.0, 1.0, 0.0],
                          [0.0, 1.0, 0.0, 1.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]])
            newState = kalman(self.curr_state[i], self.curr_P[i], self.curr_observed_state[i])
            self.curr_state[i] = newState[0]
            self.curr_P[i] = newState[1]
            self.curr_state[i] = motionUpdate(self.curr_state[i], F)
            self.curr_P[i] = motionUncertaintyUpdate(F, self.curr_P[i])

        list = []
        for i in self.curr_state:
            list.append((i, self.curr_state[i][0], self.curr_state[i][1]))
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
        
        # <STUDENT IMPLEMENTATION GOES HERE>

        angle = 0
        speed = 1
        self.estimate_asteroid_locs()
        distdict = dict()
        for i in self.curr_state:
            distdict[i] = math.sqrt((self.curr_state[i][0] - craft_state.x)**2 + (self.curr_state[i][1] - craft_state.y)**2)
        sortedList = []
        for i in distdict:
            sortedList.append(distdict[i])
        sortedList.sort()
        headingdict = dict()
        for i in self.curr_state:
            opp = (self.curr_state[i][1] - craft_state.y)
            adj = (self.curr_state[i][0] - craft_state.x)
            if (adj == 0):
                if (opp > 0):
                    headingdict[i] = 3 * math.pi / 2
                else:
                    headingdict[i] = math.pi/2
                continue
            if (opp == 0):
                if (adj > 0):
                    headingdict[i] = math.pi
                else:
                    headingdict[i] = 0
                continue
            tangent = math.atan(opp/adj)
            if (opp > 0 and adj > 0):
                headingdict[i] = (tangent + math.pi) % (2 * math.pi)
            elif (opp > 0 and adj < 0):
                headingdict[i] = ((tangent + math.pi) + math.pi) % (2 * math.pi)
            elif (opp < 0 and adj < 0):
                headingdict[i] = ((tangent + math.pi) + math.pi) % (2 * math.pi)
            elif (opp < 0 and adj > 0):
                headingdict[i] = (tangent + math.pi) % (2 * math.pi)

        #sortedlist is a list of all asteroid's distance from the craft
        #distdict is a dictionary of all asteroids and their distance from the craft
        #heading dict is a dictionary of the direction of all asteroids relative to the craft
        closestheadingsdict = dict()
        for i in self.curr_state:
            #only interested in 15 closest and if theyre close enough to be a problem; if so, add to closest dict
            if distdict[i] < sortedList[4] and distdict[i] < 0.11:
                closestheadingsdict[i] = headingdict[i]
        total = 0
        avgtotal = 0
        for i in closestheadingsdict:
            total += 1/distdict[i]
            avgtotal += distdict[i]
        wallheadingdict = dict()
        walldistdict = dict()
        temp = 0
        if (craft_state.x - 1.0 > -0.31):
            wallheadingdict[2] = 5 * math.pi / 6
            walldistdict[2] = abs(craft_state.x - 1.0)
            total += 1/abs(craft_state.x - 1.0)
            avgtotal += abs(craft_state.x - 1.0)
        if (craft_state.x + 1.0 < 0.31):
            wallheadingdict[0] = math.pi / 6
            walldistdict[0] = abs(craft_state.x + 1.0)
            total += 1/abs(craft_state.x + 1.0)
            avgtotal += abs(craft_state.x + 1.0)
        if (craft_state.y + 1.2 < 0.31):
            wallheadingdict[1] = math.pi/2
            walldistdict[1] = abs(craft_state.y + 1.2)
            total += 1/abs(craft_state.y + 1.2)
            avgtotal += abs(craft_state.y + 1.2)
            temp += abs(craft_state.y + 1.2)
        totalheading = 0
        #takes a weighted average by how far it is
        for i in closestheadingsdict:
            totalheading += (closestheadingsdict[i]) * ((1/distdict[i])/total)
        for i in wallheadingdict:
            totalheading += (wallheadingdict[i]) * (1/walldistdict[i]/total)
        
        if not bool(closestheadingsdict) and not bool(wallheadingdict):
            #if empty
            speed = 0
            angle = angleDirection(craft_state.h, math.pi/2)
        else:
            print(avgtotal/(len(closestheadingsdict) + len(walldistdict)))
            if avgtotal/(len(closestheadingsdict) + len(walldistdict)) < 0.3:
                if (avgtotal/(len(closestheadingsdict) + len(walldistdict)) < 0.2 and abs(craft_state.h - totalheading) > 2):
                    speed = 0
                else:
                    speed = 1
                angle = angleDirection(craft_state.h, totalheading)
            else:
                speed = 0
                if (craft_state.x < 0):
                    angle = angleDirection(craft_state.h, math.pi)
                else:
                    angle = angleDirection(craft_state.h, 6.28)
        return (angle, speed)