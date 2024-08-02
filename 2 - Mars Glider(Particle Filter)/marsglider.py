######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

# These import statements give you access to library functions which you may
# (or may not?) want to use.
from math import *
from glider import *
import random
import numpy

# If you see (vastly) different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


# This is the function you will have to write for part A.
# -The argument 'height' is a floating point number representing
# the number of meters your glider is above the average surface based upon 
# atmospheric pressure. (You can think of this as height above 'sea level'
# except that Mars does not have seas.) Note that this sensor may be
# slightly noisy.
# This number will go down over time as your glider slowly descends.
#
# -The argument 'radar' is a floating point number representing the
# number of meters your glider is above the specific point directly below
# your glider based off of a downward facing radar distance sensor. Note that
# this sensor has random Gaussian noise which is different for each read.

# -The argument 'mapFunc' is a function that takes two parameters (x,y)
# and returns the elevation above "sea level" for that location on the map
# of the area your glider is flying above.  Note that although this function
# accepts floating point numbers, the resolution of your map is 1 meter, so
# that passing in integer locations is reasonable.
#
#
# -The argument OTHER is initially None, but if you return an OTHER from
# this function call, it will be passed back to you the next time it is
# called, so that you can use it to keep track of important information
# over time.
#


def measurement_prob(glider_diff, particle_dist, sigma):
    diff = glider_diff - particle_dist
    return exp(-(diff ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


# def resample(particles, weights):
#     N = len(particles)
#     indices = []
#     C = [0.] + [sum(weights[:i+1]) for i in range(N)]
#     u0, j = random.uniform(0, 1./N), 0
#     for u in [u0 + i/N for i in range(N)]:
#         while u > C[j]:
#             j += 1
#         indices.append(j-1)
#     particles[:] = [particles[i] for i in indices]

def resample(particles, weights):
    N = len(particles)
    re = []
    index = int(random.random() * N)
    beta = 0.0
    max_weight = max(weights)
    newPart = glider()
    for i in range(N):
        beta += random.random() * 2.0 * max_weight
        while weights[index] < beta:
            beta -= weights[index]
            index = (index + 1) % N
        newPart = glider(particles[index].x, particles[index].y, particles[index].z, particles[index].heading, particles[index].mapFunc, 0, particles[index].speed)
        re.append(newPart)
    return re

N = 2000

def initialize_particles(num_particles):
    particles = []
    for _ in range(num_particles):
        xpos = random.uniform(-250, 250)
        ypos = random.uniform(-250, 250)
        zpos = random.uniform(4950, 5050)
        heading = random.gauss(0, pi / 4)
        particle = glider(x=xpos, y=ypos, z=zpos, heading=heading)
        particles.append(particle)
    return particles

def estimate_next_position(height, radar, mapFunc, OTHER=None):
    if OTHER is None:
        OTHER = initialize_particles(N)
    
    weights = []
    gliderDiff = height - radar
    sigma = 31
    for particle in OTHER:
        particleDist = mapFunc(particle.x, particle.y)
        weight = measurement_prob(gliderDiff, particleDist, sigma)
        weights.append(weight)
    
    OTHER = resample(OTHER, weights)

    percentageFuzz = 30
    hFuzz = pi / 10
    xyStdDev = percentageFuzz / 4
    headingFuzzStdDev = hFuzz / 4

    for _ in range(int(N * 0.3)):
        randIndex= random.randint(0, N - 1)
        xFuzz = random.gauss(0, xyStdDev)
        yFuzz = random.gauss(0, xyStdDev)
        OTHER[randIndex].x += xFuzz
        OTHER[randIndex].y += yFuzz

    for _ in range(int(N * 0.05)):
        randIndex = random.randint(0, N - 1)
        fuzz_heading = random.gauss(0, headingFuzzStdDev)
        OTHER[randIndex].heading += fuzz_heading

    for i in range(N):
        OTHER[i] = OTHER[i].glide()

    xTotal = sum(p.x for p in OTHER)
    yTotal = sum(p.y for p in OTHER)
    xy_estimate = (xTotal / N, yTotal / N)

    optionalPointsToPlot = [(p.x, p.y, p.heading) for p in OTHER]
    return xy_estimate, OTHER, optionalPointsToPlot
    

    # You may optionally also return a list of (x,y,h) points that you would like
    # the PLOT_PARTICLES=True visualizer to plot for visualization purposes.
    # If you include an optional third value, it will be plotted as the heading
    # of your particle.

    optionalPointsToPlot = [(1, 1), (2, 2), (3, 3)]  # Sample (x,y) to plot
    optionalPointsToPlot = [(1, 1, 0.5), (2, 2, 1.8), (3, 3, 3.2)]  # (x,y,heading)


# This is the function you will have to write for part B. The goal in part B
# is to navigate your glider towards (0,0) on the map steering # the glider 
# using its rudder. Note that the Z height is unimportant.

#
# The input parameters are exactly the same as for part A.

#
#

def next_turn_angle(height, radar, mapFunc, OTHER=None):
    # Get the current estimate of position and updated particles

    xy_estimate, OTHER, optionalPointsToPlot= estimate_next_position(height, radar, mapFunc, OTHER)
    xpos, ypos = xy_estimate
    xdiff = 0 - xpos
    ydiff = 0 - ypos
    currHeadingX = 0
    currHeadingY = 0
    for particle in OTHER:
        currHeadingX += cos(particle.heading)
        currHeadingY += sin(particle.heading)
    currHeading = atan2(currHeadingX,currHeadingY)
    ##Error here not sure if that is the right way to get average heading
    target = atan2((xdiff),(ydiff))
    targetHeading = currHeading - target
    targetHeading = angle_trunc(targetHeading)
    #Gotta be some angle voodoo goin on up there

    if targetHeading >= pi/8.0:
        turn_angle = pi/8.0
    elif targetHeading <= -pi/8.0:
        turn_angle = -pi/8.0
    else:
        turn_angle = targetHeading
    #This is going in  the right angle
    for particle in OTHER:
        particle.heading = particle.heading + turn_angle
    return turn_angle, OTHER, optionalPointsToPlot



def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith122).
    whoami = 'pmarneni3'
    return whoami
