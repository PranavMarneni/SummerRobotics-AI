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

"""
 === Introduction ===

   The assignment is broken up into two parts.

   Part A:
        Create a SLAM implementation to process a series of landmark measurements (location of tree centers) and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.

    Part B:
        Here you will create the action planner for the drone.  The returned actions will be executed with the goal being to navigate to
        and extract the treasure from the environment marked by * while avoiding obstacles (trees).
        Actions:
            'move distance steering'
            'extract treasure_type x_coordinate y_coordinate'
        Example Actions:
            'move 1 1.570963'
            'extract * 1.5 -0.2'

    Note: All of your estimates should be given relative to your drone's starting location.

    Details:
    - Start position
      - The drone will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future drone location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from trees located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D', 'radius':0.5}, ...}
      - Only trees that are within the horizon distance will return measurements. Therefore new trees may appear as you move through the environment.
    - Movements
      - Action: 'move 1.0 1.570963'
        * The drone will turn counterclockwise 90 degrees [1.57 radians] first and then move 1.0 meter forward.
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the drone will not move.
      - Action: 'extract * 1.5 -0.2'
        * The drone will attempt to extract the specified treasure (*) from the current location of the drone (1.5, -0.2).
      - The drone must be within 0.25 distance to successfully extract a treasure.

    The drone will always execute a measurement first, followed by an action.
    The drone will have a time limit of 10 seconds to find and extract all of the needed treasures.
"""

from typing import Dict
from rait import matrix
import math

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module."""

    def __init__(self):
        """Initialize SLAM components here."""
        self.Omega = matrix([[1.0, 0.0],
                             [0.0, 1.0]])
        self.Xi = matrix([[0],
                          [0]])
        self.dim = 2
        self.treeNum = 0  # helps us when putting indexes to trees
        self.treeList = {}  # treeDict id to position
        self.orientation = 0
        self.motionNoise = 3
        self.measurementNoise = 2.345
    # Provided Functions
    def get_coordinates(self):
        """
        Retrieves the estimated (x, y) locations in meters of the drone and all landmarks (trees) when called.

        Args: None

        Returns:
            The (x, y) coordinates in meters of the drone and all landmarks (trees) in the format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        mu = self.Omega.inverse() * self.Xi
        coordinates = {'self': (mu[0][0], mu[1][0])}
        for tree, idx in self.treeList.items():
            coordinates[tree] = (mu[2 * idx + 2][0], mu[2 * idx + 3][0])
        return coordinates

    def process_measurements(self, measurements: Dict):
        """
        Process a new series of measurements and update (x,y) location of drone and landmarks.

        Args:
            measurements: Collection of measurements of tree positions and radius
                in the format {'landmark_id': {'distance': float <meters>, 'bearing': float <radians>, 'type': char, 'radius': float <meters>}, ...}
        """
        for locationID, data in measurements.items():
            distance = data['distance']
            bearing = data['bearing'] + self.orientation
            delta_x = distance * math.cos(bearing)
            delta_y = distance * math.sin(bearing)

            if locationID not in self.treeList:
                self.treeList[locationID] = self.treeNum
                self.treeNum += 1

                # Expand to take in the new location
                old_dim = self.dim
                self.dim += 2
                self.Omega = self.Omega.expand(self.dim, self.dim, list(range(old_dim)), list(range(old_dim)))
                self.Xi = self.Xi.expand(self.dim, 1, list(range(old_dim)), [0])

                m = 2 * (self.treeList[locationID] + 1)
                for b in range(2):
                    self.Omega.value[b][b] += 1.0 / self.measurementNoise
                    self.Omega.value[m + b][m + b] += 1.0 / self.measurementNoise
                    self.Omega.value[b][m + b] += -1.0 / self.measurementNoise
                    self.Omega.value[m + b][b] += -1.0 / self.measurementNoise
                    if b == 0:
                        self.Xi.value[b][0] += -delta_x / self.measurementNoise
                        self.Xi.value[m + b][0] += delta_x / self.measurementNoise
                    else:
                        self.Xi.value[b][0] += -delta_y / self.measurementNoise
                        self.Xi.value[m + b][0] += delta_y / self.measurementNoise

    def process_movement(self, distance: float, steering: float):
        """
        Process a new movement and update (x,y) location of drone.

        Args:
            distance: distance to move in meters
            steering: amount to turn in radians
        """
        idxs = [0, 1] + list(range(4, self.dim + 2))
        self.Omega = self.Omega.expand(self.dim + 2, self.dim + 2, idxs, idxs)
        self.Xi = self.Xi.expand(self.dim + 2, 1, idxs, [0])

        self.orientation = steering + self.orientation
        delta_x = distance * math.cos(self.orientation)
        delta_y = distance * math.sin(self.orientation)

        for b in range(4):
            self.Omega.value[b][b] += 1.0 / self.motionNoise
        for b in range(2):
            self.Omega.value[b + 0][2 + b] += -1.0 / self.motionNoise
            self.Omega.value[2 + b][0 + b] += -1.0 / self.motionNoise
            if b == 0:
                self.Xi.value[0 + b][0] += -delta_x / self.motionNoise
                self.Xi.value[2 + b][0] += delta_x / self.motionNoise
            else:
                self.Xi.value[0 + b][0] += -delta_y / self.motionNoise
                self.Xi.value[2 + b][0] += delta_y / self.motionNoise

        newidxs = list(range(2, len(self.Omega.value)))
        a = self.Omega.take([0, 1], newidxs)
        b = self.Omega.take([0, 1])
        c = self.Xi.take([0, 1], [0])
        self.Omega = self.Omega.take(newidxs) - a.transpose() * b.inverse() * a
        self.Xi = self.Xi.take(newidxs, [0]) - a.transpose() * b.inverse() * c

import drone
class IndianaDronesPlanner:
    """
    Create a planner to navigate the drone to reach and extract the treasure marked by * from an unknown start position while avoiding obstacles (trees).
    """

    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the drone can travel in a single move in meters.
            max_steering: the max steering angle the drone can turn in a single move in radians.
        """
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.slam = SLAM()
        self.current_position = (0, 0)
        self.current_orientation = 0
        self.energy_reserve = 100.0
        self.previous_actions = []

    def next_move(self, measurements: Dict, treasure_location: Dict):
        """Next move based on the current set of measurements.

        Args:
            measurements: Collection of measurements of tree positions and radius in the format
                          {'landmark id':{'distance': float <meters>, 'bearing':float <radians>, 'type': char, 'radius':float <meters>}, ...}
            treasure_location: Location of Treasure in the format {'x': float <meters>, 'y':float <meters>, 'type': char '*'}

        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the drone.
                allowed:
                    'move distance steering'
                    'move 1.0 1.570963'  - Turn left 90 degrees and move 1.0 distance.

                    'extract treasure_type x_coordinate y_coordinate'
                    'extract * 1.5 -0.2' - Attempt to extract the treasure * from your current location (x = 1.5, y = -0.2).
                                           This will succeed if the specified treasure is within the minimum sample distance.

            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the drone estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        # Process measurements and update the SLAM system
        self.slam.process_measurements(measurements)
        estimated_positions = self.slam.get_coordinates()
        self.current_position = estimated_positions.get('self', (0, 0))
        self.current_orientation = self.slam.orientation

        # Extract treasure coordinates
        target_x = treasure_location['x']
        target_y = treasure_location['y']

        # Compute distances and angles
        delta_x = target_x - self.current_position[0]
        delta_y = target_y - self.current_position[1]
        distance_to_treasure = math.sqrt(delta_x ** 2 + delta_y ** 2)
        angle_to_treasure = math.atan2(delta_y, delta_x)

        # Energy cost parameters
        energy_cost_per_meter = 1.0
        energy_cost_per_radian = 0.5
        required_energy = (distance_to_treasure * energy_cost_per_meter) + (abs(angle_to_treasure) * energy_cost_per_radian)

        # Check if energy is sufficient
        if self.energy_reserve < required_energy:
            action = "conserve energy"
            points_to_plot = self.slam.get_coordinates()
            return action, points_to_plot
        if distance_to_treasure <= 0.25:
            action = f"extract * {target_x:.2f} {target_y:.2f}"
            points_to_plot = self.slam.get_coordinates()
            self.previous_actions.append(action)
            return action, points_to_plot
        
        steering_angle = (angle_to_treasure - self.current_orientation + math.pi) % (2 * math.pi) - math.pi

        # Adjust the steering angle if it exceeds the maximum allowed
        if abs(steering_angle) > self.max_steering:
            steering_angle = math.copysign(self.max_steering, steering_angle)

        # Determine the distance to move
        travel_distance = min(self.max_distance, distance_to_treasure)
        action = f"move {travel_distance:.2f} {steering_angle:.2f}"

        # Update SLAM with the movement
        self.slam.process_movement(travel_distance, steering_angle)
        self.energy_reserve -= (travel_distance * energy_cost_per_meter) + (abs(steering_angle) * energy_cost_per_radian)
        points_to_plot = self.slam.get_coordinates()
        self.previous_actions.append(action)
        return action, points_to_plot

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'pmarneni3'
    return whoami
