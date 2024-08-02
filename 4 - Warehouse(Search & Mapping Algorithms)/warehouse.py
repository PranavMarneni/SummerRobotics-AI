import heapq as hq
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

import math
import state
from state import State

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class DeliveryPlanner_PartA:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

      plan_delivery(self, debug = False):
       Stubbed out below.  You may not change the method signature
        as it will be called directly by the autograder but you
        may modify the internals as needed.

      __init__:
        Required to initialize the class.  Signature can NOT be changed.
        Basic template starter code is provided.  You may choose to
        use this starter code or modify and replace it based on
        your own solution.
    """

    def __init__(self, warehouse_viewer, dropzone_location, todo, box_locations):
        self.warehouse_viewer = warehouse_viewer
        self.dropzone_location = dropzone_location
        self.todo = todo
        self.box_locations = box_locations

    def checkMovement(self, xy):
        if xy == (-1, 0):
            return ('n', 2, 'move n')
        elif xy == (1, 0):
            return ('s', 2, 'move s')
        elif xy == (0, -1):
            return ('w', 2, 'move w')
        elif xy == (0, 1):
            return ('e', 2, 'move e')
        elif xy == (-1, -1):
            return ('nw', 3, 'move nw')
        elif xy == (-1, 1):
            return ('ne', 3, 'move ne')
        elif xy == (1, -1):
            return ('sw', 3, 'move sw')
        elif xy == (1, 1):
            return ('se', 3, 'move se')

    def checkEmpty(self, location):

        for dx, dy in state.DELTA_DIRECTIONS:
            direction = self.checkMovement((dx, dy))[2]
            nx = location[0] + dx
            ny = location[1] + dy
            if self.warehouse_viewer[nx][ny] == '.':
                return nx, ny, direction
        return None
    

    def plan_delivery(self, debug=True):

        def diagonalDistance(x1, y1, x2, y2):
            return 2 * math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
            # dx = abs(x1 - x2)
            # dy = abs(y1 - y2)
            # return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

        def adjacentCheck1(position, location):
            
            boxAdjacent = False
            x, y = position
            valid = []
            for dx, dy in state.DELTA_DIRECTIONS:
                nx = x + dx
                ny = y + dy
                direction, cost, directionName = self.checkMovement((dx, dy))
                try:
                    if self.warehouse_viewer[nx][ny] == '.' or self.warehouse_viewer[nx][ny] == '@':
                        valid.append(((nx, ny), direction, cost, directionName))
                    else:
                        if (nx, ny) == location:
                            boxAdjacent = True
                except:
                    continue
            return valid, boxAdjacent

        def adjacentCheck2(position, location):

            boxAdjacent = False
            x, y = position
            valid = []
            actualDirection = ""
            for dx, dy in state.DELTA_DIRECTIONS:
                x2, y2 = x + dx, y + dy
                direction, cost, directionName = self.checkMovement((dx, dy))
                try:
                    if self.warehouse_viewer[x2][y2] == '.':
                        valid.append(((x2, y2), direction, cost, directionName))
                    else:
                        if (x2, y2) == location:
                            actualDirection = direction
                            boxAdjacent = True
                except:
                    continue
            return valid, boxAdjacent, actualDirection

        def search(start, goal, targetBox, otwToBox):
            goalX, goalY = goal
            pq = [(0, start, [])]
            hq.heapify(pq)
            gList = {start : 0}
            visited = set()

            while pq:

                f, current, path = hq.heappop(pq)
                if current in visited:
                    continue
                visited.add(current)
                if otwToBox:
                    valid_moves, isAdjacent = adjacentCheck1(current, goal)
                else:
                    valid_moves, isAdjacent, direction = adjacentCheck2(current, goal)
                if isAdjacent and otwToBox:
                    s = 'lift ' + targetBox
                    path += [s]
                    return path, current

                if isAdjacent and not otwToBox:
                    s = 'down ' + direction
                    path += [s]
                    return path, current
                for spotCoords, direction, cost, directionName in valid_moves:
                    newg = gList[current] + cost
                    if spotCoords not in gList or newg < gList[spotCoords]:
                        gList[spotCoords] = newg
                        totalCost = newg + diagonalDistance(spotCoords[0], spotCoords[1], goalX, goalY)
                        newPath = path + [directionName]
                        hq.heappush(pq, (totalCost, spotCoords, newPath))
                
            return [], start

        moves = []
        start = self.dropzone_location


        for i in self.todo:
            box = self.box_locations[i]
            otwToBox = True

            nextPath, pickup = search(start, box, i, otwToBox)
            moves.extend(nextPath)
            self.warehouse_viewer[self.box_locations[i][0]][self.box_locations[i][1]] = '.'
            robotLocation = pickup

            if robotLocation == self.dropzone_location:
                empty_location = self.checkEmpty(robotLocation)
                if empty_location:
                    dx, dy, direction = empty_location
                    moves.extend([direction])
                    robotLocation = (dx, dy)
            otwToBox = False
            returnPath, dropOff = search(robotLocation, self.dropzone_location, i, otwToBox)
            start = dropOff
            moves.extend(returnPath)
            self.warehouse_viewer[self.box_locations[i][0]][self.box_locations[i][1]] = '.'


        print(moves)
        return moves


class DeliveryPlanner_PartB:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug=False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class. Signature can NOT be changed.
         Basic template starter code is provided. You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo):
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.warehouse = warehouse

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse (list of list): the warehouse map.
        """
        self.rows = len(warehouse)
        self.cols = len(warehouse[0])

        self.warehouse_state = [[None for _ in range(self.cols)] for _ in range(self.rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(self.rows):
            for j in range(self.cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'
                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'
                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)
                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)
    def findOpposite(self,direction):
        opposites = {
            'n': 's',
            's': 'n',
            'w': 'e',
            'e': 'w',
            'nw': 'se',
            'ne': 'sw',
            'sw': 'ne',
            'se': 'nw'
        }
        return opposites.get(direction)

        curr = movements.index(direction)
        return movements[(curr + 1)]
    def isValid(self, position):
        """Check if the position given is valid."""
        x, y = position
        return 0 <= x < self.rows and 0 <= y < self.cols and self.warehouse[x][y] != '#'

    def valueIteration(self, goal, toBox, debug=False):
        movements = {
            'move n': (-1, 0, 2),
            'move s': (1, 0, 2),
            'move w': (0, -1, 2),
            'move e': (0, 1, 2),
            'move nw': (-1, -1, 3),
            'move ne': (-1, 1, 3),
            'move sw': (1, -1, 3),
            'move se': (1, 1, 3)
        }

        values = [[float('inf') for _ in range(self.cols)] for _ in range(self.rows)]
        policy = [["-1" for _ in range(self.cols)] for _ in range(self.rows)]

        goalx, goaly = goal
        values[goalx][goaly] = 0
        policy[goalx][goaly] = "B"
        done = False
        
        while not done:
            done = True
            for i in range(self.rows):
                for j in range(self.cols):
                    if (i, j) == goal or self.warehouse[i][j] == '#':
                        continue
                    for action, (di, dj, cost) in movements.items():
                        ni, nj = i + di, j + dj
                        if self.isValid((ni, nj)):
                            new_cost = values[ni][nj] + cost + self.warehouse_cost[ni][nj]
                            if new_cost < values[i][j]:
                                done = False
                                values[i][j] = new_cost
                                policy[i][j] = (
                                    "lift 1" if toBox and (ni, nj) == goal else
                                    f"down {action[5:]}" if not toBox and (ni, nj) == goal else
                                    action
                                )

        for i in range(self.rows):
            for j in range(self.cols):
                if values[i][j] == float('inf'):
                    values[i][j] = -1

        for i in range(self.rows):
            for j in range(self.cols):
                if policy[i][j] == 'B':
                    for action, (di, dj, cost) in movements.items():
                        ni, nj = i + di, j + dj
                        if 0 <= ni < self.rows and 0 <= nj < self.cols:  # Check bounds
                            if policy[ni][nj].startswith("down "):
                                opposite_direction = self.findOpposite(policy[ni][nj][5:])
                                policy[i][j] = f"move {opposite_direction}"


        if debug:
            print("\nValues:")
            for row in values:
                print(row)

            print("\nPolicy:")
            for row in policy:
                print(row)

        return policy, values

    def generate_policies(self, debug=False):
        box_id = "1"
        to_box_policy, _ = self.valueIteration(self.boxes[box_id],True)
        deliver_policy, _ = self.valueIteration(self.dropzone,False)


        if debug:
            print("\nTo Box Policy:")
            for row in to_box_policy:
                print(row)

            print("\nDeliver Policy:")
            for row in deliver_policy:
                print(row)

        return to_box_policy, deliver_policy


# NOTE: Part C is optional.  It is NOT part of your grade for this project, but you are welcome to attempt it if you wish.
# run the part C testing suite to test your part C code.
class DeliveryPlanner_PartC:
    """
    [Doc string same as part B]
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.stochastic_probabilities = stochastic_probabilities

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # The following is the hard coded solution to test case 1
        to_box_policy = [
            ['B', 'lift 1', 'move w'],
            ['lift 1', -1, 'move nw'],
            ['move n', 'move nw', 'move n'],
        ]

        to_zone_policy = [
            ['move e', 'move se', 'move s'],
            ['move se', -1, 'down s'],
            ['move e', 'down e', 'move n'],
        ]

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        to_box_values = None
        to_zone_values = None
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'pmarneni3'
    return whoami


if __name__ == "__main__":
    """
    You may execute this file to develop and test the search algorithm prior to running
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    print('\n~~~ Testing for part A: ~~~\n')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    # testcase 1
    warehouse = [
        '######',
        '#....#',
        '#.1#2#',
        '#..#.#',
        '#...@#',
        '######',
    ]
    todo = list('12')
    benchmark_cost = 23
    viewed_cell_count_threshold = 20
    dropzone = (4,4)
    box_locations = {
        '1': (2,2),
        '2': (2,4),
    }
    # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, dropzone, todo, box_locations)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\n~~~ Testing for part B: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.generate_policies(debug=True)

    # Testing for Part C
    # testcase 1
    print('\n~~~ Testing for part C: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    stochastic_probabilities = {
        'as_intended': .70,
        'slanted': .1,
        'sideways': .05,
    }

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partC.generate_policies(debug=True)