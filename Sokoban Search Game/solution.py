#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
import heapq
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def heur_manhattan_distance(state):
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage
    # point nearest to it is such a heuristic. When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan_distance_sum = 0
    for box in state.boxes:
        manhattan_distances = []
        for storage in state.storage:
            box_to_storage_distance = manhattan_distance(box, storage)
            manhattan_distances.append(box_to_storage_distance)
        if len(manhattan_distances) > 0:
            manhattan_distance_sum += min(manhattan_distances)
    return manhattan_distance_sum


def manhattan_distance(p1, p2):
    # Implementation of manhattan distance between 2 arbitrary points
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''Trivial admissible heuristic accumulated the number of boxes in state boxes not in state storage'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    boxes = 0
    for box in state.boxes:
        if box not in state.storage:
            boxes += 1
    return boxes


def heur_alternate(state):
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between
    # the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # The improvement is made by the following:
    # 1. Check if given the current state of the game, there are any boxes that can't move
    #    A More detailed explanation of what is meant by a box can't move is in the function box_cant_move
    # If there are any boxes that can't move then return infinity since there will be no solution in that state since
    # there is at least 1 box that can't move.
    res = 0
    if box_cant_move(state):
        return math.inf
    # 2. Calculate the sum of the distances from the robots to the boxes.
    if state.robots:
        res += get_robot_to_box_distances(state)
    # 3. Calculate the sum of the distances from the boxes to the storage spaces.
    if state.boxes:
        res += get_box_to_storage_distances(state)
    return res


def get_box_to_storage_distances(state):
    res = 0
    for box in state.boxes:
        box_to_storage_distances = [math.inf]
        storages = get_storage_spaces(box, state)
        for storage in storages:
            box_to_storage_distances.append(
                manhattan_distance(storage, box))
        res += min(box_to_storage_distances)
    return res


def get_robot_to_box_distances(state):
    res = 0
    for robot in state.robots:
        robot_to_box_distances = [math.inf]
        for box in state.boxes:
            robot_to_box_distances.append(manhattan_distance(robot, box))
        res += min(robot_to_box_distances)
    return res


def get_storage_spaces(box, state):
    # Return all the storage spaces available to the box by creating a possible list of storage spaces and populating
    # the list with [box] if the box is already in storage. Also remove any storage spaces that are already taken
    storages_spaces = []
    for space in state.storage:
        storages_spaces.append(space)
    if box in storages_spaces:
        return [box]
    for stored_box in state.boxes:
        if box != stored_box and stored_box in storages_spaces:
            storages_spaces.remove(stored_box)
    return storages_spaces


def box_cant_move(state):
    # For each box, check if its in storage space. If its not in a storage space its possible the box won't be
    # able to move to a storage space. Any given box won't be able to move to a storage space it's:
    # 1. In a corner of the gameboard
    # 2. In a corner between a gameboard wall and an obstacle
    # 3. In a corner between two obstacles
    # 4. Against a wall of the gameboard that has no storage space
    for box in state.boxes:
        storage_spaces = get_storage_spaces(box, state)
        if box not in storage_spaces:
            if is_in_gameboard_corner(box, state) or \
                    is_in_gameboard_wall_obstacle_corner(box, state) or \
                    is_in_obstacle_corner(box, state) or \
                    is_against_wall_with_no_storage(box, state):
                return True
    return False


def is_in_gameboard_corner(box, state):
    # Check Condition 1. In a corner of the gameboard
    # The West-North, West-South, East-North and North-West corners need to be checked
    (x, y) = box
    if x == state.width - 1 and (y == 0 or y == state.height - 1):
        return True
    if x == 0 and (y == 0 or y == state.height - 1):
        return True
    return False


def is_in_gameboard_wall_obstacle_corner(box, state):
    # Check Condition 2. In a corner between a gameboard wall and an obstacle
    # Check if there is an obstacle-wall directly North, East, South or West
    obstacles = state.obstacles.union(state.boxes)
    (x, y) = box
    north = (x, y + 1)
    south = (x, y - 1)
    east = (x - 1, y)
    west = (x + 1, y)
    if x == state.width - 1 and ((north in obstacles) or (south in obstacles)):
        return True
    if x == 0 and ((north in obstacles) or (south in obstacles)):
        return True
    if y == state.height - 1 and ((east in obstacles) or (west in obstacles)):
        return True
    if y == 0 and ((east in obstacles) or (west in obstacles)):
        return True


def is_in_obstacle_corner(box, state):
    # Check Condition 3. In a corner between two obstacles
    # Check if there is an obstacle pair forming a North-East, North-West, South-East, South-West Obstacle Corner
    (x, y) = box
    north = (x, y + 1)
    south = (x, y - 1)
    east = (x - 1, y)
    west = (x + 1, y)
    if north in state.obstacles and ((east in state.obstacles) or (west in state.obstacles)):
        return True
    if south in state.obstacles and ((east in state.obstacles) or (west in state.obstacles)):
        return True
    return False


def is_against_wall_with_no_storage(box, state):
    # Check Condition 4. Check if box is against a wall of the gameboard that has no storage space
    # Boxes could be on the any of the left, right, top or bottom gameboard walls
    # To do so, check the x and y coordinates of the storage spaces and compare those to the boxes coordinates
    (x, y) = box
    possible_storage_positions = get_storage_spaces(box, state)
    x_list = []
    y_list = []
    for coord in possible_storage_positions:
        x_list.append(coord[0])
        y_list.append(coord[1])
    # If the box is along east wall
    if x == 0 and (0 not in x_list):
        return True
    # If the box is along west wall
    elif x == (state.width - 1) and ((state.width - 1) not in x_list):
        return True
    # if the box is along north wall
    elif y == 0 and (0 not in y_list):
        return True
    # if the box is along south wall
    elif y == (state.height - 1) and ((state.width - 1) not in y_list):
        return True
    else:
        return False


def get_obstacles_num(start, goal, state):
    # Find the number of obstacles including robots to get from the origin to goal
    res = 0
    robots_and_obstacles = state.obstacles.union(set(state.robots))
    for obstacle in robots_and_obstacles:
        north = min(start[1], goal[1])
        south = max(start[1], goal[1])
        west = min(start[0], goal[0])
        east = max(start[0], goal[0])
        if west <= obstacle[0] <= east and north <= obstacle[0] <= south:
            res += 1
    return res


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + (weight * sN.hval)


def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    search_engine = SearchEngine('astar', 'full')
    search_engine.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn,
                              fval_function=(lambda sN: fval_function(sN, weight)))

    start = os.times()[0]
    cost = math.inf
    if initial_state.gval:
        cost = initial_state.gval
    goal = search_engine.search(timebound, costbound=(cost, math.inf, math.inf))
    if goal:
        time_remaining = timebound - (os.times()[0] - start)
        best = goal
        while time_remaining > 0 and not search_engine.open.empty:
            initial_time = os.times()[0]
            if best.gval:
                cost = best.gval
            new_goal = search_engine.search(time_remaining, costbound=(cost, math.inf, math.inf))
            time_remaining = time_remaining - (os.times()[0] - initial_time)  # Update remaining time.
            if new_goal.__lt__(new_goal):
                best = new_goal
        return best
    else:
        return False


def iterative_astar(initial_state, heur_fn, weight=1,
                    timebound=5):  # uses f(n), see how autograder initializes a search line 88
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    search_engine = SearchEngine('astar', 'full')
    search_engine.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    initial_time = os.times()[0]
    cost = math.inf
    if initial_state.gval:
        cost = initial_state.gval
    goal = search_engine.search(timebound, costbound=(cost, math.inf, math.inf))
    if goal:
        time_remaining = timebound - (os.times()[0] - initial_time)
        best = goal
        while time_remaining > 0 and not search_engine.open.empty:
            if best.gval:
                cost = best.gval
            new_goal = search_engine.search(time_remaining, costbound=(cost, math.inf, math.inf))
            if new_goal.__lt__(goal):
                best = new_goal
            time_remaining = time_remaining - (os.times()[0] - initial_time)
        return best
    else:
        return False


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    search_engine = SearchEngine('best_first', 'full')
    search_engine.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)

    initial_time = os.times()[0]
    cost = math.inf
    if initial_state.gval:
        cost = initial_state.gval
    goal = search_engine.search(timebound, costbound=(cost, math.inf, math.inf))
    if goal:
        time_remaining = timebound - (os.times()[0] - initial_time)
        best = goal
        while time_remaining > 0 and not search_engine.open.empty:
            if best.gval:
                cost = best.gval
            new_goal = search_engine.search(time_remaining, costbound=(cost, math.inf, math.inf))
            if new_goal.__lt__(goal):
                best = new_goal
            time_remaining = time_remaining - (os.times()[0] - initial_time)
        return best
    else:
        return False
