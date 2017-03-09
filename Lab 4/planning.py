
#author1: Paras Jain
#author2: Connor Lindquist

import asyncio
from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

# State globals
ix = 0 # initial x position
iy = 0 # initial y position
grid_scale = 25.6

# Utility functions
def grid_dist(a, b):
    ax, ay = a
    bx, by = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def c(start):
    return (int(round(start[0])), int(round(start[1])))


def poseToGrid(pose: cozmo.util.Pose):
    return c(poseToGridRaw(pose))

def poseToGridRaw(pose: cozmo.util.Pose):
    pos = pose.position
    x = ((pos.x - ix) / grid_scale) + 3
    y = ((pos.y - iy) / grid_scale) + 2
    return (x, y)


# A*
def astar(grid: CozGrid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start = grid.getStart()
    goal = grid.getGoals()[0]
    obstacles = grid._obstacles

    path, visited = astarImpl(heuristic, start, goal, obstacles)

    grid.setPath(path)
    for v in visited:
        grid.addVisited(v)


def astarImpl(heuristic, start, goal, grid: CozGrid):
    start = (int(round(start[0])), int(round(start[1])))
    goal = (int(round(goal[0])), int(round(goal[1])))

    visited = [start]
    costs = {}
    costs[start] = 0

    front = PriorityQueue()
    front.put((0 + heuristic(start, goal), 0, (start, [start])))



    while not front.empty():
        node = front.get()
        cost, counter, data = node
        cell, path = data

        visited.append(cell)
        if grid_dist(cell, goal) < 1:
            return path, visited
        else:
            for neighborWithCost in grid.getNeighbors(cell):
                neighbor, cost = neighborWithCost
                if neighbor in grid._obstacles:
                    print(neighbor)
                newCost = costs[cell] + cost
                if neighbor not in costs or newCost < costs[neighbor]:
                    costs[neighbor] = newCost
                    priority = newCost + heuristic(neighbor, goal)
                    newpath = path[:]
                    newpath.append(neighbor)
                    front.put((priority, counter + 1, (neighbor, newpath)))

    return [start], visited


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    currx, curry = current
    goalx, goaly = goal
    return pow(pow(goaly - curry, 2) + pow(goalx - currx, 2), 0.5)


# add an obstacle to the map
def add_obstacle_to_grid(grid: CozGrid, obstacle):
    ox, oy = c(obstacle)
    for dx in [-2, -1, 0, 1, 2]:
        for dy in [-2, -1, 0, 1, 2]:
            grid.addObstacle((ox + dx, oy + dy))

def add_goal_obstacle_to_grid(grid: CozGrid, obstacle, cube):
    print("Goal cube 1 added")
    ox, oy = c(obstacle)
    oz = 180 + cube.pose.rotation.angle_z.degrees
    for dx in [-2, -1, 0, 1, 2]:
        for dy in [-2, -1, 0, 1, 2]:
            grid.addObstacle((ox + dx, oy + dy))
    if oz > 60 and oz < 120: #set goal above
        grid.addGoal((ox, oy + 3))
        return(ox, oy + 3)
    elif oz > 150 and oz < 210:
        grid.addGoal((ox - 3, oy))
        return (ox - 3, oy)
    elif oz > 240 and oz < 300:
        grid.addGoal((ox, oy - 3))
        return (ox, oy -3)
    elif (oz > 330 and oz <= 360) or (0 <= oz and oz < 30):
        grid.addGoal((ox + 3, oy))
        return (ox + 3, oy)

    # grid.addGoal((ox, oy))
# find any new cubes and place them into the map if necessary. returns true if any cubes were added
def find_and_update_cubes(robot: cozmo.robot.Robot, seen_cubes: dict, grid: CozGrid, oldGoalPosition):
    gP = oldGoalPosition
    try:
        cubes = list(robot.world.visible_objects)
    except asyncio.TimeoutError:
        print("find_and_update_cubes: timeout error")
        return False, seen_cubes
    else:
        changed = False
        for cube in cubes:
            is_cube_1 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
            is_cube_2 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube2Id].object_id
            is_cube_3 = cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube3Id].object_id
            if 1 not in seen_cubes and is_cube_1:
                print("I found cube 1 at " + str(poseToGridRaw(cube.pose)))
                seen_cubes[1] = cube
                changed = True
                gP = add_goal_obstacle_to_grid(grid, poseToGridRaw(cube.pose), seen_cubes[1])
            elif 2 not in seen_cubes and is_cube_2:
                print("I found cube 2 at " + str(poseToGridRaw(cube.pose)))
                seen_cubes[2] = cube
                changed = True
                add_obstacle_to_grid(grid, poseToGridRaw(cube.pose))
            elif 3 not in seen_cubes and is_cube_3:
                print("I found cube 3 at " + str(poseToGridRaw(cube.pose)))
                seen_cubes[3] = cube
                changed = True
                add_obstacle_to_grid(grid, poseToGridRaw(cube.pose))
        return changed, seen_cubes, gP


# straight line drive to
def drive_to(robot: cozmo.robot.Robot, pos):
    nx, ny = pos

    rx, ry = poseToGridRaw(robot.pose)
    dx = nx - rx
    dy = ny - ry

    rz = robot.pose.rotation.angle_z
    rotz = math.degrees(math.atan2(dy, dx))
    robot.turn_in_place(degrees(rotz) - rz).wait_for_completed()

    rx, ry = poseToGridRaw(robot.pose)
    dx = nx - rx
    dy = ny - ry

    rotd = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    robot.drive_straight(distance_mm(rotd * 25.6), speed_mmps(25)).wait_for_completed()


def init(robot: cozmo.robot.Robot):
    global ix, iy
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    ix = robot.pose.position.x
    iy = robot.pose.position.y


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment document for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    state = "stopped"
    init(robot)
    print("Initial calibration ", str((ix, iy)))

    cubes = {}
    goalPosition = None
    path = []
    path_pos = -1
    grid.setStart((2,2))

    while not stopevent.is_set():
        cubes_changed, cubes, goalPosition = find_and_update_cubes(robot, cubes, grid, goalPosition)
        if cubes_changed:
            state = "stopped"

        grid.setStart(poseToGrid(robot.pose))
        print("-- ", state, " --")

        if state == "stopped":
            if 1 not in cubes:
                path, visited = astarImpl(heuristic, grid.getStart(), (13, 9), grid)
                grid.setPath(path)
                grid.clearVisited()
                for v in visited:
                    grid.addVisited(v)
                path_pos = 0
                grid.clearGoals()
                grid.addGoal((13, 9))
                print("Plotted path for center")
                state = "go_to_center"
            else:
                dest = goalPosition
                path, visited = astarImpl(heuristic, grid.getStart(), dest, grid) # todo choose right side of cube
                grid.setPath(path)
                grid.clearVisited()
                for v in visited:
                    grid.addVisited(v)
                path_pos = 0
                grid.clearGoals()
                grid.addGoal(dest)
                print("Plotted path for cube 1")
                state = "drive"

        elif state == "go_to_center":
            if 1 not in cubes:
                if grid_dist(poseToGridRaw(robot.pose), (13, 9)) < 1.41:
                    state = "search"
                elif path_pos == len(path):
                    print("Huh... out of path options but I still am not in the center!")
                    state = "stopped"
                else:
                    print("go_to_center driving from " + str(poseToGridRaw(robot.pose)) + " to " + str(path[path_pos]))
                    drive_to(robot, path[path_pos])
                    path_pos += 1
            else:
                state = "stopped"

        elif state == "search":
            if 1 not in cubes:
                robot.turn_in_place(degrees(30)).wait_for_completed()
            else:
                state = "drive"

        elif state == "drive":
            # None
            # if 1 in cubes:
            #     print(cubes[1].pose.rotation.angle_z.degrees)
            # if 1 in cubes:
            #     if toGoal > -1:
            #         drive_to(robot, path[toGoal])
            #         toGoal += 1
            # else:
            #     state = "stopped"
            print("In drive, goal is: ", goalPosition)






######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

