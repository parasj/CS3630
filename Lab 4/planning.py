
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

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    goal = grid.getGoals()[0]

    front = PriorityQueue()
    front.put((0 + heuristic(grid.getStart(), goal), 0, (grid.getStart(), [grid.getStart()])))
    grid.addVisited(grid.getStart())

    costs = {}
    costs[grid.getStart()] = 0

    while not front.empty():
        node = front.get()
        cost, counter, data = node
        cell, path = data

        grid.addVisited(cell)
        if cell == goal:
            grid.setPath(path)
            return path
        else:
            for neighborWithCost in grid.getNeighbors(cell):
                neighbor, cost = neighborWithCost
                newCost = costs[cell] + cost
                if neighbor not in costs or newCost < costs[neighbor]:
                    costs[neighbor] = newCost
                    priority = newCost + heuristic(neighbor, goal)
                    newpath = path[:]
                    newpath.append(neighbor)
                    front.put((priority, counter + 1, (neighbor, newpath)))
    return [grid.getStart()]


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    currx, curry = current
    goalx, goaly = goal
    return pow(pow(goaly - curry, 2) + pow(goalx - currx, 2), 0.5)

def updateGrid(robot: cozmo.robot.Robot, grid: CozGrid):
    try:
        cubes = list(robot.world.visible_objects)
    except asyncio.TimeoutError:
        return

    for cube in cubes:
        if cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
            grid.clearGoals()
            print("found cube1, marking goal", poseToGridCube(cube.pose), str(cube))
            grid.addGoal(poseToGridCube(cube.pose))
            cube.set_lights(cozmo.lights.green_light.flash())
        else: # update obstacle
            print("found obstacle at", str(poseToGridCube(cube.pose)), str(cube))
            grid.addObstacle(poseToGridCube(cube.pose))
            cube.set_lights(cozmo.lights.red_light.flash())

ix = 0
iy = 0

def poseToGrid(pose: cozmo.util.Pose):
    global ix, iy

    pos = pose.position
    x = ((pos.x - ix) / 25.6) + 3
    y = ((pos.y - iy) / 25.6) + 2
    return x, y

def poseToGridCube(pose: cozmo.util.Pose):
    global ix, iy

    pos = pose.position
    x = ((pos.x - ix) / 25.6)
    y = ((pos.y - iy) / 25.6)
    return x, y


def init(robot: cozmo.robot.Robot):
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()


def in_center(robot: cozmo.robot.Robot):
    x,y = poseToGrid(robot.pose)
    return abs(x-13) + abs(y-9) < 2

def robot_go_to(robot: cozmo.robot.Robot, grid, pos):
    oldGoals = grid.getGoals()

    grid.setStart(poseToGrid(robot.pose))
    grid.clearGoals()
    grid.addGoal(pos)

    path = astar(grid, heuristic)

    if len(path) > 1:
        print("Going to " + str(path[1]))
        nx, ny = path[1]
        rx, ry = poseToGrid(robot.pose)
        rz = robot.pose.rotation.angle_z

        dx = nx - rx
        dy = ny - ry

        rotd = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if dx == 0:
            dx = 0.0001
        rotz = math.degrees(math.atan(dy / dx))

        robot.turn_in_place(degrees(rotz) - rz).wait_for_completed()
        robot.drive_straight(distance_mm(rotd * 25.6), speed_mmps(25)).wait_for_completed()
    
    grid.clearGoals()
    for goal in oldGoals:
        grid.addGoal(goal)

def at_cube():
    if len(grid.getGoals()) > 0:
        gx, gy = grid.getGoals()[0]
        x, y = poseToGrid(robot.pose)
        return abs(x - gx) + abs(y - gy) < 2
    return False

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment document for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """


    global grid, stopevent, ix, iy

    state = "go_to_center"
    init(robot)

    ix = robot.pose.position.x
    iy = robot.pose.position.y

    grid.setStart((3,2))
    while not stopevent.is_set():
        updateGrid(robot, grid)

        if state == "go_to_center":
            if len(grid.getGoals()) == 0:
                if in_center(robot):
                    state = "search"
                else:
                    robot_go_to(robot, grid, (13, 9))
            else:
                state = "drive"

        elif state == "drive":
            goals = grid.getGoals()
            if len(goals) < 1:
                state = "go_to_center"
            else:
                if at_cube():
                    state = "orient"
                else:
                    robot_go_to(robot, grid, goals[0])

        elif state == "search":
            if len(grid.getGoals()) == 0:
                robot.turn_in_place(degrees(30)).wait_for_completed()
            else:
                state = "drive"





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

