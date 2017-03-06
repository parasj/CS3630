
#author1:
#author2:

import asyncio
from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, distance_mm

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
            return
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
    cubes = []

    try:
        cubes = list(robot.world.visible_objects)
    except asyncio.TimeoutError:
        print("Didn't find a cube")
        return

    for cube in cubes:
        if len(grid.getGoals()) == 0 and cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
            print("found cube1, marking goal", poseToGrid(cube.pose), str(cube))
            grid.addGoal(poseToGrid(cube.pose))
            grid.addObstacle(poseToGrid(cube.pose))
            cube.set_lights(cozmo.lights.green_light.flash())
        else: # update obstacle
            print("found obstacle at", str(poseToGrid(cube.pose)), str(cube))
            grid.addObstacle(poseToGrid(cube.pose))
            cube.set_lights(cozmo.lights.red_light.flash())



def poseToGrid(pose: cozmo.util.Pose):
    pos = pose.position()
    x = pos.x / 25
    y = pos.y / 25
    return x, y

def init(robot: cozmo.robot.Robot):
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()


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

    state = "go_to_center"
    currentPos = (0,0,0)

    init(robot)

    while not stopevent.is_set():
        if state == "go_to_center":
            updateGrid(robot, grid)
            if len(grid.getGoals()) == 0:
                print("Didn't find goal cube")
                if in_center(robot):
                    state = "search"
                else:
                    robot_go_to(robot, (13, 9))
            else:
                state = "drive"
                print("drive")

        elif state == "drive":
            cubes = list(robot.world.visible_objects)
            if cubes is not None and len(cubes) > 0:
                for cube in cubes:
                    x = world.light_cubes[cozmo.objects.LightCube1Id].x/25
                    y = world.light_cubes[cozmo.objects.LightCube1Id].y/25
                    if world.light_cubes[cozmo.objects.LightCube1Id].object_id == 1:
                        if grid.getGoals is  None:
                            grid.addGoal(self.grid, (x,y))
                    else:
                        grid.addObstacle(self.grid, (x,y))
            astar(self.grid, heuristic)

        elif state == "search":
            updateGrid(robot, grid)
            if len(grid.getGoals()) == 0:
                print("Didn't find goal cube")
                robot.drive_wheels(10, -10)
            else:
                state = "drive"
                print("Drive")





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

