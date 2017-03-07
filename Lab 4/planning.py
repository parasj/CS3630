
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


def grid_dist(a, b):
    ax, ay = a
    bx, by = b
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

def c(start):
    return (int(round(start[0])), int(round(start[1])))


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

def updateGrid(robot: cozmo.robot.Robot, grid: CozGrid, cubes):
    try:
        cubes = list(robot.world.visible_objects)
    except asyncio.TimeoutError:
        return

    for cube in cubes:
        if cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
            if 1 not in cubes:
                ox, oy = poseToGrid(cube.pose)

                ox = int(round(ox))
                oy = int(round(oy))

                grid.addObstacle((ox, oy))
                grid.addObstacle((ox - 1, oy - 1))
                grid.addObstacle((ox - 1, oy + 1))
                grid.addObstacle((ox + 1, oy - 1))
                grid.addObstacle((ox + 1, oy + 1))
                grid.addObstacle((ox, oy - 1))
                grid.addObstacle((ox, oy + 1))
                grid.addObstacle((ox - 1, oy))
                grid.addObstacle((ox + 1, oy))
                grid.addObstacle((ox - 2, oy - 2))
                grid.addObstacle((ox - 2, oy + 2))
                grid.addObstacle((ox + 2, oy - 2))
                grid.addObstacle((ox + 2, oy + 2))

                grid.addGoal((ox, oy))
                cube.set_lights(cozmo.lights.green_light.flash())

        else: # update obstacle
            print("found obstacle at", str(poseToGrid(cube.pose)), str(cube))
            ox, oy = poseToGrid(cube.pose)
            ox = int(round(ox))
            oy = int(round(oy))

            grid.addObstacle((ox, oy))
            grid.addObstacle((ox - 1, oy - 1))
            grid.addObstacle((ox - 1, oy + 1))
            grid.addObstacle((ox + 1, oy - 1))
            grid.addObstacle((ox + 1, oy + 1))
            grid.addObstacle((ox, oy - 1))
            grid.addObstacle((ox, oy + 1))
            grid.addObstacle((ox - 1, oy))
            grid.addObstacle((ox + 1, oy))

            cube.set_lights(cozmo.lights.red_light.flash())

ix = 0
iy = 0

def poseToGrid(pose: cozmo.util.Pose):
    global ix, iy

    pos = pose.position
    x = ((pos.x - ix) / 25.6) + 3
    y = ((pos.y - iy) / 25.6) + 2
    return x, y


def init(robot: cozmo.robot.Robot):
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()

def robot_go_to(robot: cozmo.robot.Robot, grid: CozGrid, goal):
    oldGoals = grid.getGoals()

    start = c(poseToGrid(robot.pose))
    goal = c(goal)
    path, visited = astarImpl(heuristic, start, goal, grid)

    # update viz
    grid.clearStart()
    grid.clearGoals()
    grid.addGoal(goal)
    grid.setStart(start)
    grid.setPath(path)


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

    gx = 0
    gy = 0

    print("Initial calibration ", str((ix, iy)))

    cubes = {}

    grid.setStart((3,2))
    while not stopevent.is_set():
        print("-- ", state, " --")
        updateGrid(robot, grid, cubes)

        if state == "go_to_center":
            if len(grid.getGoals()) == 0:
                if grid_dist(poseToGrid(robot.pose), (13, 9)) < 1:
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
                try:
                    cubes = list(robot.world.visible_objects)
                except asyncio.TimeoutError:
                    state = "go_to_center"
                    continue

                for cube in cubes:
                    if cube.object_id == robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id:
                        gx, gy = poseToGrid(cube.pose)
                        z = cube.pose.rotation.angle_z.degrees

                        gx = gx + 3 * math.cos(math.radians(z + 180))
                        gy = gy + 3 * math.sin(math.radians(z + 180))

                        gx, gy = c((gx, gy))

                if grid_dist(poseToGrid(robot.pose), (gx, gy)) < 1:
                    state = "orient"
                else:
                    robot_go_to(robot, grid, (gx, gy))

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

