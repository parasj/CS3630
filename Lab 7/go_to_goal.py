# Paras Jain, Connor Lindquist
# !/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import threading

import cozmo
import cv2
import numpy as np
import sys
from cozmo.util import degrees, distance_mm, speed_mmps, Pose
from numpy.linalg import inv
import find_ball

from ar_markers.hamming.detect import detect_markers
from gui import GUIWindow
from particle_filter import *
from utils import *

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

# marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0, 0, 0, angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6, 10, 0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


async def image_processing(robot):
    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)

    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)

    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        # print("ID =", marker.id);
        # print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers


# calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    marker2d_list = [];

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
        R_2_2p = np.matrix([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        # print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2, 0], R_2p_1p[0, 0])

        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y, 'theta =', yaw)

        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x, y, math.degrees(yaw)))

    return marker2d_list


# compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
                             last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
                             curr_pose.rotation.angle_z.degrees
    print(curr_h)
    dx, dy = rotate_point(curr_x - last_x, curr_y - last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))


class BallAnnotator(cozmo.annotate.Annotator):
    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:
            # double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball, 2)

            # define and display bounding box with params:
            # msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[1] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[2] * 2, BallAnnotator.ball[2] * 2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


# particle filter functionality
class ParticleFilter:
    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):
        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


def beep():
    print("\a")


async def run(robot: cozmo.robot.Robot):
    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    await robot.set_head_angle(degrees(0)).wait_for_completed()
    await robot.set_lift_height(0).wait_for_completed()

    # Start particle filter
    pf = ParticleFilter(grid)

    state = 'searching'
    curr_action = None

    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    '''
    Determine distance
    '''
    # while True:
    #     event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
    #     opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
    #     ball_found = find_ball.find_ball(opencv_image)
    #     BallAnnotator.ball = ball_found
    #     if ball_found is not None:
    #         x, y, radius = ball_found
    #         print(radius)

    while True:
        print(state, "Is picked up: ", robot.is_picked_up, curr_action)

        if abs(robot.head_angle.degrees) > 5:
            await robot.set_head_angle(degrees(0)).wait_for_completed()

        if robot.is_picked_up:  # Begin search agent again -> add particles?
            pf.particles = Particle.create_random(PARTICLE_COUNT, grid)
            state = 'searching'
            curr_action = None
            await robot.set_head_angle(degrees(0)).wait_for_completed()
            await robot.set_lift_height(0).wait_for_completed()
        else:
            # Obtain odometry information
            odom = compute_odometry(robot.pose)
            last_pose = robot.pose

            # Obtain list of currently seen markers and their poses
            markers = await image_processing(robot)
            markerPose = cvt_2Dmarker_measurements(markers)

            # Update the particle filter using above information
            (m_x, m_y, m_h, m_confident) = pf.update(odom, markerPose)

            # Update particle filter GUI for debugging
            gui.show_particles(pf.particles)
            gui.show_mean(m_x, m_y, m_h, m_confident)
            gui.updated.set()

            # positional calculations
            g_x, g_y, g_theta = goal
            dx, dy = (g_x - m_x, g_y - m_y)
            cx, cy = (math.cos(math.radians(robot.pose.rotation.angle_z.degrees)) * .55,
                      math.sin(math.radians(robot.pose.rotation.angle_z.degrees)) * .55)
            dtheta = math.degrees(math.atan2(dy + cx, dx + cy))
            dist = math.sqrt((dx * 25) ** 2 + (dy * 25) ** 2)
            # degrees_to_rotate = dtheta - m_h

            if state == 'searching':
                await robot.drive_wheels(20, -20)
                if m_confident:
                    beep()
                    print("- - - - - - - - - - - -HEADING: ", m_h)
                    await robot.drive_wheels(0, 0)
                    # print("Turning", degrees_to_rotate)
                    state = 'turning'
                    # curr_action = robot.turn_in_place(degrees(degrees_to_rotate))
                    curr_action = robot.turn_in_place(degrees(0))
            elif state == 'turning':
                if curr_action.is_completed:
                    beep()
                    print("Driving", max(dist / 2, 5))
                    state = 'driving'
                    # curr_action = await robot.drive_straight(distance_mm(max(dist / 4, 5)), speed_mmps(75),
                    # should_play_anim=False).wait_for_completed()
                    curr_action = await robot.turn_in_place(degrees(-m_h)).wait_for_completed()
                    curr_action = await robot.turn_in_place(degrees(-90)).wait_for_completed()
                    m_h = -90
            elif state == 'driving':
                print("in driving")
                event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
                opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
                ball_found = find_ball.find_ball(opencv_image)
                BallAnnotator.ball = ball_found
                if ball_found is not None:
                    x, y, radius = ball_found
                    print(x, y, radius)
                    state = 'lock-in'
                else:
                    await robot.turn_in_place(degrees(50)).wait_for_completed()
                    m_h = m_h + 50;
            elif state == 'lock-in':
                print("lock-in")
                event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
                opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
                ball_found = find_ball.find_ball(opencv_image)
                BallAnnotator.ball = ball_found
                if ball_found is not None:
                    x, y, radius = ball_found
                    dx = 320 / 2 - x  # positive = ball is on left
                    deg_to_turn = dx / (320.0 / 2.0) * 30
                    print(deg_to_turn, "deg_to_turn")
                    m_h = m_h + deg_to_turn
                    await robot.turn_in_place(degrees(deg_to_turn)).wait_for_completed()
                    if deg_to_turn <= 10 and deg_to_turn >= -10:
                        state = 'align'
                if ball_found is None:
                    state = "driving"
            elif state == 'align':
                angle = (360 + m_h) % 360
                print("Angle of robot: ", angle)
                distance_to_ball = .0023 * math.pow(radius, 2) - 0.3632 * radius + 16.515
                print("Distance to ball: ", distance_to_ball)
                print("Cozmo at: ", m_x, m_y)
                ball_x = math.cos(math.radians(angle)) * distance_to_ball
                ball_y = math.sin(math.radians(angle)) * distance_to_ball
                ball_all_x = ball_x + m_x
                ball_all_y = ball_y + m_y
                print("X_ball component: ", ball_x, " Y_ball component: ", ball_y)
                print("Position x: ", m_x + ball_x, " Position y: ", m_y + ball_y)

                slopeY = 9.0 - ball_all_y
                slopeX = 26.0 - ball_all_x
                slope = slopeY / slopeX
                dx = ball_all_x - m_x
                gX = ball_all_x - dx
                gY = ball_all_y - dx * slope

                # await robot.turn_in_place(degrees(m_h)).wait_for_completed()
                print("Values: ", slope, dx, gX, gY)
                lat_dist = m_y - gY
                if (lat_dist < 0):
                    print("drive forward", lat_dist)
                    curr_action = await robot.turn_in_place(degrees(-m_h - 90)).wait_for_completed()
                    curr_action = await robot.drive_straight(distance_mm(lat_dist * 25),
                                                             speed_mmps(75)).wait_for_completed()
                else:
                    curr_action = await robot.turn_in_place(degrees(-m_h + 90)).wait_for_completed()
                    curr_action = await robot.drive_straight(distance_mm(lat_dist * 25),
                                                             speed_mmps(75)).wait_for_completed()
                    print("drive backward")

                    # slope = (9 - ball_all_y)/(26- ball_all_x)
                    # b = 9 - (slope * 26)
                    # print("y="+str(slope)+"x+"+str(b))
                    # a = input()
                    # await robot.turn_in_place(cozmo.util.Angle(degrees=(-1 * m_h))).wait_for_completed()
                    # print(last_pose.rotation.angle_z.degrees)
                    # if(float(last_pose) < 0):
                    #     x_dist = math.sin(last_pose) * distance_to_ball
                    #     y_dist = math.cos(last_pose) * distance_to_ball
                    #     print(x_dist, y_dist)


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)


# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):
    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:
            # double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball, 2)

            # define and display bounding box with params:
            # msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[1] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[2] * 2, BallAnnotator.ball[2] * 2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':
    # cozmo thread
    # cozmo_thread = CozmoThread()
    # cozmo_thread.start()
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
    # init
    # grid = CozGrid(Map_filename)
    # gui = GUIWindow(grid)
    # gui.start()
