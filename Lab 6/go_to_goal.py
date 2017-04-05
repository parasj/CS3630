#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import threading

import cozmo
import cv2
import numpy as np
from cozmo.util import degrees, distance_mm, speed_mmps
from numpy.linalg import inv

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
        print('x =', x, 'y =', y, 'theta =', yaw)

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

    dx, dy = rotate_point(curr_x - last_x, curr_y - last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))


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


async def run(robot: cozmo.robot.Robot):
    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    await robot.set_head_angle(degrees(0)).wait_for_completed()
    await robot.set_lift_height(0).wait_for_completed()

    while True:
        # Start particle filter
        pf = ParticleFilter(grid)

        # Obtain odometry information
        odom = compute_odometry(robot.pose)

        # Obtain list of currently seen markers and their poses
        markers = await image_processing(robot)
        markerPose = cvt_2Dmarker_measurements(markers)
        # cv2.waitKey(1)

        # Update the particle filter using above information
        (m_x, m_y, m_h, m_confident) = pf.update(odom, markerPose)

        # Update particle filter GUI for debugging
        gui.show_particles(pf.particles)
        gui.show_mean(m_x, m_y, m_h, m_confident)
        gui.updated.set()

        print(str((m_x, m_y, m_h, m_confident)), markerPose)

        # Determine COZMO actions based on state of localization
        if m_confident:
            await robot.drive_wheels(0, 0)

            # Have robot drive to goal, play animation then stand still
            g_x, g_y, g_theta = goal

            dx = m_x - g_x
            dy = m_y = g_y
            dist = math.sqrt(dx ** 2 + dy ** 2) * 25
            dtheta = math.degrees(math.atan2(dy, dx))

            degrees_to_rotate = dtheta - m_h

            await robot.turn_in_place(degrees(degrees_to_rotate)).wait_for_completed()
            await robot.drive_straight(distance_mm(dist * 0.75), speed_mmps(dist * 0.5)).wait_for_completed()
            await robot.say_text("Confident", use_cozmo_voice=False, duration_scalar=0.5, voice_pitch=1).wait_for_completed()
        else:
            print("Turning")
            if len(markerPose) > 0:
                await robot.drive_wheels(0, 0)
            else:
                await robot.drive_wheels(10, -10)




        # Make robust to kidnapping
        # Begin searching again
        if robot.is_picked_up: # Begin search agent again -> add particles?
            await robot.play_anim_trigger(cozmo.anim.Triggers.ReactToPickup).waitForCompleted()


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':
    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()
