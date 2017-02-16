#!/usr/bin/env python3

# Paras Jain, Connor Lindquist

import sys

import cozmo
import cv2
import numpy as np
from cozmo.util import degrees

import find_ball
from cozmosearcher import CozmoSearcher

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


class StateAnnotator(cozmo.annotate.Annotator):
    state = "init"

    def apply(self, image, scale):
        if StateAnnotator.state is not None:
            d = ImageDraw.Draw(image)
            bounds = (0, 0, image.width, image.height)
            text = cozmo.annotate.ImageText('STATE %s            ' % StateAnnotator.state, color='red')
            text.render(d, bounds)


class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='yellow')
        text.render(d, bounds)


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


### Constants
SEARCH_SPIN_SPEED = 50
PURSUE_MIN_BALL_SIZE = 20
PURSUE_MAX_BALL_SIZE = 90
PURSUE_MIN_SPIN_SPEED = 0
PURSUE_MAX_SPIN_SPEED = 150
PURSUE_MIN_FORWARD_SPEED = 10
PURSUE_MAX_FORWARD_SPEED = 150


def solve_for_kinematics(ballx, bally, ballr):
    # phi1 = 0
    # phi2 = 0
    # drive_left = 1
    # if ballx > 320 / 2:
    #     drive_left = -1
    # ballr_norm = max(PURSUE_MIN_BALL_SIZE, min(PURSUE_MAX_BALL_SIZE, ballr))
    # ball_size_percentage = 1.0 - float(ballr_norm - PURSUE_MIN_BALL_SIZE) / float(PURSUE_MAX_BALL_SIZE - PURSUE_MIN_BALL_SIZE)
    # forward_velocity = ball_size_percentage * (PURSUE_MAX_FORWARD_SPEED - PURSUE_MIN_FORWARD_SPEED) + PURSUE_MIN_FORWARD_SPEED
    # angular_velocity = PURSUE_MIN_SPIN_SPEED + float(dx) / float(320 / 2) * (PURSUE_MAX_SPIN_SPEED - PURSUE_MIN_SPIN_SPEED)
    # phi1 = forward_velocity + -1 * drive_left * angular_velocity
    # phi2 = forward_velocity + drive_left * angular_velocity

    dx = ballx - 320 / 2

    phi1 = dx / 320 * 200 + 200
    phi2 = -dx / 320 * 200 + 200

    return (int(phi1), int(phi2))


def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)
    robot.world.image_annotator.add_annotator('state', StateAnnotator)

    fsm = CozmoSearcher(robot)
    try:
        samples_missing_ball = 0
        samples_big_ball = 0

        while True:
            StateAnnotator.state = fsm.state

            event = robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            ball_found = find_ball.find_ball(opencv_image)
            BallAnnotator.ball = ball_found

            if fsm.state == "search":
                if fsm.lastSeen == None or fsm.lastSeen[0] > 320 / 2:
                    robot.drive_wheels(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED)
                else:
                    robot.drive_wheels(-SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED)

                if ball_found is not None:
                    samples_missing_ball = 0
                    samples_big_ball = 0
                    robot.drive_wheels(0, 0)
                    fsm.save_ball_location(ball_found)
                    fsm.ball_found()

            elif fsm.state == "pursue":  # check ball size and if big then go to atBall
                if ball_found is None:
                    samples_missing_ball += 1
                    samples_big_ball = 0

                    if samples_missing_ball > 10:
                        robot.drive_wheels(0, 0)
                        fsm.ball_lost()
                elif ball_found[2] > 90 and abs(ball_found[0] - 320 / 2) < 320 / 4:
                    samples_missing_ball = 0
                    samples_big_ball += 1

                    robot.drive_wheels(0, 0)

                    if samples_big_ball > 3:
                        fsm.at_ball()
                else:
                    samples_missing_ball = 0
                    samples_big_ball = 0

                    fsm.save_ball_location(ball_found)

                    ballx, bally, ballr = ball_found
                    phi1, phi2 = solve_for_kinematics(ballx, bally, ballr)
                    robot.drive_wheels(phi1, phi2)

            elif fsm.state == "touch":
                robot.set_lift_height(0).wait_for_completed()
                robot.set_lift_height(1).wait_for_completed()
                robot.set_lift_height(0).wait_for_completed()
                robot.play_anim(name="ID_pokedB").wait_for_completed()
                fsm.finish()

            elif fsm.state == "done":
                robot.set_lift_height(0).wait_for_completed()
                robot.set_head_angle(degrees(30)).wait_for_completed()

    except KeyboardInterrupt:
        print("Exit requested by user")
        robot.drive_wheels(0, 0)
    except cozmo.RobotBusy as e:
        print(e)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
