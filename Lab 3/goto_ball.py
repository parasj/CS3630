#!/usr/bin/env python3

# Paras Jain, Connor Lindquist

import sys
import cv2
import numpy as np
import find_ball
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
from cozmosearcher import CozmoSearcher

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

class BallAnnotator(cozmo.annotate.Annotator):
    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


### Constants
SEARCH_SPIN_SPEED = 75
PURSUE_MIN_BALL_SIZE = 20
PURSUE_MAX_BALL_SIZE = 90
PURSUE_MIN_SPIN_SPEED = 0
PURSUE_MAX_SPIN_SPEED = 75
PURSUE_MIN_FORWARD_SPEED = 5
PURSUE_MAX_FORWARD_SPEED = 75




def solve_for_kinematics(ballx, bally, ballr):
    phi1 = 0
    phi2 = 0

    drive_left = 1
    if ballx > 320 / 2:
        drive_left = -1

    ballr_norm = max(PURSUE_MIN_BALL_SIZE, min(PURSUE_MAX_BALL_SIZE, ballr))

    dx = abs(ballx - 320/2)
    ball_size_percentage = 1.0 - float(ballr_norm - PURSUE_MIN_BALL_SIZE) / float(PURSUE_MAX_BALL_SIZE - PURSUE_MIN_BALL_SIZE)

    forward_velocity = ball_size_percentage * (PURSUE_MAX_FORWARD_SPEED - PURSUE_MIN_FORWARD_SPEED) + PURSUE_MIN_FORWARD_SPEED

    phi1 = forward_velocity
    phi2 = forward_velocity

    return (int(phi1), int(phi2))


def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    fsm = CozmoSearcher(robot)
    try:
        while True:
            event = robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            ball_found = find_ball.find_ball(opencv_image)
            BallAnnotator.ball = ball_found

            if fsm.state == "search":
                if fsm.lastSeen == None or fsm.lastSeen[0] < 320 / 2:
                    robot.drive_wheels(-SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED)
                else:
                    robot.drive_wheels(SEARCH_SPIN_SPEED, -SEARCH_SPIN_SPEED)

                if ball_found is not None:
                    robot.drive_wheels(0, 0)
                    fsm.save_ball_location(ball_found)
                    fsm.ball_found()

            elif fsm.state == "pursue": # check ball size and if big then go to atBall
                if ball_found is None:
                    fsm.ball_lost()
                else:
                    fsm.save_ball_location(ball_found)

                    ballx, bally, ballr = ball_found
                    phi1, phi2 = solve_for_kinematics(ballx, bally, ballr)
                    print("driving " + str(phi1) + " " + str(phi2) + " because ball=" + str(ball_found))

                    robot.drive_wheels(phi1, phi2)

            # elif cozmo.state == "touch":
            #     # ram it
            #     robot.set_lift_height(0).wait_for_completed()
    except KeyboardInterrupt:
        print("Exit requested by user")
        robot.drive_wheels(0,0)
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)