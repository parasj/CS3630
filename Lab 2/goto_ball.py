#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np
import find_ball

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


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

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    try:
        state = 'searching'

        while True:
            # find a ball
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            ball_found = find_ball.find_ball(opencv_image)
            BallAnnotator.ball = ball_found

            print(ball_found)

            print("STATE: " + state)
            if state == 'searching':
                await robot.set_head_angle(degrees(0)).wait_for_completed()
                await robot.set_lift_height(0).wait_for_completed()
                if ball_found is not None:
                    x, y, radius = ball_found
                    await robot.play_anim_trigger(cozmo.anim.Triggers.CubePounceFake).wait_for_completed()
                    state = 'lock-in'
                else:
                    await robot.turn_in_place(degrees(60)).wait_for_completed()
            elif state == 'lock-in':
                if ball_found is None:
                    state = 'searching'
                else:
                    x, y, radius = ball_found
                    dx = 320/2 - x # positive = ball is on left
                    deg_to_turn = dx / (320.0/2.0) * 30
                    
                    if deg_to_turn <= 10 and deg_to_turn >= -10:
                        state = 'drive'
                    await robot.turn_in_place(degrees(deg_to_turn)).wait_for_completed()
            elif state == 'drive':
                if ball_found is not None:
                    x, y, radius = ball_found
                    
                    dx = 320/2 - x # positive = ball is on left
                    deg_to_turn = dx / (320.0/2.0) * 30
                    if deg_to_turn < -5 or deg_to_turn > 5:
                        await robot.turn_in_place(degrees(deg_to_turn)).wait_for_completed()

                    if radius < 20:
                        await robot.drive_straight(distance_mm(250), speed_mmps(1000)).wait_for_completed()
                    elif radius < 40:
                        await robot.drive_straight(distance_mm(100), speed_mmps(500)).wait_for_completed()
                    elif radius < 90:
                        await robot.drive_straight(distance_mm(20), speed_mmps(300)).wait_for_completed()
                    else:
                        await robot.drive_straight(distance_mm(30), speed_mmps(10)).wait_for_completed()
                        await robot.play_anim_trigger(cozmo.anim.Triggers.PopAWheelieInitial).wait_for_completed()
                        state = 'done'
                else:
                    state = 'searching'
            elif state == 'done':
                await robot.turn_in_place(degrees(180)).wait_for_completed()
                await robot.drive_straight(distance_mm(200), speed_mmps(100)).wait_for_completed()
                state = 'searching'


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)