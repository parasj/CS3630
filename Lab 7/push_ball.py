import asyncio
import sys

import cv2
import numpy as np

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

async def run(robot: cozmo.robot.Robot):
	while True:
		try:
			await robot.set_head_angle(degrees(0)).wait_for_completed()
			await robot.set_lift_height(.55).wait_for_completed()
			await robot.drive_straight(distance_mm(25), speed_mmps(50)).wait_for_completed()
		except KeyboardInterrupt:
			print("")
			print("Exit requested by user")
		except cozmo.RobotBusy as e:
			print(e)

if __name__ == '__main__':
	cozmo.run_program(run)