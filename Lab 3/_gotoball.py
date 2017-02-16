#     state = 'searching'

#     while True:
#         # find a ball
#         event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
#         opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
#         ball_found = find_ball.find_ball(opencv_image)
#         BallAnnotator.ball = ball_found

#         print(ball_found)

#         print("STATE: " + state)
#         if state == 'searching':
#             await robot.set_head_angle(degrees(0)).wait_for_completed()
#             await robot.set_lift_height(1).wait_for_completed()
#             if ball_found is not None:
#                 x, y, radius = ball_found
#                 # await robot.play_anim_trigger(cozmo.anim.Triggers.CubePounceFake).wait_for_completed()
#                 state = 'lock-in'
#             else:
#                 await robot.turn_in_place(degrees(60)).wait_for_completed()
#         elif state == 'lock-in':
#             if ball_found is None:
#                state = 'searching'
#             else:
#                 x, y, radius = ball_found
#                 dx = 320/2 - x # positive = ball is on left
#                 deg_to_turn = dx / (320.0/2.0) * 30

#                 if deg_to_turn <= 10 and deg_to_turn >= -10:
#                     state = 'drive'
#                 await robot.turn_in_place(degrees(deg_to_turn)).wait_for_completed()
#         elif state == 'drive':
#             if ball_found is not None:
#                 x, y, radius = ball_found

#                 dx = 320/2 - x # positive = ball is on left
#                 deg_to_turn = dx / (320.0/2.0) * 30
#                 if deg_to_turn < -5 or deg_to_turn > 5:
#                     await robot.turn_in_place(degrees(deg_to_turn)).wait_for_completed()

#                 if radius < 20:
#                     await robot.drive_straight(distance_mm(250), speed_mmps(1000)).wait_for_completed()
#                 elif radius < 40:
#                     await robot.drive_straight(distance_mm(100), speed_mmps(500)).wait_for_completed()
#                 elif radius < 90:
#                     await robot.drive_straight(distance_mm(15), speed_mmps(300)).wait_for_completed()
#                 else:
#                     await robot.drive_straight(distance_mm(19), speed_mmps(10)).wait_for_completed()
#                     # await robot.play_anim_trigger(cozmo.anim.Triggers.PopAWheelieInitial).wait_for_completed()
#                     await robot.set_lift_height(0).wait_for_completed()

#                     state = 'done'
#             else:
#                 state = 'searching'
#         elif state == 'done':
#             # await robot.turn_in_place(degrees(180)).wait_for_completed()
#             # await robot.drive_straight(distance_mm(200), speed_mmps(100)).wait_for_completed()
#             # state = 'searching'
#             return