# rcj_soccer_player controller - ROBOT B1

# Griffins - Goalkeeper

import math
import time

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from Mathematics.Mathematics import Clamp
from Mathematics.Vector2 import Vector2
import utils

class MyRobot1(RCJSoccerRobot):
    def __init__(self, robot):
        super().__init__(robot)

        self.position: Vector2 = Vector2.zero
        self.rotation: float = 0

        self.ballPosition: Vector2 = Vector2.zero
        self.prevBallPosition: Vector2 = Vector2.zero

        GK_BLUE_SPOT: Vector2 = Vector2(0.0, 0.5)
        GK_YELLOW_SPOT: Vector2 = Vector2(0.0, -0.5)

        self.attackSpot: Vector2
        self.polarity = 0
        if self.name.startswith('B'):
            self.attackSpot = GK_BLUE_SPOT
            self.polarity = 1
        if self.name.startswith('Y'):
            self.attackSpot = GK_YELLOW_SPOT
            self.polarity = -1


    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            # Update game data
            self.position = self.get_gps_coordinates()
            self.rotation = self.get_compass_heading()

            teamData = []

            while self.is_new_team_data():
                teamData.append(self.get_new_team_data())

            ballData = {}
            if self.is_new_ball_data():
                ballData = self.get_new_ball_data()
                localBallPosition = utils.ball_position(ballData, self.position, self.rotation)

            
            self.ballPosition = utils.avg_ball_position(ballData, self.position, self.rotation, teamData)
            self.send_data_to_team(
                    self.player_id,
                    self.position,
                    utils.ball_position(ballData, self.position, self.rotation)
                )

            if self.ballPosition.x == -100:
                self.ballPosition = self.prevBallPosition

            self.prevBallPosition = self.ballPosition

            ballYDelta = (self.attackSpot.y * self.polarity) - (self.ballPosition.y * self.polarity)

            if ballYDelta > 0:
                # The ball is in front of the goalkeeper 
            
                if ballYDelta < 0.6:
                    if self.position.y < self.attackSpot.y + 0.09 and self.position.y > self.attackSpot.y - 0.09:
                        if self.rotation < math.radians(345) and self.rotation > math.radians(15):
                            self.RotateTo(math.radians(0), 5)
                        else:
                            if self.position.y < self.attackSpot.y + 0.07 and self.position.y > self.attackSpot.y - 0.07 and self.RotateTo(math.radians(0), 5):
                                pass
                            else:
                                if self.position.x < self.ballPosition.x:
                                    if self.position.y < self.attackSpot.y:
                                        self.SetMotorVelocity(-10, -10 + (abs(self.position.y - self.attackSpot.y)) * 20)
                                    else:
                                        self.SetMotorVelocity(-10 + (abs(self.position.y - self.attackSpot.y)) * 20, -10)
                                else:
                                    if self.position.y < self.attackSpot.y:
                                        self.SetMotorVelocity(10, 10 - (abs(self.position.y - self.attackSpot.y)) * 20)
                                    else:
                                        self.SetMotorVelocity(10 - (abs(self.position.y - self.attackSpot.y)) * 20, 10)
                    else:
                        self.GoToPosition(Vector2(self.position.x, self.attackSpot.y), 10)

                else:
                    if self.position.y < self.attackSpot.y + 0.09 and self.position.y > self.attackSpot.y - 0.09:
                        if self.rotation < math.radians(345) and self.rotation > math.radians(15):
                            self.RotateTo(math.radians(0), 5)
                        else:
                            if self.position.y < self.attackSpot.y + 0.07 and self.position.y > self.attackSpot.y - 0.07 and self.RotateTo(math.radians(0), 5):
                                pass
                            else:
                                if self.position.x < Clamp(self.ballPosition.x, -0.35, 0.35) :
                                    if self.position.y < self.attackSpot.y:
                                        self.SetMotorVelocity(-10, -10 + (abs(self.position.y - self.attackSpot.y)) * 20)
                                    else:
                                        self.SetMotorVelocity(-10 + (abs(self.position.y - self.attackSpot.y)) * 20, -10)
                                else:
                                    if self.position.y < self.attackSpot.y:
                                        self.SetMotorVelocity(10, 10 - (abs(self.position.y - self.attackSpot.y)) * 20)
                                    else:
                                        self.SetMotorVelocity(10 - (abs(self.position.y - self.attackSpot.y)) * 20, 10)
                    else:
                        self.GoToPosition(Vector2(self.position.x, self.attackSpot.y), 10)
            else:
                # The ball is behind the goalkeeper
                self.GoToPosition(Vector2(self.ballPosition.x, 0.7 * self.polarity), 10)


    def Rotate(self, speed):
        # Reversed Positive numbers to be clockwise
        self.SetMotorVelocity(speed, -speed)

    def StopMotors(self):
        self.SetMotorVelocity(0, 0)

    def RotateTo(self, angle, speed) -> bool:
            # This function should be call in a loop
            # Returns True while rotating

            angleDifference = abs(math.degrees(self.rotation) - angle)

            if not (angleDifference >= 345 or angleDifference <= 15):
                if math.degrees(self.rotation)  < angle:
                    if abs(math.degrees(self.rotation) - angle) <= 180:
                        self.Rotate(speed)
                    else:
                        self.Rotate(-speed)
                else:
                    if abs(math.degrees(self.rotation)  - angle) <= 180:
                        self.Rotate(-speed)
                    else:
                        self.Rotate(speed)
                return True
            else:
                self.StopMotors()
                return False

    def GoToPosition(self, position: Vector2, speed) -> bool:
        # This function should be call in a loop
        # Returns True while moving

        positionDifference = self.position - position
        angle = math.degrees((self.position - position).GetAngle())

        if abs(positionDifference.x) >= 0.02 or abs(positionDifference.y) >= 0.02:
            if not (self.RotateTo(angle, speed)):
                self.GoForward(speed)
            return True
        else:
            self.StopMotors()
            return False

    def SetMotorVelocity(self, leftMotor: float, rightMotor: float):
        # Reverse Forward to be positive numbers
        self.left_motor.setVelocity(-leftMotor)
        self.right_motor.setVelocity(-rightMotor)

    def GoForward(self, speed):
        self.SetMotorVelocity(speed, speed)

"""
import math

import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from Mathematics.Vector2 import *

GOALKEEPER_SPOT: Vector2 = Vector2(0, 0.5)

class MyRobot1(RCJSoccerRobot):    
    def run(self):
        # START

        # UPDATE
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    ball_direction = Vector2(ball_data["direction"][0], ball_data["direction"][1])
                    ball_angle = angle_between_vectors(Vector2.north, ball_direction)

                    #print(math.sqrt(1 / ball_data["strength"]))

                heading = self.get_compass_heading()

                angle = math.degrees(heading) + 270

                robot_data = self.get_gps_coordinates()
                robot_position: Vector2 = Vector2(robot_data[0], robot_data[1])
                robot_rotation = (math.degrees(heading) + 270) % 360



                goalkeeper_spot_angle = (angle_between_vectors(Vector2.north, (robot_position - GOALKEEPER_SPOT).Normalized()) + robot_rotation) % 360
                #print(goalkeeper_spot_angle)
                #print(utils.ball_position(ball_data, robot_data, heading))             
                #print(angle)

                print(math.degrees(heading))


    

def get_direction(ball_angle: float) -> int:
    if ball_angle >= 345 or ball_angle <= 15:
        return 0
    return -1 if ball_angle < 180 else 1

def angle_between_vectors(vector1: Vector2, vector2: Vector2) -> float:
    return (math.degrees(math.atan2(vector1.y, vector1.x)) - math.degrees(math.atan2(vector2.y, vector2.x)) + 360) % 360











    def run(self):
        timer: utils.Timer = utils.Timer()

        #self.left_motor.setVelocity(7.068)
        #self.right_motor.setVelocity(-7.068)

        # Update
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                
                # TODO: Get and do something with supervisor data
                #data = self.get_new_data()

                #while self.is_new_team_data():
                    #team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data


                # Get ball data
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    print("Robot does not see the ball")
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
    
                # Get data from sonars
                #sonar_values = self.get_sonar_values()  # noqa: F841


                #print(math.degrees(math.atan(ball_data["direction"][1] / ball_data["direction"][0])))
                # print(ball_data["direction"])

                angle = math.degrees(heading) + 270

                if angle > 360:
                    angle -= 360

                # Set the speed to motors

                ballAngle = math.degrees(math.atan2(0, 1)) - math.degrees(math.atan2(ball_data["direction"][1], ball_data["direction"][0]))

                if ballAngle < 0:
                    ballAngle += 360

                ballAngle += 360 - angle

                if ballAngle > 360:
                    ballAngle -= 360

                ballAngle = 360 - ballAngle

                #print(1 / math.sqrt(ball_data["strength"]))
                #print(angle)

                if angle > 85 and angle < 95:
                    print(timer.GetTime())
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    return

                elif timer.GetTime() > 4:
                    self.left_motor.setVelocity(2.5)
                    self.right_motor.setVelocity(2.5)


                # Send message to team robots
                self.send_data_to_team(self.player_id)
    """
            