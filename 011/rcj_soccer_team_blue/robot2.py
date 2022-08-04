from errno import ESTALE
import math
import utils
import struct
#import geometry
import time
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def readData(self):
        self.heading = self.get_compass_heading()*180/math.pi
        # if(self.name[0] == 'B'):
        #     self.heading = self.heading + 180
        #     if self.heading > 180: self.heading -= 360
        #     if self.heading <-180: self.heading += 360
        self.robot_pos = self.get_gps_coordinates()
        if(self.name[0] == 'Y'):
            self.robot_pos[1] *= -1
            self.robot_pos[0] *= -1
        self.sonar = self.get_sonar_values()
        if self.is_new_ball_data():
            self.isBall = True
            self.ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(self.ball_data['direction'][1], self.ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.0166666/(abs(self.ball_data['direction'][2])/math.sqrt(1 - self.ball_data['direction'][2]**2)))
            self.ball_x =-math.sin((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            self.ball_y = math.cos((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_pos = [self.ball_x, self.ball_y]
        else:
            self.isBall = False
        self.robot_x = self.robot_pos[1]
        self.robot_y = self.robot_pos[0]
        self.behind_ball = [self.ball_x, self.ball_y + 0.2]
    def moveToAngle(self, angle):
        if angle > 180: angle -= 360
        if angle <-180: angle += 360
        if -90 < angle < 90:
            if angle > 40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            elif angle <-40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            else:
                self.right_motor.setVelocity(utils.velocity(10 + angle/5))
                self.left_motor.setVelocity(utils.velocity(10 - angle/5))
        else:
            if angle < 0: angle = -180 - angle
            elif angle > 0: angle =  180 - angle
            if angle > 40:
                self.right_motor.setVelocity(-10)
                self.left_motor.setVelocity(10)
            elif angle <-40:
                self.right_motor.setVelocity(10)
                self.left_motor.setVelocity(-10)
            else:
                self.right_motor.setVelocity(utils.velocity(-10 - angle/5))
                self.left_motor.setVelocity(utils.velocity(-10 + angle/5))
    def move(self, dest, stop=False):
        if(dest[1]-self.robot_pos[1]!=0):
            dest_angle = math.atan2(self.robot_pos[0]-dest[0],dest[1]-self.robot_pos[1])*180/math.pi
        else:
            dest_angle = 0
        angle = self.heading - dest_angle
        if stop and utils.getDistance(dest, self.robot_pos) < 0.1:
            self.stop()
        else:
            self.moveToAngle(angle)
    def stop(self):
        self.right_motor.setVelocity(0)
        self.left_motor.setVelocity(0)
    def sendTeamData(self):
        packet = struct.pack(utils.dataFormat, self.robot_id, self.robot_x, self.robot_y, self.isBall, self.ball_x, self.ball_y)
        self.team_emitter.send(packet)
    def getTeamData(self):
        self.robot_positions[self.robot_id - 1][1] = self.robot_x
        self.robot_positions[self.robot_id - 1][0] = self.robot_y
        while self.is_new_team_data():
            packet = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            unpacked = struct.unpack(utils.dataFormat, packet)
            self.robot_positions[unpacked[0] - 1][1] = unpacked[1]
            self.robot_positions[unpacked[0] - 1][0] = unpacked[2]
            if not self.isBall and unpacked[3]:
                self.ball_x = unpacked[4]
                self.ball_y = unpacked[5]
                self.ball_pos = [self.ball_x, self.ball_y]
                self.behind_ball = [self.ball_x, self.ball_y - 0.2]
                self.isBall = True
        distances = [0, 0, 0]
        distances[1] = utils.getDistance([self.robot_positions[1][0], self.robot_positions[1][1]], self.ball_pos)
        distances[0] = utils.getDistance([self.robot_positions[0][0], self.robot_positions[0][1]], self.ball_pos)
        distances[2] = utils.getDistance([self.robot_positions[2][0], self.robot_positions[2][1]], self.ball_pos)
        if distances[self.robot_id - 1] == max(distances):
        # if self.robot_id == 3:    
            self.gaolKeeper = True
        else:
            self.gaolKeeper = False
        
        if distances[self.robot_id - 1] == min(distances):
        # if self.robot_id == 2:
            self.forward = True
        else:
            self.forward = False
        ######################### Lack Of Progress Detection
        if utils.getDistance(self.ball_pos, self.startBallPos) > 0.1:
            self.startTime = time.time()
            self.startBallPos = self.ball_pos
            self.lackOfProgress = False
        else:
            if time.time() - self.startTime > 4:
                self.lackOfProgress = True
            else:
                self.lackOfProgress = False
    def ocupiedSpot(self, spot):
        for i in range(3):
            if utils.getDistance(spot, self.robot_positions[i]) < 0.08:
                return True
        if utils.getDistance(spot, self.ball_pos) < 0.08:
            return True
        return False
    def guessNutralSpot(self):
        nutralSpots = [
            [ 0, 0],
            [ 0, 0.2],
            [ 0,-0.2],
            [ 0.3, 0.3],
            [-0.3, 0.3],
            [ 0.3,-0.3],
            [-0.3,-0.3],
        ]
        unocupiedNutralSpots = []
        for spot in nutralSpots:
            if not self.ocupiedSpot(spot):
                unocupiedNutralSpots.append(spot)
        nearestSpot = unocupiedNutralSpots[0]
        minDistance = 10
        for spot in unocupiedNutralSpots:
            distance = utils.getDistance(spot, self.ball_pos)
            if distance < minDistance:
                minDistance = distance
                nearestSpot = spot
        return nearestSpot
    def run(self):
        self.ball_x = 0
        self.ball_y = 0
        self.isBall = False
        self.T_Goal = [0, 0.65]
        self.O_Goal = [0, -0.65]
        self.ball_pos = [0, 0]
        self.robot_positions = [[0, 0] , [0, 0] , [0, 0]]
        self.robot_id = int(self.name[1])
        self.gaolKeeper = False
        self.goalKeeper_x = 0
        self.last_ball_pos = self.ball_pos
        self.lackOfProgress = False
        self.startTime = time.time()
        self.startBallPos = [0, 0]
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.waitingForKick = self.get_new_data()['waiting_for_kickoff']
                self.readData()
                self.sendTeamData()
                self.getTeamData()
                if self.waitingForKick:
                    self.stop()
                elif self.lackOfProgress:
                    nearestSpot = self.guessNutralSpot()
                    self.move([nearestSpot[0], nearestSpot[1] - 0.2], stop=True)
                elif self.gaolKeeper:
                    ball_line = utils.Line()
                    ball_line.drawLineWithTwoPoint({'x': self.ball_pos[0], 'y': self.ball_pos[1]}, {'x': self.last_ball_pos[0], 'y': self.last_ball_pos[1]})
                    goal_line = utils.Line(1, 0, -self.T_Goal[1])
                    intersection = ball_line.getIntersectionWithLine(goal_line)
                    if intersection:
                        self.goalKeeper_x = intersection['x']
                    else:
                        self.goalKeeper_x = self.ball_x
                    if self.goalKeeper_x > 0.3: self.goalKeeper_x = 0.3
                    if self.goalKeeper_x <-0.3: self.goalKeeper_x =-0.3
                    self.move([self.goalKeeper_x , self.T_Goal[1]])
                elif self.forward:
                    if(self.ball_y > 0):
                        ball_circle = utils.Circle({'x': self.ball_x, 'y': self.ball_y},0.07)
                        ball_to_goal = utils.Line()
                        ball_to_goal.drawLineWithTwoPoint({'x':self.O_Goal[0], 'y':self.O_Goal[1]},{'x': self.ball_x, 'y': self.ball_y})
                        intersection=ball_circle.getIntersectionWithLine(ball_to_goal)
                        if(len(intersection)==2):
                            x1=utils.getDistance([intersection[0]['x'], intersection[0]['y']],self.O_Goal)
                            x2=utils.getDistance([intersection[1]['x'], intersection[1]['y']],self.O_Goal)
                            if(x1>x2):
                                go_pos=[intersection[0]['x'], intersection[0]['y']]
                            else:
                                go_pos=[intersection[1]['x'], intersection[1]['y']]
                        else:
                            go_pos = go_pos=[intersection['x'], intersection['y']]
                        self.move(go_pos)

                    else:
                        if self.isBall:
                            if(self.ball_pos[1]<self.robot_pos[1]):
                                direction = utils.get_direction(self.ball_data["direction"])
                                if direction == 0:
                                    left_speed = 10
                                    right_speed = 10
                                else:
                                    left_speed = direction * 7
                                    right_speed = direction * -7
                                self.left_motor.setVelocity(left_speed)
                                self.right_motor.setVelocity(right_speed)
                            else:
                                self.stop()
                        else: 
                            self.move(self.T_Goal)

                else:  
                    self.move([self.T_Goal[0],self.T_Goal[1]/1.5])
            else:
                    pass