# rcj_soccer_player controller - ROBOT Y2

# Feel free to import built-in libraries
import math
# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot3(RCJSoccerRobot):
    def __init__(self, robot):
        super(MyRobot3, self).__init__(robot)
        self.isBall = False
        self.ball_data = 0
        self.ball_data = 0
        self.ball_angle = 0
        self.ball_distance = 0
        self.ball_x =0
        self.ball_y = 0
        self.ball_pos = 0
    def readData(self):
        self.heading = self.get_compass_heading()*180/math.pi
        self.robot_pos = self.get_gps_coordinates()
        self.sonar = self.get_sonar_values()
        if self.is_new_ball_data():
            self.isBall = True
            self.ball_data = self.get_new_ball_data()
            self.ball_angle = math.atan2(self.ball_data['direction'][1], self.ball_data['direction'][0])*180/math.pi
            self.ball_distance = abs(0.0166666/(abs(self.ball_data['direction'][2])/math.sqrt(1 - self.ball_data['direction'][2]**2)))
            self.ball_x =-math.sin((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[0]
            self.ball_y = math.cos((self.ball_angle + self.heading)*math.pi/180) * self.ball_distance + self.robot_pos[1]
            self.ball_pos = [self.ball_x, self.ball_y]
        else:
            self.isBall = False
        self.robot_x = self.robot_pos[0]
        self.robot_y = self.robot_pos[1]
        self.behind_ball = [self.ball_x, self.ball_y - 0.1]
    def run(self):
        if self.name[0] == "Y":
            isBackOK2 = 0
            while self.robot.step(TIME_STEP) != -1:
                self.readData()
                if self.isBall == False:
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0,-0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, -0.5])==True):
                            isBackOK2=0
                    gps = self.get_gps_coordinates()
                    send_value = int(gps[1] * 10000)
                    self.send_data_to_team(send_value)
                    continue
                elif self.ball_data['strength'] > 55:
                    try:
                        direction = utils.get_direction(self.ball_data["direction"])
                        if direction == 0:
                            left_speed = 10
                            right_speed = 10
                        else:
                            left_speed = direction * 8
                            right_speed = direction * -8
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                    except: pass
                elif self.ball_y < 0:
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0,-0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, -0.5])==True):
                            isBackOK2=0
                    gps = self.get_gps_coordinates()
                    send_value = int(gps[1] * 10000)
                    self.send_data_to_team(send_value)
                    



                elif self.ball_y < self.robot_y:
                    print("HIIIII")
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0,-0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, -0.5])==True):
                            isBackOK2=0





                elif self.ball_y > 0 :
                    try:
                        direction = utils.get_direction(self.ball_data["direction"])
                        if direction == 0:
                            left_speed = 10
                            right_speed = 10
                        else:
                            left_speed = direction * 8
                            right_speed = direction * -8
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                    except: pass
                    gps = self.get_gps_coordinates()
                    send_value = int(1000000)
                    self.send_data_to_team(send_value)
                elif self.is_new_data():
                    kickoff_data = self.get_new_data()
                    if kickoff_data["waiting_for_kickoff"] == True:
                        utils.stop(self)
        else:
            isBackOK2 = 0
            while self.robot.step(TIME_STEP) != -1:
                self.readData()
                if self.isBall == False:
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0, 0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, 0.5])==True):
                            isBackOK2=0
                    gps = self.get_gps_coordinates()
                    send_value = int(gps[1] * 10000)
                    self.send_data_to_team(send_value)
                    continue
                elif self.ball_data['strength'] > 55:
                    try:
                        direction = utils.get_direction(self.ball_data["direction"])
                        if direction == 0:
                            left_speed = 10
                            right_speed = 10
                        else:
                            left_speed = direction * 8
                            right_speed = direction * -8
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                    except: pass
                elif self.ball_y < 0:
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0, 0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, 0.5])==True):
                            isBackOK2=0
                    gps = self.get_gps_coordinates()
                    send_value = int(gps[1] * 10000)
                    self.send_data_to_team(send_value)
                elif self.ball_y < self.robot_y:
                    if(isBackOK2==0):
                        if(utils.go_pos(self,[0, 0.2])==True):
                            isBackOK2=1
                    elif(isBackOK2==1):
                        if(utils.go_pos(self,[0, 0.5])==True):
                            isBackOK2=0
                elif self.ball_y > 0 :
                    try:
                        direction = utils.get_direction(self.ball_data["direction"])
                        if direction == 0:
                            left_speed = 10
                            right_speed = 10
                        else:
                            left_speed = direction * 8
                            right_speed = direction * -8
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                    except: pass
                    gps = self.get_gps_coordinates()
                    send_value = int(1000000)
                    self.send_data_to_team(send_value)
                elif self.is_new_data():
                    kickoff_data = self.get_new_data()
                    if kickoff_data["waiting_for_kickoff"] == True:
                        utils.stop(self)
         