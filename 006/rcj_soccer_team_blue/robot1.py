# rcj_soccer_player controller - ROBOT Y2

# Feel free to import built-in libraries
import math
import time
# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def __init__(self, robot):
        super(MyRobot1, self).__init__(robot)
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
        self.behind_ball = [self.ball_x, self.ball_y - 0.2]

    def run(self):
        isBackOK = 0
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                if(self.is_new_ball_data()):
                    pass
                self.readData()
                if self.is_new_team_data():
                    y = self.get_new_team_data()
                    y = int(y['robot_id']) / 10000
                    if y < 1:
                        if utils.go_pos(self, [0.25, y + 0.05]) == True:
                            utils.go_angle3(self, 0)
                        
                    else:
                        if(isBackOK==0):
                            if(utils.go_pos(self,[0.3,-0.2])==True):
                                isBackOK=1
                        elif(isBackOK==1):
                            if(utils.go_pos(self,[0.3, 0.2])==True):
                                isBackOK=0

                kickoff_data = self.get_new_data()
                if kickoff_data["waiting_for_kickoff"] == True:
                    utils.stop(self)
                    
                
