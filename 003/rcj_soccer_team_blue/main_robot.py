import math

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from offense import Offense
from offense2 import Offense2
from defense import Defense
from role import RoleSwitching


class MainRobot(RCJSoccerRobot):
    team = ""
    robot_id = 0
    robot_x, robot_y = 0, 0
    robot_heading = 0
    robot_detect = False
    ball_x, ball_y = 0, 0
    ball_distance = 0
    ball_angle = 0
    teammate1_id = 0
    teammate1_x, teammate1_y = 0, 0
    teammate1_heading = 0
    teammate1_detect = False
    teammate1_ball_x, teammate1_ball_y = 0, 0
    teammate1_ball_distance = 0
    teammate2_id = 0
    teammate2_x, teammate2_y = 0, 0
    teammate2_heading = 0
    teammate2_detect = False
    teammate2_ball_x, teammate2_ball_y = 0, 0
    teammate2_ball_distance = 0
    pred_x = 0
    pred_y = 0
    last_x = 0
    last_y = 0
    pred_constant = 0
    time_limit2 = 2
    timer = 0
    temp_timer = 0

    def __init__(self, robot):
        super().__init__(robot)
        if "B" in self.name:
            self.team = "B"
        else:
            self.team = "Y"
        if int(self.name[1]) == 1:
            self.robot_id = 1
            self.teammate1_id = 2
            self.teammate2_id = 3
        elif int(self.name[1]) == 2:
            self.robot_id = 2
            self.teammate1_id = 1
            self.teammate2_id = 3
        else:
            self.robot_id = 3
            self.teammate1_id = 1
            self.teammate2_id = 2
        self.pred_constant = 50  # changable value for prediction (30-70)

    def run(self):
        offense = Offense()
        offense2 = Offense2()
        defense = Defense()
        role = RoleSwitching(self)

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                sonar_values = self.get_sonar_values()

                team_data = []
                while self.is_new_team_data():
                    team_data.append(self.get_new_team_data())

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    MainRobot.updateRobotData(
                        self, self.get_gps_coordinates(), self.get_compass_heading(), ball_data)
                    self.send_data_to_team(
                        self.player_id * 10, self.ball_x, self.ball_y, self.ball_distance)
                else:
                    MainRobot.updateRobotData(
                        self, self.get_gps_coordinates(), self.get_compass_heading())
                    self.send_data_to_team(self.player_id * 10)

                MainRobot.receiveData(self, team_data)

                MainRobot.updatePrediction(self)

                MainRobot.run_task(self, offense, offense2, defense, role)

                self.last_x, self.last_y = self.ball_x, self.ball_y

                self.send_data_to_team(
                    self.player_id, self.robot_x, self.robot_y, self.robot_heading)

    def run_task(self, offense, offense2, defense, role):
        self.timer += (TIME_STEP / 1000)
        if self.ball_y > 0.76 or self.ball_y < -0.76:
            self.temp_timer = self.timer
        if (self.timer - self.temp_timer) < self.time_limit2:
            if self.team == "B":
                if self.robot_id == 2:
                    defense.run_blue(self, [[0,0],[0,0],[0,0]],1)
                elif self.robot_id == 1:
                    offense2.run_blue(self, [[0,0],[0,0],[0,0]],1)
                else:
                    offense.run_blue(self, [[0,0],[0,0],[0,0]],1)
            else:
                if self.robot_id == 2:
                    defense.run_yellow(self, [[0,0],[0,0],[0,0]],1)
                elif self.robot_id == 1:
                    offense2.run_yellow(self, [[0,0],[0,0],[0,0]],1)
                else:
                    offense.run_yellow(self, [[0,0],[0,0],[0,0]],1)
        else:
            role.run(self, offense, offense2, defense)

    def updateRobotData(self, robot_pos: list, robot_heading: float, ball_data=None):
        self.robot_x, self.robot_y = robot_pos[0], robot_pos[1]
        if self.team == "B":
            self.robot_heading = (-math.degrees(robot_heading)
                                  + 180 + 360) % 360
        else:
            self.robot_heading = (-math.degrees(robot_heading) + 360) % 360
        if ball_data is None:
            self.robot_detect = False
        else:
            self.robot_detect = True
            ball_radian = math.atan2(
                ball_data["direction"][0], ball_data["direction"][1]) + (math.pi / 2)
            if ball_radian < -math.pi:
                ball_radian = ball_radian + (2 * math.pi)
            self.ball_angle = (math.degrees(ball_radian) + 360) % 360
            self.ball_distance = math.pow(1 / ball_data["strength"], 0.5)
            if self.team == "B":
                abs_heading_radian = math.radians(
                    (self.robot_heading + self.ball_angle) % 360)
                x_distance = math.sin(abs_heading_radian) * self.ball_distance
                y_distance = math.cos(abs_heading_radian) * self.ball_distance
                self.ball_x, self.ball_y = self.robot_x - x_distance, self.robot_y - y_distance
            else:
                abs_heading_radian = math.radians(
                    (self.robot_heading + self.ball_angle) % 360)
                x_distance = math.sin(abs_heading_radian) * self.ball_distance
                y_distance = math.cos(abs_heading_radian) * self.ball_distance
                self.ball_x, self.ball_y = self.robot_x + x_distance, self.robot_y + y_distance

    def receiveData(self, team_data):
        if team_data:
            for packet in team_data:
                packet_id = packet["packet_id"]
                packet_value1 = packet["value1"]
                packet_value2 = packet["value2"]
                packet_value3 = packet["value3"]
                if packet_id == self.teammate1_id:
                    self.teammate1_x = packet_value1
                    self.teammate1_y = packet_value2
                    self.teammate1_heading = packet_value3
                elif packet_id == self.teammate2_id:
                    self.teammate2_x = packet_value1
                    self.teammate2_y = packet_value2
                    self.teammate2_heading = packet_value3
                elif packet_id == self.teammate1_id * 10:
                    if packet_value3 > 0:
                        self.teammate1_detect = True
                        self.teammate1_ball_x = packet_value1
                        self.teammate1_ball_y = packet_value2
                        self.teammate1_ball_distance = packet_value3
                    else:
                        self.teammate1_detect = False
                        self.teammate1_ball_distance = math.sqrt(
                            math.pow(self.teammate1_x - self.ball_x, 2) + math.pow(self.teammate1_y - self.ball_y, 2))
                elif packet_id == self.teammate2_id * 10:
                    if packet_value3 > 0:
                        self.teammate2_detect = True
                        self.teammate2_ball_x = packet_value1
                        self.teammate2_ball_y = packet_value2
                        self.teammate2_ball_distance = packet_value3
                    else:
                        self.teammate2_detect = False
                        self.teammate2_ball_distance = math.sqrt(
                            math.pow(self.teammate2_x - self.ball_x, 2) + math.pow(self.teammate2_y - self.ball_y, 2))

        if (self.teammate1_detect or self.teammate2_detect) and not self.robot_detect:
            if self.teammate1_detect and not self.teammate2_detect:
                self.ball_x, self.ball_y = self.teammate1_ball_x, self.teammate1_ball_y
            elif self.teammate2_detect and not self.teammate1_detect:
                self.ball_x, self.ball_y = self.teammate2_ball_x, self.teammate2_ball_y
            else:
                if self.teammate1_ball_distance <= self.teammate2_ball_distance:
                    self.ball_x, self.ball_y = self.teammate1_ball_x, self.teammate1_ball_y
                else:
                    self.ball_x, self.ball_y = self.teammate2_ball_x, self.teammate2_ball_y
            self.ball_distance = math.sqrt(math.pow(
                self.ball_x - self.robot_x, 2) + math.pow(self.ball_y - self.robot_y, 2))
            if self.team == "B":
                self.ball_angle = (math.degrees(math.atan2(
                    self.ball_x - self.robot_x, self.ball_y - self.robot_y)) - self.robot_heading + 180) % 360
            else:
                self.ball_angle = (math.degrees(math.atan2(
                    self.ball_x - self.robot_x, self.ball_y - self.robot_y)) - self.robot_heading + 360) % 360

    def getTeamDetect(self):
        return self.robot_detect or self.teammate1_detect or self.teammate2_detect

    def updatePrediction(self):
        ball_delta_x = self.ball_x - self.last_x
        ball_delta_y = self.ball_y - self.last_y
        self.pred_x = self.ball_x + ball_delta_x * \
            self.ball_distance * self.pred_constant
        self.pred_y = self.ball_y + ball_delta_y * \
            self.ball_distance * self.pred_constant
