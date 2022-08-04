# Import all libraries here
import math
import numpy as np

# Define all constants here

class RoleSwitching:
    # Initialize all instance variables here
    goal_x = 0
    goal_y = 0
    goal2_x = 0
    goal2_y = 0
    off_x = 0
    off_y = 0

    def __init__(self, robot):
        self.team = robot.name[0]

    # Create all your methods here
    def run(self, robot, offense, offense2, defense):
        robot_x, robot_y = robot.robot_x, robot.robot_y
        robot_heading = robot.robot_heading
        ball_x, ball_y = robot.ball_x, robot.ball_y
        ball_angle = robot.ball_angle
        ball_distance = robot.ball_distance
        team_detect = robot.getTeamDetect()

        if team_detect:
            if self.team == "B":
                if robot.ball_y > 0.5 and robot.ball_x > 0:
                    self.goal_x = 0.35
                    self.goal_y = 0.7
                    self.goal2_x = 0.25
                    self.goal2_y = 0.7
                    self.off_x = 0
                    self.off_y = 0
                elif robot.ball_y > 0.5 and robot.ball_x < 0:
                    self.goal_x = -0.35
                    self.goal_y = 0.7
                    self.goal2_x = -0.25
                    self.goal2_y = 0.7
                    self.off_x = 0
                    self.off_y = 0
                else:
                    self.goal_x = 0
                    self.goal_y = 0.7
                    self.goal2_x = 0
                    self.goal2_y = 0.7
                    self.off_x = ball_x
                    self.off_y = ball_y
            else:
                if robot.ball_y < -0.5 and robot.ball_x > 0:
                    self.goal_x = 0.35
                    self.goal_y = -0.7
                    self.goal2_x = 0.25
                    self.goal2_y = -0.7
                    self.off_x = 0
                    self.off_y = 0
                elif robot.ball_y < -0.5 and robot.ball_x < 0:
                    self.goal_x = -0.35
                    self.goal_y = -0.7
                    self.goal2_x = -0.25
                    self.goal2_y = -0.7
                    self.off_x = 0
                    self.off_y = 0
                else:
                    self.goal_x = 0
                    self.goal_y = -0.7
                    self.goal2_x = 0
                    self.goal2_y = -0.7
                    self.off_x = ball_x
                    self.off_y = ball_y
        else:
            if self.team == "B":
                self.goal_x = 0
                self.goal_y = 0.7
                self.goal2_x = 0
                self.goal2_y = 0.7
                self.off_x = 999
                self.off_y = 999
            else:
                self.goal_x = 0
                self.goal_y = -0.7
                self.goal2_x = 0
                self.goal2_y = -0.7
                self.off_x = 999
                self.off_y = 999

        if "1" in robot.name:
            idx = [0, 1, 2]
        elif "2" in robot.name:
            idx = [1, 0, 2]
        else:
            idx = [1, 2, 0]

        team_position = np.array([[robot.robot_x, robot.robot_y], [
            robot.teammate1_x, robot.teammate1_y], [robot.teammate2_x, robot.teammate2_y]])
        robot_distances = np.array([RoleSwitching.getRobotDistance(self, robot.robot_x, robot.robot_y), RoleSwitching.getRobotDistance(
            self, robot.teammate1_x, robot.teammate1_y), RoleSwitching.getRobotDistance(self, robot.teammate2_x, robot.teammate2_y)])

        team_position = team_position[idx]
        robot_distances = robot_distances[idx]

        # robot_distances = np.array([RoleSwitching.getRobotDistance(self, robot.robot_x, robot.robot_y), RoleSwitching.getRobotDistance(self, robot.teammate1_x, robot.teammate1_y), RoleSwitching.getRobotDistance(self, robot.teammate2_x, robot.teammate2_y)])
        robot_distances_id = np.argsort(robot_distances)

        def_bot = robot_distances_id[0] + 1
        if self.off_x == 0 and self.off_y == 0:
            robot_distances2 = np.array([RoleSwitching.getRobotDistance2(self, robot.robot_x, robot.robot_y), RoleSwitching.getRobotDistance2(
                self, robot.teammate1_x, robot.teammate1_y), RoleSwitching.getRobotDistance2(self, robot.teammate2_x, robot.teammate2_y)])
            robot_distances2 = robot_distances2[idx]
            robot_distances2_id = np.argsort(robot_distances2)
            def2_bot = robot_distances2_id[0] + \
                1 if robot_distances2_id[0] != robot_distances_id[0] else robot_distances2_id[1] + 1
        elif self.off_x == 999 and self.off_y == 999:
            def2_bot = (def_bot+1)%3 + 1
        else:
            robot_distances2 = np.array([RoleSwitching.getRobotDistance3(self, robot.robot_x, robot.robot_y), RoleSwitching.getRobotDistance3(
                self, robot.teammate1_x, robot.teammate1_y), RoleSwitching.getRobotDistance3(self, robot.teammate2_x, robot.teammate2_y)])
            robot_distances2 = robot_distances2[idx]
            robot_distances2_id = np.argsort(robot_distances2)
            def2_bot = robot_distances2_id[1] + \
                1 if robot_distances2_id[1] != robot_distances_id[0] else robot_distances2_id[2] + 1

        team_position = team_position[[
            def_bot - 1, def2_bot - 1, 6 - def_bot - def2_bot - 1]]

        # print(F'DEF:{def_bot} OFF2:{def2_bot} OFF:{6-def_bot-def2_bot}')

        # RoleSwitching condition
        if self.team == "B":
            if robot.robot_id == def_bot:
                defense.run_blue(robot, team_position)
            elif robot.robot_id == def2_bot:
                offense2.run_blue(robot, team_position)
            else:
                offense.run_blue(robot, team_position)
        else:
            if robot.robot_id == def_bot:
                defense.run_yellow(robot, team_position)
            elif robot.robot_id == def2_bot:
                offense2.run_yellow(robot, team_position)
            else:
                offense.run_yellow(robot, team_position)

    def getRobotDistance(self, x, y):
        return math.sqrt(math.pow(x - self.goal_x, 2) + math.pow(y - self.goal_y, 2))

    def getRobotDistance2(self, x, y):
        return math.sqrt(math.pow(x - self.goal2_x, 2) + math.pow(y - self.goal2_y, 2))

    def getRobotDistance3(self, x, y):
        return math.sqrt(math.pow(x - self.off_x, 2) + math.pow(y - self.off_y, 2))
