# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from typing import Tuple
from Mathematics.Vector2 import Vector2

class MyRobot2(RCJSoccerRobot):
    def __init__(self, robot):
        super().__init__(robot)
        self.prev_time = -1
        self.prev_ball_pos_x = -100
        self.prev_ball_pos_y = -100
        self.robot_pos = 0, 0

    #vrati rychlost motorov
    def get_speed (self, angle: float) -> Tuple[float, float]:
        if angle >= 270 or angle <= 90:    
            if angle >= 345 or angle <= 15:
                left_speed = 10
                right_speed = 10
            elif angle > 180:
                left_speed = -4
                right_speed = 10
            else:
                left_speed = 10
                right_speed = -4
        else:
            if angle >= 165 and angle <= 195:
                left_speed = -10
                right_speed = -10
            elif angle > 180:
                left_speed = 4
                right_speed = -10
            else:
                left_speed = -10
                right_speed = 4
        return left_speed, right_speed    

    def get_ball_vel(self, act_time, ball_pos):
        if (self.prev_time < 0) or (self.prev_ball_pos_x == -100):
            self.prev_time = act_time
            self.prev_ball_pos_x = ball_pos[0]
            self.prev_ball_pos_y = ball_pos[1]
            return 0, 0

        # Calculate the speed of ball and predict its position
        timedel = act_time - self.prev_time
        if timedel <= 0:
            return 0, 0

        ball_vel_x = (ball_pos[1] - self.prev_ball_pos_x) / timedel
        ball_vel_y = (ball_pos[1] - self.prev_ball_pos_y) / timedel
        if abs(ball_vel_x) > 5:
            ball_vel_x = 0
        if abs(ball_vel_y) > 5:
            ball_vel_y = 0

        self.prev_time = act_time
        self.prev_ball_pos_x = ball_pos[0]
        self.prev_ball_pos_y = ball_pos[1]
        return ball_vel_x, ball_vel_y

    def get_new_ball_pos(self, ball_pos, robot_pos, ball_vel_x, ball_vel_y):
        new_ball_x = ball_pos[0]
        new_ball_y = ball_pos[1]
        for _ in range(3):
            rel_x = abs(new_ball_x - robot_pos[0])
            rel_y = abs(new_ball_y - robot_pos[1])
            ball_distance = math.hypot(rel_x, rel_y)
            move_time = ball_distance/(10 * 0.0182)    #self.MAX_SPEED*self.REAL_SPEED)
            new_ball_x = ball_pos[0] + move_time * ball_vel_x
            new_ball_y = ball_pos[1] + move_time * ball_vel_y
        return new_ball_x, new_ball_y

    def get_angle(self, ball_pos: dict, robot_pos: dict, heading: float):
        robot_angle = heading
        
        #print('angle_pos', ball_pos, 'robot_pos', robot_pos)

        angle = math.atan2(
            ball_pos[1] - robot_pos[1],
            ball_pos[0] - robot_pos[0],
        )

        #print('angle_before', math.degrees(angle))

        if angle < 0:
            angle = 2 * math.pi + angle

        #print('angle_after', math.degrees(angle))

        robot_ball_angle = math.degrees(angle + robot_angle)
        robot_angle = math.degrees(robot_angle)
        
        #print('angle1', math.degrees(angle), 'robot_ball_angle1', robot_ball_angle)

        #robot_ball_angle -= 180
        if robot_ball_angle > 360:
            robot_ball_angle -= 360

        #print('angle2', math.degrees(angle), 'robot_ball_angle2', robot_ball_angle)

        return robot_ball_angle, robot_angle

    def is_valid_pos (self, x: float, y: float):
        if x > 0.70 or x < -0.70:
            return False
        if y > 0.60 or y < -0.60:
            return False
        return True

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                goal_pos         = [0.00, -0.90]
                goal_my_pos      = [0.00, 0.90]
                real_goal_my_pos = [0.00, 0.75]
                no_ball_pos      = [0.30, 0.30]
                if self.name.startswith('Y'):
                    goal_pos         = [0.00, 0.90]
                    goal_my_pos      = [0.00, -0.90]
                    real_goal_my_pos = [0.00, -0.75]
                    no_ball_pos      = [-0.30, -0.30]

                team_data = []
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    if not isinstance(team_data, list):
                        team_data = [team_data]
                    # Do something with team data

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                self.robot_pos = list(self.get_gps_coordinates())  # noqa: F841

                #ball_pos = [0, 0]
                ball_data = {}
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                
                ball_pos = list(utils.avg_ball_position(ball_data, self.robot_pos, heading, team_data))

                if ball_pos == [-100, -100]:
                    ball_pos = no_ball_pos

                ball_vel_x, ball_vel_y = self.get_ball_vel(self.robot.getTime(), ball_pos)
                new_ball_x, new_ball_y = self.get_new_ball_pos(ball_pos, self.robot_pos, ball_vel_x, ball_vel_y)
                #print('ball_angle')
                ball_angle, robot_angle = self.get_angle(ball_pos, self.robot_pos, heading)
                
                to_ball_x = ball_pos[0] - self.robot_pos[0]
                to_ball_y = ball_pos[1] - self.robot_pos[1]
                to_ball_dist = math.hypot(to_ball_x, to_ball_y)

                #vypocet miesta odkial ma robo strielat aby strelil loptu do brany
                dx = ball_pos[0] - goal_pos[0]
                dy = ball_pos[1] - goal_pos[1]
                dd = math.hypot(dx, dy)
                dd2 = dd + 0.08
                if dd > 0.01:
                    strike_pos_x = goal_pos[0] + dx * dd2 / dd
                    strike_pos_y = goal_pos[1] + dy * dd2 / dd
                else: 
                    strike_pos_x = ball_pos[0]
                    strike_pos_y = ball_pos[1]
                
                #obchadzanie lopty pri mantineli
                dz_x = ball_pos[0] - real_goal_my_pos[0]
                dz_y = ball_pos[1] - real_goal_my_pos[1] 
                dz = math.hypot( dz_x, dz_y )
                dz2 = dz - 0.08
                if dz > 0.01:
                    striker_near_barrier_x = real_goal_my_pos[0] + dz_x * dz2 / dz
                    striker_near_barrier_y = real_goal_my_pos[1] + dz_y * dz2 / dz
                else:
                    striker_near_barrier_x = ball_pos[0]
                    striker_near_barrier_y = ball_pos[1]

                if not self.is_valid_pos(strike_pos_x, strike_pos_y):
                    strike_pos_x = striker_near_barrier_x
                    strike_pos_y = striker_near_barrier_y
                
                #my goal - ak tlacime loptu do nasej brany tak ju obideme 
                dd1 = math.hypot(goal_my_pos[0] - self.robot_pos[0], goal_my_pos[1] - self.robot_pos[1])
                dd2 = math.hypot(goal_my_pos[0] - ball_pos[0], goal_my_pos[1] - ball_pos[1])
                
                #ochrana proti vlastnemu golu
                avoid_ball = 0
                # robot je dalej od brany ako lopta (tlaci ju do brany)
                if dd1 > dd2 - 0.03:
                    avoid_ball = 1
                    p1_pos = [ball_pos[1] + 0.08, ball_pos[0] + goal_my_pos[0] / 20] 
                    p2_pos = [ball_pos[1] - 0.08, ball_pos[0] + goal_my_pos[0] / 20]
                    p1_dist = math.hypot(p1_pos[0] - self.robot_pos[0], p1_pos[1] - self.robot_pos[1])
                    p2_dist = math.hypot(p2_pos[0] - self.robot_pos[0], p2_pos[1] - self.robot_pos[1])
                    if (p1_dist < p2_dist and p1_pos[0] <= 0.60) or p2_pos[0] < -0.60:
                        strike_pos_x = p1_pos[0]
                        strike_pos_y = p1_pos[1]
                    else:
                        strike_pos_x = p2_pos[0]
                        strike_pos_y = p2_pos[1]
                
                #print('avoid_ball:', avoid_ball)
                #print('strike_pos', [strike_pos_x, strike_pos_y])

                to_strike_dist = math.hypot(strike_pos_x - self.robot_pos[0], strike_pos_y - self.robot_pos[1])
                strike_angle, a1 = self.get_angle([strike_pos_x, strike_pos_y], self.robot_pos, heading)
                #print('goal_angle')
                goal_angle, a1 = self.get_angle(goal_pos, self.robot_pos, heading)

                go_to_ball = 1
                go_to_strike = 0
                go_to_goal = 0

                new_strike_x = new_ball_x + strike_pos_x - ball_pos[0]
                new_strike_y = new_ball_y + strike_pos_y - ball_pos[1]
                if not self.is_valid_pos(new_strike_x, new_strike_y):
                    new_strike_x = ball_pos[0]
                    new_strike_y = ball_pos[1]

                to_strike_dist = math.hypot(new_strike_x - self.robot_pos[0], new_strike_y - self.robot_pos[1])
                #print('new_strike_angle')
                strike_angle, a1 = self.get_angle([new_strike_x, new_strike_y], self.robot_pos, heading)

                if  to_ball_dist <= 0.08:
                    go_to_ball = 1
                
                if go_to_ball > 0:
                    if  to_ball_dist <= 0.1:
                        go_to_goal = 1
                    elif to_strike_dist > 0.05:
                        go_to_strike = 1
           
                if avoid_ball > 0 and go_to_ball > 0:
                    go_to_strike = 1

                go_to_angle = 0

                if go_to_goal > 0:
                    #print('goal')
                    go_to_angle = goal_angle
                elif go_to_strike > 0:
                    #print('strike')
                    go_to_angle = strike_angle
                elif go_to_ball > 0:
                    #print('ball')
                    go_to_angle = ball_angle

                left_speed, right_speed = self.get_speed(go_to_angle)

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                self.send_data_to_team(
                    self.player_id,
                    self.robot_pos,
                    utils.ball_position(ball_data, self.robot_pos, heading)
                )

                #print('ball_pos', ball_pos)
                #print('go_to_angle', go_to_angle)
                ##print(left_speed, right_speed)
                #print(ball_vel_x, ball_vel_y)
                ##print('to_ball_dist', to_ball_dist)
                #print('ball_pos_x, ball_pos_y', ball_pos_x, ball_pos_y)
                #print('heading', heading)
                ##print('ball_angle', ball_angle, 'robot_angle', robot_angle, 'goal_angle', goal_angle)