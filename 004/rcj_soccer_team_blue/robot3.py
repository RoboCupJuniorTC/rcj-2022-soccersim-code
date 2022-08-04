# rcj_soccer_player controller - ROBOT B3

# Feel free to import built-in libraries
import math  # noqa: F401
from Mathematics.Vector2 import Vector2

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot3(RCJSoccerRobot):
    def __init__(self, robot):
        super().__init__(robot)
        # Constants definition
        self.robot_pos = 0, 0
        self.avoid_time = -1000
        self.kickoff_time = -1000
        self.waited_for_kickoff = True
        self.FIELD = ((-0.65, 0.65), (-0.85, 0.85))
        self.GOAL = 0, 0.85 if self.name.startswith('Y') else -0.85
        self.REAL_SPEED = 0.0182
        self.MAX_SPEED = 10
        self.SHOOT_DIST = 0.08
        self.MAX_DRIVE_DIST = 0.2
        self.AVOID_EL = 1/4

    def run(self):
        time = 0
        prev_ball_pos = [-100, -100]
        target_x = target_y = 0
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                act_time = self.robot.getTime()
                team_data = []

                # If the robot kicks off, wait, and then shoot
                if self.waited_for_kickoff:
                    self.kickoff_time = act_time
                self.waited_for_kickoff = data["waiting_for_kickoff"]
                if act_time-self.kickoff_time <= 0.1:
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                while self.is_new_team_data():
                    team_data.append(self.get_new_team_data())  # noqa: F841

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                self.robot_pos = list(self.get_gps_coordinates())  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                ball_data = {}
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()

                ball_pos = list(utils.avg_ball_position(ball_data, self.robot_pos, heading, team_data))

                if ball_pos == [-100, -100]:
                    ball_pos = [0, 0]

                if prev_ball_pos[0] == -100:
                    prev_ball_pos = ball_pos.copy()

                # Calculate the speed of ball and predict its position
                timedel = act_time-time
                if timedel == 0:
                    timedel = 1/1000
                time = act_time
                ball_x_vel = (prev_ball_pos[0]-ball_pos[0])/timedel
                ball_y_vel = (prev_ball_pos[1]-ball_pos[1])/timedel
                if abs(ball_x_vel) > 5:
                    ball_x_vel = 0
                if abs(ball_y_vel) > 5:
                    ball_y_vel = 0
                prev_ball_pos = ball_pos.copy()
                new_ball_x = ball_pos[0]
                new_ball_y = ball_pos[1]
                for _ in range(3):
                    rel_x = abs(new_ball_x-self.robot_pos[0])
                    rel_y = abs(new_ball_y-self.robot_pos[1])
                    ball_distance = math.hypot(rel_x, rel_y)
                    move_time = ball_distance/(self.MAX_SPEED*self.REAL_SPEED)*1.1
                    temp_x = ball_pos[0] - move_time*ball_x_vel
                    temp_y = ball_pos[1] - move_time*ball_y_vel
                    if self.is_valid_pos(temp_x, temp_y):
                        new_ball_x, new_ball_y = temp_x, temp_y
                    else:
                        break

                # Calculate coordinates for shooting
                target_x, target_y = new_ball_x, new_ball_y
                rel_ball_x, rel_ball_y = new_ball_x-self.GOAL[0], new_ball_y-self.GOAL[1]
                ball_goal_hypot = math.hypot(abs(rel_ball_x), abs(rel_ball_y))
                hypot_ratio = (ball_goal_hypot+self.SHOOT_DIST)/ball_goal_hypot
                temp_x = self.GOAL[0] + rel_ball_x*hypot_ratio
                temp_y = self.GOAL[1] + rel_ball_y*hypot_ratio
                if self.is_valid_pos(temp_x, temp_y):
                    target_x, target_y = temp_x, temp_y

                # Realize whether the robot should shoot
                target_coords = [target_x, target_y]
                if self.at_dest(target_x, target_y):
                    target_coords = self.GOAL
                if act_time-self.kickoff_time <= 1.5:
                    target_coords = ball_pos
                target_angle = math.atan2( \
                    target_coords[1] - self.robot_pos[1],
                    target_coords[0] - self.robot_pos[0],
                ) + heading

                # Realize whether the robot is between ball and goal. If so, avoid the ball
                el_goal_x = self.GOAL[0] + rel_ball_x*self.AVOID_EL
                el_goal_y = self.GOAL[1] + rel_ball_y*self.AVOID_EL
                el_ball_x = self.GOAL[0] + rel_ball_x*(1-self.AVOID_EL)
                el_ball_y = self.GOAL[1] + rel_ball_y*(1-self.AVOID_EL)
                b_rel_x, b_rel_y = self.robot_pos[0]-el_ball_x, self.robot_pos[1]-el_ball_y
                g_rel_x, g_rel_y = self.robot_pos[0]-el_goal_x, self.robot_pos[1]-el_goal_y
                ball_hypot = math.hypot(abs(b_rel_x), abs(b_rel_y))
                goal_hypot = math.hypot(abs(g_rel_x), abs(g_rel_y))
                if ball_hypot+goal_hypot < ball_goal_hypot:
                    self.avoid_time = act_time

                if act_time-self.avoid_time <= 0.2:
                    target_angle = (target_angle+3/18*math.pi) % (2*math.pi)

                # Change the direction of move if advantageous
                polarity = 1
                if self.get_angle_diff(math.degrees(target_angle)) > 120:
                    target_angle = (target_angle+math.pi) % (2*math.pi)
                    polarity = -1

                # Calculate speed of motors
                speed = self.MAX_SPEED * polarity

                if target_angle > math.pi:
                    target_angle -= 2*math.pi

                target_dist = math.hypot( \
                    target_coords[0]-self.robot_pos[0],
                    target_coords[1]-self.robot_pos[1]
                )
                k = min((target_dist**2/self.MAX_DRIVE_DIST**2)*2-1, 1)
                if target_dist < 0.15 and abs(math.degrees(target_angle)) < 15:
                    k = 0.8
                speed2 = (1-abs(target_angle)/(math.pi/2)) * k * speed

                if target_angle < 0:
                    left = speed2
                    right = speed
                else:
                    left = speed
                    right = speed2

                # Set the speed to motors
                self.left_motor.setVelocity(left)
                self.right_motor.setVelocity(right)

                self.send_data_to_team(
                    self.player_id,
                    self.robot_pos,
                    utils.ball_position(ball_data, self.robot_pos, heading)
                )

    def at_dest(self, x, y):
        # Check whether the robot is on the given position
        tol = self.SHOOT_DIST
        rx, ry = self.robot_pos[0], self.robot_pos[1]
        return x-tol <= rx <= x+tol and y-tol <= ry <= y+tol

    def is_valid_pos(self, x, y):
        (x0, x1), (y0, y1) = self.FIELD
        return x0 <= x <= x1 and y0 <= y <= y1

    def get_angle_diff(self, angle):
        return angle if angle <= 180 else 360-angle

    def get_direction(self, ball_angle: float) -> int:
        ball_angle = math.degrees(ball_angle)
        if ball_angle < 0:
            ball_angle += 360
        if ball_angle >= 345 or ball_angle <= 15:
            return 0
        return 1 if ball_angle < 180 else -1
