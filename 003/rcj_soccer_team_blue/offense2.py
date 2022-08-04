import math

from rcj_soccer_robot import TIME_STEP


class Offense2:
    timer = 0
    last_x = 0
    last_y = 0
    state = 0
    threshold = 0.006
    time_limit = 2.5
    attack_side = 0
    def_flag = 0
    onDefPoint = 0
    old_attack_side = []
    shake_timer = 0

    def run_blue(self, robot, team_position, kickoff=0):
        left_speed, right_speed = 0, 0

        robot_x, robot_y = robot.robot_x, robot.robot_y
        robot_heading = robot.robot_heading
        ball_x, ball_y = robot.ball_x, robot.ball_y
        ball_angle = robot.ball_angle
        ball_distance = robot.ball_distance
        pred_x, pred_y = robot.pred_x, robot.pred_y
        team_detect = robot.getTeamDetect()
        self.attack_side = -1
        angle_range = 10
        spin_speed = 6

        off1_x, off1_y = team_position[2][0], team_position[2][1]
        distance_off1 = math.sqrt(
            math.pow(off1_x - robot_x, 2) + math.pow(off1_y - robot_y, 2))
        if distance_off1 < 2.0:
            if ball_y < -0.6 and -0.45 < ball_x < 0:
                pred_x = pred_x - 0.1
            elif ball_y < -0.6 and 0 < ball_x < 0.45:
                pred_x = pred_x + 0.1
            else:
                pred_y = pred_y + 0.1

        ball_pred_angle = (math.degrees(math.atan2(
            pred_x - robot_x, pred_y - robot_y)) - robot_heading + 360 + 180) % 360
        temp_ball_angle = ball_pred_angle
        if temp_ball_angle > 180:
            temp_ball_angle = temp_ball_angle - 360  # ANGLE CONVERTION
        absolute_ball_angle = (
            temp_ball_angle + robot_heading + 360) % 360

        if team_detect:
            # if ball_y < -0.58:
            #     if ball_x > 0:
            #         target_angle = 90
            #         dest_x = 0.2
            #     else:
            #         target_angle = 270
            #         dest_x = -0.2
            #     dest_y = -0.7
            #     if robot_y < -0.7:
            #         dest_x = robot_x
            #     # target_angle = 0
            #     if self.def_flag and not (dest_y - 0.02 < robot_y < dest_y + 0.02):
            #         self.def_flag = 0
            #     if not self.def_flag:
            #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint(
            #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
            #     if self.onDefPoint:
            #         self.def_flag = 1
            #         left_speed, right_speed = Offense2.towardAngle(self, robot_heading, target_angle, spin_speed, angle_range)
            if ball_y > 0.58 and not(ball_distance<0.08 and ((ball_x<0 and absolute_ball_angle<180) or (ball_x>0 and absolute_ball_angle>180))):
                angle_range = 2
                # if -0.45 < ball_x < 0 and team_position[2][1] > -0.6:
                #     dest_x, dest_y = -0.155, -0.8   
                #     target_angle = 270
                #     if -0.35 < robot_x < 0.35:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_y(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                #     else:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_x(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                # elif 0 < ball_x < 0.45 and team_position[2][1] > -0.6:
                #     dest_x, dest_y = 0.155, -0.8
                #     target_angle = 90
                #     if -0.35 < robot_x < 0.35:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_y(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                #     else:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_x(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                # else:
                if ball_x > 0:
                    target_angle = 270
                    dest_x = team_position[0][0] - 0.2
                else:
                    target_angle = 90
                    dest_x = team_position[0][0] + 0.2
                dest_y = 0.695

                # target_angle = 0
                x_point_range = 0.05
                y_point_range = 0.015
                
                if -0.2 < robot_x < 0.2:
                    left_speed, right_speed, _ = Offense2.toPoint_y(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                else:
                    left_speed, right_speed, _ = Offense2.toPoint_x(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)

                if (dest_y - y_point_range < robot_y < dest_y + y_point_range):
                    if ball_distance < 0.17:
                        angle_range = 20
                    if robot_x < -0.2 or robot_x > 0.2:
                        target_angle = 100 if 0 < absolute_ball_angle < 180 else 260
                    else:
                        target_angle = 90 if 0 < absolute_ball_angle < 180 else 270
                    left_speed, right_speed = Offense2.towardAngle(
                        self, robot_heading, target_angle, spin_speed, angle_range)

                #print('off2', left_speed, right_speed)
            else:
                self.def_flag = 0
                delta = math.sqrt(
                    math.pow(ball_x - self.last_x, 2) + math.pow(ball_y - self.last_y, 2))
                # print(delta)
                if delta < self.threshold:
                    self.timer += (TIME_STEP / 1000)
                else:
                    self.timer = 0
                if self.timer < self.time_limit:
                    state = 0
                else:
                    state = 1
                #print(round(delta,3), self.timer)
                if state == 1 and (ball_x < -0.37 or ball_x > 0.37) and (ball_y < -0.3):
                    # (ball_y > 0.66 and robot_y < 0.1):
                    # CAMPER
                    dest_x, dest_y = 0, 0.12
                    target_angle = 180
                    left_speed, right_speed, _ = Offense2.toPoint(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range=4, x_point_range=0.005)
                # ---BOUNCE---
                elif (ball_x <= -0.5 or ball_x >= 0.5) and delta > 0.002 and math.sqrt(
                                        math.pow(ball_x - team_position[2][0], 2) + math.pow(ball_y - team_position[2][1], 2)) > 0.07: 
                    #print("BOUNCE")                                 
                    if ball_x > 0:
                        if delta > 0.0065:
                            dest_x = ball_x - 0.11
                        else:
                            dest_x = ball_x - 0.06
                    else:
                        if delta > 0.0065:
                            dest_x = ball_x + 0.11
                        else:
                            dest_x = ball_x + 0.06
                    dest_y = ball_y + 0.02
                    target_angle = 15 if ball_x > 0 else 345 
                    left_speed, right_speed, _ = Offense2.toPoint(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range=4, x_point_range=0.005)
                else:
                    # normal chase
                    offense_angle = 15
                    if ball_distance >= 0.2 or kickoff:
                        aim_angle = 0
                        angle_offset = 0
                    else:
                        if robot_y < 0:
                            if robot_x > 0:
                                aim_angle = -robot_x * 45 + robot_y * 10
                            else:
                                aim_angle = -robot_x * 45 - robot_y * 10
                        else:
                            aim_angle = -robot_x * 25

                        if -0.18 < ball_x < 0.18:
                            aim_angle = 0

                        if offense_angle < absolute_ball_angle < 90:
                            angle_offset = 30 * \
                                (0.2 - ball_distance) / 0.2 + 30
                        elif 270 < absolute_ball_angle < 360 - offense_angle:
                            angle_offset = -30 * \
                                (0.2 - ball_distance) / 0.2 - 30
                        elif offense_angle < absolute_ball_angle <= 180:
                            angle_offset = 60 * \
                                (0.2 - ball_distance) / 0.2 + 45
                        elif 180 < absolute_ball_angle < 360 - offense_angle:
                            angle_offset = -60 * \
                                (0.2 - ball_distance) / 0.2 - 45
                        else:
                            angle_offset = 0

                    if 160 < absolute_ball_angle <= 180 and ball_distance < 0.15:
                        aim_angle = 0
                        angle_offset = 90
                    elif 180 < absolute_ball_angle < 200 and ball_distance < 0.15:
                        aim_angle = 0
                        angle_offset = -90

                    absolute_move_direction = (
                        temp_ball_angle + angle_offset + aim_angle + robot_heading + 360) % 360

                    closest_angle, _ = Offense2.getCloserAngle(
                        self, (temp_ball_angle + angle_offset + aim_angle + 360) % 360, 0, 180)

                    if len(self.old_attack_side) >= 6:
                        if -3 < sum(self.old_attack_side) < 3 and self.old_attack_side[-1] != self.old_attack_side[-2]:
                            self.shake_timer = 1
                    if self.shake_timer == 0:
                        self.attack_side = 1 if closest_angle == 0 else -1
                    else:
                        self.shake_timer += (TIME_STEP / 1000)
                    if self.shake_timer > 1.8:
                        self.shake_timer = 0

                    if self.attack_side == 1:
                        move_direction = temp_ball_angle + angle_offset + aim_angle
                    else:
                        if temp_ball_angle + angle_offset + aim_angle >= 0:
                            move_direction = temp_ball_angle + angle_offset + aim_angle - 180
                        else:
                            move_direction = 180 - \
                                abs((temp_ball_angle + angle_offset + aim_angle))

                        # print(closest_angle, attack_side, temp_ball_angle, move_direction)

                    left_speed, right_speed = Offense2.robotAttack(
                        self, move_direction, self.attack_side)

                    # avoid wall hitting
                    if robot_x > 0.585 and -0.68 < robot_y < 0.68 and absolute_move_direction > 180:
                        # left move up
                        target_angle = 330
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_x < -0.585 and -0.68 < robot_y < 0.68 and absolute_move_direction < 180:
                        # right move up
                        target_angle = 30
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y > 0.685 and 180 < absolute_move_direction < 270:
                        # bottom move left
                        target_angle = 260
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y > 0.685 and 90 < absolute_move_direction < 180:
                        # bottom move right
                        target_angle = 100
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y < -0.685 and 180 < absolute_move_direction < 360 and ball_distance < 0.15:
                        # top move left
                        target_angle = 300
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y < -0.685 and 0 < absolute_move_direction < 180 and ball_distance < 0.15:
                        # top move right
                        target_angle = 60
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)

            self.last_x, self.last_y = ball_x, ball_y
            if len(self.old_attack_side) < 6:
                self.old_attack_side.append(self.attack_side)
            else:
                self.old_attack_side.pop(0)
                self.old_attack_side.append(self.attack_side)

        else:  # no team_detect -> robot to center
            if ball_y > 0:
                if team_position[2][0] > 0 and robot_x < 0:
                    dest_x, dest_y = -0.3, 0.3
                else:
                    dest_x, dest_y = -0.3, 0.3
            else:
                if team_position[2][0] > 0 and robot_x < 0:
                    dest_x, dest_y = -0.3, 0.1
                else:
                    dest_x, dest_y = -0.3, 0.1

            target_angle = 0
            angle_range = 8
            left_speed, right_speed, _ = Offense2.toPoint_y(
                self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range)

        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)

    def run_yellow(self, robot, team_position, kickoff=0):
        left_speed, right_speed = 0, 0

        robot_x, robot_y = robot.robot_x, robot.robot_y
        robot_heading = robot.robot_heading
        ball_x, ball_y = robot.ball_x, robot.ball_y
        ball_angle = robot.ball_angle
        ball_distance = robot.ball_distance
        pred_x, pred_y = robot.pred_x, robot.pred_y
        team_detect = robot.getTeamDetect()
        self.attack_side = -1
        angle_range = 10
        spin_speed = 6

        off1_x, off1_y = team_position[2][0], team_position[2][1]
        distance_off1 = math.sqrt(
            math.pow(off1_x - robot_x, 2) + math.pow(off1_y - robot_y, 2))
        if distance_off1 < 2.0:
            pred_y = pred_y - 0.1

        ball_pred_angle = (math.degrees(math.atan2(
            pred_x - robot_x, pred_y - robot_y)) - robot_heading + 360) % 360
        temp_ball_angle = ball_pred_angle
        if temp_ball_angle > 180:
            temp_ball_angle = temp_ball_angle - 360  # ANGLE CONVERTION
        absolute_ball_angle = (
            temp_ball_angle + robot_heading + 360) % 360

        if team_detect:
            # if ball_y < -0.58:
            #     if ball_x > 0:
            #         target_angle = 90
            #         dest_x = 0.2
            #     else:
            #         target_angle = 270
            #         dest_x = -0.2
            #     dest_y = -0.7
            #     if robot_y < -0.7:
            #         dest_x = robot_x
            #     # target_angle = 0
            #     if self.def_flag and not (dest_y - 0.02 < robot_y < dest_y + 0.02):
            #         self.def_flag = 0
            #     if not self.def_flag:
            #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint(
            #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
            #     if self.onDefPoint:
            #         self.def_flag = 1
            #         left_speed, right_speed = Offense2.towardAngle(self, robot_heading, target_angle, spin_speed, angle_range)
            if ball_y < -0.58 and not(ball_distance<0.08 and ((ball_x>0 and absolute_ball_angle<180) or (ball_x<0 and absolute_ball_angle>180))):
                angle_range = 2
                # if -0.45 < ball_x < 0 and team_position[2][1] > -0.6:
                #     dest_x, dest_y = -0.155, -0.8   
                #     target_angle = 270
                #     if -0.35 < robot_x < 0.35:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_y(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                #     else:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_x(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                # elif 0 < ball_x < 0.45 and team_position[2][1] > -0.6:
                #     dest_x, dest_y = 0.155, -0.8
                #     target_angle = 90
                #     if -0.35 < robot_x < 0.35:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_y(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                #     else:
                #         left_speed, right_speed, self.onDefPoint = Offense2.toPoint_x(
                #             self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, 0.02)
                # else:
                if ball_x > 0:
                    target_angle = 90
                    dest_x = team_position[0][0] - 0.2
                else:
                    target_angle = 270
                    dest_x = team_position[0][0] + 0.2
                dest_y = -0.695

                # target_angle = 0
                x_point_range = 0.05
                y_point_range = 0.015
                
                if -0.2 < robot_x < 0.2:
                    left_speed, right_speed, _ = Offense2.toPoint_y(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                else:
                    left_speed, right_speed, _ = Offense2.toPoint_x(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)

                if (dest_y - y_point_range < robot_y < dest_y + y_point_range):
                    if robot_x < -0.2 or robot_x > 0.2:
                        target_angle = 100 if 0 < absolute_ball_angle < 180 else 260
                        if ball_distance < 0.17:
                            angle_range = 20
                    else:
                        target_angle = 90 if 0 < absolute_ball_angle < 180 else 270
                    left_speed, right_speed = Offense2.towardAngle(
                        self, robot_heading, target_angle, spin_speed, angle_range)

                # print('off2', left_speed, right_speed)
            else:
                self.def_flag = 0
                delta = math.sqrt(
                    math.pow(ball_x - self.last_x, 2) + math.pow(ball_y - self.last_y, 2))
                if delta < self.threshold:
                    self.timer += (TIME_STEP / 1000)
                else:
                    self.timer = 0
                if self.timer < self.time_limit:
                    state = 0
                else:
                    state = 1
                #print(round(delta,3), self.timer)
                if state == 1 and (ball_x < -0.37 or ball_x > 0.37) and (ball_y > 0.3):
                    # (ball_y > 0.66 and robot_y < 0.1):
                    # CAMPER
                    dest_x, dest_y = 0, -0.12
                    target_angle = 180
                    left_speed, right_speed, _ = Offense2.toPoint(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range=4, x_point_range=0.005)
                # ---BOUNCE---
                elif (ball_x <= -0.5 or ball_x >= 0.5) and delta > 0.002 and math.sqrt(
                                        math.pow(ball_x - team_position[2][0], 2) + math.pow(ball_y - team_position[2][1], 2)) > 0.07:                                  
                    if ball_x > 0:
                        if delta > 0.0065:
                            dest_x = ball_x - 0.11
                        else:
                            dest_x = ball_x - 0.06
                    else:
                        if delta > 0.0065:
                            dest_x = ball_x + 0.11
                        else:
                            dest_x = ball_x + 0.06
                    dest_y = ball_y - 0.02
                    target_angle = 15 if ball_x < 0 else 345 
                    left_speed, right_speed, _ = Offense2.toPoint(
                        self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range=4, x_point_range=0.005)
                else:
                    # normal chase
                    offense_angle = 15
                    if ball_distance >= 0.2 or kickoff:
                        aim_angle = 0
                        angle_offset = 0
                    else:
                        if robot_y > 0:
                            if robot_x > 0:
                                aim_angle = robot_x * 45 + robot_y * 10
                            else:
                                aim_angle = robot_x * 45 - robot_y * 10
                        else:
                            aim_angle = robot_x * 25

                        if -0.18 < ball_x < 0.18:
                            aim_angle = 0

                        if offense_angle < absolute_ball_angle < 90:
                            angle_offset = 30 * \
                                (0.2 - ball_distance) / 0.2 + 30
                        elif 270 < absolute_ball_angle < 360 - offense_angle:
                            angle_offset = -30 * \
                                (0.2 - ball_distance) / 0.2 - 30
                        elif offense_angle < absolute_ball_angle <= 180:
                            angle_offset = 60 * \
                                (0.2 - ball_distance) / 0.2 + 45
                        elif 180 < absolute_ball_angle < 360 - offense_angle:
                            angle_offset = -60 * \
                                (0.2 - ball_distance) / 0.2 - 45
                        else:
                            angle_offset = 0

                    if 160 < absolute_ball_angle <= 180 and ball_distance < 0.15:
                        aim_angle = 0
                        angle_offset = 90
                    elif 180 < absolute_ball_angle < 200 and ball_distance < 0.15:
                        aim_angle = 0
                        angle_offset = -90

                    absolute_move_direction = (
                        temp_ball_angle + angle_offset + aim_angle + robot_heading + 360) % 360

                    closest_angle, _ = Offense2.getCloserAngle(
                        self, (temp_ball_angle + angle_offset + aim_angle + 360) % 360, 0, 180)

                    if len(self.old_attack_side) >= 6:
                        if -3 < sum(self.old_attack_side) < 3 and self.old_attack_side[-1] != self.old_attack_side[-2]:
                            self.shake_timer = 1
                    if self.shake_timer == 0:
                        self.attack_side = 1 if closest_angle == 0 else -1
                    else:
                        self.shake_timer += (TIME_STEP / 1000)
                    if self.shake_timer > 1.8:
                        self.shake_timer = 0

                    if self.attack_side == 1:
                        move_direction = temp_ball_angle + angle_offset + aim_angle
                    else:
                        if temp_ball_angle + angle_offset + aim_angle >= 0:
                            move_direction = temp_ball_angle + angle_offset + aim_angle - 180
                        else:
                            move_direction = 180 - \
                                abs((temp_ball_angle + angle_offset + aim_angle))

                        # print(closest_angle, attack_side, temp_ball_angle, move_direction)

                    left_speed, right_speed = Offense2.robotAttack(
                        self, move_direction, self.attack_side)

                    # avoid wall hitting
                    if robot_x < -0.585 and -0.68 < robot_y < 0.68 and absolute_move_direction > 180:
                        # left move up
                        target_angle = 330
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_x > 0.585 and -0.68 < robot_y < 0.68 and absolute_move_direction < 180:
                        # right move up
                        target_angle = 30
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y < -0.685 and 180 < absolute_move_direction < 270:
                        # bottom move left
                        target_angle = 260
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y < -0.685 and 90 < absolute_move_direction < 180:
                        # bottom move right
                        target_angle = 100
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y > 0.685 and 180 < absolute_move_direction < 360 and ball_distance < 0.15:
                        # top move left
                        target_angle = 300
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    elif robot_y > 0.685 and 0 < absolute_move_direction < 180 and ball_distance < 0.15:
                        # top move right
                        target_angle = 60
                        angle_range = 20
                        left_speed, right_speed = Offense2.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)

            self.last_x, self.last_y = ball_x, ball_y
            if len(self.old_attack_side) < 6:
                self.old_attack_side.append(self.attack_side)
            else:
                self.old_attack_side.pop(0)
                self.old_attack_side.append(self.attack_side)

        else:  # no team_detect -> robot to center
            if ball_y < 0:
                if team_position[2][0] < 0 and robot_x > 0:
                    dest_x, dest_y = 0.3, -0.3
                else:
                    dest_x, dest_y = 0.3, -0.3
            else:
                if team_position[2][0] < 0 and robot_x > 0:
                    dest_x, dest_y = 0.3, -0.1
                else:
                    dest_x, dest_y = 0.3, -0.1

            target_angle = 0
            angle_range = 8
            left_speed, right_speed, _ = Offense2.toPoint_y(
                self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range)

        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)

    def getCloserAngle(self, input_angle, angle1, angle2):
        angle_options = [(input_angle - angle1 + 360) % 360, (angle1 - input_angle + 360) % 360,
                         (input_angle - angle2 + 360) % 360, (angle2 - input_angle + 360) % 360]
        min_difference = min(angle_options)
        if min_difference == angle_options[0]:
            return angle1, -angle_options[0]
        elif min_difference == angle_options[1]:
            return angle1, angle_options[1]
        elif min_difference == angle_options[2]:
            return angle2, -angle_options[2]
        else:
            return angle2, angle_options[3]

    def getCoordDirection(self, input_angle, initial_x, initial_y, final_x, final_y):
        absolute_angle = (math.degrees(math.atan2(
            initial_x - final_x, initial_y - final_y))) % 360
        closer_angle, closer_angle_difference = Offense2.getCloserAngle(
            self, input_angle, absolute_angle, (absolute_angle + 180) % 360)
        return closer_angle, absolute_angle

    def robotSpin(self, robot_heading, target_angle, spin_speed):
        closer_angle, closer_angle_difference = Offense2.getCloserAngle(
            self, robot_heading, target_angle, (target_angle + 180) % 360)
        if closer_angle_difference > 0:
            left_speed, right_speed = -spin_speed, spin_speed
        else:
            left_speed, right_speed = spin_speed, -spin_speed
        return left_speed, right_speed

    def robotAttack(self, move_direction, attack_side, angle=60):
        left_speed = attack_side * -10 * \
            (angle + move_direction * attack_side) / angle
        right_speed = attack_side * -10 * \
            (angle - move_direction * attack_side) / angle

        if left_speed > 10:
            left_speed = 10
        elif left_speed < -10:
            left_speed = -10
        if right_speed > 10:
            right_speed = 10
        elif right_speed < -10:
            right_speed = -10

        return left_speed, right_speed

    def toPoint(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Offense2.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                      dest_y)
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                if "B" in robot.name:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = -10, -10
                    else:
                        left_speed, right_speed = 10, 10
                else:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = 10, 10
                    else:
                        left_speed, right_speed = -10, -10
        else:
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range):
            dest_y = robot_y
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Offense2.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                      dest_y)
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                if "B" in robot.name:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = -10, -10
                    else:
                        left_speed, right_speed = 10, 10
                else:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = 10, 10
                    else:
                        left_speed, right_speed = -10, -10
        else:
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_y - y_point_range < robot_y < dest_y + y_point_range):
            dest_x = robot_x
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Offense2.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                      dest_y)
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                if "B" in robot.name:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = -10, -10
                    else:
                        left_speed, right_speed = 10, 10
                else:
                    if closer_angle == robot_heading:
                        left_speed, right_speed = 10, 10
                    else:
                        left_speed, right_speed = -10, -10
        else:
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Offense2.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def towardAngle(self, robot_heading, target_angle, spin_speed, angle_range):
        closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                        (robot_heading + 180) % 360)
        if abs(closer_angle_difference) > angle_range:
            left_speed, right_speed = Offense2.robotSpin(
                self, robot_heading, target_angle, spin_speed)
        else:
            closer_angle, closer_angle_difference = Offense2.getCloserAngle(self, target_angle, robot_heading,
                                                                            (robot_heading + 180) % 360)
            if closer_angle == robot_heading:
                left_speed, right_speed = -10, -10
            else:
                left_speed, right_speed = 10, 10
        return left_speed, right_speed
