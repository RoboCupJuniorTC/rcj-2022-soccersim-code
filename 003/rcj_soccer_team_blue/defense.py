import math


class Defense:
    sp = 0
    def_flag = 0
    onDefPoint = 0
    wiggle = 0

    def run_blue(self, robot, team_position, kickoff=0):
        left_speed, right_speed = 0, 0

        robot_x, robot_y = robot.robot_x, robot.robot_y
        robot_heading = robot.robot_heading
        ball_x, ball_y = robot.ball_x, robot.ball_y
        ball_angle = robot.ball_angle
        ball_distance = robot.ball_distance
        pred_x, pred_y = robot.pred_x, robot.pred_y
        team_detect = robot.getTeamDetect()

        spin_speed = 6
        angle_range = 6

        # print(robot_heading)
        ball_pred_angle = (math.degrees(math.atan2(
            pred_x - robot_x, pred_y - robot_y)) - robot_heading + 360 + 180) % 360
        temp_ball_angle = ball_pred_angle
        if temp_ball_angle > 180:
            temp_ball_angle = temp_ball_angle - 360  # ANGLE CONVERTION
        absolute_ball_angle = (
            temp_ball_angle + robot_heading + 360) % 360

        if kickoff: # Kickoff -> defense walk straight to point
            x_point_range = 0.05
            y_point_range = 0.05
            dest_x, dest_y = 0, 0.54
            target_angle = 90
            left_speed, right_speed, _ = Defense.toPoint(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range) 
        else:
        # Robot Can Detect Ball Location
            if team_detect:
                if 0.54 <= robot_y <= 0.58 and -0.28 <= robot_x <= 0.28:
                    # inside
                    sp = 1
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if ball_x < robot_x:
                            if closer_angle == robot_heading:
                                sp = 21
                                left_speed, right_speed = -10, -10
                            else:
                                sp = 22
                                left_speed, right_speed = 10, 10
                        elif ball_x > robot_x:
                            if closer_angle == robot_heading:
                                sp = 23
                                left_speed, right_speed = 10, 10
                            else:
                                sp = 24
                                left_speed, right_speed = -10, -10
                elif robot_x < -0.28 and robot_y < 0.56:
                    # right up outside
                    sp = 2
                    target_angle, absolute_angle = Defense.getCoordDirection(
                        self, robot_heading, robot_x, robot_y, -0.2, 0.59)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(
                        self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = -10, -10
                        else:
                            left_speed, right_speed = 10, 10
                elif robot_x > 0.28 and robot_y < 0.56:
                    # left up outside
                    sp = 3
                    target_angle, absolute_angle = Defense.getCoordDirection(
                        self, robot_heading, robot_x, robot_y, 0.2, 0.59)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(
                        self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = -10, -10
                        else:
                            left_speed, right_speed = 10, 10
                elif robot_x < -0.28 and (robot.teammate1_y and robot.teammate2_y) < 0.58:
                    # right outside
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        sp = 12
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if closer_angle == robot_heading:
                            sp = 4
                            left_speed, right_speed = 10, 10
                        else:
                            sp = 5
                            left_speed, right_speed = -10, -10
                elif robot_x > 0.28 and (robot.teammate1_y and robot.teammate2_y) < 0.58:
                    # left outside
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        sp = 13
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if closer_angle == robot_heading:
                            sp = 6
                            left_speed, right_speed = -10, -10
                        else:
                            sp = 7
                            left_speed, right_speed = 10, 10

                elif robot_y < 0.54 or robot_y > 0.58:
                    # up down outside
                    sp = 8
                    if ball_x < 0.27 and ball_x > -0.27:
                        target_angle, absolute_angle = Defense.getCoordDirection(
                            self, robot_heading, robot_x, robot_y, robot_x, 0.58)
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(
                            self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    else:
                        target_angle, absolute_angle = Defense.getCoordDirection(
                            self, robot_heading, robot_x, robot_y, robot_x, 0.58)
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(
                            self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = -10, -10
                        else:
                            left_speed, right_speed = 10, 10

                # ball in corner
                # print(self.onDefPoint)
                if ball_y > 0.58:
                    x_point_range = 0.05
                    y_point_range = 0.015
                    angle_range = 2

                    # if 150<absolute_ball_angle<210:
                    #     if ball_x > 0:
                    #         dest_x = ball_x - 0.15
                    #     else: 
                    #         dest_x = ball_x + 0.15
                    #     dest_y = 0.695
                    #     target_angle = 0
                    #     left_speed, right_speed, _ = Defense.toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range) 
                    # else:    
                    if ball_x < 0:
                        target_angle = 90
                        dest_x = -0.35
                    else:
                        target_angle = 270
                        dest_x = 0.35  
                    dest_y = 0.695
                    
                    if ((ball_x < 0 and robot_x < 0 and ball_x > robot_x) or (ball_x > 0 and robot_x > 0 and ball_x < robot_x)) and robot_y > 0.6:
                        target_angle = 180
                        left_speed, right_speed, _ = Defense.toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                    elif -0.3 < robot_x < 0.3:
                        left_speed, right_speed, _ = Defense.toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                    else:
                        left_speed, right_speed, _ = Defense.toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
            
                    if (dest_y - y_point_range < robot_y < dest_y + y_point_range):
                        if ball_distance < 0.17:
                            angle_range = 20
                        if robot_x < -0.2 or robot_x > 0.2:
                            target_angle = 100 if 0 < absolute_ball_angle < 180 else 260
                        else:
                            target_angle = 90 if 0 < absolute_ball_angle < 180 else 270
                        left_speed, right_speed = Defense.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)

                    #print('def', left_speed, right_speed, angle_range)
            else:
                sp = 11
                if robot_y < 0.52 or robot_y > 0.58 or robot_x > 0.01 or robot_x < -0.01:
                    target_angle, absolute_angle = Defense.getCoordDirection(self, robot_heading, robot_x, robot_y, 0,
                                                                            0.56)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = -10, -10
                        else:
                            left_speed, right_speed = 10, 10
                else:
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        left_speed, right_speed = 10, 10
            # if target_angle==180 or target_angle==0:
            # print("sp = ", sp)
        
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

        spin_speed = 6
        angle_range = 6
        # print(robot_heading)
        ball_pred_angle = (math.degrees(math.atan2(
            pred_x - robot_x, pred_y - robot_y)) - robot_heading + 360) % 360
        temp_ball_angle = ball_pred_angle
        if temp_ball_angle > 180:
            temp_ball_angle = temp_ball_angle - 360  # ANGLE CONVERTION
        absolute_ball_angle = (
            temp_ball_angle + robot_heading + 360) % 360

        if kickoff: # Kickoff -> defense walk straight to point
            angle_range = 50
            x_point_range = 0.05
            y_point_range = 0.05
            dest_x, dest_y = 0, -0.54
            target_angle = 90
            left_speed, right_speed, _ = Defense.toPoint(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range) 
        else:
            # Robot Can Detect Ball Location
            if team_detect:
                if robot_y < -0.54 and robot_y > -0.58 and robot_x >= -0.28 and robot_x <= 0.28:
                    # inside
                    sp = 1
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if ball_x > robot_x:
                            if closer_angle == robot_heading:
                                sp = 21
                                left_speed, right_speed = -10, -10
                            else:
                                sp = 22
                                left_speed, right_speed = 10, 10
                        elif ball_x < robot_x:
                            if closer_angle == robot_heading:
                                sp = 23
                                left_speed, right_speed = 10, 10
                            else:
                                sp = 24
                                left_speed, right_speed = -10, -10
                elif robot_x > 0.28 and robot_y > -0.56:
                    # right up outside
                    sp = 2
                    target_angle, absolute_angle = Defense.getCoordDirection(
                        self, robot_heading, robot_x, robot_y, 0.2, -0.59)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(
                        self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = 10, 10
                        else:
                            left_speed, right_speed = -10, -10
                elif robot_x < -0.28 and robot_y > -0.56:
                    # left up outside
                    sp = 3
                    target_angle, absolute_angle = Defense.getCoordDirection(
                        self, robot_heading, robot_x, robot_y, -0.2, -0.59)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(
                        self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = 10, 10
                        else:
                            left_speed, right_speed = -10, -10
                elif robot_x > 0.28 and (robot.teammate1_y and robot.teammate2_y) > -0.58:
                    # right outside
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        sp = 12
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if closer_angle == robot_heading:
                            sp = 4
                            left_speed, right_speed = 10, 10
                        else:
                            sp = 5
                            left_speed, right_speed = -10, -10
                elif robot_x < -0.28 and (robot.teammate1_y and robot.teammate2_y) > -0.58:
                    # left outside
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        sp = 13
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        if closer_angle == robot_heading:
                            sp = 6
                            left_speed, right_speed = -10, -10
                        else:
                            sp = 7
                            left_speed, right_speed = 10, 10

                elif robot_y > -0.54 or robot_y < -0.58:
                    # up down outside
                    sp = 8
                    if ball_x < 0.27 and ball_x > -0.27:
                        target_angle, absolute_angle = Defense.getCoordDirection(
                            self, robot_heading, robot_x, robot_y, ball_x, -0.58)
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(
                            self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    else:
                        target_angle, absolute_angle = Defense.getCoordDirection(
                            self, robot_heading, robot_x, robot_y, robot_x, -0.58)
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(
                            self, target_angle, robot_heading, (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = 10, 10
                        else:
                            left_speed, right_speed = -10, -10

                # ball in corner
                # print(self.onDefPoint)
                if ball_y < -0.58:
                    angle_range = 2
                    x_point_range = 0.05
                    y_point_range = 0.015

                    # if ball_y < robot_y - 0.05 and ball_x + 0.1 > robot_x > ball_x - 0.1:
                    #     if ball_x > 0:
                    #         dest_x = ball_x - 0.15
                    #     else: 
                    #         dest_x = ball_x + 0.15
                    #     dest_y = ball_y - 0.12
                    #     target_angle = 0
                    #     left_speed, right_speed, _ = Defense.toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range) 
                    # else:
                    if ball_x > 0:
                        target_angle = 90
                        dest_x = 0.35
                    else:
                        target_angle = 270
                        dest_x = -0.35  
                    dest_y = -0.695
                
                    if ((ball_x > 0 and robot_x > 0 and ball_x < robot_x) or (ball_x < 0 and robot_x < 0 and ball_x > robot_x)) and robot_y < -0.6:
                        target_angle = 180
                        left_speed, right_speed, _ = Defense.toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                    elif -0.3 < robot_x < 0.3:
                        left_speed, right_speed, _ = Defense.toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                    else:
                        left_speed, right_speed, _ = Defense.toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range, y_point_range)
                                
                    if (dest_y - y_point_range < robot_y < dest_y + y_point_range):
                        if robot_x < -0.2 or robot_x > 0.2:
                            target_angle = 100 if 0 < absolute_ball_angle < 180 else 260
                            if ball_distance < 0.17:
                                angle_range = 20
                        else:
                            target_angle = 90 if 0 < absolute_ball_angle < 180 else 270
                        
                        left_speed, right_speed = Defense.towardAngle(
                            self, robot_heading, target_angle, spin_speed, angle_range)
                    #print('def', left_speed, right_speed, angle_range)
            else:
                sp = 11
                if robot_y > -0.52 or robot_y < -0.58 or robot_x > 0.01 or robot_x < -0.01:
                    target_angle, absolute_angle = Defense.getCoordDirection(self, robot_heading, robot_x, robot_y, 0,
                                                                            -0.56)
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
                                                                                    (robot_heading + 180) % 360)
                        if closer_angle == robot_heading:
                            left_speed, right_speed = 10, 10
                        else:
                            left_speed, right_speed = -10, -10
                else:
                    target_angle = 90
                    closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                                (robot_heading + 180) % 360)
                    if abs(closer_angle_difference) > angle_range:
                        left_speed, right_speed = Defense.robotSpin(
                            self, robot_heading, target_angle, spin_speed)
                    else:
                        left_speed, right_speed = -10, -10
            # if target_angle==180 or target_angle==0:
            # print("sp = ", sp)
        
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
        closer_angle, closer_angle_difference = Defense.getCloserAngle(
            self, input_angle, absolute_angle, (absolute_angle + 180) % 360)
        return closer_angle, absolute_angle

    def robotSpin(self, robot_heading, target_angle, spin_speed):
        closer_angle, closer_angle_difference = Defense.getCloserAngle(
            self, robot_heading, target_angle, (target_angle + 180) % 360)
        if closer_angle_difference > 0:
            left_speed, right_speed = -spin_speed, spin_speed
        else:
            left_speed, right_speed = spin_speed, -spin_speed
        return left_speed, right_speed

    def toPoint(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Defense.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                     dest_y)
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
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
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def toPoint_x(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range):
                dest_y = robot_y
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Defense.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                     dest_y)
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
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
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def toPoint_y(self, left_speed, right_speed, robot, robot_x, robot_y, robot_heading, dest_x, dest_y, target_angle, spin_speed, angle_range, x_point_range=0.03, y_point_range=0.03):
        onPoint = 0
        if not (dest_y - y_point_range < robot_y < dest_y + y_point_range):
                dest_x = robot_x
        if not (dest_x - x_point_range < robot_x < dest_x + x_point_range and dest_y - y_point_range < robot_y < dest_y + y_point_range):
            target_angle, absolute_angle = Defense.getCoordDirection(self, robot_heading, robot_x, robot_y, dest_x,
                                                                     dest_y)
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                closer_angle, closer_angle_difference = Defense.getCloserAngle(self, absolute_angle, robot_heading,
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
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if abs(closer_angle_difference) > angle_range:
                left_speed, right_speed = Defense.robotSpin(
                    self, robot_heading, target_angle, spin_speed)
            else:
                onPoint = 1
        return left_speed, right_speed, onPoint

    def towardAngle(self, robot_heading, target_angle, spin_speed, angle_range):
        closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                       (robot_heading + 180) % 360)
        if abs(closer_angle_difference) > angle_range:
            left_speed, right_speed = Defense.robotSpin(
                self, robot_heading, target_angle, spin_speed)
        else:
            closer_angle, closer_angle_difference = Defense.getCloserAngle(self, target_angle, robot_heading,
                                                                           (robot_heading + 180) % 360)
            if closer_angle == robot_heading:
                left_speed, right_speed = -10, -10
            else:
                left_speed, right_speed = 10, 10
        return left_speed, right_speed
