import math
def get_direction(ball_vector: list) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_vector (list of floats): Current vector of the ball with respect
            to the robot.

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if -0.13 <= ball_vector[1] <= 0.13:
        return 0
    return -1 if ball_vector[1] < 0 else 1




def go_angle1(robot_angle,go_to_angle) -> float:
    heading=robot_angle-go_to_angle
    if(heading>180):
        heading=heading-360
    elif(heading<-180):
        heading=heading+360

    [left_speed,right_speed]=[0,0]            
    if(-1<heading and heading<1):
        return [left_speed,right_speed]
    elif(heading>0):
        [left_speed,right_speed]=[-5,5]
        return [left_speed,right_speed]
    elif(heading<0):
        [left_speed,right_speed]=[5,-5]
        return [left_speed,right_speed]

def go_angle2(robot,go_to_angle):
    heading=math.degrees(robot.get_compass_heading())-go_to_angle

    if(heading>180):
        heading=heading-360
    elif(heading<-180):
        heading=heading+360

    left_speed=-heading/5
    right_speed=heading/5
    
    if(left_speed>10):
        left_speed=10 
    if(left_speed<-10):
        left_speed=-10
    if(right_speed>10):
        right_speed=10
    if(right_speed<-10):
        right_speed=-10
    return [left_speed,right_speed]

def go_angle3(robot,go_to_angle):
    heading=math.degrees(robot.get_compass_heading())-go_to_angle

    if(heading>180):
        heading=heading-360
    elif(heading<-180):
        heading=heading+360

    left_speed=-heading/5
    right_speed=heading/5
    
    if(left_speed>10):
        left_speed=10 
    if(left_speed<-10):
        left_speed=-10
    if(right_speed>10):
        right_speed=10
    if(right_speed<-10):
        right_speed=-10
    
    robot.right_motor.setVelocity(right_speed)
    robot.left_motor.setVelocity(left_speed)

def go_pos(robot,position):
    if (robot.get_gps_coordinates()[0]-position[0]) == 0:
        return False
    angle=math.degrees(math.atan((robot.get_gps_coordinates()[1]-position[1])/(robot.get_gps_coordinates()[0]-position[0])))
    if(robot.get_gps_coordinates()[0]>=position[0]):
        angle=angle+90
    elif(robot.get_gps_coordinates()[0]<position[0]):
        angle=angle-90
    
    if(abs(angle-math.degrees(robot.get_compass_heading()))>90):
        if(robot.get_gps_coordinates()[0]>=position[0]):
            angle=angle-180
        elif(robot.get_gps_coordinates()[0]<position[0]):
            angle=angle+180
        go=-9
    else:
        go=9


    if(fisa(robot,position)>0.05):
        [left_speed,right_speed]=go_angle2(robot,angle) 
        left_speed=left_speed+go
        right_speed=right_speed+go
        if(left_speed>10):
            left_speed=10 
        if(left_speed<-10):
            left_speed=-10
        if(right_speed>10):
            right_speed=10
        if(right_speed<-10):
            right_speed=-10
        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)
        return False
    else:
        [left_speed,right_speed]=[0,0]
        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)
        return True

def fisa(robot,pos):
    d=math.sqrt((robot.get_gps_coordinates()[1]-pos[1])**2+(robot.get_gps_coordinates()[0]-pos[0])**2)
    return d


def stop(robot):
    robot.left_motor.setVelocity(0)
    robot.right_motor.setVelocity(0)

def set_motor_velocity(robot, right, left):
    robot.left_motor.setVelocity(left)
    robot.right_motor.setVelocity(right)   

def is_between(m, x, n) -> bool:
    if m > x and x > n:
        return True
    else:
        return False

def go_straight(robot,position):
    y = robot.get_gps_coordinates()[1]
    heading = robot.get_compass_heading()
    print(heading)
    if is_between(0 + 3, abs(heading), 0 - 3):
        if position - y > 0:
            go = 9
        else:
            go = -9
        if fisa(robot, [0, position]) >= 0.1:
            robot.left_motor.setVelocity(go)
            robot.right_motor.setVelocity(go)
            return False
        else:
            robot.left_motor.setVelocity(0)
            robot.right_motor.setVelocity(0)
            return True
    else:
        go_angle3(robot, 0)

def go_with_index(robot, goal, index):
    index1 = index[0]
    index2 = index[1]
    print(index1, index2)
    isOK = False
    if isOK == False:
        if index1 > goal:
            set_motor_velocity(robot, 0, 0)
            isOK = True
        else:
                index1 += 1
                set_motor_velocity(robot, 9, 9)
    else:
        print("yes")
        if index2 > goal:
            set_motor_velocity(robot, 0, 0)
        else:
            index2 += 1
            set_motor_velocity(robot, -9, -9)           
    return [index1, index2]

def sleep(robot, goal):
    index = 0
    while robot.robot.step(32) != -1:
        if index > goal:
            return
        index += 1