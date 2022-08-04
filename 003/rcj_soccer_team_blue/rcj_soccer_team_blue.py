from controller import Robot
from main_robot import MainRobot

robot = Robot()
name = robot.getName()
robot_number = int(name[1])

if robot_number == 1:
    robot_controller = MainRobot(robot)
elif robot_number == 2:
    robot_controller = MainRobot(robot)
else:
    robot_controller = MainRobot(robot)

robot_controller.run()
