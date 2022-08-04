import struct
from typing import Iterable

from utils import *
from utils import Vector2D

TIME_STEP = 32
TP_COOLDOWN = 9
TP_THRESHOLD = 0.1
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

ORIGIN = Vector2D(0, 0)
OPP_GOAL = Vector2D(0.75, 0)
OUR_GOAL = -OPP_GOAL

ROTATE = Vector2D(1, -1)


class Teammate:
    def __init__(self):
        self.pos = [0, 0]


class Handler:
    def __init__(self, robot):
        self.rotated = True
        self.arrived = True

        self.robot = robot
        self.name = self.robot.getName()
        self.team = self.name[0]
        self.play_direction = 1 if self.team == "Y" else -1
        self.player_id = int(self.name[1])
        self.team_list = [i for i in range(1, 4) if i != self.player_id]
        self.teleport_cooldown = 0
        self.shooting = 0

        self.receiver = self.robot.getDevice("supervisor receiver")
        self.receiver.enable(TIME_STEP)

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)

        self.prev_ball_pos = Vector2D(0, 0)
        self.ball_pos = Vector2D(0, 0)
        self.ball_direction = 0
        self.ball_speed = 0
        self.distance_to_ball = 0
        self.ball_receiver = self.robot.getDevice("ball receiver")
        self.ball_receiver.enable(TIME_STEP)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.compass = self.robot.getDevice("compass")
        self.compass.enable(TIME_STEP)

        self.sonar_left = self.robot.getDevice("distancesensor left")
        self.sonar_left.enable(TIME_STEP)
        self.sonar_right = self.robot.getDevice("distancesensor right")
        self.sonar_right.enable(TIME_STEP)
        self.sonar_front = self.robot.getDevice("distancesensor front")
        self.sonar_front.enable(TIME_STEP)
        self.sonar_back = self.robot.getDevice("distancesensor back")
        self.sonar_back.enable(TIME_STEP)

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float("+inf"))
        self.right_motor.setPosition(float("+inf"))

        self.set_speeds(0)

        self.teammates = [Teammate(), Teammate()]

    def parse_supervisor_msg(self, packet: str) -> dict:
        """Parse message received from supervisor

        Returns:
            dict: Location info about each robot and the ball.
            Example:
                {
                    'waiting_for_kickoff': False,
                }
        """
        # True/False telling whether the goal was scored
        struct_fmt = "?"
        unpacked = struct.unpack(struct_fmt, packet)

        data = {"waiting_for_kickoff": unpacked[0]}
        return data

    def get_new_data(self) -> dict:
        """Read new data from supervisor

        Returns:
            dict: See `parse_supervisor_msg` method
        """
        packet = self.receiver.getData()
        self.receiver.nextPacket()

        return self.parse_supervisor_msg(packet)

    def is_new_data(self) -> bool:
        """Check if there is new data from supervisor to be received

        Returns:
            bool: Whether there is new data received from supervisor.
        """
        return self.receiver.getQueueLength() > 0

    def parse_team_msg(self, packet: str) -> dict:
        """Parse message received from team robot

        Returns:
            dict: Parsed message stored in dictionary.
        """
        struct_fmt = "i"
        unpacked = struct.unpack(struct_fmt, packet)
        data = {
            "robot_id": unpacked[0],
        }
        return data

    def get_new_team_data(self) -> dict:
        """Read new data from team robot

        Returns:
            dict: See `parse_team_msg` method
        """
        packet = self.team_receiver.getData()
        self.team_receiver.nextPacket()
        return self.parse_team_msg(packet)

    def is_new_team_data(self) -> bool:
        """Check if there is new data from team robots to be received

        Returns:
            bool: Whether there is new data received from team robots.
        """
        return self.team_receiver.getQueueLength() > 0

    def get_new_ball_data(self) -> dict:
        """Read new data from IR sensor

        Returns:
            dict: Direction and strength of the ball signal
            Direction is normalized vector indicating the direction of the
            emitter with respect to the receiver's coordinate system.
            Example:
                {
                    'direction': [0.23, -0.10, 0.96],
                    'strength': 0.1
                }
        """
        _ = self.ball_receiver.getData()
        data = {
            "direction": self.ball_receiver.getEmitterDirection(),
            "strength": self.ball_receiver.getSignalStrength(),
        }
        self.ball_receiver.nextPacket()
        return data

    def is_new_ball_data(self) -> bool:
        """Check if there is new data from ball to be received

        Returns:
            bool: Whether there is new data received from ball.
        """
        return self.ball_receiver.getQueueLength() > 0

    def get_gps_coordinates(self) -> Vector2D:
        """Get new GPS coordinates

        Returns:
            List containing x and y values
        """
        x, y, _ = self.gps.getValues()
        return Vector2D(y, -x) * self.play_direction

    def get_compass_heading(self) -> float:
        """Get compass heading in radians

        Returns:
            float: Compass value in radians
        """
        x, y, z = self.compass.getValues()
        direction = math.atan2(y, -x)
        if self.team == "Y":
            direction += math.pi

        return normalize_angle(direction)

    def get_sonar_values(self) -> dict:
        """Get new values from sonars.

        Returns:
            dict: Value for each sonar.
        """
        return {
            "left": self.sonar_left.getValue(),
            "right": self.sonar_right.getValue(),
            "front": self.sonar_front.getValue(),
            "back": self.sonar_back.getValue(),
        }

    def read_ball_data(self):
        ball_data = self.get_new_ball_data()

        x, y, z = ball_data["direction"]
        self.distance_to_ball = math.sqrt(1 / ball_data['strength'])
        ball_pos_relative = Vector2D(x, y) * self.distance_to_ball

        self.ball_pos = self.robot_pos + ball_pos_relative.rotate(self.robot_angle)

        ball_delta = self.prev_ball_pos - self.ball_pos
        self.ball_direction = ball_delta.angle
        self.ball_speed = ball_delta.length * 500

        self.prev_ball_pos = self.ball_pos

    def send_data(self):
        packet = struct.pack(
            'idd?ddddd',
            self.player_id,
            *self.robot_pos,
            self.ball_visible,
            *self.ball_pos,
            self.distance_to_ball,
            self.ball_direction,
            self.ball_speed,
        )

        self.team_emitter.send(packet)

    def get_data(self):
        while self.is_new_team_data():
            packet = self.team_receiver.getData()
            self.team_receiver.nextPacket()

            teammate_id, x, y, ball_visible, ball_x, ball_y, space_to_ball, ball_direction, ball_speed = struct.unpack(
                'idd?ddddd', packet)
            teammate_index = self.team_list.index(teammate_id)
            teammate = self.teammates[teammate_index]
            teammate.pos = Vector2D(x, y)

            if ball_visible:
                self.ball_visible = True
                self.ball_pos = Vector2D(ball_x, ball_y)
                self.distance_to_ball = space_to_ball
                self.ball_direction = ball_direction
                self.ball_speed = ball_speed

    def read_data(self):
        gps_pos = self.get_gps_coordinates()
        self.prev_robot_pos = self.robot_pos if hasattr(self, 'robot_pos') else gps_pos
        self.robot_pos = gps_pos
        self.robot_angle = self.get_compass_heading()
        self.sonar_front_value = self.sonar_front.getValue()
        self.sonar_back_value = self.sonar_back.getValue()
        self.sonar_right_value = self.sonar_right.getValue()
        self.sonar_left_value = self.sonar_left.getValue()

        self.ball_visible = self.is_new_ball_data()
        if self.ball_visible:
            self.read_ball_data()

        self.send_data()
        self.get_data()

    def timer_step(self):
        self.teleport_cooldown -= self.teleport_cooldown > 0
        self.shooting -= self.shooting > 0

    def set_speeds(self, left, right=None):
        if right is None:
            if isinstance(left, Iterable):
                left, right = left
            else:
                right = left

        self.left_motor.setVelocity(clip(left, -10, 10))
        self.right_motor.setVelocity(clip(right, -10, 10))

    def to_relative(self, pos: Vector2D) -> Vector2D:
        return (pos - self.robot_pos).rotate(-self.robot_angle)

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.read_data()
                self.timer_step()

                # if teleported
                if (self.robot_pos - self.prev_robot_pos).length > 0.1:
                    self.teleport_cooldown = TP_COOLDOWN

                if self.teleport_cooldown > 0:
                    self.set_speeds(0, 0)
                    continue

                self.act()


class RCJSoccerRobot(Handler):
    def stop(self) -> bool:
        self.set_speeds(0, 0)
        return False

    def align_to(self, target: Vector2D) -> bool:
        SPEED = 10
        ERROR: float = pi / 100

        angle = (target - self.robot_pos).angle
        delta = normalize_angle(angle - self.robot_angle)

        if abs(delta) > pi / 2:
            delta = normalize_angle(delta + pi)

        if abs(delta) < ERROR:
            return True

        speed = SPEED * atan_smooth(delta, sharpness=5)
        self.set_speeds(speed, -speed)
        return False

    def arc(self, radius: float, speed: float = 10):
        ROBOT_WIDTH = 0.05

        direction = sign(radius)
        radius = abs(radius)
        right = left = speed
        if radius != float('inf'):
            right *= (radius - ROBOT_WIDTH) / (radius + ROBOT_WIDTH)
        right, left = [right, left][::direction]
        self.set_speeds(left, right)
        return False

    def goto(self, target: Vector2D, priority: float = 50) -> float:
        SPEED = 10
        ERROR: float = 2e-3

        if abs(target - self.robot_pos) < ERROR:
            return True

        x, y = self.to_relative(target)

        dist = (target - self.robot_pos).length
        speed = SPEED * atan_smooth(dist, sharpness=200)
        radius = x / y / priority * sign(x) if y else float('inf')

        return self.arc(radius, speed * sign(x))

    def goto_behing_ball(
            self, target: Vector2D = OPP_GOAL,
            radius=.08,
            error: float = 1e-2,
    ) -> float:

        EPSILON_THETA = pi / 10
        radius = abs(radius)
        ball_to_robot = self.robot_pos - self.ball_pos
        behind = target.towards(self.ball_pos, radius)

        if abs(self.ball_pos + behind - self.robot_pos) < error:
            return True

        # can we directly go to the ball?
        if behind.normalized @ ball_to_robot > radius:
            return self.goto(self.ball_pos + behind)

        ball_dist = abs(ball_to_robot)
        delta_angle = normalize_angle(behind.angle - ball_to_robot.angle)

        if ball_dist > radius:
            angle = math.acos(radius / ball_dist) * sign(delta_angle)
        else:
            angle = EPSILON_THETA * sign(delta_angle)

        target = ball_to_robot.rotate(angle).normalized * radius + self.ball_pos

        return self.goto(target)

    def act(self):
        return self.stop()

    def attack(self):
        if abs(self.robot_pos.y) > .55:
            if (self.robot_pos - self.ball_pos).x < 0:
                return self.goto(self.ball_pos)
            else:
                return self.goto(ORIGIN)

        target = OPP_GOAL
        if self.shooting > 0:
            return self.goto(target)

        if self.goto_behing_ball(target):
            self.shooting = 10
            return self.stop()
