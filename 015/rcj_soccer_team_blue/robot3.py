#---------------------------------------------------------------------------------------------------libraries
import struct
import math

#---------------------------------------------------------------------------------------------------utils
TIME_STEP = 32
ROBOT_NAMES = ["B1", "B2", "B3", "Y1", "Y2", "Y3"]
N_ROBOTS = len(ROBOT_NAMES)

class RCJSoccerRobot:
    def __init__(self, robot):
        self.robot = robot
        self.name = self.robot.getName()
        self.team = self.name[0]
        self.player_id = int(self.name[1])

        self.receiver = self.robot.getDevice("supervisor receiver")
        self.receiver.enable(TIME_STEP)

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)

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

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

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

    def send_data_to_team(self, robot_id) -> None:
        """Send data to the team

        Args:
             robot_id (int): ID of the robot
        """
        struct_fmt = "i"
        data = [robot_id]
        packet = struct.pack(struct_fmt, *data)
        self.team_emitter.send(packet)

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

    def get_gps_coordinates(self) -> list:
        """Get new GPS coordinates

        Returns:
            List containing x and y values
        """
        gps_values = self.gps.getValues()
        return [gps_values[0], gps_values[1]]

    def get_compass_heading(self) -> float:
        """Get compass heading in radians

        Returns:
            float: Compass value in radians
        """
        compass_values = self.compass.getValues()

        # Add math.pi/2 (90) so that the heading 0 is facing opponent's goal
        rad = math.atan2(compass_values[0], compass_values[1]) + (math.pi / 2)
        if rad < -math.pi:
            rad = rad + (2 * math.pi)

        return rad

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

    def run(self):
        raise NotImplementedError
#---------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------config
DOOSTAM_1 = 1
DOOSTAM_2 = 2

POSHTE_TOOP_MIN = 80
POSHTE_TOOP_MAX = 280
TURN_STOP_MIN = 10
TURN_STOP_MAX = 350
DARVAZE_GHER = 250
JOLOE_TOOP_FOLLOW = 40
MOHAJEM = "m"
KOMAKI = "k"

#---------------------------------------------------------------------------------------------------variables
x = 0
y = 1
keep_turning_flag = False
lost_flag = False
flag_ekhtelaf_scan = False
doostam_mige = False
stuck_time = 0
ball_stuck_time = 0
gher_time = 0
jarime_time = 0
robot_x_last = 0
robot_y_last = 0
ball_x_last = 0 
ball_y_last = 0

doostam_1_x = 0
doostam_1_y = 0
doostam_1_zavie_toop = 360
doostam_1_fasele_toop = 1000
doostam_2_x = 0
doostam_2_y = 0
doostam_2_zavie_toop = 360
doostam_2_fasele_toop = 1000


#---------------------------------------------------------------------------------------------------MyRobot3
class MyRobot3(RCJSoccerRobot):

#---------------------------------------------------------------------------------------------------taghsim_vazayef
    def taghsim_vazayef(self):

        global naghsh

        if fasele_toop < doostam_2_fasele_toop:
            naghsh = MOHAJEM
        elif fasele_toop > doostam_2_fasele_toop:
            naghsh = KOMAKI
        else:
            naghsh = MOHAJEM

#---------------------------------------------------------------------------------------------------fake_ball_angle
    def fake_ball_angle(self):        
        
        global zavie_toop 
        global jahate_toop
        global zavie_toop_alamat
        global strength
        global fasele_toop
        global lost_flag
        global toop_be_zamin_x
        global toop_be_zamin_y

        if doostam_1_x != 0 and doostam_2_x != 0:
            toop_be_zamin_x = (doostam_1_x + doostam_2_x)/2
            toop_be_zamin_y = (doostam_1_y + doostam_2_y)/2
        elif doostam_1_x != 0:
            toop_be_zamin_x = doostam_1_x
            toop_be_zamin_y = doostam_1_y
        elif doostam_2_x != 0:
            toop_be_zamin_x = doostam_2_x
            toop_be_zamin_y = doostam_2_y
        else:
            toop_be_zamin_x = 0
            toop_be_zamin_y = 0
            lost_flag = True

        if doostam_1_zavie_toop != 360 and doostam_2_zavie_toop != 360:
            zavie_toop = (doostam_1_zavie_toop + doostam_2_zavie_toop)/2
        elif doostam_1_zavie_toop != 360:
            zavie_toop = doostam_1_zavie_toop
        elif doostam_2_zavie_toop != 360:
            zavie_toop = doostam_2_zavie_toop
        else:
            zavie_toop = 0
        
        zavie_toop = zavie_toop - robot_angle
        zavie_toop_alamat = 360 - ((zavie_toop + 180) % 360) - 180
        jahate_toop = (robot_angle + zavie_toop) % 360
        strength = 0
        fasele_toop = 1000

#---------------------------------------------------------------------------------------------------go_to
    def go_to(self,x_maghsad,y_maghsad):   

        fasele_noghat = math.sqrt((robot_y - y_maghsad)**2 +(robot_x - x_maghsad)**2) 
        fasele_noghat = fasele_noghat * 20    
        zavie_noghat = math.atan2(robot_y - y_maghsad , robot_x - x_maghsad)
        if zavie_noghat < 0:
            zavie_noghat = 2 * math.pi + zavie_noghat
        zavie_noghat = math.degrees(zavie_noghat)
        darvaze_pos = (zavie_noghat - robot_angle + 360 + 90)%360
        darvaze_pos = (darvaze_pos - 180) % 360
        darvaze_pos = 360 - ((darvaze_pos + 180) % 360) - 180

        divar_error = 0
        if abs(robot_y) > 0.65 or abs(robot_x) > 0.55:
            divar_error = darvaze_pos  #robot_angle - zavie_noghat

        darvaze_pos = darvaze_pos / 3
        left_error = fasele_noghat + darvaze_pos + divar_error
        right_error = fasele_noghat - darvaze_pos - divar_error
        self.move(10 + left_error, 10 + right_error)

#---------------------------------------------------------------------------------------------------main_ball
    def main_ball(self):

        if jahate_toop > 320 or jahate_toop < 40:
            self.ball_follow()
        else:
            self.turn()
        
#---------------------------------------------------------------------------------------------------turn
    def turn(self):

        jahate_toop_temp = jahate_toop
        jahate_toop_temp_yellow = jahate_toop
        ball_range = jahate_toop
        x_range = jahate_toop
        y_range = jahate_toop

        if name[0] == "B":
            zarib = 0.43
            if jahate_toop > 180 and jahate_toop < 360:
                jahate_toop_temp = jahate_toop - 360
            y_range = jahate_toop_temp / (180 / zarib)
            y_range = abs(y_range)
            if jahate_toop > 180 and  jahate_toop < 270:
                ball_range = jahate_toop - 180
            elif jahate_toop > 270 and jahate_toop < 360:
                ball_range = jahate_toop - 360
            elif jahate_toop > 90 and jahate_toop < 180:
                ball_range = (jahate_toop - 90) * -1
            x_range = ball_range / (90 / zarib) 
            self.go_to(toop_be_zamin_x + x_range, toop_be_zamin_y + y_range)
        else:
            zarib = 0.17
            jahate_toop_temp_yellow -= 180
            if jahate_toop_temp_yellow < 0:
                jahate_toop_temp_yellow += 360

            if jahate_toop_temp_yellow > 180 and jahate_toop_temp_yellow < 360:
                jahate_toop_temp = jahate_toop_temp_yellow - 360
            y_range = jahate_toop_temp / (180 / zarib) 
            y_range = abs(y_range) 
            if jahate_toop_temp_yellow > 180 and  jahate_toop_temp_yellow < 270:
                ball_range = (jahate_toop_temp_yellow - 180) 
            elif jahate_toop_temp_yellow > 270 and jahate_toop_temp_yellow < 360:
                ball_range = (jahate_toop_temp_yellow - 360) 
            elif jahate_toop_temp_yellow > 90 and jahate_toop_temp_yellow < 180:
                ball_range = (jahate_toop_temp_yellow - 90) * -1
            x_range = ball_range / (90 / zarib)
            self.go_to(toop_be_zamin_x - x_range, toop_be_zamin_y - y_range)

#---------------------------------------------------------------------------------------------------ball_follow
    def ball_follow(self):        
        if strength > 180:
            self.darvaze()
        else:            
            self.go_to(toop_be_zamin_x, toop_be_zamin_y)

#---------------------------------------------------------------------------------------------------darvaze
    def darvaze(self):
        if name[0] == "B":
            self.go_to(0,-0.81)
        else:
            self.go_to(0,0.81)

#---------------------------------------------------------------------------------------------------send_data
    def send_data(self, toop_x, toop_y, zavie_toop, fasele_toop):
        message_format = "idddd"
        packet = struct.pack(message_format, self.player_id, toop_x, toop_y, zavie_toop, fasele_toop)
        self.team_emitter.send(packet)
        
#---------------------------------------------------------------------------------------------------receive_data
    def receive_data(self):

        global doostam_1_x
        global doostam_1_y
        global doostam_1_zavie_toop
        global doostam_1_fasele_toop
        global doostam_2_x
        global doostam_2_y
        global doostam_2_zavie_toop
        global doostam_2_fasele_toop

        while self.is_new_team_data():
            packet = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            message_format = "idddd"
            unpack = struct.unpack(message_format, packet)
            id = unpack[0]
            if id == DOOSTAM_1:
                doostam_1_x = unpack[1]
                doostam_1_y = unpack[2]
                doostam_1_zavie_toop = unpack[3]
                doostam_1_fasele_toop = unpack[4]
            elif id == DOOSTAM_2:
                doostam_2_x = unpack[1]
                doostam_2_y = unpack[2]
                doostam_2_zavie_toop = unpack[3]
                doostam_2_fasele_toop = unpack[4]

#---------------------------------------------------------------------------------------------------sensors_update
    def sensors_update(self):

        global kick_off_data
        global ball_data
        global heading
        global robot_pos
        global sonar        
        global strength
        global robot_x
        global robot_y
        global ball_x
        global ball_y
        global name

        name = self.robot.getName()
        sonar = self.get_sonar_values()
        kick_off_data = self.get_new_data()['waiting_for_kickoff']
        heading = self.get_compass_heading()  
        robot_pos = self.get_gps_coordinates()  
        robot_x = robot_pos[x]
        robot_y = robot_pos[y]
        self.get_robot_angles()
        if self.is_new_ball_data():
            ball_data = self.get_new_ball_data()  
            strength = ball_data["strength"] 
            ball_x = ball_data["direction"][x]
            ball_y = ball_data["direction"][y]
            self.get_ball_angles() 
            self.get_ball_dist()
        else:
            return False       
        return True

#---------------------------------------------------------------------------------------------------get_ball_dist
    def get_ball_dist(self):

        global fasele_toop

        fasele_toop = math.sqrt((robot_y - ball_y)**2 +(robot_x - ball_x)**2) 

#---------------------------------------------------------------------------------------------------get_ball_angles
    def get_ball_angles(self):

        global zavie_toop 
        global jahate_toop
        global zavie_toop_alamat

        zavie_toop = math.atan2(ball_y, ball_x)
        if zavie_toop < 0:
            zavie_toop = 2 * math.pi + zavie_toop
        zavie_toop = math.degrees(zavie_toop)
        zavie_toop_alamat = 360 - ((zavie_toop + 180) % 360) - 180
        jahate_toop = (robot_angle + zavie_toop) % 360

#---------------------------------------------------------------------------------------------------get_robot_angles
    def get_robot_angles(self):

        global robot_angle
        global robot_angle_alamat
        
        robot_angle = heading
        if robot_angle < 0:
            robot_angle = 2 * math.pi + robot_angle
        robot_angle = math.degrees(robot_angle)
        robot_angle_alamat = 360 - ((robot_angle + 180) % 360) - 180

#---------------------------------------------------------------------------------------------------move
    def move(self, left, right):
        left = min(max(left, -10), 10)     
        right = min(max(right, -10), 10)
        right = right * 1.0
        left = left * 1.0
        self.left_motor.setVelocity(right)
        self.right_motor.setVelocity(left)

#---------------------------------------------------------------------------------------------------timers
    def timers(self):

        global stuck_time
        global robot_x_last
        global robot_y_last
        global gher_time
        global jarime_time
        global ball_stuck
        global ball_stuck_time
        global ball_x_last
        global ball_y_last

        stuck = abs(robot_x - robot_x_last + robot_y - robot_y_last)
        if stuck > 0.001:
            stuck_time = 0
        else:
            stuck_time += 1
        if stuck_time > 30:
            gher_time = 10
        if gher_time > 0:
            gher_time -= 1
        robot_x_last = robot_x
        robot_y_last = robot_y
        if robot_x > -0.34 and robot_x < 0.34 and robot_y > 0.57:
            jarime_time += 1
        else:
            jarime_time = 0
        ball_stuck = abs(toop_be_zamin_x - ball_x_last + toop_be_zamin_y - ball_y_last)
        if ball_stuck > 0.01:
            ball_stuck_time = 0
        else:
            if ball_stuck_time < 100:
                ball_stuck_time += 1 
        ball_x_last = toop_be_zamin_x
        ball_y_last = toop_be_zamin_y

#---------------------------------------------------------------------------------------------------toop_be_zamin   
    def toop_be_zamin(self):

        global toop_be_zamin_x
        global toop_be_zamin_y

        dist = math.sqrt(ball_x**2+ball_y**2)
        r = 0.11886*dist - 0.02715
        theta = (jahate_toop + 90) % 360
        if theta < 0:
            theta += 360
        theta = math.radians(theta)
        pos_x = r * math.cos(theta)
        pos_y = r * math.sin(theta)
        toop_be_zamin_x = robot_x - 2*pos_x
        toop_be_zamin_y = robot_y - 2*pos_y

#---------------------------------------------------------------------------------------------------fasele_noghat
    def fasele_noghat(self, x_1, y_1):
        ekhtelaf = abs(x_1 - robot_x + y_1 - robot_y)
        return ekhtelaf

#---------------------------------------------------------------------------------------------------scan
    def scan(self):

        global flag_ekhtelaf_scan

        if self.fasele_noghat(-0.20, -0.50) < 0.05:
            flag_ekhtelaf_scan = True
        if flag_ekhtelaf_scan == True:
            self.go_to(-0.20, 0.50)
        if self.fasele_noghat(-0.20, 0.50) < 0.05:
            flag_ekhtelaf_scan = False
        if flag_ekhtelaf_scan == False:
            self.go_to(-0.20, -0.50)

#---------------------------------------------------------------------------------------------------run
    def run(self):

        global lost_flag
        global stuck_time
        global jarime_time
        global robot_x_last
        global robot_y_last
        global ball_x_last
        global ball_y_last
        global ball_stuck_time
        global doostam_mige
        
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                
                self.receive_data()
                lost_flag = False
                doostam_mige = False
                
                if self.sensors_update() == False:  
                    doostam_mige = True    
                    self.send_data(0, 0, 360, 1000)
                    self.fake_ball_angle()
                                
                self.taghsim_vazayef()         
                if doostam_mige == False:
                    self.toop_be_zamin() 
                    self.send_data(toop_be_zamin_x, toop_be_zamin_y, jahate_toop, fasele_toop)                    

                if gher_time > 0 and lost_flag == False:
                    self.move(-10,-4)
                elif stuck_time < 100 and kick_off_data == False and jarime_time < 180:
                    if lost_flag == False:
                        if ball_stuck_time > 800 and naghsh == KOMAKI:
                            if name[0] == "B":
                                self.go_to(0, 0.09)                                       
                            else:
                                self.go_to(0, -0.09)                                       
                        elif strength > 70 or naghsh == MOHAJEM:
                            if strength > DARVAZE_GHER:
                                self.darvaze()
                            else:
                                self.main_ball()
                        else:
                            if name[0] == "B":
                                self.go_to(-min(max(toop_be_zamin_x,-0.3), 0.3), toop_be_zamin_y + 0.075) #-.06
                            else:
                                self.go_to(-min(max(toop_be_zamin_x,-0.3), 0.3), toop_be_zamin_y - 0.075) #-.06
                    else:
                        self.scan()    
                else:
                    self.move(0,0)                
                self.timers()