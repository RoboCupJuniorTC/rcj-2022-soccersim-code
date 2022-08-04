import utils
from utils import Position as Position
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot3(RCJSoccerRobot):

    def run(self):
        # トランシーバーの初期化
        self.transceiver = utils.Transceiver(self)
        self.balllog = utils.Manage_log(200)
        self.robotlog = utils.Manage_log(20)
        self.move=utils.Move(self.robotlog,self.balllog)
        # このロボットをキーパーに設定
        self.mode = "attacker2"  # keeper or attacker or attacker2
        self.init = False
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():

                ########## 共有データの取得 ##########
                teamData = self.transceiver.getData()

                ########## 自分のデータの取得 ##########
                gpsdata = self.get_gps_coordinates()
                position = Position(gpsdata[0],gpsdata[1], self.get_compass_heading())
                self.robotlog.set_data(position)
                if self.is_new_ball_data():
                    ballData = self.get_new_ball_data()
                    myballPos = utils.get_ballpos(position,ballData["direction"][1],ballData["direction"][0],ballData["strength"])
                else:
                    ballData = None
                    myballPos = None

                ballPos = myballPos
                ##########     init     ##########aa
                if(self.init==False):
                    """
                    teamcolor: -1  blue
                    1  yellow
                    """
                    if(position.y>=0):
                        teamcolor=-1
                    else:
                        teamcolor=1
                    self.init=True
                        
                ########## 動作の決定 ##########
                if ballPos==None:
                    for i in teamData.values():
                        if i["ballpos"]!=None:
                            ballPos = i["ballpos"]

                self.balllog.set_data(ballPos)
                if self.mode == "keeper":
                    motorOut = self.move.keeper(position,self.balllog.get_PrePos(),teamcolor)
                elif self.mode == "attacker":
                    motorOut = self.move.attacker(position,self.balllog.get_PrePos(),teamcolor)
                else:
                    motorOut = self.move.attacker2(position,self.balllog.get_PrePos(),teamcolor)
                #print(self.balllog.get_PrePos())
                #print("--------------------------------------")
                self.left_motor.setVelocity(motorOut[0])
                self.right_motor.setVelocity(motorOut[1])
                
                #self.left_motor.setVelocity(0)
                #self.right_motor.setVelocity(0)
                ########## 共有データの更新 ##########
                self.transceiver.sendData(self.player_id, position, myballPos)