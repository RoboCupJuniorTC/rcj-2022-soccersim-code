from rcj_soccer_robot import RCJSoccerRobot as sr
import numpy as np
import math
from collections import deque # 先頭に

class Position:
    def __init__(self, x: float, y: float, dir: float = 0):
        self.x = x
        self.y = y
        self.dir = dir
        
    def __repr__(self):
        return f'x:{self.x}  y:{self.y}  dir:{self.dir}'
    def __str__(self):
        return f'x:{self.x}  y:{self.y}  dir:{self.dir}'

GOAL_YELLOW_POS_CENTER = Position(0,-0.745)
GOAL_YELLOW_POS_UPPER = Position(0.2,-0.745)
GOAL_YELLOW_POS_LOWER = Position(-0.2,-0.745)
GOAL_BLUE_POS_CENTER = Position(0,0.745)
GOAL_BLUE_POS_UPPER = Position(0.2,0.745)
GOAL_BLUE_POS_LOWER = Position(-0.2,0.745)


def get_distance(posa:Position,posb:Position = Position(0,0))->float:
    return ((posa.x-posb.x)**2+(posa.y-posb.y)**2)**0.5

def get_direction(ball_vector: float) -> int:
    # Get direction to navigate robot to face the ball

    # Args:
    #     ball_vector (list of floats): Current vector of the ball with respect
    #         to the robot.

    # Returns:
    #     int: 0 = forward, -1 = right, 1 = left

    if -0.13 <= ball_vector <= 0.13:
        return 0
    return -1 if ball_vector < 0 else 1


def get_ballpos(rpos:Position,ball_dirs:float,ball_dirc:float,ball_str:float) -> Position:
    sdir = np.arcsin(ball_dirs)
    cdir = np.arccos(ball_dirc)
    cdir -= np.pi/2
    newdir = np.abs(sdir)
    if sdir<0 and cdir>0:
        newdir=np.pi + newdir
    elif sdir<0:
        newdir=np.pi*2 - newdir
    elif cdir>0:
        newdir=np.pi - newdir
    dir = change_dir(rpos.dir+newdir)
    x = rpos.x - np.sin(dir) * (ball_str ** -0.50813)
    y = rpos.y + np.cos(dir) * (ball_str ** -0.50813)
    retpos = Position(x,y,0)
    return retpos

def get_motorOut(nowPos: Position, newPos: Position) -> tuple:
    """newPosに向かうモーターの出力を計算する

    Args:
        nowPos (Position): 今いる場所(向いている方向も含める)
        newPos (Position): 行きたい場所(向きたい方向も含める)

    Returns:
        tuple: _description_
    """

    dir = math.atan2(newPos.x - nowPos.x, newPos.y - nowPos.y)
    dir = change_dir(dir + nowPos.dir)
    leftMotor = 10
    rightMotor = 10
    
    if np.sin(dir) > 0:
        if np.cos(dir) > 0:
            rightMotor -= max(0, abs(dir) * 12)

        elif np.cos(dir) < 0:
            # 右後
            leftMotor *= -1
            rightMotor *= -1
            rightMotor += max(0, (math.pi - abs(dir)) * 12)

    elif np.sin(dir) < 0:
        if np.cos(dir) > 0:
            # 左前
            leftMotor -= max(0, abs(dir) * 12)

        elif np.cos(dir) < 0:
            # 左後
            leftMotor *= -1
            rightMotor *= -1
            leftMotor += max(0, (math.pi - abs(dir)) * 12)

    return rightMotor, leftMotor

# 角度を-pi ~ piにして返す
def change_dir(dir):
    if dir > np.pi:
        dir -= 2*np.pi
    if dir < -np.pi:
        dir += 2*np.pi
    return dir

class Transceiver:
    def __init__(self,robot:sr):
        self.robo = robot

    def isdata(self)->bool:
        return self.robo.is_new_team_data()

    def getData(self)->dict:
        ret:dict = {}
        while self.isdata():
            data = self.robo.get_new_team_data()
            pos = Position(data["pos_x"],data["pos_y"],data["pos_dir"])
            if(data["is_ball_data"]):
                ball_pos = Position(data["ball_x"],data["ball_y"],0)
            else :
                ball_pos = None
            tmp = {
                "robotpos":pos,
                "ballpos":ball_pos
            }
            ret[data["robot_id"]] = tmp
        return ret
    
    def sendData(self,robotid:int,robotpos:Position,ballpos:Position = None):
        """_summary_

        Args:
            robotid (int): ロボットのID
            robotpos (Position): ロボットの座標・方向
            ballpos (_type_, optional): ボールの座標 Defaults to None.
            ballcos : ball_data["direction"][1]
            ballsin : ball_data["direction"][0]
            ballstrenge : ball_data["strenge"]

        Returns:
            _type_: None
        """
        if ballpos == None:
            data = [robotid,robotpos.x,robotpos.y,robotpos.dir,False,-100,-100]
        else :
            data = [robotid,robotpos.x,robotpos.y,robotpos.dir,True,ballpos.x,ballpos.y]
        self.robo.send_data_to_team(data)

class Manage_log:

    def __init__(self,length: int):
        self.positionLog = deque(maxlen = length)

    ##### BALL #####
    def set_data(self,data:Position):
        self.positionLog.append(data)

    # ボールが見えているか
    def is_seen(self) -> bool:
        for i in self.positionLog:
            if i!=None:
                return True
        return False

    # ボールの進んでいる方向
    def get_Vector(self) -> Position:
        spos=None
        epos=None
        for i in self.positionLog:
            if i!=None:
                if spos==None:
                    spos = i
                else:
                    epos = i
        if spos==None: return Position(0,0)
        if spos!=None and epos==None: return Position(0,0)
        diss = get_distance(spos,epos)
        if diss <= 0.00001: return Position(0,0)
        return Position((epos.x-spos.x)/diss,(epos.y-spos.y)/diss)
    
    def get_Speed(self) -> Position:
        spos=None
        epos=None
        count=1
        ctmp=1
        for i in range(1,len(list(self.positionLog))):
            if self.positionLog[-i]!=None:
                if spos==None:
                    spos = self.positionLog[-i] 
                else:
                    epos = self.positionLog[-i]
                    count+=ctmp
                    ctmp=1
            elif spos!=None:
                ctmp+=1
            if i>6 and epos!=None:
                break
        return Position(-(epos.x-spos.x)/count,-(epos.y-spos.y)/count) if spos!=None and epos!=None else None

    def get_PrePos(self) ->  Position:
        spos=None
        count=1
        if self.positionLog[-1] !=None:
            return self.positionLog[-1]
        for i in list(self.positionLog)[-1::-1]:
            if i!=None:
                spos = i
                break
            else :
                count+=1
        spd = self.get_Speed()
        if spd==None:
            spd=Position(0,0)
        return Position(spos.x+spd.x*count,spos.y+spd.y*count) if spos!=None else None
    def is_stop(self,flame:int) ->bool:
        spos=None
        epos=None
        for i in list(self.positionLog)[-1:-flame:-1]:
            if i!=None:
                if spos==None:
                    spos = i
                else:
                    epos = i
        return True if spos!=None and epos!=None and get_distance(spos,epos)<0.075 else False

    # リスポーンされたか
    def is_respawn(self) ->bool:
        if self.positionLog[-1]==None:
            return True if self.is_stop(100) else False
        else:
            spos=self.positionLog[-1]
            epos=None
            for i in self.positionLog[-2::-1]:
                if i!=None:
                    epos=i
                    break
        return False if epos==False else False if get_distance(spos,epos)<0.1 else True
        
class Move:
		########## flag ###########
    attacker_mawariflag = False
    attacker_mawariflag2 = False
    at_mawariflag = False
    at_toxzeroflag = False
    yuchan = 0
    robotlog:Manage_log= None
    balllog:Manage_log= None
    def __init__(self,robot_log,ball_log):
        self.robotlog = robot_log
        self.balllog = ball_log
    def KakuKaku(self,nowPos: Position, newPos: Position) -> tuple:
        """
        カクカク動くよ！
        """
        dir = math.atan2(newPos.x - nowPos.x, newPos.y - nowPos.y)
        dir = change_dir(dir + nowPos.dir)
        
        leftMotor = 10
        rightMotor = 10
        
        if (np.sin(dir)>0.1 and np.cos(dir)<0) or (np.sin(dir)<-0.1 and np.cos(dir)>0):
            leftMotor *= -1
        elif (np.sin(dir)<-0.1 and np.cos(dir)<0) or (np.sin(dir)>0.1 and np.cos(dir)>0):
            rightMotor *= -1
        elif np.cos(dir)<0:
            leftMotor*=-1
            rightMotor*=-1
        return rightMotor, leftMotor


    keeper_goto=None
    keeper_goto2=None
    keeper_furafura=False
    def keeper(self,robotPos: Position, ballPos: Position, teamColor: int) -> tuple:
        """
        キーパーの動きを計算し，モーターの出力値を返す
        Args:
            robotPos (position): ロボットの座標
            ballPos (position): ボールの座標
            reverse (int): 攻める向き

        Returns:
            tuple(float, float): モーターの出力値 (right, left) 
        """
        
        if ballPos != None:
            yosoux = self.balllog.get_Vector().x*(-0.01*teamColor-ballPos.y)/max(0.01,abs(self.balllog.get_Vector().y))
            if ballPos.y  *teamColor*-1< 0.3 or (abs(ballPos.x)<0.3 and  ballPos.y  *teamColor*-1 < 0.65):
                goto = Position(min(0.3, max(-0.3, ballPos.x)), -teamColor * 0.55, 0)
            elif  ballPos.y  *teamColor*-1< 0.65:
                goto = Position(min(0.25, max(-0.2, ballPos.x)), -teamColor * 0.70, 0)
            elif self.balllog.positionLog[-1] == None :
                goto = Position(0,-teamColor * 0.70)
            else:
                goto = Position(ballPos.x/abs(ballPos.x), -teamColor * 0.70, 0)

            if get_distance(ballPos,robotPos)<0.1 and abs(ballPos.y) < abs(robotPos.y) and abs(ballPos.x)>0.35:
                goto = ballPos
        else:
            goto = Position(0, -teamColor * 0.55, math.pi / 2)

        self.keeper_goto=goto
        #ふらふらしてリスポーン対策するよ
        if get_distance(robotPos,self.keeper_goto)<0.01 and self.keeper_furafura==False and (ballPos==None or (ballPos!=None and ballPos.y/abs(ballPos.y)/teamColor>0)):
            #if get_distance(robotPos,ballPos)>0.2:
            self.keeper_furafura=True
            self.keeper_goto2=Position(self.keeper_goto.x-self.keeper_goto.x/max(0.01,abs(self.keeper_goto.x))*0.1,self.keeper_goto.y)
        if self.keeper_furafura and get_distance(robotPos,self.keeper_goto2)<0.01:
             self.keeper_furafura=False

        #リスポーン予測だよ
        if self.balllog.is_stop(100) and ballPos!=None and ballPos.y/abs(ballPos.y)/teamColor>0:
            self.keeper_furafura=False
            self.keeper_goto = Position(0,-0.3*teamColor)
        if self.balllog.is_stop(100) and self.robotlog.is_stop(100):
            self.keeper_furafura=False
            self.keeper_goto = Position(0, -teamColor * 0.55, math.pi / 2)
        # print(self.balllog.is_stop(100))
        if not self.keeper_furafura: return self.KakuKaku(robotPos, self.keeper_goto)
        else: return self.KakuKaku(robotPos, self.keeper_goto2)

    def attacker(self,robotPos:Position, ballPos:Position, teamcolor: int) -> tuple:
        """
        とにかく回り込んで相手のゴールの方にボールを運ぶ
        Args:
            robotPos (position): ロボットの座標
            ballPos (position): ボールの座標
        Returns:
            tuple(float, float): モーターの出力値 (right, left) 
        """

        def in_range(dir1, dir2, dif) -> bool:
            mi = abs((dir1 - dir2) % (math.pi * 2))
            mx = math.pi * 2 - mi
            return min(mi, mx) <= dif

        ########## ballが見えないとき ##########
        if ballPos == None:
            return get_motorOut(robotPos, Position(0, -teamcolor * 0.3))

        ########### 進みたい方向の範囲内であるとき ##########
        # robotとballの角度
        nowDir = math.atan2(ballPos.x - robotPos.x, ballPos.y - robotPos.y)
        # 進みたい方向
        nextDir = math.atan2(-ballPos.x, GOAL_BLUE_POS_CENTER.y * teamcolor - ballPos.y)
        # 進みたい方向からの許容角度
        difDir = math.pi / 3

        if in_range(nextDir, nowDir, difDir):
            # 半径
            moveR = get_distance(robotPos, ballPos)
            # 次に行く場所
            nextPos = Position(
                ballPos.x + moveR * math.sin(nextDir),
                ballPos.y + moveR * math.cos(nextDir)
            )
            return get_motorOut(robotPos, nextPos)

        if get_distance(robotPos, ballPos) >= 0.4:
            return get_motorOut(robotPos, Position(ballPos.x - np.sign(ballPos.x) * 0.2, ballPos.y - teamcolor * 0.3))
    
        if teamcolor * ballPos.y < -0.4:
            return self.keeper(robotPos, ballPos, teamcolor)

        ########## まわりこみ ##########        
        vec = self.robotlog.get_Vector()
        # robotが進んでいる方向
        robot_gotoDir = math.atan2(vec.x, vec.y)

        # 回転したい方向 (正転方向 : 1、反転方向 : -1)
        rotate = -teamcolor * np.sign(math.sin(nowDir))
        if in_range(math.pi, robot_gotoDir, math.pi / 24):
            rotate = - teamcolor * np.sign(vec.x)

        # 半径
        moveR = max(0, get_distance(robotPos, ballPos) - 0.02)

        # ball基準に変更
        nowDir = change_dir(nowDir + math.pi)

        # 次に行く場所
        nextPos = Position(
            ballPos.x + moveR * math.sin(nowDir + rotate * math.pi / 36),
            ballPos.y + moveR * math.cos(nowDir + rotate * math.pi / 36)
        )

        return get_motorOut(robotPos, nextPos)


    def attacker2(self,robotPos:Position, ballPos:Position, teamcolor: int) -> tuple:
        """
        とにかく回り込んで相手のゴールの方にボールを運ぶ
        Args:
            robotPos (position): ロボットの座標
            ballPos (position): ボールの座標
        Returns:
            tuple(float, float): モーターの出力値 (right, left) 
        """
        if(teamcolor==-1):
            if ballPos == None:
                dis=((0-robotPos.x)**2+(0-robotPos.y)**2)**0.5
                if(dis>0.01):
                    returndata=get_motorOut(robotPos,Position(0,0))
                else:
                    returndata=(10,-10)
                self.yuchan=100
            else:
                if(ballPos.y>robotPos.y or self.at_mawariflag==True):
                    self.at_mawariflag=True
                    if(ballPos.x>0):
                        if(robotPos.x > ballPos.x or self.at_toxzeroflag==True):
                            self.at_toxzeroflag=True
                            returndata=get_motorOut(robotPos,Position(-0.7,ballPos.y-0.15))
                            self.yuchan=1
                            if ballPos.x-0.25 > robotPos.x:
                                self.at_toxzeroflag=False
                        else:
                            returndata=get_motorOut(robotPos,Position(ballPos.x-0.15,ballPos.y+0.15))
                            self.yuchan=2
                    else:
                        if(ballPos.x > robotPos.x or self.at_toxzeroflag==True):
                            self.at_toxzeroflag=True
                            returndata=get_motorOut(robotPos,Position(0.7,ballPos.y-0.15))
                            self.yuchan=3
                            if(robotPos.x > ballPos.x-0.25):
                                self.at_toxzeroflag=False
                        else:
                            returndata=get_motorOut(robotPos,Position(ballPos.x+0.15,ballPos.y+0.15))
                            self.yuchan=4
                    if(robotPos.y>ballPos.y+0.1):
                        self.at_mawariflag=False
                        self.at_toxzeroflag=False
                else:
                    balldis=get_distance(robotPos,ballPos)
                    if(balldis>0.1):
                        shoota=(ballPos.x-GOAL_YELLOW_POS_CENTER.x) / (ballPos.y-GOAL_YELLOW_POS_CENTER.y)
                        shooty=ballPos.y+0.05
                        shootx=shoota * (shooty-ballPos.y) + ballPos.x
                        returndata=get_motorOut(robotPos,Position(shootx,shooty))
                        
                    else:
                        returndata=get_motorOut(robotPos,ballPos)
                    self.yuchan=10
        else:
            if ballPos == None:
                dis=((0-robotPos.x)**2+(0-robotPos.y)**2)**0.5
                if(dis>0.01):
                    returndata=get_motorOut(robotPos,Position(0,0))
                else:
                    returndata=(10,-10)
                self.yuchan=100
            else:
                if(ballPos.y<robotPos.y or self.at_mawariflag==True):
                    self.at_mawariflag=True
                    if(ballPos.x>0):
                        if(robotPos.x > ballPos.x or self.at_toxzeroflag==True):
                            self.at_toxzeroflag=True
                            returndata=get_motorOut(robotPos,Position(-0.7,ballPos.y+0.15))
                            self.yuchan=1
                            if ballPos.x-0.25 > robotPos.x:
                                self.at_toxzeroflag=False
                        else:
                            returndata=get_motorOut(robotPos,Position(ballPos.x-0.15,ballPos.y-0.15))
                            self.yuchan=2
                    else:
                        if(ballPos.x > robotPos.x or self.at_toxzeroflag==True):
                            self.at_toxzeroflag=True
                            returndata=get_motorOut(robotPos,Position(0.7,ballPos.y+0.15))
                            self.yuchan=3
                            if(robotPos.x > ballPos.x+0.25):
                                self.at_toxzeroflag=False
                        else:
                            returndata=get_motorOut(robotPos,Position(ballPos.x+0.15,ballPos.y-0.15))
                            self.yuchan=4
                    if(robotPos.y<ballPos.y-0.05):
                        self.at_mawariflag=False
                        self.at_toxzeroflag=False
                else:
                    balldis=get_distance(robotPos,ballPos)
                    if(balldis>0.1):
                
                        shoota=(ballPos.x-GOAL_BLUE_POS_CENTER.x) / (ballPos.y-GOAL_BLUE_POS_CENTER.y)
                        shooty=ballPos.y-0.05
                        shootx=shoota * (shooty-ballPos.y) + ballPos.x
                        returndata=get_motorOut(robotPos,Position(shootx,shooty))
                        
                    else:
                        returndata=get_motorOut(robotPos,ballPos)
                        self.yuchan=10
        return returndata