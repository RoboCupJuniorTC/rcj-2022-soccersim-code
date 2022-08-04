from rcj_soccer_robot import *


class MyRobot3(RCJSoccerRobot):
    def act(self):
        if not self.ball_visible:
            return self.goto(
                OUR_GOAL - Vector2D(0, .12)
            ) and self.align_to(OPP_GOAL) and self.stop()

        return self.attack()
