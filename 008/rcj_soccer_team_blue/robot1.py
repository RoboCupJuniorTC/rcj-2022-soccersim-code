from rcj_soccer_robot import *


class MyRobot1(RCJSoccerRobot):
    def act(self):
        if not self.ball_visible:
            return self.goto(
                OUR_GOAL + Vector2D(0, .12)
            ) and self.align_to(OPP_GOAL) and self.stop()

        if (OUR_GOAL + Vector2D(.15, 0) - self.ball_pos).length < .2:
            return self.attack()

        origin = OUR_GOAL  # - Vector2D(.82, 0)
        target = origin + origin.towards(self.ball_pos, .22)

        return self.goto(target) and self.stop()
