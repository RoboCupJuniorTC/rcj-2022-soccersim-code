from rcj_soccer_robot import *


class MyRobot2(RCJSoccerRobot):
    def act(self):
        if not self.ball_visible:
            return self.goto(
                OUR_GOAL + Vector2D(.12, 0)
            ) and self.align_to(OPP_GOAL) and self.stop()

        if (self.robot_pos - self.ball_pos).length < .2:
            return self.attack()

        origin = OUR_GOAL  # - Vector2D(.82, 0)
        target = origin + origin.towards(self.ball_pos, .25)

        return self.goto(target) and self.stop()
