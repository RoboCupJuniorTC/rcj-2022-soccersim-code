import math
from Mathematics.Vector2 import Vector2

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

def ball_position(ball_data, robot_pos, heading):
    if not ball_data:
        return Vector2(-100, -100)
    r = math.sqrt(1 / ball_data["strength"])
    ballAngle = - heading + math.atan2(ball_data["direction"][1], ball_data["direction"][0])
    x = r * math.cos(ballAngle)
    y = r * math.sin(ballAngle)
    return Vector2(*robot_pos) + Vector2(x, y)

def avg_ball_position(ball_data, robot_pos, heading, team_data):
    ball_positions = []
    if team_data:
        ball_positions = [list(t["ball_pos"]) for t in team_data if list(t["ball_pos"]) != [-100, -100]]
    
    own_ball_position = list(ball_position(ball_data, robot_pos, heading))
    if own_ball_position != [-100, -100]:
        ball_positions.append(own_ball_position)

    if not ball_positions:
        return Vector2(-100, -100)

    return Vector2(
        sum(b[0] for b in ball_positions)/len(ball_positions),
        sum(b[1] for b in ball_positions)/len(ball_positions)
    )
