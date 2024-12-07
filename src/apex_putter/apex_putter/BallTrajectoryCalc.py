import numpy as np

class BallTrajectoryCalculator:
    def __init__(self, ball_position, hole_position):
        self.ball_position = np.array(ball_position)
        self.hole_position = np.array(hole_position)

    def calculate_trajectory(self):
        direction = self.hole_position - self.ball_position
        distance = np.linalg.norm(direction)
        unit_direction = direction / distance
        return unit_direction, distance
