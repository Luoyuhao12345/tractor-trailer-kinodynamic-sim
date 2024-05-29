from math import cos, sin, tan, pi, hypot
from constant import *


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi


class KinoModel:
    def __init__(self, x1=0, y1=0, yaw1=0, x2=0, y2=0, yaw2=0):
        self.x1 = x1
        self.y1 = y1
        self.yaw1 = yaw1
        self.x2 = x2-Lt-L2
        self.y2 = y2
        self.yaw2 = yaw2

    def update(self, v, delta):
        x1_dot = v*cos(self.yaw1)
        y1_dot = v*sin(self.yaw1)
        yaw1_dot = v*tan(delta)/L1
        yaw2_dot = v*(sin(self.yaw1-self.yaw2)-Lt/L1*tan(delta)*cos(self.yaw1-self.yaw2))/L2
        self.x1 += x1_dot*dt
        self.y1 += y1_dot*dt
        self.yaw1 += yaw1_dot*dt
        self.yaw2 += yaw2_dot*dt
        self.yaw1 = pi_2_pi(self.yaw1)
        self.yaw2 = pi_2_pi(self.yaw2)
        self.x2 = self.x1 - Lt * cos(self.yaw1) - L2 * cos(self.yaw2)
        self.y2 = self.y1 - Lt * sin(self.yaw1) - L2 * sin(self.yaw2)

    def calc_distance(self, point_x, point_y):
        dx = self.x1 - point_x
        dy = self.y1 - point_y
        return hypot(dx, dy)

