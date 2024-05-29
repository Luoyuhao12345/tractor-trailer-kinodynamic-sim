import matplotlib.pyplot as plt
from math import cos, sin, tan, pi
from constant import *
from scipy.spatial.transform import Rotation as Rot
import numpy as np

# tractor
LL1 = L1*1.2
W1 = LL1/1.618  # width of car
LF1 = 0.7*LL1  # distance from rear to vehicle front end
LB1 = 0.3*LL1  # distance from rear to vehicle back end
# vehicle rectangle vertices
VRX1 = [LF1, LF1, -LB1, -LB1, LF1]
VRY1 = [W1 / 2, -W1 / 2, -W1 / 2, W1 / 2, W1 / 2]

# trailer
LL2 = L1*0.8
W2 = LL2/1.618  # width of car
LF2 = 0.7*LL2  # distance from rear to vehicle front end
LB2 = 0.3*LL2  # distance from rear to vehicle back end
# vehicle rectangle vertices
VRX2 = [LF2, LF2, -LB2, -LB2, LF2]
VRY2 = [W2 / 2, -W2 / 2, -W2 / 2, W2 / 2, W2 / 2]


def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


class Visualization:
    def __init__(self):
        self.xs1 = []
        self.ys1 = []
        self.xs2 = []
        self.ys2 = []

    def plot_arrow(self, x1, y1, yaw1, x2, y2, yaw2):
        plt.cla()
        plt.axis("equal")
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.arrow(x1, y1, L1*cos(yaw1), L1*sin(yaw1),
                  fc="r", ec="k", head_width=0.5, head_length=1)
        plt.arrow(x2, y2, L2*cos(yaw2), L1*sin(yaw2),
                  fc="b", ec="k", head_width=0.5, head_length=1)
        plt.pause(0.02)

    def plot_car(self, x1, y1, yaw1, x2, y2, yaw2):
        car_color1 = '-r'
        rot1 = rot_mat_2d(-yaw1)
        car_outline_x1, car_outline_y1 = [], []
        for rx, ry in zip(VRX1, VRY1):
            converted_xy = np.stack([rx, ry]).T @ rot1
            car_outline_x1.append(converted_xy[0] + x1)
            car_outline_y1.append(converted_xy[1] + y1)

        car_color2 = '-b'
        rot2 = rot_mat_2d(-yaw2)
        car_outline_x2, car_outline_y2 = [], []
        for rx, ry in zip(VRX2, VRY2):
            converted_xy = np.stack([rx, ry]).T @ rot2
            car_outline_x2.append(converted_xy[0] + x2)
            car_outline_y2.append(converted_xy[1] + y2)

        # plt.cla()
        # plt.axis("equal")
        # plt.xlim(-10, 10)
        # plt.ylim(-10, 10)
        plt.arrow(x1, y1, -Lt*cos(yaw1), -Lt*sin(yaw1),
                  fc="r", ec="k", head_width=0.1, head_length=0.1)
        plt.arrow(x2, y2, L2*cos(yaw2), L2*sin(yaw2),
                  fc="b", ec="k", head_width=0.1, head_length=0.1)
        plt.plot(car_outline_x1, car_outline_y1, car_color1)
        plt.plot(car_outline_x2, car_outline_y2, car_color2)
        plt.pause(0.02)

    def traj_append(self, x1, y1, x2, y2):
        self.xs1.append(x1)
        self.ys1.append(y1)
        self.xs2.append(x2)
        self.ys2.append(y2)

    def plot_traj(self):
        plt.cla()
        plt.subplots(1)
        plt.axis("equal")
        plt.plot(self.xs1, self.ys1, marker='.', label="tractor")
        plt.plot(self.xs2, self.ys2, marker='.', label="trailer")
        plt.legend(loc="upper right")
        plt.grid(True)
        plt.show()
