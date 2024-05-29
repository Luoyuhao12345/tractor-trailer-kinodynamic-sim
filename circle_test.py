from kino_model import KinoModel
from visualization import Visualization
from constant import *
import math
import matplotlib.pyplot as plt


if __name__ == "__main__":
    car = KinoModel()
    visual = Visualization()
    sim_time = 18
    cur_time = 0
    v = 1
    delta = 30/180*math.pi
    while cur_time<sim_time:
        car.update(v, delta)
        # visual.plot_arrow(car.x1, car.y1, car.yaw1,
        #                   car.x2, car.y2, car.yaw2)
        plt.cla()
        plt.axis("equal")
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        visual.plot_car(car.x1, car.y1, car.yaw1,
                        car.x2, car.y2, car.yaw2)
        visual.traj_append(car.x1, car.y1, car.x2, car.y2)
        cur_time += dt
    visual.plot_traj()
