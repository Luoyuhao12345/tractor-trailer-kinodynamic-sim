from kino_model import KinoModel
from visualization import Visualization
from constant import *
import math
import matplotlib.pyplot as plt
import numpy as np
from pure_pursuit1 import pure_pursuit_steer_control, TargetCourse


if __name__ == "__main__":
    car = KinoModel(x1=-0.0, y1=0.0, yaw1=0.0)
    visual = Visualization()

    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    target_speed = 10.0 / 3.6  # [m/s]
    T = 100.0  # max simulation time

    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(car)
    lastIndex = len(cx) - 1
    time = 0.0

    while T >= time and lastIndex > target_ind:
        delta, target_ind = pure_pursuit_steer_control(
            car, target_course, target_ind)
        car.update(target_speed, delta)
        time += dt

        plt.cla()
        plt.axis("equal")
        plt.xlim(-10, 60)
        plt.ylim(-50, 50)
        plt.plot(cx, cy, "-k", label="course")
        visual.plot_car(car.x1, car.y1, car.yaw1,
                        car.x2, car.y2, car.yaw2)
        visual.traj_append(car.x1, car.y1, car.x2, car.y2)

    visual.plot_traj()
