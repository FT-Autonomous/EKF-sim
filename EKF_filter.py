import numpy as np
from time import time
from collections import deque

    
def multiply(mu1, var1, mu2, var2, var_min = 0.001):
    if var1 <= 0: var1 = var_min 
    if var2 <= 0: var2 = var_min 
    mean = (var1*mu2 + var2*mu1) / (var1+var2)
    variance = 1 / (1/var1 + 1/var2)
    return (mean, variance)

class Filter:
    def __init__(self, init_x, init_y):
        self.x_1 = init_x
        self.y_1 = init_y


    def kalman_filter(self, odom, lidar):
        # True in sim, not in real life
        dt = 1 

        # Predict Step
        v_x, v_y = odom
        pred_x, pred_y = self.x_1 + v_x, self.y_1 + v_y

        # Measurement Step
        lidar_x, lidar_y = lidar
        measured_x, measured_y = 400 - lidar_x, 300 - lidar_y

        x = pred_x + measured_x
        y = pred_y + measured_y
        x /= 2
        y /= 2
    
        return x, y
        

    def calc_pose(self, odom, lidar):
        self.x_1, self.y_1 = self.kalman_filter(odom, lidar)

        return self.x_1, self.y_1

