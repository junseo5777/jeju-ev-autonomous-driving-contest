

import numpy as np
from math import *

class Control:
    def __init__(self):
        self.last_q = 0
        self.WB = 0.75
        self.k_error = 1.6

    def find_nearest_index(self, x, y, rx, ry):
        dists = [hypot(px - x, py - y) for px, py in zip(rx, ry)]
        return int(np.argmin(dists))

    def find_pp_target(self, x, y, rx, ry, Ld):
        nearest_idx = self.find_nearest_index(x, y, rx, ry)

        dist_sum = 0.0
        target_idx = nearest_idx

        for i in range(nearest_idx, len(rx) - 1):
            seg = hypot(rx[i+1] - rx[i], ry[i+1] - ry[i])
            dist_sum += seg
            if dist_sum >= Ld:
                target_idx = i + 1
                break

        return target_idx
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi
    
    def pp_steer(self, x, y, heading, goal, speed=50):
        rx = goal[0]
        ry = goal[1]

        if len(rx) < 2 or len(ry) < 2:
            return 0.0, [x, y]

        Ld = 1.2 * speed + 1.0

        target_idx = self.find_pp_target(x, y, rx, ry, Ld)

        tx = rx[target_idx]
        ty = ry[target_idx]
        goal_point = [tx, ty]
        theta = self.normalize_angle(np.arctan2(ty-y, tx-x) - heading)
        theta = self.normalize_angle(theta)

        delta = np.arctan2(2 * self.WB * np.sin(theta), Ld)

        steer = np.rad2deg(delta)

        
        return float(np.clip(steer, -23, 23)), goal_point
    
    
    def stanley(self, x, y, heading, wb, path, speed):
        if len(path) < 3:
            return 0.0

        selected_rx = path[0]
        selected_ry = path[1]
        selected_ryaw = path[2]

        if len(selected_rx) < 2 or len(selected_ry) < 2 or len(selected_ryaw) < 2:
            return 0.0

        s_x = x + wb * np.cos(heading)
        s_y = y + wb * np.sin(heading)

        dis = 1e5
        idx = 0
        for i in range(len(selected_rx)):
            d = sqrt((s_x - selected_rx[i]) ** 2 + (s_y - selected_ry[i]) ** 2)
            if d < dis:
                dis = d
                idx = i


        path_yaw = selected_ryaw[idx]
        
        heading_error = self.normalize_angle(path_yaw - heading)


        ref_x = np.asarray(path[0], dtype=float)
        ref_y = np.asarray(path[1], dtype=float)


        dx = s_x - ref_x
        dy = s_y - ref_y
        front_axle_vec = np.array([
                -np.cos(heading + np.pi/2),
                -np.sin(heading + np.pi/2)], dtype=float)
        
        err_vec = np.array([dx[idx], dy[idx]], dtype=float)

        error_front_axle = float(np.dot(err_vec, front_axle_vec))


        delta = heading_error + np.arctan2(self.k_error * error_front_axle, speed + 1.0)
        stanley_steer = np.rad2deg(delta)

        return float(np.clip(stanley_steer, -23, 23))
    