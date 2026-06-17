

import os
import sys

import numpy as np

from dwa import DWAControl
from global_path import GlobalPath
from PP_STANLEY_for_dwa import Control
import cubic_spline_planner
from math import *

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

class Path_Tracking_DWA:
    def __init__(self, path, file=0):
        self.PP = Control()
        
        if file == 0:
            glob_path = path
        self.global_path = glob_path
        self.path_planner = DWAControl(glob_path) 
        self.wheel_base = 0.75
    

    def gps_tracking(self, pose, s, q, heading, speed=0.0, steer=0.0, obs_xy=None, target_xy=None, no_gps = False):
        if obs_xy is None:
            obs_xy = [] 
        if target_xy is None:
            target_xy = []
            
        x, y = pose[0], pose[1] 
        rx_far, ry_far = self.global_path.get_far_rxry()
        goal_point =  [[rx_far], [ry_far]]
        try:
            selected_path, candidate_paths = self.path_planner.DWA(x, y, s,q ,heading, speed, steer, obs_xy, target_xy, no_gps)
        except:
            print("DWA 오류")

            pure_pursuit_val, goal = self.PP.pp_steer(x, y, heading, goal_point)
            return 1.2 * pure_pursuit_val, [], [], goal
        
        if selected_path is None or len(selected_path) < 3:
            pure_pursuit_val, goal = self.PP.pp_steer(x, y, heading, goal_point)
            print("경로없어용")
            return 1.2 * pure_pursuit_val, [], [], goal


        Kp = 2.0
        K_stanley = 2.0


        selected_path_xy = [list(x) for x in zip(*selected_path)][0:2] 
        try:
            if len(selected_path_xy[0]) >= 2:
                selected_rx, selected_ry, selected_ryaw, selected_rk, _, selected_s = cubic_spline_planner.calc_spline_course(selected_path_xy[0], selected_path_xy[1], ds=0.1)
                goal = [selected_rx[0:], selected_ry[0:]]

        except:
            x_sp, y_sp = selected_path[-1][0], selected_path[-1][1]
            goal = [[x_sp], [y_sp]]

            path_x1 = selected_path[1][0]
            path_y1 = selected_path[1][1]
            path_x2 = selected_path[2][0]
            path_y2 = selected_path[2][1]
            dif = atan2(path_y2 - path_y1, path_x2 - path_x1) 
            dif = heading - dif
            dif= (dif + pi) % (2*pi) - pi
            stanley_steer = np.rad2deg(dif)


            pure_pursuit_val, goal_point = self.PP.pp_steer(x, y, heading, goal)
            P_steer =  pure_pursuit_val
            target_steer = Kp * P_steer + K_stanley * np.clip(stanley_steer,-23,23) 
            return np.clip(-target_steer, -23, 23), selected_path, candidate_paths, goal_point
        

        gps2fw = 0.75

        path = [selected_rx, selected_ry, selected_ryaw]
        stanley_steer = self.PP.stanley(x, y, heading, gps2fw, path, speed)


        pure_pursuit_val, goal_point = self.PP.pp_steer(x, y, heading, goal,speed)
        P_steer =  pure_pursuit_val
        target_steer = P_steer * 1.4 + stanley_steer * 0.4
        print(f"P_steer: {P_steer}, dis : 제외, stanley_dif : {stanley_steer}")

        
        return np.clip(target_steer, -23, 23), selected_path, candidate_paths, goal_point