
from math import * 
import numpy as np

def angle_diff(a, b):
    diff = a - b
    return (diff + np.pi) % (2 * np.pi) - np.pi

class DWAControl():
    def __init__(self, glob_path):
        self.glob_path = glob_path
        

        self.max_speed = 8
        self.max_steer = np.deg2rad(21)
        self.max_steer_a = np.deg2rad(21)
        self.max_a = 0.0
        
        self.width = 0.68
        self.wheel_base = 0.75

        self.current_s = 0.0
        self.current_q = 0.0

        self.predict_time = 0.13
        self.search_frame = 5
        self.DWA_search_size = 21
        self.obstacle_avoid_dis = 1.20
        self.obstacle_avoid_dis_m = 0.90
        self.obstacle_avoid_dis_f = 0.50


        self.w1 = 10.0
        self.w2 = 4.0
        self.w3 = 5.0
        self.w4 = 15.5
        self.w5 = 0.8
        self.w6 = 2.0
        self.w7 = 0.0

        self.w8 = 25.0


        self.targeting = False

    def calc_dynamic_window(self, velocity, steer=0.0):
        DWA_velocity = velocity + self.max_a
        DWA_step_rot = 2 * self.max_steer_a / (self.DWA_search_size - 1)
            
        DWA_steer = [steer - self.max_steer_a + DWA_step_rot * i for i in range(self.DWA_search_size) if
                    abs(steer - self.max_steer_a + DWA_step_rot * i) <= self.max_steer]
    
        dw = [DWA_velocity, DWA_steer]
        return dw
        

    def convert_coordinate_l2g(self, dx, dy, dtheta):
        dtheta = -pi / 2 + dtheta
        trans_matrix = np.array([[cos(dtheta), -sin(dtheta), 0],
                                [sin(dtheta), cos(dtheta), 0],
                                [0, 0, 1]])
        dtheta = pi / 2 + dtheta
        return np.dot(trans_matrix, np.transpose([dx, dy, dtheta]))


    def generate_predict_point(self, x, y, velocity, steer, heading):  
        tan_dis = velocity * self.predict_time


        if steer != 0:
            R = self.wheel_base / tan(-steer)
        else:
            R = float('inf')

        theta, future_pos = 0.0, []
        for i in range(self.search_frame):
            if R == float('inf'):
                predict_point = [0, tan_dis * (i + 1), theta]                    
            else:
                theta += tan_dis / R
                predict_point = [R * (1 - cos(theta)), R * sin(theta), theta]
            pos = np.transpose(self.convert_coordinate_l2g(predict_point[0], predict_point[1], theta + heading))
            future_pos.append([x + pos[0], y + pos[1], pos[2]])
        return future_pos


    def calc_dist_far_global_path(self, pos): 
        
        rx_far, ry_far = self.glob_path.get_far_rxry()
        dis_far = abs(sqrt((pos[0] - rx_far) ** 2 + (pos[1] -ry_far) ** 2))

        return dis_far
    

    def calc_far_global_path_yaw(self, heading):
        forward = False
        ryaw_far = self.glob_path.get_far_yaw()
        diff = ((heading) - ryaw_far + np.pi) % (2*np.pi) - np.pi
        if diff >= np.deg2rad(60):
            forward = True
        return ryaw_far, forward
    

    def calc_curvature_cost(self, future_pos):
        total_yaw_change = 0.0
        for i in range(len(future_pos) - 1):
            theta1 = future_pos[i][2]
            theta2 = future_pos[i+1][2]
            dtheta = abs(theta2 - theta1)
            total_yaw_change += dtheta
        return total_yaw_change / (len(future_pos) - 1)


    def filter_obs(self, obs_xy):
        return [obs for obs in obs_xy if -1.0 < obs[1]]
    

    def collision_and_center_check(self, future_pos, obs_xy):
      
        blocker_ids = set()
        
        for obs_idx, obs in enumerate(obs_xy):
            for traj_pt in future_pos[:5]:
                dx = traj_pt[0] - obs[0]
                dy = traj_pt[1] - obs[1]
                dist = np.hypot(dx, dy)

                if dist <= 0.70:
                    blocker_ids.add(obs_idx)
                    break

        collision = len(blocker_ids) > 0
        return collision, blocker_ids

    def targeting_filter(self, target_xy):
        gp_separation_sq = self.glob_path.xy2sl(target_xy[0][0], target_xy[0][1], mode=0)
        separation_q = gp_separation_sq[1]
        print(f"separation_q {separation_q}")
        if abs(separation_q) >= 1.6:
            return False
        else:
            return True

    def DWA(self, x, y, s,q,heading, speed, steer, obs_xy=None, target_xy = None, no_gps = False):
 
        speed = 5.0
        if obs_xy is None or len(obs_xy) == 0:
            obs_xy = []

        if target_xy is None or len(target_xy) == 0:
            self.targeting = False
        else:
            true_targeting = self.targeting_filter(target_xy)
            if true_targeting:
                self.targeting = True
            else:
                self.targeting = False
        
    
        if no_gps:
            self.w1, self.w2, self.w5, self.w7, self.w6 = 0,0,0,0,2.0
        else:
            self.w1, self.w2, self.w3, self.w4, self.w5, self.w6, self.w7, self.w8 = 10.0,0.0, 5.0, 15.5, 0.0, 2.0, 0.0, 25.0
        

        self.current_s, self.current_q = s,q
        
        def cost_function(pos_end,pos_midf, pos_first, future_pos):
            gp_separation_sq = self.glob_path.xy2sl(pos_end[0], pos_end[1], mode=0)
            gp_separation = abs(gp_separation_sq[1])
                
            cost1 = gp_separation / 1.2 if gp_separation <= 1.2 else gp_separation * 10
           
            dis_far = self.calc_dist_far_global_path(pos_end)
 
            cost2 = dis_far / 100
            
            if obs_xy is None or len(obs_xy) ==0:
                cost3 ,cost4, cost8  = 0.0, 0.0, 0.0
            else:
                obs_d = min([np.hypot(pos_end[0] - obstacle[0], pos_end[1] - obstacle[1]) - self.width / 2 for obstacle in obs_xy])
                cost3 = (self.obstacle_avoid_dis - obs_d) / self.obstacle_avoid_dis if obs_d < self.obstacle_avoid_dis else 0
                
                obs_d_with_m = min([np.hypot(pos_midf[0] - obstacle[0], pos_midf[1] - obstacle[1]) - self.width / 2 for obstacle in obs_xy])  
                cost4 = (self.obstacle_avoid_dis_m - obs_d_with_m) / self.obstacle_avoid_dis_m if obs_d_with_m < self.obstacle_avoid_dis_m else 0
                
                obs_d_with_f = min([np.hypot(pos_first[0] - obstacle[0], pos_first[1] - obstacle[1]) - self.width / 2 for obstacle in obs_xy])  
                cost8 = (self.obstacle_avoid_dis_f - obs_d_with_f) / self.obstacle_avoid_dis_f if obs_d_with_f < self.obstacle_avoid_dis_f else 0


            cost5 = self.calc_curvature_cost(future_pos)
            
            ryaw_far, forward = self.calc_far_global_path_yaw(heading)
            yaw_error = abs(angle_diff(pos_end[2], ryaw_far))
            cost6 = yaw_error

            
            if self.targeting:
                target_d = np.hypot(pos_end[0] - target_xy[0][0], pos_end[1] - target_xy[0][1])
                cost7 = target_d / 0.15 if target_d <= 0.15 else target_d * 10
        
            else:
                cost7 = 0

            return self.w1 * cost1 + self.w2 * cost2 + self.w3 * cost3  + self.w4 * cost4 + self.w5 * cost5 + self.w6 * cost6 + self.w7 * cost7  + self.w8 * cost8     
                           

        optimal_cost = float('inf')
        best_actual = [speed, steer]
        candidate_paths, selected_path = [], []

        dw = self.calc_dynamic_window(best_actual[0])
        velocity = dw[0] 
        mid_index = len(dw[1]) // 2
        blocked_mid = [mid_index -2, mid_index-1, mid_index, mid_index+1, mid_index + 2]
        skipped = []
        skipped_blockers = {}

        for idx, steer in enumerate(dw[1]):
            future_pos = self.generate_predict_point(x, y, velocity, steer, heading)

            is_blocked, blocker_ids = self.collision_and_center_check(future_pos, obs_xy)
            if is_blocked:
                skipped.append(idx)
                skipped_blockers[idx] = blocker_ids
                continue

            candidate_paths.append((future_pos, idx))

        if all(i in skipped for i in blocked_mid):
            mid_blocker_ids = skipped_blockers.get(mid_index, set())

            left_skipped = 0
            right_skipped = 0

            for i in skipped:
                if i in blocked_mid:
                    continue

                blocker_ids = skipped_blockers.get(i, set())

                effective_blockers = blocker_ids - mid_blocker_ids

                if len(effective_blockers) == 0:
                    continue

                if i < mid_index:
                    left_skipped += 1
                elif i > mid_index:
                    right_skipped += 1

            if right_skipped > left_skipped and right_skipped > 6 and len(obs_xy) > 3:
                candidate_paths = [p for p in candidate_paths if p[1] < mid_index]
            elif left_skipped > right_skipped and left_skipped > 6 and len(obs_xy) > 3:
                candidate_paths = [p for p in candidate_paths if p[1] > mid_index]

        optimal_cost = float('inf')
        selected_path = []

        for future_pos, idx in candidate_paths:
            cost = cost_function(future_pos[-1], future_pos[2], future_pos[1], future_pos)
            if cost < optimal_cost:
                optimal_cost = cost
                selected_path = future_pos
        
        return selected_path, [p[0] for p in candidate_paths]
