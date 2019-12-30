import ObstacleDetector
import statistics
import math

class GuidingLaw():
    def __init__(self, d2t_yaw, d2t_distance, rf_max_range):
        self.rf_max_range = rf_max_range
        distance = min(d2t_distance, rf_max_range * 0.6)
        d2t_yaw = d2t_yaw * math.pi/180
        self.norm_target_power_x = math.cos(d2t_yaw) * distance
        self.norm_target_power_y = math.sin(d2t_yaw) * distance

        self.max_distance = rf_max_range * 0.8

        print("terget power_x {0} power_y {1}".format(self.norm_target_power_x, self.norm_target_power_y))

        self.norm_obstacle_power_x = 0.0
        self.norm_obstacle_power_y = 0.0

        print("rf_range_max {}".format(rf_max_range))

    def update_low2obstacle(self, yaw, distance_list):
        mediam_distance = statistics.median(distance_list)
        # self.max_distance = min(mediam_distance*0.6, self.max_distance)
        norm_distance = mediam_distance/self.rf_max_range

        norm_poewr = norm_distance**2 - 1

        norm_power_x = norm_poewr * math.cos(yaw)
        norm_power_y = norm_poewr * math.sin(yaw)

        self.norm_obstacle_power_x += norm_power_x
        self.norm_obstacle_power_y += norm_power_y

    def get_next_point(self):
        target_late = 1
        obstacle_late = 2.5

        power_x = self.norm_target_power_x * target_late + self.norm_obstacle_power_x * obstacle_late
        power_y = self.norm_target_power_y * target_late + self.norm_obstacle_power_y * obstacle_late

        print("power_x {0} power_y {1}".format(power_x, power_y))

        yaw2next_point = math.atan2(power_y, power_x) * 180/math.pi
        distance = math.sqrt(power_x**2 + power_y**2)

        distance = min(self.max_distance, distance)

        print("yaw2next_point {0} distance {1}".format(yaw2next_point, distance))

        return yaw2next_point, distance
            

class Low2Obstacle():
    def __init__(self):
        self.x = 0
        self.y = 0

class location_gps():
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon