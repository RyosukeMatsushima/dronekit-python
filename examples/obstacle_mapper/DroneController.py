#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# for WindowController
import tkinter

# Set up option parsing to get connection string
import argparse
import math
import statistics

import ObstacleDetector
from GuidingLaw import GuidingLaw
from RequestState import RequestState

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

distance_sensor_state = None

class DroneController():
    def __init__(self):
        self.click_point_lat = None
        self.click_point_lon = None

        self.altitude = 0
        self.draw_obstacle_stock = []
        self.requestState = RequestState.STOP_DRONE
        
        # self.arm_and_takeoff(self.alttiude)

        print("Set default/target airspeed to 3")
        vehicle.airspeed = 3

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def go_to_click_point(self):
        vehicle.mode = VehicleMode("GUIDED")
        print("task")
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, self.click_point_lat, self.click_point_lon)
        print("d2c yaw {0} distance {1}".format(result['azimuth1'], result['distance']))

        point1 = LocationGlobalRelative(self.click_point_lat, self.click_point_lon, self.altitude)
        vehicle.simple_goto(point1, groundspeed=10)
        print("go to lat{0}, lon{1}".format(self.click_point_lat, self.click_point_lon))

    def guid_to_click_point(self):
        vehicle.mode = VehicleMode("GUIDED")
        while not self.is_reached(self.click_point_lat, self.click_point_lon, 3):
            time.sleep(3)
            next_point = self.get_next_point_gps()
            
            if self.should_stop():
                print("stop drone")
                break

            print("go next point")
            self.go_to(next_point['lat'], next_point['lon'], self.altitude)

        print("reached click point")


    def go_to(self, lat, lon, alt):
        point1 = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point1, groundspeed=10)
        print("go to lat{0}, lon{1}".format(self.click_point_lat, self.click_point_lon))

        while vehicle.mode.name=="GUIDED":
            time.sleep(2)
            print('I am going')
            if self.is_reached(lat, lon, 2):
                print('reached to point')
                break
            
            if self.should_stop():
                print("stop drone")
                break


    def is_reached(self, target_lat, target_lon, threshold):
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, target_lat, target_lon)
        return result['distance'] < threshold

    def should_stop(self):
        is_stop_mode = vehicle.mode.name != "GUIDED"
        is_stop_request = self.requestState == RequestState.STOP_DRONE
        return is_stop_mode or is_stop_request

        
    def condition_yaw(self, heading, relative=False):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

        For more information see: 
        http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        """
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)

    def get_next_point_gps(self):
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, self.click_point_lat, self.click_point_lon)
        print("d2c yaw {0} distance {1}".format(result['azimuth1'], result['distance']))
        guidingLaw = GuidingLaw(result['azimuth1'], result['distance'], float(distance_sensor_state.max_distance)/100)
        for i in range(0, 37):
            time.sleep(0.5)

            
            if self.should_stop():
                print("stop drone")
                break

            obstacle_distance_list = []
            for j in range(0, 3):
                time.sleep(0.2)
                distance = self.get_obstacle_distance()
                
                # distance = None #for test
                # if 0 < vehicle.attitude.yaw < 30 * math.pi/180:  #for test
                #     distance = 6
                print("distance {}".format(distance))

                if not distance == None:
                    obstacle_distance_list.append(distance)

            print("distance{}".format(obstacle_distance_list))
            # if len(obstacle_distance_list) > 30:
            if not obstacle_distance_list == []:
                yaw = vehicle.attitude.yaw
                guidingLaw.update_low2obstacle(yaw, obstacle_distance_list)
                median_distance = statistics.median(obstacle_distance_list)
                self.add_obstacle_to_map(median_distance, yaw)

            self.condition_yaw(10, True)

        yaw2next_point, distance = guidingLaw.get_next_point()
        self.condition_yaw(yaw2next_point)
        time.sleep(2)
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon

        return ObstacleDetector.vincenty_direct(lat, lon, yaw2next_point, distance)

    @vehicle.on_message('DISTANCE_SENSOR')
    def listener(self, name, message):
        global distance_sensor_state
        distance_sensor_state = message
        # print(message.max_distance)
        # print(distance_sensor_state.current_distance)
        
    def get_obstacle_distance(self):
        if distance_sensor_state == None or distance_sensor_state.current_distance == None:
            print("distance_sensor_state is None")
            return None
        rngfnd_distance = float(distance_sensor_state.current_distance)/100
        distance_sensor_state.current_distance = None
        print("law distance {}".format(rngfnd_distance))
        rngfnd1_max = float(distance_sensor_state.max_distance)/100

        altitude = vehicle.location.global_frame.alt
        pitch = vehicle.attitude.pitch
        rngfnd1_max = min(rngfnd1_max, abs(altitude/math.sin(pitch)))
        if rngfnd_distance == None:
            return None
        if rngfnd_distance > rngfnd1_max:
            print("out of range max{}".format(rngfnd1_max))
            return None

        drone2obstacle = rngfnd_distance * math.cos(vehicle.attitude.pitch)
        return drone2obstacle

    def add_obstacle_to_map(self, distance, yaw):
        
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        yaw *= 180/math.pi
        result = ObstacleDetector.vincenty_direct(lat, lon, yaw, distance)
        self.draw_obstacle_stock.append(result)

    def get_draw_obstacle_stock(self):
        stock = self.draw_obstacle_stock
        self.draw_obstacle_stock = []

        return stock

    def get_drone_position(self):
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon

        return lat, lon
        

    def deinit(self):
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        vehicle.close()

        # Shut down simulator if it was started.
        if self.sitl:
            self.sitl.stop()