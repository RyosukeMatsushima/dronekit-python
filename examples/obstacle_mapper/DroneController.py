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

class DroneController():
    def __init__(self):
        self.click_point_lat = None
        self.click_point_lon = None


        self.parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
        self.parser.add_argument('--connect',
                            help="Vehicle connection target string. If not specified, SITL automatically started and used.")
        self.args = self.parser.parse_args()

        self.connection_string = self.args.connect
        self.sitl = None

        self.draw_obstacle_stock = []

        self.requestState = RequestState.STOP_DRONE


        # Start SITL if no connection string specified
        if not self.connection_string:
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()


        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=True)

        
        # self.arm_and_takeoff(20)

        print("Set default/target airspeed to 3")
        self.vehicle.airspeed = 3

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def go_to_click_point(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        print("task")
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, self.click_point_lat, self.click_point_lon)
        print("d2c yaw {0} distance {1}".format(result['azimuth1'], result['distance']))

        point1 = LocationGlobalRelative(self.click_point_lat, self.click_point_lon, 20)
        self.vehicle.simple_goto(point1, groundspeed=10)
        print("go to lat{0}, lon{1}".format(self.click_point_lat, self.click_point_lon))

    def guid_to_click_point(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.is_reached(self.click_point_lat, self.click_point_lon, 3):
            time.sleep(3)
            next_point = self.get_next_point_gps()
            
            if self.should_stop():
                print("stop drone")
                break

            print("go next point")
            self.go_to(next_point['lat'], next_point['lon'], 20)

        print("reached click point")


    def go_to(self, lat, lon, alt):
        point1 = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(point1, groundspeed=10)
        print("go to lat{0}, lon{1}".format(self.click_point_lat, self.click_point_lon))

        while self.vehicle.mode.name=="GUIDED":
            time.sleep(2)
            print('I am going')
            if self.is_reached(lat, lon, 2):
                print('reached to point')
                break
            
            if self.should_stop():
                print("stop drone")
                break


    def is_reached(self, target_lat, target_lon, threshold):
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, target_lat, target_lon)
        return result['distance'] < threshold

    def should_stop(self):
        is_stop_mode = self.vehicle.mode.name != "GUIDED"
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
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def get_next_point_gps(self):
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        result = ObstacleDetector.vincenty_inverse(lat, lon, self.click_point_lat, self.click_point_lon)
        print("d2c yaw {0} distance {1}".format(result['azimuth1'], result['distance']))
        guidingLaw = GuidingLaw(result['azimuth1'], result['distance'], float(self.vehicle.parameters.get('RNGFND1_MAX_CM'))/100)
        for i in range(0, 37):
            time.sleep(0.5)

            
            if self.should_stop():
                print("stop drone")
                break

            obstacle_distance_list = []
            for j in range(0, 100):
                distance = self.get_obstacle_distance()

                # if 0 < self.vehicle.attitude.yaw < 30 * math.pi/180:  #for test
                #     distance = 6

                if not distance == None:
                    obstacle_distance_list.append(distance)

            if len(obstacle_distance_list) > 30:
                yaw = self.vehicle.attitude.yaw
                guidingLaw.update_low2obstacle(yaw, obstacle_distance_list)
                median_distance = statistics.median(obstacle_distance_list)
                self.add_obstacle_to_map(median_distance, yaw)

            self.condition_yaw(10, True)

        yaw2next_point, distance = guidingLaw.get_next_point()
        self.condition_yaw(yaw2next_point)
        time.sleep(2)
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon

        return ObstacleDetector.vincenty_direct(lat, lon, yaw2next_point, distance)
        
    def get_obstacle_distance(self):
        rngfnd_distance = self.vehicle.rangefinder.distance
        rngfnd1_max = float(self.vehicle.parameters.get('RNGFND1_MAX_CM'))/100

        altitude = self.vehicle.location.global_frame.alt
        pitch = self.vehicle.attitude.pitch
        rngfnd1_max = min(rngfnd1_max, altitude/math.sin(pitch))
        if rngfnd_distance == None or rngfnd_distance > rngfnd1_max:
            return None

        drone2obstacle = self.vehicle.rangefinder.distance * math.sin(self.vehicle.attitude.pitch)
        return drone2obstacle

    def add_obstacle_to_map(self, distance, yaw):
        
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        yaw *= 180/math.pi
        result = ObstacleDetector.vincenty_direct(lat, lon, yaw, distance)
        self.draw_obstacle_stock.append(result)

    def get_draw_obstacle_stock(self):
        stock = self.draw_obstacle_stock
        self.draw_obstacle_stock = []

        return stock

    def get_drone_position(self):
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon

        return lat, lon
        

    def deinit(self):
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()

        # Shut down simulator if it was started.
        if self.sitl:
            self.sitl.stop()