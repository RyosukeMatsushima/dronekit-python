#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# for WindowController
import tkinter

# Set up option parsing to get connection string
import argparse

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


        # Start SITL if no connection string specified
        if not self.connection_string:
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()


        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=True)

        
        self.arm_and_takeoff(20)

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
        print("task")    
        point1 = LocationGlobalRelative(self.click_point_lat, self.click_point_lon, 20)
        self.vehicle.simple_goto(point1, groundspeed=10)
        print("go to lat{0}, lon{1}".format(self.click_point_lat, self.click_point_lon))

    def deinit(self):
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()

        # Shut down simulator if it was started.
        if self.sitl:
            self.sitl.stop()