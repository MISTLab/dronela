#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""




import glob
import os
import sys
import time
import math
#from pymavlink import mavutil
import random
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import time

from simple_pid import PID
import numpy as np


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class Odometry:
    def __init__(self, imu_sensor):
        self.imu_sensor = imu_sensor
        self.previous_timestamp = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x=0
        self.y=0
        self.z=0

    def imu_callback(self, imu_data):
        current_timestamp = imu_data.timestamp
        delta_time = (current_timestamp - self.previous_timestamp)   # Convert to seconds

        # Extract angular velocity
        angular_velocity = imu_data.gyroscope

        # Integrate angular velocity to obtain angles
        self.roll += angular_velocity.x * delta_time
        self.pitch += angular_velocity.y * delta_time
        self.yaw += angular_velocity.z * delta_time

        acceleration=imu_data.accelerometer
        # Update previous timestamp
        self.previous_timestamp = current_timestamp

        # Use the calculated angles as needed
        print(delta_time,current_timestamp)
        print("Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(math.degrees(self.roll), math.degrees(self.pitch),math.degrees( self.yaw)))



# Example PID Controller for Yaw
yaw_desired = 0
yaw_pid = PID(1.0, 0, 1, setpoint=yaw_desired, output_limits=(-1, 1))
 

pitch_desired = 0 
pitch_pid = PID(1, 0, 1, setpoint=pitch_desired, output_limits=(-1, 1))

roll_desired = 0
roll_pid = PID(1, 0, 1, setpoint=roll_desired, output_limits=(-1, 1))



vx_desired = .51  
vx_pid = PID(1, 0.001, 1, setpoint=vx_desired, output_limits=(-20, 20))


vy_desired = 0 
vy_pid = PID(1, 0.001, 1, setpoint=vy_desired, output_limits=(-20, 20))

vz_desired = 10 # Desired altitude (adjust as needed)
vz_current = 0.0   # Current altitude (initially at ground level)
vz_pid = PID(1, 0.4, 0, setpoint=vz_desired, output_limits=(0, 1))



def main():
    
    # Connect to the CARLA simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and set to synchronous mode
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.01  # Set the time step for the simulation
    world.apply_settings(settings)

    # Spawn a vehicle named 'lea'
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('lea')[0]  
    spawn_point = carla.Transform(carla.Location(x=0, y=10, z=1), carla.Rotation(yaw=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  # Adjust these values as needed
    # Create the IMU sensor
    imu_bp = world.get_blueprint_library().find('sensor.other.imu')
    imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    vehicle_odom=Odometry(imu_sensor)
    imu_sensor.listen(lambda imu_data: vehicle_odom.imu_callback(imu_data))
    

    try: 
        spectator = world.get_spectator()
        vehicle_transform = vehicle.get_transform()
        vehicle_velocity= vehicle.get_velocity()
        spectator.set_transform(carla.Transform(vehicle_transform.location + carla.Location(z=2), vehicle_transform.rotation))
        # Main loop
        while True:

           
            time.sleep( settings.fixed_delta_seconds)
            world.tick()  # Tick the simulation
  
           
            vehicle_transform = vehicle.get_transform()
            vehicle_velocity= vehicle.get_velocity()
            spectator.set_transform(carla.Transform(vehicle_transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
            
            vx_output =vx_pid(vehicle_velocity.x)
            pitch_pid.setpoint=0#-vx_output
            pitch_output =pitch_pid(vehicle_transform.rotation.pitch)#1/(100*math.sin(math.radians(abs(vehicle_transform.rotation.pitch)))+0.1)*
           
            vy_output =vy_pid(vehicle_velocity.y)
            roll_pid.setpoint=2#vy_output
            roll_output =  roll_pid(vehicle_transform.rotation.roll)
            # roll_pid.setpoint=clamp(roll_output,-45,45)

            # roll_output = roll_pid(vehicle_transform.rotation.roll)
            # pitch_output = pitch_pid(vehicle_transform.rotation.pitch)
            yaw_output = 0.1#yaw_pid(vehicle_transform.rotation.yaw)
           
            throttle_output =vz_pid(vehicle_velocity.z)
           
            #print(vehicle_velocity)
            print(vehicle_transform)
            #print(roll_output)
            #print("pitch",vehicle_transform.rotation.pitch)

            speed_set=1
            front_left =  speed_set*throttle_output  + +roll_output + +pitch_output - yaw_output
            front_right = speed_set*throttle_output  + -roll_output + +pitch_output + yaw_output 
            rear_left = speed_set*throttle_output    + +roll_output + -pitch_output + yaw_output
            rear_right = speed_set*throttle_output   + -roll_output + -pitch_output -yaw_output
            
            # front_left=1
            # rear_right=1
            # front_right=1
            # rear_left=1



            vehicle.apply_motor_speed(front_left*3000,front_right*3000,rear_left*3000,rear_right*3000)
            # vehicle.apply_control_d()
                            
    finally:
        if vehicle is not None: 
          
            vehicle.destroy()
            print('Vehicle destroyed.')
        # If something goes wrong, try to disable synchronous mode
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print('Synchronous mode disabled.')

if __name__ == '__main__':
    main()
