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

def pid_controller(desired, current, kp, ki, kd, output_limits):
    pid = PID(kp, ki, kd, setpoint=desired, output_limits=output_limits)
    return pid(current)


roll_desired = -0.1
roll_current = 0.0
roll_pid = PID(1.0, 0.01, 0.4, setpoint=roll_desired, output_limits=(-1, 1))


pitch_desired = 0.0
pitch_current = 0.0
pitch_pid = PID(1.0 ,0.01, 0.4, setpoint=pitch_desired, output_limits=(-1, 1))


z_desired = 5.0  # Desired altitude (adjust as needed)
z_current = 0.0   # Current altitude (initially at ground level)
z_pid = PID(1, 0.01, 0.4, setpoint=z_desired, output_limits=(0, 1))

# Example PID Controller for Yaw
yaw_desired = 0
yaw_current = 0
yaw_pid = PID(1.0, 0.01, 0.4, setpoint=yaw_desired, output_limits=(-1, 1))
 

x_desired = 50.0  # Desired altitude (adjust as needed)
x_current = 0.0   # Current altitude (initially at ground level)
x_pid = PID(.3, 0, 1, setpoint=x_desired, output_limits=(-1, 1))

y_desired = 12.0  # Desired altitude (adjust as needed)
y_current = 0.0   # Current altitude (initially at ground level)
y_pid = PID(.3, 0, 1, setpoint=y_desired, output_limits=(-1, 1))



def main():
    

    # Connect to the CARLA simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and set to synchronous mode
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # Set the time step for the simulation
    world.apply_settings(settings)

    # Spawn a vehicle named 'lea'
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('lea')[0]  
    spawn_point = carla.Transform(carla.Location(x=0, y=10, z=1), carla.Rotation(yaw=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    
    

    try:
        
        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=2), transform.rotation))
        # Main loop
        while True:
            world.tick()  # Tick the simulation
           
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
            x_output = x_pid(transform.location.x+random.uniform(-.02,0.02))
            #pitch_pid.setpoint=-x_output*10
            y_output = y_pid(transform.location.y+random.uniform(-.02,0.02))
            #roll_pid.setpoint=y_output*10

            roll_output = roll_pid(transform.rotation.roll+random.uniform(-.002,0.002))
            pitch_output = pitch_pid(transform.rotation.pitch+random.uniform(-.002,0.002))
            yaw_output = yaw_pid(transform.rotation.yaw)
           
            throttle_output = z_pid(transform.location.z+random.uniform(-.02,0.02))
           
            print(transform.location,y_output)
            

            speed_set=1
            front_left =  speed_set*throttle_output + (roll_output + pitch_output -yaw_output)
            front_right = speed_set*throttle_output + (-roll_output + pitch_output + yaw_output )
            rear_left = speed_set*throttle_output + (+roll_output - pitch_output+yaw_output)
            rear_right = speed_set*throttle_output + (-roll_output - pitch_output -yaw_output)
            
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
