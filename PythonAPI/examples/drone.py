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

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import carla
import time


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
    vehicle_bp = blueprint_library.filter('lea')[0]  # Assuming 'model3' is the car model you want to spawn
    spawn_point = carla.Transform(carla.Location(x=80, y=0, z=1), carla.Rotation(yaw=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    
    try:
        # Set the spectator 2 meters above the vehicle
        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=2), transform.rotation))

        # Main loop
        while True:
            world.tick()  # Tick the simulation
            vehicle.apply_control_d()
            # Adjust the spectator's transform if needed
            transform = vehicle.get_transform()
            print(transform)
            spectator.set_transform(carla.Transform(transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))

            time.sleep(settings.fixed_delta_seconds)  # Wait for the next tick

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
