


import glob
import os
import sys
import time
import math

import pygame
from pygame.locals import QUIT,KEYUP, KEYDOWN, K_ESCAPE, K_w, K_a, K_s, K_d, K_SPACE, K_LEFT, K_RIGHT, K_UP, K_DOWN

import numpy as np
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



class Drone_sensors_ctl:
    def __init__(self,world, 
                 drone_bp, spawn_point,
                 camera_bp=None,camera_transform=None, screen=None,
                 imu_bp=None, imu_transform=None,
                 lidar_bp=None, lidar_transform=None,
                 controller=None
                 ):
        self.drone= world.spawn_actor(drone_bp, spawn_point)
        self.screen=screen
        if (camera_bp!=None):
            if(camera_transform==None):
                camera_transform=carla.Transform(carla.Location(x=-2 ,z=1))
            self.camera=world.spawn_actor(camera_bp, camera_transform, attach_to=self.drone)
            self.camera.listen(lambda image: self.on_camera_image(image,screen))
    

        else:
            self.camera=None
        if (lidar_bp!=None):
            if(lidar_transform==None):
                lidar_transform= carla.Transform(carla.Location(x=0,y=0, z=-0.20))
            self.lidar=world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.drone)
        else:
            self.lidar=None
        if (imu_bp!=None):
            if(imu_transform==None):
                imu_transform= carla.Transform(carla.Location(x=0,y=0, z=0))
            self.imu=world.spawn_actor(imu_bp, imu_transform, attach_to=self.drone)
            self.angular_rate = carla.Vector3D(x=0,y=0,z=0)
            self.imu.listen(lambda imu_data: self.imu_callback(imu_data))
        else:
            self.imu=None
            self.angular_rate=None
        self.controller=controller

    def on_camera_image(self,image,screen):
    # Get the raw image data
        image_data = image.raw_data

        # Convert the image data to a NumPy array
        image_array = np.frombuffer(image_data, dtype=np.uint8)

        # Reshape the array to match the image dimensions and channels
        image_array = image_array.reshape((image.height, image.width, 4))

        # Extract RGB channels and discard the alpha channel
        rgb_image = image_array[:, :, :3]

        # Correct color channels order
        rgb_image = np.flip(rgb_image, axis=2)

        # Rotate the image 90 degrees counter-clockwise
        rgb_image = np.rot90(rgb_image)

        # Resize the image to match the Pygame window dimensions
        pygame_surface = pygame.surfarray.make_surface(rgb_image)
        pygame_surface = pygame.transform.scale(pygame_surface, (2000, 1500))
        if screen!=None:
            # Blit the Pygame surface onto the entire screen
            screen.blit(pygame_surface, (0, 0))

            # Update the Pygame display
            pygame.display.flip()


    def imu_callback(self, imu_data):
        self.angular_rate = imu_data.gyroscope

    def get_angular_velocity(self):
        return self.angular_rate
    
    def destroy_all(self):
        if (self.imu!=None):
            self.imu.destroy()
        if (self.camera!=None):
            self.camera.destroy()
        if (self.lidar!=None):
            self.lidar.destroy()
        self.drone.destroy()
        



def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)








class controller:
    def __init__(self):

        self.yaw_rate_pid = PID(1.0, 0, 1, setpoint=0, output_limits=(-1, 1))
        self.pitch_pid = PID(1, 0, 1, setpoint=0, output_limits=(-1, 1))
        self.roll_pid = PID(1, 0, 1, setpoint=0, output_limits=(-1, 1))       
        self.vforward_pid = PID(10, 0.01, 10, setpoint=0, output_limits=(-35, 35))
        self.vleft_pid = PID(10, 0.01, 10, setpoint=0, output_limits=(-35, 35))
        self.vz_pid = PID(1, 0.4, 0, setpoint=0, output_limits=(0, 1))

    def handle_ros(self, vforward_desired=0, vleft_desired=0,vz_desired=0,yaw_rate_desired=0):
        self.vforward_pid.setpoint=vforward_desired
        self.vleft_desired.setpoint=vleft_desired
        self.vz_desired.setpoint=vz_desired
        self.yaw_rate_desired.setpoint=yaw_rate_desired
        
    def pid_calculate(self,vehicle_transform,vehicle_velocity,vehicle_angular_rate):
        vforward_output =self.vforward_pid(self.forward_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        self.pitch_pid.setpoint=-vforward_output
        pitch_output =self.pitch_pid(vehicle_transform.rotation.pitch)#1/(100*math.sin(math.radians(abs(vehicle_transform.rotation.pitch)))+0.1)*
        
        vleft_output =self.vleft_pid(self.right_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        self.roll_pid.setpoint=vleft_output
        roll_output =  self.roll_pid(vehicle_transform.rotation.roll)
       
        yaw_rate_output = self.yaw_rate_pid(vehicle_angular_rate.z)
        throttle_output =self.vz_pid(vehicle_velocity.z)
        
        #print(vehicle_velocity)
        #print(vehicle_transform)
        #print(roll_output)
        #print("forward_speed",forward_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        #print("right_speed",right_speed(vehicle_velocity,math.radians(vehicle_transform.rotation.yaw)))
        #print("yawRate",vehicle_angular_rate.yaw_rate)
        speed_set=1
        front_left =  speed_set*throttle_output  + +roll_output + +pitch_output - yaw_rate_output
        front_right = speed_set*throttle_output  + -roll_output + +pitch_output + yaw_rate_output 
        rear_left = speed_set*throttle_output    + +roll_output + -pitch_output + yaw_rate_output
        rear_right = speed_set*throttle_output   + -roll_output + -pitch_output -yaw_rate_output
        
        return front_left,front_right,rear_left,rear_right

    
    def handle_keys(self,type,key,maxspeed=12,maxyaw=2):
        if key==K_w:
            if type==KEYDOWN:
                self.vforward_pid.setpoint=maxspeed
            elif type==KEYUP:
                self.vforward_pid.setpoint=0
        if key==K_s:
            if type==KEYDOWN:
                self.vforward_pid.setpoint=-maxspeed
            elif type==KEYUP:
                self.vforward_pid.setpoint=0
        #-----------left
        if key==K_a:
            if type==KEYDOWN:
                self.vleft_pid.setpoint=maxspeed
            elif type==KEYUP:
                self.vleft_pid.setpoint=0
        if key==K_d:
            if type==KEYDOWN:
                self.vleft_pid.setpoint=-maxspeed
            elif type==KEYUP:
                self.vleft_pid.setpoint=0
        #---------up
        if key==K_UP:
            if type==KEYDOWN:
                self.vz_pid.setpoint=maxspeed
            elif type==KEYUP:
                self.vz_pid.setpoint=0
        if key==K_DOWN:
            if type==KEYDOWN:
                self.vz_pid.setpoint=-maxspeed
            elif type==KEYUP:
                self.vz_pid.setpoint=0

        #---------yaw
        if key==K_LEFT:
            if type==KEYDOWN:
                self.yaw_rate_pid.setpoint=maxyaw
            elif type==KEYUP:
                self.yaw_rate_pid.setpoint=0
        if key==K_RIGHT:
            if type==KEYDOWN:
                self.yaw_rate_pid.setpoint=-maxyaw
            elif type==KEYUP:
                self.yaw_rate_pid.setpoint=0
    def forward_speed(cls,world_speed,world_yaw):
        return world_speed.x*math.cos(world_yaw)+world_speed.y*math.sin(world_yaw)


    def right_speed(cls,world_speed,world_yaw):
        return -world_speed.x*math.sin(world_yaw)+world_speed.y*math.cos(world_yaw)

def world_tick(drone_s_ctl_1, world_snap_shot):
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == KEYDOWN or event.type == KEYUP:
            drone_s_ctl_1.controller.handle_keys(event.type,event.key)
            
    #time.sleep( settings.fixed_delta_seconds)
    
    vehicle_transform = drone_s_ctl_1.drone.get_transform()
    vehicle_velocity= drone_s_ctl_1.drone.get_velocity()
    vehicle_angular_rate=drone_s_ctl_1.get_angular_velocity()
    #spectator.set_transform(carla.Transform(vehicle_transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
    front_left,front_right,rear_left,rear_right=drone_s_ctl_1.controller.pid_calculate(vehicle_transform,vehicle_velocity,vehicle_angular_rate)

    drone_s_ctl_1.drone.apply_motor_speed(front_left*3000,front_right*3000,rear_left*3000,rear_right*3000)
    # vehicle.apply_control_d()


def main():
    pygame.init()
    screen = pygame.display.set_mode((2000, 1500), pygame.HWSURFACE | pygame.DOUBLEBUF)
    clock = pygame.time.Clock()
    # Connect to the CARLA simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and set to synchronous mode
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.03  # Set the time step for the simulation
    world.apply_settings(settings)
   
    
    drone_blue_print =world.get_blueprint_library().filter('lea')[0]  
    spawn_point =carla.Transform(carla.Location(x=0, y=10, z=1), carla.Rotation(yaw=0))
    spawn_point2 =carla.Transform(carla.Location(x=3, y=10, z=1), carla.Rotation(yaw=0))
    #spawn_point_rand = random.select(world.get_map().get_spawn_points())
    
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  # Adjust these values as needed
    imu_bp = world.get_blueprint_library().find('sensor.other.imu')
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    

    drone_s_ctl_1=Drone_sensors_ctl(world=world,
                                    drone_bp=drone_blue_print,spawn_point=spawn_point,
                                    camera_bp=camera_bp,screen=screen,
                                    imu_bp=imu_bp,
                                    controller=controller())
    
    world.on_tick(lambda world_snap_shot:world_tick(drone_s_ctl_1,world_snap_shot))
    # drone_s_ctl_2=Drone_sensors_ctl(world=world,
    #                                 drone_bp=drone_blue_print,spawn_point=spawn_point2,
    #                                 camera_bp=camera_bp,screen=screen,
    #                                 imu_bp=imu_bp,
    #                                 controller=controller())

    
    

    try:    
        while True:
          time.sleep(1)
    except KeyboardInterrupt:
        if drone_s_ctl_1 is not None: 
            drone_s_ctl_1.destroy_all()
            
            print('Vehicle destroyed.')
        # if drone_s_ctl_2 is not None: 
        #     drone_s_ctl_2.destroy_all()
        # If something goes wrong, try to disable synchronous mode
        settings.synchronous_mode = False
        world.apply_settings(settings)
        print('Synchronous mode disabled.')

if __name__ == '__main__':
    main()
