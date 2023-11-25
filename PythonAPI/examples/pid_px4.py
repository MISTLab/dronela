#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""


class CarlaIMUData:
    def __init__(self, timestamp=0.0, xmag=0,ymag=0,zmag=0):
        self.timestamp = timestamp
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag

class CarlaGPSData:
    def __init__(self, timestamp=0.0, latitude=0.0, longitude=0.0, altitude=0.0):
        self.timestamp = timestamp
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude


import glob
import os
import sys
import time
import math
from pymavlink import mavutil
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
gps_data_px4 = None
imu_data_px4 = None



from simple_pid import PID

def pid_controller(desired, current, kp, ki, kd, output_limits):
    pid = PID(kp, ki, kd, setpoint=desired, output_limits=output_limits)
    return pid(current)


roll_desired = 0.0
roll_current = 0.0
roll_pid = PID(1.0, 0.01, 0.4, setpoint=roll_desired, output_limits=(-1, 1))


pitch_desired = 0.0
pitch_current = 0.0
pitch_pid = PID(1.0 ,0.01, 0.4, setpoint=pitch_desired, output_limits=(-1, 1))


z_desired = 5.0  # Desired altitude (adjust as needed)
z_current = 0.0   # Current altitude (initially at ground level)
z_pid = PID(1, 0.01, 0.4, setpoint=z_desired, output_limits=(0, 1))

# Example PID Controller for Yaw
yaw_desired = 1
yaw_current = 0
yaw_pid = PID(1.0, 0.01, 0.4, setpoint=yaw_desired, output_limits=(-1, 1))
 

x_desired = 50.0  # Desired altitude (adjust as needed)
x_current = 0.0   # Current altitude (initially at ground level)
x_pid = PID(.3, 0, 1, setpoint=x_desired, output_limits=(-1, 1))

y_desired = 12.0  # Desired altitude (adjust as needed)
y_current = 0.0   # Current altitude (initially at ground level)
y_pid = PID(.3, 0, 1, setpoint=y_desired, output_limits=(-1, 1))



def connect_px4():

    global px4_connection

    print("Waiting to connect...")
    px4_connection = mavutil.mavlink_connection('tcpin:localhost:4560')

    msg = px4_connection.recv_match(blocking = True)
    if msg.get_type() != "COMMAND_LONG":
        raise Exception("error")
    print( "<==", msg)

    msg = px4_connection.recv_match(blocking = True)
    if msg.get_type() != "HEARTBEAT":
        raise Exception("error")
    print( "<==", msg)


# =========================================================
# =========================================================
# =========================================================

def pressure_at_altitude(h):
    P = ((1.0 - (h / 44330.0)) ** 5.254999) * 101325.0
    # print(" height ", h, " pressure ", P)
    return P +random.uniform(-1,1)


def process_imu_data(data, imu_data_px4):
   
    if data is not None:
        imu_data_px4.timestamp = int(data.timestamp*1e6)
        imu_data_px4.accelerometer = data.accelerometer
        imu_data_px4.gyroscope = data.gyroscope
        imu_data_px4.xmag = 0.3*math.cos(data.compass+random.uniform(-0.0001,0.0001))
        imu_data_px4.ymag = 0.3*-math.sin(data.compass+random.uniform(-0.0001,0.0001))
        imu_data_px4.zmag = 0.4+random.uniform(-0.0001,0.0001)
        # print("IMU data received:", data)
   

def process_gps_data(data, gps_data_px4):
    if data is not None:
        gps_data_px4.timestamp = int(data.timestamp*1e6)
        gps_data_px4.latitude = int(data.latitude * 1e7)
        gps_data_px4.longitude = int(data.longitude * 1e7)
        gps_data_px4.altitude = int(data.altitude*1000)
        # print("GPS data received:", data)



def main():
    

    # Connect to the CARLA simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and set to synchronous mode
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0.01  # Set the time step for the simulation
    #world.apply_settings(settings)

    # Spawn a vehicle named 'lea'
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('lea')[0]  
    spawn_point = carla.Transform(carla.Location(x=0, y=10, z=1), carla.Rotation(yaw=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    imu_blueprint = world.get_blueprint_library().find('sensor.other.imu')
    imu_blueprint.set_attribute('noise_accel_stddev_x', '0.001')
    imu_blueprint.set_attribute('noise_accel_stddev_y', '0.001')
    imu_blueprint.set_attribute('noise_accel_stddev_z', '0.001')
    imu_blueprint.set_attribute('noise_gyro_bias_x', '0')
    imu_blueprint.set_attribute('noise_gyro_bias_y', '0')
    imu_blueprint.set_attribute('noise_gyro_bias_z', '0')
    imu_blueprint.set_attribute('noise_gyro_stddev_x', '0.0001')
    imu_blueprint.set_attribute('noise_gyro_stddev_y', '0.0001')
    imu_blueprint.set_attribute('noise_gyro_stddev_z', '0.0001')

    imu_location = carla.Location(x=0, y=0, z=0)
    imu_sensor = world.spawn_actor(imu_blueprint, carla.Transform(imu_location,carla.Rotation(yaw=0)), attach_to=vehicle)
    gps_blueprint = world.get_blueprint_library().find('sensor.other.gnss')
    gps_blueprint.set_attribute('noise_alt_bias', '0.01')
    gps_blueprint.set_attribute('noise_alt_stddev', '0.01')
    gps_blueprint.set_attribute('noise_lat_bias', '0.0000001')
    gps_blueprint.set_attribute('noise_lat_stddev', '0.0000001')
    gps_blueprint.set_attribute('noise_lon_bias', '0.0000001')
    gps_blueprint.set_attribute('noise_lon_stddev', '0.0000001')

    gps_location = carla.Location(x=0, y=0, z=0)
    gps_sensor = world.spawn_actor(gps_blueprint, carla.Transform(gps_location), attach_to=vehicle)


    imu_data_px4=CarlaIMUData
    gps_data_px4=CarlaGPSData
    


    gps_sensor.listen(lambda data: process_gps_data(data, gps_data_px4))
    imu_sensor.listen(lambda data: process_imu_data(data, imu_data_px4))
   


    try:
        
        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=2), transform.rotation))
        connect_px4()
        # Main loop
        while True:
            world.tick()  # Tick the simulation
            # vehicle.apply_control_d()
            #vehicle.apply_motor_speed(1500,1500,1400,1400)
            # Adjust the spectator's transform if needed
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location+carla.Location(x=0) +carla.Location(y=-0) + carla.Location(z=4), carla.Rotation(pitch=-90)))
            # print(transform)
        
            #vehicle.apply_motor_speed(0,0,0,5000)
                
            if gps_data_px4 is not None and imu_data_px4 is not None:
                # time.sleep(settings.fixed_delta_seconds)  # Wait for the next tick

                time_usec           =  int(time.time() * 1e6)               # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
                fix_type            = 3                         # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
                lat                 = gps_data_px4.latitude   # Latitude (WGS84) [degE7] (type:int32_t)
                lon                 = gps_data_px4.longitude        # Longitude (WGS84) [degE7] (type:int32_t)
                alt                 = gps_data_px4. altitude        # Altitude (MSL). Positive for up. [mm] (type:int32_t)
                eph                 = 30       # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                epv                 = 30      # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                vel                 = 0     # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
                vn                  = 0      # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
                ve                  = 0       # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
                vd                  = 0     # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
                cog                 = 0      # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
                satellites_visible  = 10                        # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
                the_id              = 0                         # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
                yaw                 = 0                         # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)

                if px4_connection != None:
                    px4_connection.mav.hil_gps_send(
                        time_usec           = time_usec             ,
                        fix_type            = fix_type              ,
                        lat                 = lat                   ,
                        lon                 = lon                   ,
                        alt                 = alt                   ,
                        eph                 = eph                   ,
                        epv                 = epv                   ,
                        vel                 = vel                   ,
                        vn                  = vn                    ,
                        ve                  = ve                    ,
                        vd                  = vd                    ,
                        cog                 = cog                   ,
                        satellites_visible  = satellites_visible    ,
                        id                  = the_id                ,
                        yaw                 = yaw                   ,
                    )

            #     print ( "--> HIL_GPS {",
            #     "time_usec :"           , time_usec,
            #     ",",
            #     "fix_type :"            , fix_type,
            #     ",",
            #     "lat :"                 , lat,
            #     ",",
            #     "lon :"                 , lon,
            #     ",",
            #     "alt :"                 , alt,
            #     ",",
            #     "eph :"                 , eph,
            #     ",",
            #     "epv :"                 , epv,
            #     ",",
            #     "vel :"                 , vel,
            #     ",",
            #     "vn :"                  , vn,
            #     ",",
            #     "ve :"                  , ve,
            #     ",",
            #     "vd :"                  , vd,
            #     ",",
            #     "cog :"                 , cog,
            #     ",",
            #     "satellites_visible :"  , satellites_visible,
            #     ",",
            #     "id :"                  , the_id,
            #     ",",
            #     "yaw :"                 , yaw,
            # "}")




                time_usec           = int(time.time() * 1e6)                    # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
                xacc                = imu_data_px4.accelerometer.x        # X acceleration [m/s/s] (type:float)
                yacc                = imu_data_px4.accelerometer.y         # Y acceleration [m/s/s] (type:float)
                zacc                = -imu_data_px4.accelerometer.z        # Z acceleration [m/s/s] (type:float)
                xgyro               = imu_data_px4.gyroscope.x       # Angular speed around X axis in body frame [rad/s] (type:float)
                ygyro               = imu_data_px4.gyroscope.y      # Angular speed around Y axis in body frame [rad/s] (type:float)
                zgyro               = -imu_data_px4.gyroscope.z       # Angular speed around Z axis in body frame [rad/s] (type:float)
                xmag                = imu_data_px4.xmag       # X Magnetic field [gauss] (type:float)
                ymag                = imu_data_px4.ymag        # Y Magnetic field [gauss] (type:float)
                zmag                = imu_data_px4.zmag        # Z Magnetic field [gauss] (type:float)
                abs_pressure        =  pressure_at_altitude(gps_data_px4.altitude/1000) # Absolute pressure [hPa] (type:float)
                diff_pressure       = 0 # Differential pressure (airspeed) [hPa] (type:float)
                pressure_alt        = gps_data_px4.altitude/1000    # Altitude calculated from pressure (type:float)
                temperature         = 20  # Temperature [degC] (type:float)
                fields_updated      = 7167                          # Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
                the_id              = 0                             # Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)

                if px4_connection != None:
                    px4_connection.mav.hil_sensor_send(
                        time_usec           = time_usec         ,
                        xacc                = xacc              ,
                        yacc                = yacc              ,
                        zacc                = zacc              ,
                        xgyro               = xgyro             ,
                        ygyro               = ygyro             ,
                        zgyro               = zgyro             ,
                        xmag                = xmag              ,
                        ymag                = ymag              ,
                        zmag                = zmag              ,
                        abs_pressure        = abs_pressure      ,
                        diff_pressure       = diff_pressure     ,
                        pressure_alt        = pressure_alt      ,
                        temperature         = temperature       ,
                        fields_updated      = fields_updated    ,
                        id                  = the_id            ,
                    )

                print ( "--> HIL_SENSOR {",
                "time_usec :"       , time_usec,
                ",",
                "xacc :"            , xacc,
                ",",
                "yacc :"            , yacc,
                ",",
                "zacc :"            , zacc,
                ",",
                "xgyro :"           , xgyro,
                ",",
                "ygyro :"           , ygyro,
                ",",
                "zgyro :"           , zgyro,
                ",",
                "xmag :"            , xmag,
                ",",
                "ymag :"            , ymag,
                ",",
                "zmag :"            , zmag,
                ",",
                "abs_pressure :"    , abs_pressure,
                ",",
                "diff_pressure :"   , diff_pressure,
                ",",
                "pressure_alt :"    , pressure_alt,
                ",",
                "temperature :"     , temperature,
                ",",
                "fields_updated :"  , fields_updated,
                ",",
                "id :"              , the_id,
            "}")


                if px4_connection != None:
                    msg = px4_connection.recv_match(blocking = False)
                    if msg != None: 
                        print(msg.get_type())       
                        if msg.get_type() == 'HIL_ACTUATOR_CONTROLS':
                            print( "<==", msg.controls[0:4])
                            #fl3-2 fr1-0 rl2-1 rr4-3
                            # vehicle.apply_motor_speed(msg.controls[2]*2000,msg.controls[0]*2000,msg.controls[1]*2000,msg.controls[3]*2000)
                            x_output = x_pid(transform.location.x+random.uniform(-.02,0.02))
                            #pitch_pid.setpoint=-x_output*10
                            y_output = y_pid(transform.location.y+random.uniform(-.02,0.02))
                            #roll_pid.setpoint=y_output*10
                            if (transform.location.z>40):
                                print("arrived")
                                roll_pid.setpoint=10

                            roll_output = roll_pid(transform.rotation.roll+random.uniform(-.2,0.2))
                            pitch_output = pitch_pid(transform.rotation.pitch+random.uniform(-.2,0.2))
                            yaw_output = yaw_pid(transform.rotation.yaw)
                        
                            throttle_output = z_pid(transform.location.z+random.uniform(-.02,0.02))
                        
                            
                            

                            speed_set=10
                            front_left = 3+ speed_set*throttle_output + (roll_output + pitch_output -yaw_output)
                            front_right =3+  speed_set*throttle_output + (-roll_output + pitch_output + yaw_output )
                            rear_left =3+ speed_set*throttle_output + (+roll_output - pitch_output+yaw_output)
                            rear_right =3+ speed_set*throttle_output + (-roll_output - pitch_output -yaw_output)
                            
                            # front_left=1
                            # rear_right=1
                            # front_right=1
                            # rear_left=1


                            print(">>pidCtl",front_left,front_right,rear_left,rear_right)
                            vehicle.apply_motor_speed(front_left*100,front_right*100,rear_left*100,rear_right*100)
                                
                            
    finally:
        if vehicle is not None:
            imu_sensor.stop()
            imu_sensor.destroy()

            gps_sensor.stop()
            gps_sensor.destroy()
            vehicle.destroy()
            print('Vehicle destroyed.')
        # If something goes wrong, try to disable synchronous mode
        #settings.synchronous_mode = False
        #world.apply_settings(settings)
        print('Synchronous mode disabled.')

if __name__ == '__main__':
    main()
