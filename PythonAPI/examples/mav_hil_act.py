import subprocess
import time
from pymavlink import mavutil

def start_mavproxy():
    mavproxy_cmd = "mavproxy.py --master=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14552"
    subprocess.Popen(mavproxy_cmd, shell=True)
    time.sleep(2)  # Wait for MAVProxy to start

def monitor_mavlink_messages():
    master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Connect to the forwarded MAVLink messages

    while True:
        msg = master.recv_msg()
        if msg:
            print(msg)

if __name__ == "__main__":
    # Start MAVProxy in a separate process
    start_mavproxy()

    # Monitor MAVLink messages
    monitor_mavlink_messages()
