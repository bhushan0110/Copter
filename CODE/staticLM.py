###############################################################
###############################################################

# GET PARAMETERS AND CALCULATE TARGET LATTITUDE AND LONGITUDE #

###############################################################
###############################################################

import time
import math
from pymavlink import mavutil
from math import asin, atan2, cos, degrees, radians, sin


#  Connection 
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


def getParameters():
    the_connection.mav.request_data_stream_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, # Data stream ID
        1,    # Send at 1Hz
        1     # Start sending
    )

    # Angle from NORTH
    while True:
        msg = the_connection.recv_msg()
        if msg and msg.get_type() == 'ATTITUDE':
            global angle
            angle = math.degrees(msg.yaw)
            break
        time.sleep(0.1)

        
    # Current GPS coordinates
    while True:
        msg = the_connection.recv_msg()
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            global lat
            global lon
            global alt
            global time_boot_ms
            lat = msg.lat
            lon = msg.lon
            alt = msg.alt
            time_boot_ms = msg.time_boot_ms
            break  # Exit the loop after receiving the message
        time.sleep(0.1)  # You can adjust the sleep duration as needed

def get_point_at_distance(lat1, lon1, d, bearing, R=6371):
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    a = radians(bearing)
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1) * sin(d/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(d/R) * cos(lat1),
        cos(d/R) - sin(lat1) * sin(lat2)
    )
    global target_lat 
    global target_lon
    target_lat = format(degrees(lat2),'.9f')
    target_lon = format(degrees(lon2),'.9f')

    return (target_lat,target_lon)





getParameters()
c1,c2 = get_point_at_distance( lat/10e6, lon/10e6, 150/1000, 0)
print(c1,c2)