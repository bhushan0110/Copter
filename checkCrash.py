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
        time.sleep(0.01)

        
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
            alt = ((msg.relative_alt)/1000)
            time_boot_ms = msg.time_boot_ms
            break  # Exit the loop after receiving the message
        time.sleep(0.01)  # You can adjust the sleep duration as needed

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

    target_lat = format(degrees(lat2),'.7f')
    target_lon = format(degrees(lon2),'.7f')

    return (target_lat,target_lon)

def calculate_distance(lat1, lat2, lon1, lon2):
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    dlon = lon2 - lon1 
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * asin(math.sqrt(a)) 
    r = 6371000  
    return(c * r)


getParameters()
t1,t2 = get_point_at_distance(lat/10e6,lon/10e6,150/1000,0)
t1 = float(t1)
t2 = float(t2)

print("Current Lat Lon", lat, lon)

print("\nCalculated lat lon", t1,t2)
t1 = int(float(t1)*10e6)
t2 = int(float(t2)*10e6)
print(t1,t2,type(t1))

i=0
while(1):
    getParameters()
    # d = calculate_distance(t1,lat/10e6,t2,lon/10e6)
    # c1,c2= get_point_at_distance(lat/10e6,lon/10e6,(1.5*d)/1000,0)
    # print(c1,c2)
    
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            0,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b111111000000),
            t1, t2,-3*alt,0,0,0,0,0,0,0,0
        )
    )
    print(alt)
    print(lat,lon)
    i+=1
    if(alt<=1):
        break

# -353604910
# -353619142

# def hitTarget():
#     getParameters()
#     c1,c2 = get_point_at_distance( lat/10e6, lon/10e6, 1500/1000, 0)
    # c1 = int(c1*10e6)
    # c2 = int(c2*10e6)
    # print(c1,c2)
    # the_connection.mav.send(
    #     mavutil.mavlink.MAVLink_set_position_target_global_int_message(
    #         0,
    #         the_connection.target_system,
    #         the_connection.target_component,
    #         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    #         int(0b111111000000),
    #         c1, c2,-1000,0,0,0,0,0,0,0,0
    #     )
    # )

    # msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
    # print(msg)

    # while(1):
    #     getParameters()
    #     print("Lat:"+str(lat)+" Lon:"+str(lon)+" att:"+str(alt))
    #     if(alt<=0):
    #         break
    #     time.sleep(0.1)


# hitTarget()