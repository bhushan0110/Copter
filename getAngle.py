from pymavlink import mavutil
import time
import math


the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

global lat
global lon
global alt

global target_lat
global target_lon
global target_alt

global angle
global time_boot_ms

the_connection.mav.request_data_stream_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, # Data stream ID
    1,    # Send at 1Hz
    1     # Start sending
)

while True:
    msg = the_connection.recv_msg()
    if msg and msg.get_type() == 'ATTITUDE':
        angle = math.degrees(msg.yaw)
        break  # Exit the loop after receiving the message
    time.sleep(0.1)  # You can adjust the sleep duration as needed

while True:
    msg = the_connection.recv_msg()
    if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
        print(msg)
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        time_boot_ms = msg.time_boot_ms
        break  # Exit the loop after receiving the message
    time.sleep(0.1)  # You can adjust the sleep duration as needed

print(lat,lon,alt)

X = 0
Y = 0
x=100000
angle =0
if angle == 0:
    X = x
elif angle == 180:
    X = -x
elif angle == -90:
    Y = -x
elif angle == 90:
    Y = x

elif angle < 0 and angle > -90:
    deg = -angle
    X = x*math.cos(deg)
    Y = x*math.sin(deg)
    print("\n ELIF\n  X: "+str(X) + " Y: "+str(Y)+"\n")

elif angle < -90:
    deg = -(angle + 90)
    X = x*math.sin(deg)
    Y = x*math.cos(deg)

elif angle >0 and angle <90:
    deg = angle
    Y = x*math.sin(deg)
    X = x*math.cos(deg)

else:
    deg = (angle -90)
    Y = x*math.cos(deg)
    X = x*math.sin(deg)

target_lon = lon + X
target_lat = lat + Y  

print("Heading angle from North:  " +str(angle)+"\n")

print("Drone Lat:  "+str(lat))
print("      Lon:  "+str(lon)+"\n")

print("Target_Lat: "+str(target_lat))
print("Target_Lon: "+str(target_lon))


# calculate coordinate to go with high speed
# slope = (target_lon-lon)/(target_lat-lat)
# intercept = (lon - slope*lat)


vt_lon = int(lon + 5*X)
vt_lat = int(lat + 5*Y)
vt_alt = -int(5*alt)


print("\n\nVirtual Target")
print(vt_lat, vt_lon, vt_alt)


# the_connection.mav.send(
#     mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#         time_boot_ms,
#         0,
#         the_connection.target_system,
#         the_connection.target_component,
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#         int(0b111111000000),
#         0,0,0,255,255,255,0,0,0,0,0
#     )
# )


# msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
# print(msg)
from math import asin, atan2, cos, degrees, radians, sin

print(lat)
lat = radians(lat)
a = radians(angle)
R=6371
print(a)
lat2 = asin(sin(lat) * cos(0.2/R) + cos(lat) * sin(0.2/R) * cos(a))

print(degrees(lat2))

# Origin Lat        Lon        alt
#        -353632622 1491652375 583950

#200m   Lat lon alt
#       -353614657 1491652377 633960