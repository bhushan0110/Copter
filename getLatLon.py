from math import asin, atan2, cos, degrees, radians, sin
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

# Wait for a message response (GLOBAL_POSITION_INT)
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


print("Heading angle from North:  " +str(angle)+"\n")

print("Drone Lat:  "+str(lat))
print("      Lon:  "+str(lon)+"\n")




def get_point_at_distance(lat1, lon1, d, bearing, R=6371):
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    a = radians(bearing)
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1) * sin(d/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(d/R) * cos(lat1),
        cos(d/R) - sin(lat1) * sin(lat2)
    )
    return (degrees(lat2), degrees(lon2),)



c1,c2 = get_point_at_distance( -35.3632619,149.1652375,150/1000, 0)
print(c1,c2)


