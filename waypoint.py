import time
from pymavlink import mavutil
from math import asin, cos, radians, sin, degrees, atan2


#  Connection 
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

def setAutoMode():
    mode_id = the_connection.mode_mapping()["AUTO"]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    print("Switched to AUTO mode")

    msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
    print(msg)

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

Altitude = 50
Distance = 150
h = 5
d = (h*Distance)/Altitude

print(d)

i = 2
x = d
y = Altitude-5
waypoints = []

while 1:
    if y<-5:
        break
    lat0,lon0 = get_point_at_distance(-35.3632623, 149.1652376,x/1000,0)
    lat0 = int(float(lat0)*10e6)
    lon0 = int(float(lon0)*10e6)

    waypoints.append((lat0,lon0,y))

    x = d*i
    y -= 5
    i += 1


# setAutoMode()
 
for lat, lon, alt in waypoints:
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            0,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b111111000000),
            lat,lon,alt,0,0,0,0,0,0,0,0
        )
    )



