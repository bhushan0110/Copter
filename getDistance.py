import math
from pymavlink import mavutil
from math import asin, cos, radians, sin


#  Connection 
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

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

d = calculate_distance(-35.3630036,-35.3632622,149.1652375,149.1652375)
print(d)