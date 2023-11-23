import time
import math
from pymavlink import mavutil
from math import asin, atan2, cos, degrees, radians, sin


#  Connection 
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.request_data_stream_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, # Data stream ID
    1,    # Send at 1Hz
    1     # Start sending
)

while True:
        msg = the_connection.recv_msg()
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat
            lon = msg.lon

            alt = ((msg.relative_alt)/1000)
            print(alt)
            if alt<=1:
                break