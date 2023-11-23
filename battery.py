import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Request the GLOBAL_POSITION_INT message
the_connection.mav.request_data_stream_send(
    the_connection.target_system,    # System ID
    the_connection.target_component,    # Component ID
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, # Data stream ID
    1,    # Send at 1Hz
    1     # Start sending
)
while True:
    msg = the_connection.recv_msg()
    if msg and msg.get_type()=='SYS_STATUS':
        print(msg)
        print(msg.battery_remaining)
        break

