import time
from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

mode_id = the_connection.mode_mapping()["GUIDED"]
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

print("Switched to GUIDED mode")

the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)


print("Waiting for the vehicle to arm")
the_connection.motors_armed_wait()
print('Armed!')

msg = the_connection.recv_match(type='GLOBAL_POSITION_INT',blocking= True)
print(msg)

the_connection.mav.request_data_stream_send(
    the_connection.target_system,  # Target system ID
    the_connection.target_component,  # Target component ID
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Data stream ID for position information
    1,  # Rate (1 Hz in this example, adjust as needed)
    1  # Start or stop (1 to start, 0 to stop)
)
global_position = the_connection.messages['GLOBAL_POSITION_INT']
print(global_position)




# Close the MAVLink connection
the_connection.close()