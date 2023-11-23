from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# the_connection.mav.command_long_send(
#     the_connection.target_system,
#     the_connection.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     0,
#     0,0,0,0,0,0
# )

# print("Waiting for the vehicle to arm")
# the_connection.motors_armed_wait()
# print('Armed!')


msg = the_connection.recv_match(type='HEARTBEAT',blocking= True)
print(msg)