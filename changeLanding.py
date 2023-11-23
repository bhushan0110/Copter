from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,0,0,0,0,0
)

print("Waiting for the vehicle to arm")
the_connection.motors_armed_wait()

mode_id = the_connection.mode_mapping()["GUIDED"]
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

print("Switched to GUIDED mode BVB")

the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,100
    )

msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
print(msg)

param_id = b'MPC_LAND_SPEED'
param_value = 3.0  # Update with your desired descent rate

# Send the parameter value
the_connection.mav.param_set_send(
    the_connection.target_system,
    the_connection.target_component,
    param_id,
    param_value,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

the_connection.mav.send(
    mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b111111000000),
        -353623636,1491652372,0,0,0,0,0,0,0,0,0
    )
)


# the_connection.mav.send(
#     mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#         0,
#         the_connection.target_system,
#         the_connection.target_component,
#         mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
#         int(0b111111000000),
#         0,0,-40,0,0,0,0,0,0,0,0
#     )
# )
