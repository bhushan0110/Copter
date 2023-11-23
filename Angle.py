from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

print(1)
# Create a SetAttitudeTarget message
msg = the_connection.set_attitude_target(
    0,
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET
    [0, 0, 0, 0], # q
    0, # body_roll_rate
    0, # body_pitch_rate
    40, # body_yaw_rate
    200 # thrust 
)
print(2)
# Set the yaw, pitch, and roll angles
msg.body_yaw_rate = 45 # degrees
msg.body_pitch_rate = 30 # degrees
msg.body_roll_rate = 15 # degrees

# Send the message
mav.send(msg)
print(3)
# Wait for the message to be acknowledged
mav.recv_match(type='SET_ATTITUDE_TARGET')
print(4)