from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))



# PARAM 1: 1-Arm 0-Disarm
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

print("Switched to GUIDED mode")

the_connection.mav.command_long_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,0,0,0,0,0,0,200
)



# SET YAW ANGLE
# PARAM 4: Relative or Absolute
# PARAM 3: ROtation -1 anti-clock +1 clock

def setYawAngle (deg,angularSpeed,direction,frame):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,45,25,1,1,0,0,0
    )
    print("DONE")

#SET SPEED
# PARAM 2: speed m/s
def changeSpeed (x):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,mavutil.mav.SPEED_TYPE_DESCENT_SPEED,
        1000,255,0,0,0,0
    )

setYawAngle(0,0,0,0)
# changeSpeed(1000)