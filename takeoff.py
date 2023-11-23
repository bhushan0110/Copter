from pymavlink import mavutil
import time

def disArm(the_connection):
    print("IN")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,0,0,0,0,0
    )

def arm(the_connection):
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

def changeMode(the_connection):
    mode_id = the_connection.mode_mapping()["GUIDED"]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    print("Switched to GUIDED mode")

    msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
    print(msg)

def takeOff(the_connection,altitude):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,50
    )

    msg = the_connection.recv_match(type='COMMAND_ACK',blocking= True)
    print(msg)


def go_to_target(the_connection,type_mask,x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate,boot_Time=0):
    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            boot_Time,
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            int(type_mask),
            x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate
            # 0,0,0,0,0,0,0,0,0,0,0
        )
    )

# SET YAW ANGLE
# frame: Relative or Absolute
# direction: ROtation -1 anti-clock +1 clock
def setYawAngle (the_connection,deg,angularSpeed,direction,frame):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,deg,angularSpeed,direction,frame,0,0,0
    )


#SET SPEED
# SPEED_TYPE: 
    # 0 : Air_Speed
    # 1 : Ground_Speed
    # 2 : Climb_Speed
    # 3 : Descent_Speed
# speed: speed m/s
#Throttle: 
def changeSpeed (connection,speed,Throttle=0,speedType=0):
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,speedType,speed,Throttle,0,0,0,0
    )


def getParameters (the_connection):
    the_connection.mav.request_data_stream_send(
        0,    # System ID
        0,    # Component ID
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, # Data stream ID
        1,    # Send at 1Hz
        1     # Start sending
    )

    # Wait for a message response (GLOBAL_POSITION_INT)
    while True:
        msg = the_connection.recv_msg()
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            print(msg)
        time.sleep(0.1)  # You can adjust the sleep duration as needed





# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


changeMode(the_connection)  
disArm(the_connection)
arm(the_connection)
  
takeOff(the_connection,10)
# go_to_target()
# getParameters(the_connection)