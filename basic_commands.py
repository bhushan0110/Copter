# mavutil: MAVLink utility functions for setting up communication links, receiving and decoding messages, running periodic tasks, etc

from pymavlink import mavutil



#############################################   SETTING UP CONNECTION  #############################################################


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14540')        # the_connection is connection name
                                            #protocol:address:port

# Wait for the first heartbeat 
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))





############################################    <message_name>_send()  ############################################################

the_connection.mav.system_time_send(time_unix_usec, time_boot_ms)





############################################    RECEIVING MESSAGES   ###############################################################

try: 
    altitude = the_connection.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
    timestamp = the_connection.time_since('GPS_RAW_INT')
except:
    print('No GPS_RAW_INT message received')




#############   Can use the mavutil recv_match() method to wait for and intercept messages as they arrive:  ##########################

def recv_match(self, condition=None, type=None, blocking=False, timeout=None):
    pass

msg = the_connection.recv_match(blocking=True)

# Wait for a 'SYS_STATUS' message with the specified values.
msg = the_connection.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)






################################   SENDING HEARTBEAT  ######################################

# Send heartbeat from a GCS (types are define as enum in the dialect file). 
the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

# Send heartbeat from a MAVLink application. 
the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

