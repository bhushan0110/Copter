import time
from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5763')
print(the_connection)
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


# Request the SYSTEM_TIME parameter
param_name = "SYSTEM_TIME"
the_connection.param_fetch_one(param_name)

timeout = 30  # Adjust the timeout as needed
start_time = time.time()

while time.time() - start_time < timeout:
    msg = the_connection.recv_msg()
    if msg is not None:
        if msg.get_type() == 'PARAM_VALUE' and msg.param_id == param_name:
            print(f"Parameter Name: {msg.param_id}, Value: {msg.param_value}")
            break