import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Request the GLOBAL_POSITION_INT message
the_connection.mav.request_data_stream_send(
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, # Data stream ID
    1,    # Send at 1Hz
    1     # Start sending
)

# Wait for a message response (GLOBAL_POSITION_INT)
while True:
    msg = the_connection.recv_msg()
    print(msg)
    if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
        print(msg)
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        break  # Exit the loop after receiving the message
    time.sleep(0.1)  # You can adjust the sleep duration as needed
