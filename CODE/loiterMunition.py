import time
from pymavlink import mavutil

class Flight:
    def __init__(self):
        self.connection         = None            #   Connection variable
        self.time_boot_ms       = None            #   time since system boot
        self.latitude           = 0               #   Drones lat lon alt
        self.longitude          = 0     
        self.altitude_sea       = 0  
        self.altitude_ground    = 0  
        self.velocity_x         = 0               #   Drones velocity in x y z direction
        self.velocity_y         = 0  
        self.velocity_z         = 0  
        self.battery_remaining  = 100             #   Remaining battery percentage
        self.target_lat         = -1              #   Target lat lon alt 
        self.target_lon         = -1
        self.target_alt         = -1
        self.target_angle_x     = 0
        self.target_angle_y     = 0
        self.target_angle_z     = 0

    def __del__(self):
        print("Flight Object deleted")
    
    def connect(self):
        if self.connection != None :
            self.connection = mavutil.mavlink_connection('tcp:localhost:5763')

    def battery_data (self):
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1,
            1
        )
        while True:
            msg = self.connection.recv_msg()
            if msg and msg.get_type()=='SYS_STATUS':
                print(msg)
                self.battery_remaining = msg.battery_remaining
                break 

    def disArm(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,0,0,0,0,0
        )

    def arm(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,0,0,0,0,0
        )

        print("Waiting for the vehicle to arm")
        self.connection.motors_armed_wait()
        print("Armed")

    def setGuidedMode(self):
        mode_id = self.connection.mode_mapping()["GUIDED"]
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        print("Switched to GUIDED mode")

        msg = self.connection.recv_match(type='COMMAND_ACK',blocking= True)
        print(msg)

    def takeOff(self,altitude):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,0,0,0,altitude
        )

        msg = self.connection.recv_match(type='COMMAND_ACK',blocking= True)
        print(msg)


    def go_to_target(self,type_mask,x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate,boot_Time=0):
        self.connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                boot_Time,
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                int(type_mask),
                x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate
            )
        )


    def setYawAngle (self,deg,angularSpeed,direction,frame):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,deg,angularSpeed,direction,frame,0,0,0
        )

    def changeSpeed (self,speed,Throttle=0,speedType=0):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,speedType,speed,Throttle,0,0,0,0
        )
        getParameters()


    def getParameters (self):
        self.connection.mav.request_data_stream_send(
            0, 0,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1, 1
        )
        while True:
            msg = self.connection.recv_msg()
            if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
                self.time_boot_ms   = msg.time_boot_ms
                self.latitude       = msg.lat
                self.longitude      = msg.lon
                self.altitude_sea   = msg.alt
                self.altitude_ground= msg.relative_alt
                self.velocity_x     = msg.vx
                self.velocity_y     = msg.vy
                self.velocity_z     = msg.vz
                data = {
                    'Boot_time'     :   self.time_boot_ms,
                    'Latitude'      :   self.latitude,
                    'Longitude'     :   self.longitude,
                    'Altitude_SEA'  :   self.altitude_sea,
                    'Altitude_GROUND':  self.altitude_ground,
                    'Velocity_X'    :   self.velocity_x,
                    'Velocity_Y'    :   self.velocity_y,
                    'Velocity_Z'    :   self.velocity_z, 
                }
                print(data)
                break
            time.sleep(0.1)

    def return_to_launch (self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,0,0,0,0,0,0,0
        )

    def calculateAtitude (self):
        pass

    def getTarget (self):
        pass
        #Function to get target coordinates