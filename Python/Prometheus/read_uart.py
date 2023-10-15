import serial
from enum import IntEnum
import yaml

from prometheus_client import start_http_server, Gauge, Enum

STATE_ASMS = ["OFF", "ON"]
STATE_ERROR = ["WORKING", "ERROR"]
STATE_DIRECTION = ["LEFT", "RIGHT"]
STATE_MISSION = ["AS_MANUAL", "AS_OFF", "AS_READY", "AS_DRIVING", "AS_EMERGENCY", "AS_FINISH"]


class Frame_INFO(IntEnum):
    STATUS_INFO = 0
    ANGLE_TARGET = 1
    ANGLE_ACTUALL = 2
    VELOCITY_TARGET = 3
    VELOCITY_ACTUALL = 4
    AS_STATE = 5
    ELECTRICAL_INFO = 6

class Direction(IntEnum):
    LEFT = 0
    RIGHT = 1

class State(IntEnum):
    AS_MANUAL    = 0
    AS_OFF       = 1
    AS_READY     = 2
    AS_DRIVING   = 3
    AS_EMERGENCY = 4
    AS_FINISH    = 5

class AS_INFO_STATE(IntEnum):
    SHD_ERROR = 1
    EBS_ERROR = 2
    ASB_ERROR = 4
    RES_ERROR = 8
    ASMS_OFF = 16
    ROS_ERROR = 32
    DIRECTION_TARGET = 64
    DIRECTION_ACTUALL = 128

class Prometheus_exporter():
    def __init__(self) -> None:
        self.velocity_target_gauge = Gauge("velocity_target", "Target velocity")
        self.velocity_actuall_gauge = Gauge("velocity_actuall","Actual velocity")
        self.angle_target_gauge = Gauge("angle_target","Target steering angle")
        self.angle_actuall_gauge = Gauge("angle_actuall","Actuall steering angle")
        self.CPU_temperature = Gauge("CPU_temperature", "Actuall CPU temperature")

        self.mision = Enum("mission", "Actuall mission", states=STATE_MISSION)
        
        self.ASMS_ON = Enum("ASMS", "Is ASMS on", states=STATE_ASMS)

        self.ASB_ERROR = Enum("ASB_ERROR", "Is ASB ERROR", states=STATE_ERROR)
        self.EBS_ERROR = Enum("EBS_ERROR", "Is EBS ERROR", states=STATE_ERROR)
        self.RES_ERROR = Enum("RES_ERROR", "Is RES ERROR", states=STATE_ERROR)
        self.ROS_ERROR = Enum("ROS_ERROR", "Is ROS ERROR", states=STATE_ERROR)
        self.SHD_ERROR = Enum("SHD_ERROR", "Is SHD ERROR", states=STATE_ERROR)

        self.direction_target = Enum("direction_target", "Target direction", states=STATE_DIRECTION)
        self.direction_actuall = Enum("direction_actuall", "Actuall direction", states=STATE_DIRECTION)

        start_http_server(9000)

    def read_data_and_send(self,dataLogger):
        #Reading data from dataLogger
        self.velocity_target_gauge.set(dataLogger.velocity_target)
        self.velocity_actuall_gauge.set(dataLogger.velocity_actuall)
        self.angle_target_gauge.set(dataLogger.angle_target)
        self.angle_actuall_gauge.set(dataLogger.angle_actuall)
        self.CPU_temperature.set(dataLogger.CPU_temp)

        self.mision.state(STATE_MISSION[dataLogger.as_state])
        self.ASMS_ON.state(STATE_ASMS[dataLogger.is_ASMS_OFF])

        self.ASB_ERROR.state(STATE_ERROR[dataLogger.is_ASB_ERROR])
        self.EBS_ERROR.state(STATE_ERROR[dataLogger.is_EBS_ERROR])
        self.ROS_ERROR.state(STATE_ERROR[dataLogger.is_ROS_ERROR])
        self.RES_ERROR.state(STATE_ERROR[dataLogger.is_RES_ERROR])
        self.SHD_ERROR.state(STATE_ERROR[dataLogger.is_SHD_ERROR])
        
        self.direction_actuall.state(STATE_DIRECTION[dataLogger.angle_actuall_DIR.value])
        self.direction_target.state(STATE_DIRECTION[dataLogger.angle_target_DIR.value])



class Data_Logger():
    def __init__(self) -> None:
        self.velocity_actuall = 0
        self.velocity_target = 0
        self.angle_target = 0
        self.angle_actuall = 0
        self.angle_actuall_DIR = Direction.LEFT
        self.angle_target_DIR = Direction.LEFT
        self.as_state = State.AS_OFF
        self.is_SHD_ERROR = False
        self.is_EBS_ERROR = False 
        self.is_ASB_ERROR = False 
        self.is_RES_ERROR = False 
        self.is_ASMS_OFF = False 
        self.is_ROS_ERROR = False 
        self.CPU_temp = 0

    def check_ERROR_values(self, val, compare_error_state):
        if val & compare_error_state:
            return True
        else:
            return False

    def get_Direction(self, val, compare_direction_state):
        if val & compare_direction_state:
            return Direction.LEFT
        else:
            return Direction.RIGHT

    def print(self):
        print(f"Angle actuall: {self.angle_actuall}")
        print(f"Angle target: {self.angle_target}")
        print(f"Angle actuall direction: {self.angle_actuall_DIR}")
        print(f"Angle target direction: {self.angle_target_DIR}")
        print(f"Actuall velocity: {self.velocity_actuall}")
        print(f"Target velocity: {self.velocity_target}")
        print(f"AS_STATE: {self.as_state}")

        print("\nERRORS:")
        print(f"SHD ERROR: {self.is_SHD_ERROR}")
        print(f"EBS ERROR: {self.is_EBS_ERROR}")
        print(f"ASB ERROR: {self.is_ASB_ERROR}")
        print(f"RES ERROR: {self.is_RES_ERROR}")
        print(f"ROS ERROR: {self.is_ROS_ERROR}")
        print(f"ASMS OFF: {self.is_ASMS_OFF}")

        print("\nELECTRICAL INFO:")
        print(f"CPU TEMP: {self.CPU_temp}")

        print()
    


def from_byte_to_int(byte):
    return int.from_bytes(byte, "big", signed=False)

def main():
    try:
        # Reading yaml file
        with open('prometheus_read_uart.yaml', 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        
        device = data['port']
        baud_rate = data['baud_rate']
        open_prometheus = data['use_prometheus']

        # Constructors of Data_Logger and Prometheus_exporter
        data_logger = Data_Logger()
        if open_prometheus:
            prometheus = Prometheus_exporter()

        # Opening Serial with device
        with serial.Serial(device, baud_rate) as ser:
            
            while True:
                # Reading 
                if from_byte_to_int(ser.read()) == 255:
                    msg_size = from_byte_to_int(ser.read())
                    
                    for i in range(msg_size):
                        r = ser.read()
                        val = from_byte_to_int(r)

                        if i == Frame_INFO.STATUS_INFO.value:
                            # print(val)
                            data_logger.is_SHD_ERROR = data_logger.check_ERROR_values(val, AS_INFO_STATE.SHD_ERROR.value)
                            data_logger.is_ASB_ERROR = data_logger.check_ERROR_values(val, AS_INFO_STATE.ASB_ERROR.value)
                            data_logger.is_EBS_ERROR = data_logger.check_ERROR_values(val, AS_INFO_STATE.EBS_ERROR.value)
                            data_logger.is_RES_ERROR = data_logger.check_ERROR_values(val, AS_INFO_STATE.RES_ERROR.value)
                            data_logger.is_ROS_ERROR = data_logger.check_ERROR_values(val, AS_INFO_STATE.ROS_ERROR.value)
                            data_logger.is_ASMS_OFF = data_logger.check_ERROR_values(val, AS_INFO_STATE.ASMS_OFF.value)

                            data_logger.angle_actuall_DIR = data_logger.get_Direction(val, AS_INFO_STATE.DIRECTION_ACTUALL.value)
                            data_logger.angle_target_DIR = data_logger.get_Direction(val, AS_INFO_STATE.DIRECTION_TARGET.value)

                        elif i == Frame_INFO.ANGLE_TARGET.value:
                            data_logger.angle_target = val
                        elif i == Frame_INFO.ANGLE_ACTUALL.value:
                            data_logger.angle_actuall = val
                        elif i == Frame_INFO.VELOCITY_TARGET.value:
                            data_logger.velocity_target = val
                        elif i == Frame_INFO.VELOCITY_ACTUALL.value:
                            data_logger.velocity_actuall = val
                        elif i == Frame_INFO.AS_STATE.value:
                            data_logger.as_state = State(val)
                        elif i == Frame_INFO.ELECTRICAL_INFO.value:
                            data_logger.CPU_temp = val
                    
                    # Printing data
                    data_logger.print()

                    # Reading data in Prometheus
                    if open_prometheus: 
                        prometheus.read_data_and_send(data_logger)
    except:
        print("SOME ERROR WAS CAUGHT ")

if __name__ == "__main__":
    main()