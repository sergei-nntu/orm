# Importing Libraries
import serial
import time
import math
import threading
import glob
import sys

# OSP DATA INDICIES WITHIN THE MESSAGE

OSP_MSG_DEV_INDEX = 2
OSP_MSG_CMD_INDEX = 3
OSP_BYTE_PARAM_INDEX = 4
OSP_INT_PARAM_MSB_INDEX = 5
OSP_INT_PARAM_LSB_INDEX = 4 

# OSP DEVICE TYPES

OSP_DEV_GENERIC = 0
OSP_DEV_ORM = 1
OSP_DEV_OBP = 2
OSP_DEV_O2D = 3

# OSP COMMANDS 

OSP_CMD_REQ_DEV_TYPE = 0x01
OSP_INFO_DEV_TYPE = 0x11
OSP_OBP_CMD_REQ_SOC = 0x01
OSP_OBP_CMD_REQ_VOLTAGE = 0x02
OSP_OBP_CMD_REQ_CURRENT = 0x03
OSP_OBP_CMD_REQ_STATUS = 0x04

OSP_OBP_INFO_SOC = 0x11
OSP_OBP_INFO_VOLTAGE = 0x12
OSP_OBP_INFO_CURRENT = 0x13
OSP_OBP_INFO_STATUS = 0x14


OSP_ORM_JOINT_INDEX = 4
OSP_ORM_ANGLE_MSB_INDEX = 6
OSP_ORM_ANGLE_LSB_INDEX = 5

OSP_ORM_CMD_SET_ANGLE = 0x02
OSP_ORM_CMD_SET_SPEED = 0x03
OSP_ORM_CMD_SET_CORR_ANGLE= 0x05
OSP_ORM_INFO_ANGLE = 0x12
OSP_ORM_INFO_SPEED = 0x13
OSP_ORM_INFO_STATUS = 0x14

OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX = 0
OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX = 1
OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX =   2

OSP_DEV_CURRENT = OSP_DEV_OBP

OSP_COMMAND_LENGTH = 10
OSP_BUFFER_SIZE = 10


ORM_INT_ANGLE_MAX = 65536/2

class OSP:
    command_buffer_pattern = [ 0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77]
    input_index = 0
    output_buffer = []
    input_buffer = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    osp_serial = None
    osp_dev_type = 0
    output_thread = None
    input_thread = None
    closed = False 
    
    soc = 0
    voltage = 0
    current = 0
    
    # JOINT MEASUREMENTS
    joint_angle = [0,0,0,0,0,0,0]
    joint_speed = [0,0,0,0,0,0,0]
    
    # JOINT STATUS FLAGS
    joint_status_powered = [0,0,0,0,0,0,0]
    joint_status_running = [0,0,0,0,0,0,0]
    joint_status_error = [0,0,0,0,0,0,0]
    
    
    def __init__(self,port_name):
        # '/dev/ttyACM1'
        self.osp_serial = serial.Serial(port=port_name, baudrate=115200, timeout=.1)
        #print("Starting Output Thread")
        self.output_thread = threading.Thread(target=self.output_thread, daemon=True)  
        self.output_thread.start()
        #print("Starting Input Thread")
        self.input_thread = threading.Thread(target=self.input_thread, daemon=True)  
        self.input_thread.start()
    
    def stop(self):
        self.closed = True
        self.osp_serial.close()
        
    
    def get_dev_type(self):
        return self.osp_dev_type
    
    def output_thread(self):
        while self.closed is not True:
            if len(self.output_buffer) > 0:
                #print("Sending Byte"+str(self.output_buffer[0]));
                self.osp_serial.write(bytes([self.output_buffer.pop(0)]))
                time.sleep(0.001)
     
    def osp_send_command(self,cmd_bytes):
        self.output_buffer += cmd_bytes
        
    def osp_req_device_type(self):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_GENERIC
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_CMD_REQ_DEV_TYPE
        self.osp_send_command(cmd_bytes)
     

    def set_angle(self, joint, angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)
        
    def set_speed(self, joint, speed):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_SPEED
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (speed >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = speed & 0xff
        self.osp_send_command(cmd_bytes)
        
    def set_corr_angle(self,joint,angle):
        cmd_bytes = self.command_buffer_pattern.copy()
        cmd_bytes[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM
        cmd_bytes[OSP_MSG_CMD_INDEX] = OSP_ORM_CMD_SET_CORR_ANGLE
        cmd_bytes[OSP_ORM_JOINT_INDEX] = joint
        cmd_bytes[OSP_ORM_ANGLE_MSB_INDEX] = (angle >> 8) &0xff
        cmd_bytes[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xff
        self.osp_send_command(cmd_bytes)
                    
    def osp_info_dev_type(self):
        self.osp_dev_type = self.input_buffer[OSP_BYTE_PARAM_INDEX]
        #print("Device Type Received:",self.osp_dev_type);
                
    def osp_obp_info_soc(self):
        soc_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        soc_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        soc = soc_lsb | (soc_msb << 8)
        self.soc = soc
        #print("SOC: "+str(soc))
        
    def osp_obp_info_current(self):
        current_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        current_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        current = current_lsb | (current_msb << 8)
        if current >> 15 != 0:
            # Two's complement decoding
            current = -((~current & 0xffff)+1)
        self.current = current
        #print("I: "+str(current)+" mA")
        
    def osp_obp_info_voltage(self):
        v_lsb = self.input_buffer[OSP_INT_PARAM_LSB_INDEX]
        v_msb = self.input_buffer[OSP_INT_PARAM_MSB_INDEX]
        v = v_lsb | (v_msb << 8)
        self.voltage = v
        #print("V: "+str(v)+" mV")

    def orm_info_angle(self):
        #print("ORM Info Angle")
        actuator_no = self.input_buffer[4]
        angle = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if angle & 0x8000 !=0:
            angle = -((~angle & 0xffff) + 1)
        self.joint_angle[actuator_no] = angle
#        if actuator_no == 5:
        #print("Current Angle For Actuator "+str(actuator_no)+" is "+str(angle))
   
    def orm_info_speed(self):
        actuator_no = self.input_buffer[4]
        speed = self.input_buffer[5] | (self.input_buffer[6] << 8)
        if speed & 0x8000 !=0:
            speed = -((~speed & 0xffff) + 1)
        self.joint_speed[actuator_no] = speed
        #print("Current Speed For Actuator "+str(actuator_no)+" is "+str(speed))
    
    def extract_bit_with_index(self, word, bit_index):
        return (word & (1 << bit_index)) >> bit_index
        
    def orm_info_status(self):  
        actuator_no = self.input_buffer[4]
        status_word = self.input_buffer[5] | (self.input_buffer[6] << 8)
        
        running = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX)
        powered = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX)
        error = self.extract_bit_with_index(status_word, OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX)
        
        self.joint_status_running[actuator_no] = running
        self.joint_status_powered[actuator_no] = powered
        self.joint_status_error[actuator_no] = error
        #print("JOINT "+str(actuator_no)+ " RUNNING = "+str(running)+"; POWERED = "+str(powered)+"; ERROR = "+str(error))
        
    
    def orm_is_running(self):
        return 1 in self.joint_status_running
        
    def orm_set_angle(self, joint, angle):
        print("Setting Angle to ORM joint"+str(joint)+": "+str(angle))
        int_angle = int(angle * ORM_INT_ANGLE_MAX / (2* math.pi))
        self.set_angle(joint, int_angle)
        
    def orm_set_speed(self, joint, speed): # In Radians/Sec
        if speed > 0: # Sanity Check
            int_speed =  int(ORM_INT_ANGLE_MAX * speed / (2*math.pi))
            self.set_speed(joint, int_speed)
        
        
        
    def input_thread(self):
        while self.closed is not True:
            try:
                bts = self.osp_serial.read()
                #print("Bytes_Read:"+str(bts))
                if len(bts)>0:
                    #print("Bytes_Read:"+str(bts))
                    bt = bts[0]
                    if self.command_buffer_pattern[self.input_index] == bt or self.command_buffer_pattern[self.input_index] ==0:
                        self.input_buffer[self.input_index] = bt
                        self.input_index += 1
                        if self.input_index >= len(self.command_buffer_pattern):
                            #print("Command Received:"+str(self.input_buffer))
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_OBP:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_SOC:
                                    self.osp_obp_info_soc()
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_CURRENT:
                                    self.osp_obp_info_current()
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_OBP_INFO_VOLTAGE:
                                    self.osp_obp_info_voltage()
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_ORM:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] ==  OSP_ORM_INFO_ANGLE:
                                    self.orm_info_angle();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_SPEED:
                                    self.orm_info_speed();
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_ORM_INFO_STATUS:
                                    self.orm_info_status();
                            if self.input_buffer[OSP_MSG_DEV_INDEX] == OSP_DEV_GENERIC:
                                if self.input_buffer[OSP_MSG_CMD_INDEX] == OSP_INFO_DEV_TYPE:
                                    self.osp_info_dev_type()
                            self.input_index = 0
                    else:
                        self.input_index = 0
                #time.sleep(0.000001)
            except serial.serialutil.SerialException:
                break # Exiting while loop
        


def find_osp_peripheral(osp_dev_type):
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty.usb*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.usb*')
    else:
        raise EnvironmentError('Unsupported platform')
    
    devs = list(map(lambda port: OSP(port), ports))
    
    time.sleep(3) # Timeout to receive the first messages from the devices
    
    devs_of_type = filter(lambda d: d.get_dev_type() == osp_dev_type, devs)
    
    devs_of_wrong_type = filter(lambda d: d.get_dev_type() != osp_dev_type, devs)
    for dev in devs_of_wrong_type:
        dev.stop()
    
    return list(devs_of_type)


