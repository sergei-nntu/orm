# Importing Libraries
import serial
import time
import math
import threading


ARM1_COMMAND_MAKE_STEP	= 1
ARM1_COMMAND_SET_ANGLE	= 2


ARM1_INFO_CURRENT_ANGLE	= 64
ARM1_INFO_CURRENT_SPEED	= 65
ARM1_INFO_CURRNET_ADC	= 66  


class ArmOne:
    command_buffer_pattern = [ 0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77]
    input_index = 0
    output_buffer = []
    input_buffer = [0, 0, 0, 0, 0, 0, 0, 0]
    armoneserial = None
    def __init__(self,port_name):
        # '/dev/ttyACM1'
        self.armoneserial = serial.Serial(port=port_name, baudrate=115200, timeout=.1)
        output_thread = threading.Thread(target=self.output_thread, daemon=True)  
        output_thread.start()
        input_thread = threading.Thread(target=self.input_thread, daemon=True)  
        input_thread.start()
    
    def output_thread(self):
        while True:
            if len(self.output_buffer) > 0:
                #print("Sending Byte"+str(self.output_buffer[0]));
                self.armoneserial.write(bytes([self.output_buffer.pop(0)]))
                time.sleep(0.001)

    def info_current_angle(self):
        actuator_no = self.input_buffer[3]
        angle = self.input_buffer[4] | (self.input_buffer[5] << 8)
        print("Current Angle For Actuator "+str(actuator_no)+" is "+str(angle))
#        if actuator_no == 5:
#            print("Current Angle For Actuator "+str(actuator_no)+" is "+str(angle))
   
    def info_current_speed(self):
        actuator_no = self.input_buffer[3]
        speed = self.input_buffer[4] | (self.input_buffer[5] << 8)
        print("Current Speed For Actuator "+str(actuator_no)+" is "+str(speed))
        #print("Current Speed For Actuator "+str(actuator_no)+" is "+str(speed))
    
    def input_thread(self):
        while True:
            bts = self.armoneserial.read()
            #print("Bytes_Read:"+str(bts))
            if len(bts)>0:
                #print("Bytes_Read:"+str(bts))
                bt = bts[0]
                if self.command_buffer_pattern[self.input_index] == bt or self.command_buffer_pattern[self.input_index] ==0:
                    self.input_buffer[self.input_index] = bt
                    self.input_index += 1
                    if self.input_index >= len(self.command_buffer_pattern):
                        print("Command Received:"+str(self.input_buffer))
                        if self.input_buffer[2] ==  ARM1_INFO_CURRENT_ANGLE:
                            self.info_current_angle()
                        if self.input_buffer[2] ==  ARM1_INFO_CURRENT_SPEED:
                            self.info_current_speed()
                        self.input_index = 0
                else:
                    self.input_index = 0
            time.sleep(0.000001)
    
    def makeSteps(self,actuator_no, steps_count):
    	command_buffer = self.command_buffer_pattern.copy()
    	
    	command_buffer[2] = ARM1_COMMAND_MAKE_STEP
    	command_buffer[3] =  actuator_no
    	command_buffer[4] = steps_count & 0xFF
    	command_buffer[5] = (steps_count >> 8) & 0xFF
    	print("Sending Command:"+str(command_buffer))
    	self.output_buffer.extend(command_buffer)

    def set_angle(self, actuator_no, angle):
        # Fit angle in range 0 .. 1023
        angle_int = 512 + int(angle * 1024 / (math.pi*2))
        if angle_int < 0:
            angle_int = 0
        if angle_int > 1023:
            angle_int = 1023
        
        print("Setting Angle: "+str(angle_int))
        
        command_buffer = self.command_buffer_pattern.copy()
        command_buffer[2] = ARM1_COMMAND_SET_ANGLE
        command_buffer[3] =  actuator_no
        command_buffer[4] = angle_int & 0xFF
        command_buffer[5] = (angle_int >> 8) & 0xFF
        print("Sending Command:"+str(command_buffer))
        
        self.output_buffer.extend(command_buffer)
        
    	

#while True:
#  arduino.write(bytes.fromhex("FFAA000000005577"))
#  data = arduino.read(8);
#  print(data)

#def write_read(x):
#    arduino.write(bytes(x, 'utf-8'))
#    time.sleep(0.05)
#    data = arduino.readline()
#    return data
#while True:
#    num = input("Enter a number: ") # Taking input from user
#    value = write_read(num)
#    print(value) # printing the value
