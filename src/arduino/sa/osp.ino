#include "osp.h"
/*
char            osp_output_buffer[OSP_BUFFER_SIZE];
unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
int             osp_ptr=0;

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};


// CALLBACK FUNCTIONS


// CURRENT DEVICE TYPE? IS IT NEEDED? 


// HANDING OSP GENERIC COMMANDS

void osb_info_dev_type(){
  osp_prepare_output_buffer();
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_GENERIC;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_INFO_DEV_TYPE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = OSP_DEV_CURRENT; // Returning the current device type as a parameter
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void osp_handle_generic_command(){
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_CMD_REQ_DEV_TYPE) {
    // HERE WE MUST RESPONSE WITH A DEV TYPE
    osb_info_dev_type();
  }
}

void osp_prepare_output_buffer(){
  for(int i=0;i<OSP_BUFFER_SIZE;i++) {
    osp_output_buffer[i] = osp_command_template[i];
  }
}

// HANDLING OBP COMMANDS
int obp_voltage = 0;      // in mV
int obp_current_draw = 0; // in mA
int obp_soc = 2000;       // in mAh


void osp_opb_send_info_soc(){
  osp_prepare_output_buffer();
    
  unsigned int soc_int = obp_soc;
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_SOC;
  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = soc_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = soc_int >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void osp_opb_send_info_voltage(){
  osp_prepare_output_buffer();
  unsigned int voltage_int = obp_voltage ; // Voltage in mV

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_VOLTAGE;
  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = voltage_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = voltage_int >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);  
}

void osp_opb_send_info_current(){
  osp_prepare_output_buffer();
  int  current_int = obp_current_draw; // mA 
  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_OBP;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_OBP_INFO_CURRENT;

  osp_output_buffer[OSP_INT_PARAM_LSB_INDEX] = current_int & 0xFF;
  osp_output_buffer[OSP_INT_PARAM_MSB_INDEX] = current_int >> 8;
   
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);   
}

void osp_opb_send_info_status(){
  
}


void osp_handle_obp_command(){
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_OBP_CMD_REQ_SOC) {
    // RESPONSE WITH A CURRENT SOC STATE
    osp_opb_send_info_soc();
  }
  if (cmd == OSP_OBP_CMD_REQ_VOLTAGE) {
    // RESPONSE WITH A CURRENT VOLTAGE LEVEL
    osp_opb_send_info_voltage();
  }
  if (cmd == OSP_OBP_CMD_REQ_CURRENT) {
    // RESPONSE WITH A CURRENT CURRENT DRAW
    osp_opb_send_info_current();
  }
  if (cmd == OSP_OBP_CMD_REQ_STATUS) {
    // RESPONSE WITH A CURRENT BATTERY STATUS
    osp_opb_send_info_status();
  }
}

void osp_handle_command(){
  int dev = osp_input_buffer[OSP_MSG_DEV_INDEX];
  
  if (dev == OSP_DEV_GENERIC) {
    osp_handle_generic_command();
  } else if (dev == OSP_DEV_CURRENT) {
    if (dev == OSP_DEV_OBP) {
      osp_handle_obp_command();
    }
  } else {
    // ERROR CONDITION. THE CLIENT MUST NOT SEND THE COMMAND WHICH ARE NOT FOR EITHER GENERIC OR CURRENT DEV TYPE
    // DOING NOTHING FOR NOW
  }
}

void setup_osp(){
  Serial.begin(115200);
  Serial.setTimeout(0.01);  
}

void loop_osp(){
  if(Serial.available()){
    char b = Serial.read();
    if(osp_command_template[osp_ptr]==b || osp_command_template[osp_ptr]==0){
      osp_input_buffer[osp_ptr] = b;
      osp_ptr++; 
      if(osp_ptr==OSP_COMMAND_LENGTH){
        osp_handle_command();
        osp_ptr = 0;
      }
    } else {
      osp_ptr = 0;
    }
  }
}
*/