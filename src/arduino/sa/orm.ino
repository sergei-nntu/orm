#include "orm.h"

AccelStepper j0(1,X_STEP_PIN,X_DIR_PIN);
AccelStepper j1(1,Y_STEP_PIN,Y_DIR_PIN);

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

const int ORM_J_ENCODER_INPUT[JOINTS_COUNT] = {X_ENCODER_IN,Y_ENCODER_IN};/*A1,A2,A3,A4,A5};*/
const int ORM_J_ENABLE_PIN[JOINTS_COUNT] =  {X_ENABLE_PIN,Y_ENABLE_PIN};/*7,10,13,16,19};*/
/*
const short orm_j_angle_min = 0;
const short orm_j_angle_max = 1023;
*/

const short orm_j_encoder_range = 1023; // Max Value of Encoder output
const long orm_j_stepper_full_rot[JOINTS_COUNT] = {16000, 16000}; 

//AccelStepper j0(1,X_STEP_PIN,X_DIR_PIN);

void ORM::ospHandleGenericCommand(){
  // TODO
}

void ORM::cmdSetSpeed(){

}
/*
void ORM::cmdSetAngle(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    if(angle<orm_j_angle_min) {
      angle = orm_j_angle_min;
    }  
    if(angle > orm_j_angle_max) {
      angle = orm_j_angle_max;
    }
    j_angle_desired[actuator_no] = angle;
    j_goal_achieved[actuator_no] = 0;    
  }

}
*/

void ORM::cmdSetAngle(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    //joints[0]->moveTo(angle);
    j_angle_desired[actuator_no] = angle;
    
    /*if(angle<orm_j_angle_min) {
      angle = orm_j_angle_min;
    }  
    if(angle > orm_j_angle_max) {
      angle = orm_j_angle_max;
    }
    */
  }

}



void ORM::ospHandleORMCommand(){
  // 
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_ORM_CMD_SET_ANGLE) {
    // RESPONSE WITH A CURRENT SOC STATE
    cmdSetAngle();
  }
  if (cmd == OSP_ORM_CMD_SET_SPEED) {
    // RESPONSE WITH A CURRENT VOLTAGE LEVEL
    cmdSetSpeed();
  }
}

void ORM::ospHandleCommand(){
  int dev = osp_input_buffer[OSP_MSG_DEV_INDEX];
  
  if (dev == OSP_DEV_GENERIC) {
    ospHandleGenericCommand();
  } else if (dev == OSP_DEV_CURRENT) {
    if (dev == OSP_DEV_ORM) {
      ospHandleORMCommand();
    }
  } else {
    // ERROR CONDITION. THE CLIENT MUST NOT SEND THE COMMAND WHICH ARE NOT FOR EITHER GENERIC OR CURRENT DEV TYPE
    // DOING NOTHING FOR NOW
  }
}

void ORM::ospPrepareOutputBuffer(){
  for(int i=0;i<OSP_BUFFER_SIZE;i++) {
    osp_output_buffer[i] = osp_command_template[i];
  }
}

void ORM::ormInfoCurrentAngle(int actuatorNo){
  ospPrepareOutputBuffer();
    
  unsigned int angle = j_angle_current[actuatorNo]; 

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_ANGLE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xFF;
  osp_output_buffer[OSP_ORM_ANGLE_MSB_INDEX] = angle >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::ormInfoCurrentSpeed(int actuatorNo){
  ospPrepareOutputBuffer();  
  short int speed = joints[actuatorNo]->speed();// j_speed_current[actuatorNo];

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_SPEED;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_SPEED_LSB_INDEX] = speed & 0xFF;
  osp_output_buffer[OSP_ORM_SPEED_MSB_INDEX] = speed >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::ormInfoJointStatus(int actuatorNo) {
  short int status_word = 0;
  // Forming the status byte
  int powered = 1; // STUB VALUE. TO BE READ IN LATER RELEASES
  int error = 0; // STUB VALUE. TO BE READ IN LATER RELEASES

  status_word = status_word | (joints[actuatorNo]->isRunning() << OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX);
  status_word = status_word | (powered << OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX);
  status_word = status_word | (error << OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX);

  // Forming the output buffer
  ospPrepareOutputBuffer(); 

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_JOINT_STATE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_STATUS_LSB_INDEX] = status_word & 0xFF;
  osp_output_buffer[OSP_ORM_STATUS_MSB_INDEX] = status_word >> 8;

  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::sendUpdateInfo(){
  unsigned long current_millis = millis();
  if (current_millis - last_millis > UPDATE_INTERVAL){
    for (int i=0;i<JOINTS_COUNT;i++){
      ormInfoCurrentAngle(i);
      ormInfoCurrentSpeed(i);
      ormInfoJointStatus(i);
    }
    last_millis = current_millis;
  }
}

void ORM::updateActuatorsPosition(){  
  for (int i=0;i<JOINTS_COUNT;i++){
    j_angle_read[i] = ((long)analogRead(ORM_J_ENCODER_INPUT[i])) * (long)orm_j_stepper_full_rot[i] / (long)orm_j_encoder_range;
    if(!joints[i]->isRunning()){
      if(abs(j_angle_current[i] - j_angle_desired[i])>0) {
        joints[i]->move(j_angle_desired[i] - j_angle_current[i]);
        j_angle_current[i] = j_angle_desired[i];
      } else {
        if(fabs(j_angle_read[i]-j_angle_current[i])>600){
          // If encoder value differs too much - from the expected value - make the correction
          joints[i]->move(j_angle_current[i]-j_angle_read[i]);
        }
      }
    }
  }
  /*
  for (int i=0;i<JOINTS_COUNT;i++) {
    j_angle_current[i] = analogRead(ORM_J_ENCODER_INPUT[i]);
    if(j_angle_current[i] < orm_j_angle_min/2 || j_angle_current[i] > (orm_j_angle_max+1024)/2){
      // HALT THE ACTUATOR IF THE ANGLE IS OUTSIDE THE ALLOWED RANGE
      j_speed_current[i] = 0;   
      j_goal_achieved[i] = 16;  
    } else {
      if(abs(j_angle_desired[i]-j_angle_current[i])>30) {
        j_speed_current[i] = sgn(j_angle_desired[i] - j_angle_current[i])*300;
        j_goal_achieved[i] = 0;
      } else {
        j_speed_current[i] = (j_angle_desired[i] - j_angle_current[i])*10;
      }
    }
    if (j_goal_achieved[i]>=16){
      //joints[i]->disableOutputs();
    } else {
      joints[i]->enableOutputs();
    }
    joints[i]->setSpeed(j_speed_current[i]);    
  } 
  */
}

void ORM::ospSerialLoop(){
  if(Serial.available()){
    char b = Serial.read();
    if(osp_command_template[osp_ptr]==b || osp_command_template[osp_ptr]==0){
      osp_input_buffer[osp_ptr] = b;
      osp_ptr++; 
      if(osp_ptr==OSP_COMMAND_LENGTH){
        ospHandleCommand();
        osp_ptr = 0;
      }
    } else {
      osp_ptr = 0;
    }
    
  }
  updateActuatorsPosition();
  sendUpdateInfo();

  for (int i=0;i<JOINTS_COUNT;i++){
   joints[i]->run();
  }
}

void ORM::setup(){
  Serial.begin(115200);
  Serial.setTimeout(0.01);

  analogReference(EXTERNAL);

  last_millis = millis();
  joints[0] = &j0;
  joints[1] = &j1;
  for (int i=0;i<JOINTS_COUNT;i++) {
   joints[i]->setEnablePin(ORM_J_ENABLE_PIN[i]);
   joints[i]->setPinsInverted(false,false,true);
   //joints[i]->disableOutputs();
   joints[i]->setAcceleration(200000); // 200 steps per second per second
   joints[i]->enableOutputs();
   joints[i]->setMaxSpeed(500);
   //joints[i]->setSpeed(100);
 }
}

ORM::ORM(){
  
}