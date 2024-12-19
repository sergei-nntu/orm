#include <EEPROM.h>



#include "orm.h"
/*
AccelStepper j0(1,X_STEP_PIN,X_DIR_PIN);
AccelStepper j1(1,Y_STEP_PIN,Y_DIR_PIN);
AccelStepper j2(1,Z_STEP_PIN,Z_DIR_PIN);
AccelStepper j3(1,E_STEP_PIN,E_DIR_PIN);
AccelStepper j4(1,Q_STEP_PIN,Q_DIR_PIN);
AccelStepper j5(1,W_STEP_PIN,W_DIR_PIN);
*/
Servo js0;  // Joint Servo 0 
Servo js1;  // Joint Servo 1
Servo js2;  // Joint Servo 2
Servo js3;  // Joint Servo 3
Servo js4;  // Joint Servo 4
Servo js5;  // Joint Servo 5

Servo gripperServo;

const int ADC_MAX = 675;

const char osp_command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

//const int ORM_J_ENCODER_INPUT[JOINTS_COUNT] = {X_ENCODER_IN,Y_ENCODER_IN, Z_ENCODER_IN, E_ENCODER_IN, Q_ENCODER_IN, W_ENCODER_IN} ;/*A1,A2,A3,A4,A5};*/
//const int ORM_J_ENABLE_PIN[JOINTS_COUNT] =    {X_ENABLE_PIN,Y_ENABLE_PIN, Z_ENABLE_PIN, E_ENABLE_PIN, Q_ENABLE_PIN, W_ENABLE_PIN} ;/*7,10,13,16,19};*/
/*
const short orm_j_angle_min = 0;
const short orm_j_angle_max = 1023;
*/
const int ORM_J_SERVO_PINS[JOINTS_COUNT] = {2,3,4,5,6,7};

const int ORM_J_ADC_PINS[JOINTS_COUNT] = {A0,A1,A2,A3,A4,A5};

const int EEPROM_ANGLE_WIDTH_OFFSET = 32;

const int EEPROM_DEFAULT_ANGLE_OFFSET = 64;

//AccelStepper j0(1,X_STEP_PIN,X_DIR_PIN);

void ORM::ospHandleGenericCommand(){
  // TODO
}

void ORM::cmdSetSpeed(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int speed = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    if(speed>0) { // Sanity Check
      j_speed_desired[actuator_no] = speed;
      // Setting the speed to joint in steps per second
      joints[actuator_no]->setMaxSpeed((long)j_speed_desired[actuator_no] * (long)orm_j_stepper_full_rot[actuator_no] / (long)orm_max_int_angle);
    }
  }
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
  char force = osp_input_buffer[OSP_ORM_ANGLE_FORCE_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    j_angle_desired[actuator_no] = angle;
    j_angle_force[actuator_no] = force;
  }

  if (actuator_no == GRIPPER_JOINT_NO) {
    gripper_angle = angle;
  }
}

void ORM::cmdSetCorrAngle(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    j_angle_correction[actuator_no] = angle;
    EEPROM.put(actuator_no*sizeof(short), (short) angle);
    // LEGACY CODE BENEATH. Callibration was needed for stepper motors, not for servo at the moment. 
    //j_callibr_left[actuator_no] = 2; // Force the callibration after the angle correction set.
    
  }
}

void ORM::cmdSetAngleWidth(){
  int actuator_no = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  int angle = ((int)(osp_input_buffer[OSP_ORM_ANGLE_MSB_INDEX]) << 8) | osp_input_buffer[OSP_ORM_ANGLE_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    j_angle_width[actuator_no] = angle;
    EEPROM.put(EEPROM_ANGLE_WIDTH_OFFSET+actuator_no*sizeof(short), (short) angle);
    //j_callibr_left[actuator_no] = 2; // Force the callibration after the angle correction set.
    
  }
}

void ORM::cmdSetMotorPower(){
  char motor_power_param = osp_input_buffer[OSP_BYTE_PARAM_INDEX];
  motor_power = motor_power_param;
  if (motor_power!=0){
    digitalWrite(MOTOR_POWER_PIN,LOW);
  } else {
    digitalWrite(MOTOR_POWER_PIN,HIGH);
  }
}

void ORM::ospHandleORMCommand(){
  // 
  int cmd = osp_input_buffer[OSP_MSG_CMD_INDEX];  
  if (cmd == OSP_ORM_CMD_SET_ANGLE) {
    cmdSetAngle();
  }
  if (cmd == OSP_ORM_CMD_SET_SPEED) {
    cmdSetSpeed();
  }
  if (cmd == OSP_ORM_CMD_SET_CORR_ANGLE) {
    cmdSetCorrAngle();
  }
  if (cmd == OSP_ORM_CMD_SET_ANGLE_WIDTH) {
    cmdSetAngleWidth();
  }
  if (cmd == OSP_ORM_CMD_SET_MOTOR_POWER){
    cmdSetMotorPower();
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
    ospHandleORMCommand();
  }
}

void ORM::ospPrepareOutputBuffer(){
  for(int i=0;i<OSP_BUFFER_SIZE;i++) {
    osp_output_buffer[i] = osp_command_template[i];
  }
}

void ORM::ormInfoCurrentAngle(int actuatorNo){
  ospPrepareOutputBuffer();
    
  unsigned int angle = j_angle_read[actuatorNo]; 

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_ANGLE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xFF;
  osp_output_buffer[OSP_ORM_ANGLE_MSB_INDEX] = angle >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::ormInfoGripperAngle(){
  ospPrepareOutputBuffer();
  
  unsigned short actuatorNo = GRIPPER_JOINT_NO;
  unsigned int angle = gripper_angle;

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_ANGLE;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_ANGLE_LSB_INDEX] = angle & 0xFF;
  osp_output_buffer[OSP_ORM_ANGLE_MSB_INDEX] = angle >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::ormInfoCurrentSpeed(int actuatorNo){
  ospPrepareOutputBuffer();  
  short int speed = j_speed_current[actuatorNo];// j_speed_current[actuatorNo];

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_SPEED;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = actuatorNo;
  osp_output_buffer[OSP_ORM_SPEED_LSB_INDEX] = speed & 0xFF;
  osp_output_buffer[OSP_ORM_SPEED_MSB_INDEX] = speed >> 8;
  
  Serial.write(osp_output_buffer, OSP_COMMAND_LENGTH);
}

void ORM::ormInfoIRStatus(){
  ospPrepareOutputBuffer(); 

  osp_output_buffer[OSP_MSG_DEV_INDEX] = OSP_DEV_ORM;
  osp_output_buffer[OSP_MSG_CMD_INDEX] = OSP_ORM_INFO_IR_STATUS;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX] = 0;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX+1] = 0;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX+2] = 0;
  osp_output_buffer[OSP_BYTE_PARAM_INDEX+3] = 0;
  
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
    // Stepper Joints Update
    for (int i=0;i<JOINTS_COUNT;i++){
      ormInfoCurrentAngle(i);
      ormInfoCurrentSpeed(i);
      ormInfoJointStatus(i);
      
    }
    ormInfoIRStatus();
    // Gripper Angle Update
    ormInfoGripperAngle();
    last_millis = current_millis;
  }
}


short ORM::readAngle(int actuatorNo){
  /*
  // Gamme Angle Obtained from ADC. Casted to Int Angle Range
  long gamma = ((long)analogRead(ORM_J_ENCODER_INPUT[actuatorNo])) * (long)orm_max_int_angle / (long)orm_j_encoder_adc_range;
  // Applying Correction
  gamma += j_angle_correction[actuatorNo];
  // Keep the value in the range of 0 to MAX INT ANGLE
  gamma %= orm_max_int_angle;

  // Keeping track of rotation across of ZERO
  long gamma_plus = gamma + orm_max_int_angle;
  long gamma_minus = gamma - orm_max_int_angle;

  long diff_gamma = abs(gamma - j_angle_read[actuatorNo]);
  long diff_gamma_plus = abs(gamma_plus - j_angle_read[actuatorNo]);
  long diff_gamma_minus = abs(gamma_minus- j_angle_read[actuatorNo]);

  long gamma_res=0; 
  if (diff_gamma<=diff_gamma_plus){
    if(diff_gamma<=diff_gamma_minus){
      gamma_res = gamma;      
    } else {
      gamma_res = gamma_minus;
    }
  } else {
    if(diff_gamma_plus <= diff_gamma_minus){
      gamma_res = gamma_plus;
    } else {
      gamma_res = gamma_minus;
    }
  }
  // Applying statistical filtering
  // 1. Save the current value

  j_angle_sample[actuatorNo * STAT_SAMPLE_SIZE +read_samples_ptr[actuatorNo]] = gamma_res;
  read_samples_ptr[actuatorNo] ++;
  read_samples_ptr[actuatorNo] %= STAT_SAMPLE_SIZE;
  if (read_samples_size[actuatorNo]<STAT_SAMPLE_SIZE){
    read_samples_size[actuatorNo] ++;
  }

  // 2. Take an average of the available measurements
  long int gamma_sum = 0;
  for (int i=0; i<read_samples_size[actuatorNo];i++){
    gamma_sum += j_angle_sample[actuatorNo * STAT_SAMPLE_SIZE + i];
  }
  gamma_res = gamma_sum / read_samples_size[actuatorNo];
  
  return gamma_res;
  */
  return 0;
}

int isqrt(int x) {
    if (x <= 0)
        return 0;

    int r = x;
    int y = (r + x / r) / 2;

    while (y < r) {
        r = y;
        y = (r + x / r) / 2;
    }

    return r;
}

void ORM::updateActuatorsPosition(){  
  // UPDATE THE SPEED FIRST
  unsigned long current_millis = millis();
  
  if(current_millis-speed_millis>ORM_SPEED_UPDATE_INTERVAL_MS){
    speed_millis = current_millis;

    // Speed Control Routine
    for(int i=0;i<JOINTS_COUNT;i++){
      if (motor_power == 0){
        // If no motor power - just apply the same angle that is currenrly read 
        j_angle_current[i] = j_angle_read[i];
        j_angle_desired[i] = j_angle_read[i];
        j_speed_current[i] = 0;
        continue;
      }

      if(j_angle_force[i]){
        // If the angle is forced - apply it immediately, no acceleration logic
        j_angle_current[i] = j_angle_desired[i];
        j_speed_current[i] = 0;
        continue;        
      }

      long angle_diff = j_angle_desired[i] - j_angle_read[i];
      long direction = sgn(angle_diff);

      long accelerate_speed =(long)j_speed_current[i] + direction*(long)orm_j_acceleration[i]*(long)ORM_SPEED_UPDATE_INTERVAL_MS / (long)ORM_MS_IN_SECOND;

      long abs_angle_diff = angle_diff * direction;

      if (abs_angle_diff < js_small_angle_threshold[i]) {
        //j_angle_current[i] = j_angle_desired[i];
        j_speed_current[i] = 0;
        continue;        
      }

      long sqrt_angle_diff = isqrt(abs_angle_diff);

      float deaccelerate_speed = (long)direction*(long)sqrt_angle_diff * (long)ORM_SPEED_UPDATE_INTERVAL_MS*(long)orm_j_acceleration[i] / (long)ORM_MS_IN_SECOND; 
      
      long max_speed = direction * orm_j_speed_max[i];

      long result_speed = min(min(direction*max_speed,direction*accelerate_speed),direction*deaccelerate_speed);

      j_speed_current[i] = direction*result_speed;
      
      // Do not limit to the desired angle only. Allow to pass over the desired angle if deacceleration is not possible
      float new_angle = (long)j_angle_current[i] + (long)j_speed_current[i]*(long)ORM_SPEED_UPDATE_INTERVAL_MS / (long)ORM_MS_IN_SECOND ;
      long new_angle_diff = j_angle_desired[i] - new_angle;
      //if(new_angle_diff * angle_diff < 0){
        //j_angle_current[i] = j_angle_desired[i];
      //} else {
         j_angle_current[i] = new_angle;  // Update the angle
      //}
    }
  }

  // Update ORA-based joints
  for (int i=0;i<JOINTS_COUNT;i++){
    /*
    j_angle_read[i] = readAngle(i);

    if(!joints[i]->isRunning()){
      if(abs(j_angle_current[i] - j_angle_desired[i])>0) {
        long int steps_to_make = (long)(j_angle_desired[i] - j_angle_current[i]) * (long)orm_j_stepper_full_rot[i] / orm_max_int_angle;
        joints[i]->move(steps_to_make);
        j_angle_current[i] = j_angle_desired[i];
      } else {
        // Only apply the correction if enough data is accumulated
        if(read_samples_size[i] == STAT_SAMPLE_SIZE) {
          if(fabs(j_angle_read[i]-j_angle_current[i])>200) {
            j_callibr_left[i] = 2; 
          }
          if(j_callibr_left[i] >0 && read_samples_size[i] == STAT_SAMPLE_SIZE){
            // If encoder value differs too much - from the expected value - make the correction
            long int steps_to_make = ((long)(j_angle_current[i] - j_angle_read[i]) * (long)orm_j_stepper_full_rot[i] / orm_max_int_angle);
            joints[i]->move(steps_to_make);
            j_callibr_left[i] --;
          }
        }
      }
    }
    */
    
    float servo_angle = ((float)(j_angle_current[i]+j_angle_correction[i])*(float)orm_180_angle_width / (float)j_angle_width[i])*(float)360/(float)orm_max_int_angle;
    joint_servos[i]->write(servo_angle);

  }
  // Update Gripper Position
  int servo_angle = (long)gripper_angle * (long)360 / (long)orm_max_int_angle;
  gripper_servo->write(servo_angle);
}


void ORM::updateSensorsMeasurements(){
  for (int i=0;i<JOINTS_COUNT;i++){
    int adc_read = analogRead(ORM_J_ADC_PINS[i]);
    int angle_int = orm_max_int_angle * adc_read  / ADC_MAX;
    j_angle_read_samples[i][j_angle_samples_ptr] = angle_int;
  }
  j_angle_samples_ptr++;
  j_angle_samples_ptr %= ADC_SAMPLES_N;
  if(j_angle_samples_count<ADC_SAMPLES_N){
    j_angle_samples_count++;
  }
  for(int i=0;i<JOINTS_COUNT;i++){
    long int sum = 0;
    for (int j=0;j<j_angle_samples_count;j++){
      sum += j_angle_read_samples[i][j];
    }
    int filtered_angle = sum / j_angle_samples_count;

    filtered_angle = (float) j_angle_width[i] * (float)(filtered_angle - servo_zero_angle[i]) / (float)(servo_max_angle[i]-servo_zero_angle[i]);

    filtered_angle = filtered_angle-j_angle_correction[i];
    j_angle_read[i] = filtered_angle;
    //j_angle_desired[i] = j_angle_read[i];
  }

}

void ORM::ospSerialLoop(){

  while(Serial.available()){
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
  updateSensorsMeasurements();
  updateActuatorsPosition();
  sendUpdateInfo();
  /*

  for (int i=0;i<JOINTS_COUNT;i++){
   joints[i]->run();
  }
  */
}

void ORM::setup(){

  Serial.begin(115200);
  Serial.setTimeout(0.01);

  //analogReference(EXTERNAL);

  last_millis = millis();
  speed_millis = millis();

  pinMode(MOTOR_POWER_PIN, OUTPUT);
  if (motor_power!=0){
    digitalWrite(MOTOR_POWER_PIN,LOW);
  } else {
    digitalWrite(MOTOR_POWER_PIN,HIGH);
  }
  /*
  joints[0] = &j0;
  joints[1] = &j1;
  joints[2] = &j2;
  joints[3] = &j3;
  joints[4] = &j4;
  joints[5] = &j5;  
*/
  joint_servos[0] = &js0;
  joint_servos[1] = &js1;
  joint_servos[2] = &js2;
  joint_servos[3] = &js3;
  joint_servos[4] = &js4;
  joint_servos[5] = &js5;

  gripper_servo = &gripperServo;
  gripper_servo->attach(8);
  gripper_servo->write(0);

  // Initing Joints Servos 
  for (int i=0;i<JOINTS_COUNT;i++){
    joint_servos[i]->attach(ORM_J_SERVO_PINS[i],500,2500);
    joint_servos[i]->write(0);    
  }

  for (int i=0;i<JOINTS_COUNT;i++) {
    /*
   // INIT JOINT
   joints[i]->setEnablePin(ORM_J_ENABLE_PIN[i]);
   joints[i]->setPinsInverted(false,false,true);
   //joints[i]->disableOutputs();
   joints[i]->setAcceleration(200000); // 200 steps per second per second
   joints[i]->enableOutputs();
   // Populate desired speed
   j_speed_desired[i] = (long)orm_max_int_angle * (long)500 /(long)orm_j_stepper_full_rot[i];
   // Set speed in steps/second
   joints[i]->setMaxSpeed((long)j_speed_desired[i] * (long)orm_j_stepper_full_rot[i] / (long)orm_max_int_angle);
   //joints[i]->setSpeed(100);*/
   // LOAD CORRECTION ANGLE
   EEPROM.get(i*sizeof(short),j_angle_correction[i]); 
   j_angle_current[i]  = 0;

   EEPROM.get(EEPROM_ANGLE_WIDTH_OFFSET+i*sizeof(short),j_angle_width[i]); 
   if(j_angle_width[i]<=0) {
     j_angle_width[i] = 16384;
   }

   EEPROM.get(EEPROM_DEFAULT_ANGLE_OFFSET+i*sizeof(short),j_angle_desired[i]); 
 }
}

ORM::ORM(){
  
}