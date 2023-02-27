#include <AccelStepper.h>

// Convenience sign function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Update Interval in Milliseconds
#define UPDATE_INTERVAL  100
unsigned long last_millis;

#define BUFFER_SIZE  8

#define ARM1_CMD_MAKE_STEP  1
#define ARM1_CMD_SET_ANGLE  2
#define ARM1_CMD_SET_SPEED  3

#define ARM1_INFO_CURRENT_ANGLE  64
#define ARM1_INFO_CURRENT_SPEED  65
#define ARM1_INFO_CURRNET_ADC.   66  



char output_buffer[BUFFER_SIZE];
unsigned char input_buffer[BUFFER_SIZE];
int ptr=0;

#define COMMAND_LENGTH  8

const char command_template[] = {0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x55, 0x77};

#define ARM1_MSG_OP_INDEX         2
#define ARM1_MSG_JOINT_INDEX      3
#define ARM1_MSG_PARAM_LSB_INDEX  4
#define ARM1_MSG_PARAM_MSB_INDEX  5

/*

// JOINT 0
short j0_angle_desired = 512;
short j0_angle_current = 512;
const short j0_angle_min = 128;
const short j0_angle_max = 1023 - 128;
short j0_speed_current = 0;

const int J0_ENCODER_INPUT = A0;
const int j0_enable_pi = 4;
int j0_goal_achieved = 16;
*/

#define JOINTS_COUNT              6
AccelStepper j0(1,2,3);
AccelStepper j1(1,5,6);
AccelStepper j2(1,8,9);
AccelStepper j3(1,11,12);
AccelStepper j4(1,14,15);
AccelStepper j5(1,17,18);
AccelStepper* joints[JOINTS_COUNT];
short j_angle_desired[JOINTS_COUNT] = {512, 512, 512, 512, 512, 512};
short j_angle_current[JOINTS_COUNT] = {512, 512, 512, 512, 512, 512};
short j_speed_current[JOINTS_COUNT] = {0,0,0,0,0,0};
short j_goal_achieved[JOINTS_COUNT] = {16, 16, 16, 16, 16, 16};
const int J_ENCODER_INPUT[JOINTS_COUNT] = {A0,A1,A2,A3,A4,A5};
const int J_ENABLE_PIN[JOINTS_COUNT] =  {4,7,10,13,16,19};
const short j_angle_min = 210;
const short j_angle_max = 1023-210;

void setup() {
 last_millis = millis();
 Serial.begin(115200);
 Serial.setTimeout(0.01);
 //pinMode(13, OUTPUT);
 analogReference(EXTERNAL);
 //digitalWrite(13,LOW);
 //
 joints[0] = &j0;
 joints[1] = &j1;
 joints[2] = &j2;
 joints[3] = &j3;
 joints[4] = &j4;
 joints[5] = &j5;
 
 for (int i=0;i<JOINTS_COUNT;i++) {
   joints[i]->setEnablePin(J_ENABLE_PIN[i]);
   joints[i]->setPinsInverted(false,false,true);
   joints[i]->disableOutputs();
   joints[i]->setMaxSpeed(600);
   joints[i]->setSpeed(0);
 }
 
}

void cmd_make_steps(){
  int steps = ((int)(input_buffer[ARM1_MSG_PARAM_MSB_INDEX]) << 8) | input_buffer[ARM1_MSG_PARAM_LSB_INDEX];
  // Here the steps should be applied to the actuator
}

void cmd_set_angle(){
  int actuator_no = input_buffer[ARM1_MSG_JOINT_INDEX];
  int angle = ((int)(input_buffer[ARM1_MSG_PARAM_MSB_INDEX]) << 8) | input_buffer[ARM1_MSG_PARAM_LSB_INDEX];

  if (actuator_no >=0 && actuator_no<JOINTS_COUNT){
    if(angle<j_angle_min) {
      angle = j_angle_min;
    }  
    if(angle > j_angle_max) {
      angle = j_angle_max;
    }
    j_angle_desired[actuator_no] = angle;
    j_goal_achieved[actuator_no] = 0;    
  }
  /*
  if(actuator_no == 0){
    if(angle < j0_angle_min) {
      angle = j0_angle_min;
    }
    if(angle > j0_angle_max){
      angle = j0_angle_max;
    }
    
    j0_angle_desired = angle;
    j0_goal_achieved = 0;
  }*/
}

void cmd_set_speed(){
  /*int actuator_no = input_buffer[ARM1_MSG_JOINT_INDEX];
  int speed = (input_buffer[ARM1_MSG_PARAM_MSB_INDEX] << 8) | input_buffer[ARM1_MSG_PARAM_LSB_INDEX];
  if(actuator_no == 0){
    j0.setSpeed(speed);
  }*/  
}

void info_current_adc(){
  for(int i=0;i<BUFFER_SIZE;i++) {
    output_buffer[i] = command_template[i];
  }
}

void info_current_angle(int actuator_no){
  for(int i=0;i<BUFFER_SIZE;i++) {
    output_buffer[i] = command_template[i];
  }
  short int angle =j_angle_current[actuator_no]; 

  output_buffer[ARM1_MSG_OP_INDEX] = ARM1_INFO_CURRENT_ANGLE;
  output_buffer[ARM1_MSG_JOINT_INDEX] = actuator_no;
  output_buffer[ARM1_MSG_PARAM_LSB_INDEX] = angle & 0xFF;
  output_buffer[ARM1_MSG_PARAM_MSB_INDEX] = angle >> 8;

  Serial.write(output_buffer, COMMAND_LENGTH);  
}

void info_current_speed(int actuator_no){
  for(int i=0;i<BUFFER_SIZE;i++) {
    output_buffer[i] = command_template[i];
  }
  short int speed = j_speed_current[actuator_no];

  output_buffer[ARM1_MSG_OP_INDEX] = ARM1_INFO_CURRENT_SPEED;
  output_buffer[ARM1_MSG_JOINT_INDEX] = actuator_no;
  output_buffer[ARM1_MSG_PARAM_LSB_INDEX] = speed & 0xFF;
  output_buffer[ARM1_MSG_PARAM_MSB_INDEX] = speed >> 8;

  Serial.write(output_buffer, COMMAND_LENGTH); 
}

void send_update_info(){
  unsigned long current_millis = millis();
  if (current_millis - last_millis > UPDATE_INTERVAL){
    info_current_angle(0);
    info_current_speed(0);
    last_millis = current_millis;
  }
}

void update_actuators_speed(){
  for (int i=0;i<JOINTS_COUNT;i++) {
    j_angle_current[i] = analogRead(J_ENCODER_INPUT[i]);
    if(j_angle_current[i] < j_angle_min/2 || j_angle_current[i] > (j_angle_max+1024)/2){
      // HALT THE ACTUATOR IF THE ANGLE IS OUTSIDE THE ALLOWED RANGE
      j_speed_current[i] = 0;   
      j_goal_achieved[i] = 16;  
    } else {
      if(abs(j_angle_desired[i]-j_angle_current[i])>15) {
        j_speed_current[i] = sgn(j_angle_desired[i] - j_angle_current[i])*600;
        j_goal_achieved[i] = 0;
      } else if (abs(j_angle_desired[i]-j_angle_current[i])>10){
        j_speed_current[i] = sgn(j_angle_desired[i] - j_angle_current[i]) * 200;      
        if(j_goal_achieved[i]>0){
           j_goal_achieved[i]--; 
        }  
      } else if (abs(j_angle_desired[i]-j_angle_current[i]>=1)) {
        j_speed_current[i] = sgn(j_angle_desired[i] - j_angle_current[i]) * 50;  
        if(j_goal_achieved[i]>0){
           j_goal_achieved[i]--; 
        } 
      } else {
        j_speed_current[i] = 0;
        if(j_goal_achieved[i]<32){
          j_goal_achieved[i]++;
        }        
      }
    }
    if (j_goal_achieved[i]>=16){
      joints[i]->disableOutputs();
    } else {
      joints[i]->enableOutputs();
    }
    joints[i]->setSpeed(j_speed_current[i]);    
  }  
/*
  // #### J0 ####
  j0_angle_current = analogRead(J0_ENCODER_INPUT);
  if (j0_angle_current < j0_angle_min/2 || j0_angle_current > (j0_angle_max+1024)/2){
    j0_speed_current = 0;
    // SEND ERROR NOTIFICATION HERE
  } else {
    if (abs(j0_angle_desired - j0_angle_current)>15){
      j0_speed_current = sgn(j0_angle_desired - j0_angle_current) * 400;
      j0_goal_achieved = 0;
    } else if (abs(j0_angle_desired - j0_angle_current) > 10) {
      j0_speed_current = sgn(j0_angle_desired - j0_angle_current) * 100;
    } else if(abs(j0_angle_desired - j0_angle_current) > 1) {
      j0_speed_current = sgn(j0_angle_desired - j0_angle_current) * 25;  
    } else {
      j0_speed_current = 0;
      if(j0_goal_achieved<32){
        j0_goal_achieved++;
      }
    }
  }
  if (j0_goal_achieved>=16) {
    j0.disableOutputs();
  } else {
    j0.enableOutputs();
  }
  
  j0.setSpeed(j0_speed_current);
  
  // #### J1 ####
  // <TODO>*/
}

void handle_command(){
  int cmd = input_buffer[ARM1_MSG_OP_INDEX];
  if(cmd == ARM1_CMD_SET_ANGLE) {
    cmd_set_angle();    
  }
}

void loop() {
  if(Serial.available()){
    char b = Serial.read();
    if(command_template[ptr]==b || command_template[ptr]==0){
      input_buffer[ptr] = b;
      ptr++; 
      if(ptr==COMMAND_LENGTH){
        handle_command();
        ptr = 0;
      }
    } else {
      ptr = 0;
    }
  } 
  update_actuators_speed();
  send_update_info();
  for (int i=0;i<JOINTS_COUNT;i++){
    joints[i]->runSpeed();

  }
  /*
  j0.runSpeed();
  */
}
