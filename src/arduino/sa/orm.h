#ifndef ORM_H
#define ORM_H
#include <AccelStepper.h>
#include <Servo.h> 

#include "osp.h"
//#include "pins_RAMPS.h"

#define JOINTS_COUNT              6
#define GRIPPER_JOINT_NO          JOINTS_COUNT

#define STAT_SAMPLE_SIZE          20

#define JS_ANGLE_SCALE_FACTOR_DIVISOR 1024

#define ORM_SPEED_MAX             4000  // 360 * 1000 / 16000 = 22.5 degrees per second 

#define ORM_SPEED_UPDATE_INTERVAL_MS 25   // ms

#define ORM_ACCELERATION_MAX      2000 // 360 * 1000 / 16000 = 22.5 degrees per second^2

#define ORM_DEACCELERATE_ANGLE      1000  

#define ORM_MS_IN_SECOND            1000

// Update Interval in Milliseconds
#define UPDATE_INTERVAL  100 // 10 Hz

// Convenience sign function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

const long orm_j_encoder_adc_range = 1024; // Max Value of Encoder ADC output
const long orm_max_int_angle = 32768;      // Max Positive Value of Integer Angle correspnods to 2*Pi
                                                 // ORA-3. ORA-4. ORA-3. ORA-2 ORA-2 ORA-2
const short orm_j_stepper_full_rot[JOINTS_COUNT] = {16000, 16000 ,16000, 9780, 9780, 9780};  // Number of steps to reach 2*Pi Angle

const short orm_j_speed_max[JOINTS_COUNT] =     {4000, 4000, 4000, 8000, 8000, 8000};
const short orm_j_speed_min[JOINTS_COUNT] =     {400, 400, 400, 400, 400, 400};
const short orm_j_acceleration[JOINTS_COUNT] =  {4000, 4000, 4000, 8000, 8000, 8000};


class ORM {
  private:
    // OSP COMMUNICATION VARIABLES
    char            osp_output_buffer[OSP_BUFFER_SIZE];
    unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
    int             osp_ptr=0;

    // SERVO CONTROL VARIABLES
    Servo * joint_servos[JOINTS_COUNT];

    // STEPPER CONTROL VARIABLES
    AccelStepper*   joints[JOINTS_COUNT];

    unsigned long speed_millis = 0;

    short j_speed_desired[JOINTS_COUNT] =   {500, 500, 500, 500, 500, 500}; // Default Speed 11.25 degrees second. Must be populated in the constructor
    short j_speed_current[JOINTS_COUNT] =   {0, 0, 0, 0, 0, 0};
    short j_angle_desired[JOINTS_COUNT] =   {0, 0, 0, 0, 0, 0};
    short j_angle_current[JOINTS_COUNT] =   {0, 0, 0, 0, 0, 0};
    short j_angle_read[JOINTS_COUNT] =      {0, 0, 0, 0, 0, 0};
    short j_angle_correction[JOINTS_COUNT] ={0, 0, 0, 0, 0, 0};
    short j_callibr_left[JOINTS_COUNT] = {2, 2, 2, 2, 2, 2};
    short js_angle_scale_factor[JOINTS_COUNT] = {1024,1024,682,682,1024,682}; // Scale factor to be applied prior to sending the angle to servos

    // GRIPPER CONTROL VARIABLES
    short gripper_angle = 0; // INT ANGLE -16384 .. 16383
    Servo * gripper_servo;

    // STATISTICAL FILTERING 

    short j_angle_sample[JOINTS_COUNT * STAT_SAMPLE_SIZE];

    int read_samples_size[JOINTS_COUNT] = {0,0,0,0,0,0};
    int read_samples_ptr[JOINTS_COUNT] = {0,0,0,0,0,0};
    /*
    short j_speed_current[JOINTS_COUNT] = {0,};
    short j_goal_achieved[JOINTS_COUNT] = {16,};  
    */

    // Flow Control Functions
    void            ospHandleCommand();
    void            sendUpdateInfo();   
    void            updateActuatorsPosition(); 

    // Incoming Command Processing Functions
    void ospHandleORMCommand();
    void ospHandleGenericCommand();

    void cmdSetAngle();
    void cmdSetSpeed();
    void cmdMakeSteps();
    void cmdSetCorrAngle();

    // Outcoming Commands Generation Functinos
    void ospPrepareOutputBuffer();
    void ormInfoCurrentAngle(int actuatorNo);
    void ormInfoCurrentSpeed(int actuatorNo);
    void ormInfoJointStatus(int actuatorNo);
    void ormInfoGripperAngle();

    // Data Input Function
    short readAngle(int actuatorNo);

    unsigned long last_millis; 

  public:
    // Method to poll the Serial input. Should be called at least once per the execution loop
    ORM();
    void setup();
    void ospSerialLoop();
};

#endif /*ORM_H*/
