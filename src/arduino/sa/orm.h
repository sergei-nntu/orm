#ifndef ORM_H
#define ORM_H
#include <AccelStepper.h>

#include "osp.h"
#include "pins_RAMPS.h"

#define JOINTS_COUNT              4

#define STAT_SAMPLE_SIZE          1

// Update Interval in Milliseconds
#define UPDATE_INTERVAL  100 // 10 Hz

// Convenience sign function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

const long orm_j_encoder_adc_range = 1024; // Max Value of Encoder ADC output
const long orm_max_int_angle = 32768;      // Max Positive Value of Integer Angle correspnods to 2*Pi
const short orm_j_stepper_full_rot[JOINTS_COUNT] = {16000, 16000,16000, 9780};  // Number of steps to reach 2*Pi Angle

class ORM {
  private:
    // OSP COMMUNICATION VARIABLES
    char            osp_output_buffer[OSP_BUFFER_SIZE];
    unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
    int             osp_ptr=0;

    // STEPPER CONTROL VARIABLES
    AccelStepper*   joints[JOINTS_COUNT];
    short j_angle_desired[JOINTS_COUNT] =   {0, 0, 0, 0};
    short j_angle_current[JOINTS_COUNT] =   {0, 0, 0, 0};
    short j_angle_read[JOINTS_COUNT] =      {0, 0, 0, 0};
    short j_angle_correction[JOINTS_COUNT] ={0, 0, 0, 0};

    // STATISTICAL FILTERING 

    short j_angle_sample[JOINTS_COUNT * STAT_SAMPLE_SIZE];

    int read_samples_size[JOINTS_COUNT] = {0,0,0, 0};
    int read_samples_ptr[JOINTS_COUNT] = {0,0,0,0 };
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
