#ifndef ORM_H
#define ORM_H
#include <AccelStepper.h>

#include "osp.h"
#include "pins_RAMPS.h"

#define JOINTS_COUNT              2

// Update Interval in Milliseconds
#define UPDATE_INTERVAL  66 // 15 Hz

// Convenience sign function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

class ORM {
  private:
    // OSP COMMUNICATION VARIABLES
    char            osp_output_buffer[OSP_BUFFER_SIZE];
    unsigned char   osp_input_buffer[OSP_BUFFER_SIZE];
    int             osp_ptr=0;

    // STEPPER CONTROL VARIABLES
    AccelStepper*   joints[JOINTS_COUNT];
    short j_angle_desired[JOINTS_COUNT] = {8000,8000};
    short j_angle_current[JOINTS_COUNT] = {8000,8000};
    short j_angle_read[JOINTS_COUNT] = {8000,8000};
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

    // Outcoming Commands Generation Functinos
    void ospPrepareOutputBuffer();
    void ormInfoCurrentAngle(int actuatorNo);
    void ormInfoCurrentSpeed(int actuatorNo);
    void ormInfoJointStatus(int actuatorNo);

    unsigned long last_millis; 

  public:
    // Method to poll the Serial input. Should be called at least once per the execution loop
    ORM();
    void setup();
    void ospSerialLoop();
};

#endif /*ORM_H*/