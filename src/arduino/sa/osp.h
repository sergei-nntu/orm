#ifndef OSP_H
#define OSP_H

#define OSP_MSG_DEV_INDEX     2
#define OSP_MSG_CMD_INDEX     3
#define OSP_BYTE_PARAM_INDEX      4
#define OSP_INT_PARAM_MSB_INDEX   5
#define OSP_INT_PARAM_LSB_INDEX   4
#define OSP_ORM_ANGLE_MSB_INDEX   6
#define OSP_ORM_ANGLE_LSB_INDEX   5 
#define OSP_ORM_SPEED_MSB_INDEX   6
#define OSP_ORM_SPEED_LSB_INDEX   5 
#define OSP_ORM_STATUS_MSB_INDEX  6
#define OSP_ORM_STATUS_LSB_INDEX  5 

#define OSP_DEV_GENERIC       0 
#define OSP_DEV_ORM           1 // Open Robotic Manipulator
#define OSP_DEV_OBP           2 // Open Battery Pack 
#define OSP_DEV_O2D           3 // Open Differential Drive

#define OSP_CMD_REQ_DEV_TYPE  1

#define OSP_OBP_CMD_REQ_SOC       0x01
#define OSP_OBP_CMD_REQ_VOLTAGE   0x02
#define OSP_OBP_CMD_REQ_CURRENT   0x03
#define OSP_OBP_CMD_REQ_STATUS    0x04

#define OSP_INFO_DEV_TYPE         0x11
#define OSP_OBP_INFO_SOC          0x11
#define OSP_OBP_INFO_VOLTAGE      0x12
#define OSP_OBP_INFO_CURRENT      0x13
#define OSP_OBP_INFO_STATUS       0x14

#define OSP_ORM_CMD_SET_ANGLE     0x02
#define OSP_ORM_CMD_SET_SPEED     0x03
#define OSP_ORM_INFO_ANGLE        0x12
#define OSP_ORM_INFO_SPEED        0x13
#define OSP_ORM_INFO_JOINT_STATE  0x14


#define OSP_ORM_JOINT_STATUS_RUNNING_BIT_INDEX  0
#define OSP_ORM_JOINT_STATUS_POWERED_BIT_INDEX  1
#define OSP_ORM_JOINT_STATUS_ERROR_BIT_INDEX    2

#define OSP_DEV_CURRENT           OSP_DEV_ORM

#define OSP_COMMAND_LENGTH        10
#define OSP_BUFFER_SIZE           10

#endif /*OSP_H*/