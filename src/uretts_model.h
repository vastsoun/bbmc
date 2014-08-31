#ifndef _BBMC_URETTS_CARRIAGE__H_
#define _BBMC_URETTS_CARRIAGE_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Parameter and Consant Definitions Regarding the URETTS Mechanical System
 *  
 *  
 */


#define PI                              (3.14159265359)
#define PI_X2                           (6.283185307)

#define VOLT_AMPLITUDE_PWM              (12)

#define POSITION_Y_MAX_INIT             (1740000)
#define POSITION_Y_MIN_INIT             (1000)
#define POSITION_Y_MAX_THRESH           (1650000)
#define POSITION_Y_MIN_THRESH           (10000)
#define POSITION_Y_MAX_HOME             (1600000)
#define POSITION_Y_MIN_HOME             (15000)

#define POSITION_X_MAX_INIT             (500000)
#define POSITION_X_MIN_INIT             (1000)
#define POSITION_X_MAX_THRESH           (450000)
#define POSITION_X_MIN_THRESH           (50000)
#define POSITION_X_MAX_HOME             (400000)
#define POSITION_X_MIN_HOME             (100000)

#define ENC_1_LIN_PER_ROT               (4000)
#define ENC_2_LIN_PER_ROT               (4000)

#define SPEED_MODE_THRESHOLD_Y          (10000)//!TODO
#define SPEED_MODE_THRESHOLD_X          (10000)//!TODO

#define DRIVER_MODE_CURRENT
//#define DRIVER_MODE_VOLTAGE


/** Maximum Motor speed limitation - used in eQEP-capture algorithm
 *  
 *  Maximum Speed (counts/s) = (rpm*lines_per_rot)/60
 */
#define MAX_SPEED_MOTOR_1               (155334)
#define MAX_SPEED_MOTOR_2               (318667)


#define PGH_REDUCTION_1_NOM           1539
#define PGH_REDUCTION_1_DENOM         44
#define FLYWHEEL_RADIUS_1             5E-2

#define PGH_REDUCTION_2_NOM           1539
#define PGH_REDUCTION_2_DENOM         44
#define FLYWHEEL_RADIUS_2             3E-2






#ifdef __cplusplus
}
#endif


#endif /* _BBMC_URETTS_CARRIAGE_H_ */
