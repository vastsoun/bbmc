#ifndef _BBMC_URETTS_CARRIAGE__H_
#define _BBMC_URETTS_CARRIAGE_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *  
 *  
 */


/* Numerical Constants. */
#define PI                              (3.14159265359)
#define PI_X2                           (6.283185307)

/* BBMC number of actuators to control. */
#define BBMC_DOF_NUM                    (2)

/* Motor driver parameters. */
#define VOLT_AMPLITUDE_PWM              (12)

/* TaskSpace parameters. */
#define POSITION_Y_POS_INIT             (1740000)
#define POSITION_Y_NEG_INIT             (1000)
#define POSITION_Y_POS_THRESH           (1650000)
#define POSITION_Y_NEG_THRESH           (10000)
#define POSITION_Y_POS_HOME             (1600000)
#define POSITION_Y_NEG_HOME             (15000)

#define POSITION_X_POS_INIT             (500000)
#define POSITION_X_NEG_INIT             (1000)
#define POSITION_X_POS_THRESH           (450000)
#define POSITION_X_NEG_THRESH           (50000)
#define POSITION_X_POS_HOME             (400000)
#define POSITION_X_NEG_HOME             (100000)

/* Quadrature decoding parameters. */
#define ENC_1_LIN_PER_ROT               (4000)
#define ENC_2_LIN_PER_ROT               (4000)

#define SPEED_MODE_THRESHOLD_Y          (10000)//!TODO
#define SPEED_MODE_THRESHOLD_X          (10000)//!TODO

/* Motor Drier type configuration. */
#define DRIVER_MODE_CURRENT
//#define DRIVER_MODE_VOLTAGE


/* Maximum Motor speed limitation - used in eQEP-capture algorithm
 * 
 *  Maximum Speed (counts/s) = (rpm*lines_per_rot)/60
 */
#define MAX_SPEED_MOTOR_1               (155334)
#define MAX_SPEED_MOTOR_2               (318667)


/* Planetary gear transmission parameters */
#define PGH_REDUCTION_1_NOM           1539
#define PGH_REDUCTION_1_DENOM         44
#define FLYWHEEL_RADIUS_1             5E-2

#define PGH_REDUCTION_2_NOM           1539
#define PGH_REDUCTION_2_DENOM         44
#define FLYWHEEL_RADIUS_2             3E-2



/** data type for the URETTS carriage's kinematic paramters
 *  
 */
typedef struct
{
    double beta_y;
    double beta_x;
}
csl_carriage_t;



/* algorithm data */
//TODO
static csl_carriage_t               g_carriage_kinematics;


/**
 * 
 */
int bbmc_kinematics_csl_carriage_setup (void);





#ifdef __cplusplus
}
#endif


#endif /* _BBMC_URETTS_CARRIAGE_H_ */
