#ifndef _MOTOR_CONTROL__H_
#define _MOTOR_CONTROL_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** Required Headers **/
#include "device_layer.h"



/** Macros
 *  
 */
#define BBMC_CONTROL_FUNC_NUM           (8)
#define CONTROLLER_ARG_DOUBLE_NUM       (8)
#define CONTROLLER_ARG_INT_NUM          (8)


/** Define the front-end for the output to the motor. 
 * 
 */
typedef dev_output_pwm_t   bbmc_output_motor_t;


/** Define the front-end to the motor encoder.
 * 
 */
typedef dev_input_qei_t    bbmc_input_encoder_t;


/** Define the type which holds the limiting values for the joint controllerd
 *  by the motor
 *  
 */
typedef struct 
{
    unsigned int limval[2];
}
bbmc_limits_motor_t;


/** Define the data type for the jointspace variables.
 * 
 */
typedef struct 
{
    double  q;
    double  q_dot;
    double  q_ddot;
}
joint_state_t;



/** This type is the the type which packages all the necessary data for motor
 *  control, exluding the input from the encoders.
 * 
 *  arg_<type> define general purpose data containers for use when implementing
 *             the motor controller.
 * 
 */
typedef struct 
{
    joint_state_t         state_desired;
    
    double volatile       arg_double[CONTROLLER_ARG_DOUBLE_NUM];
    
    bbmc_output_motor_t   output;
    
    int                   arg_int[CONTROLLER_ARG_INT_NUM];
    
    
}
bbmc_contrl_motor_t;



/** Prototypes for I/O & Controller function types
 *  
 */
typedef int     (*bbmc_input_fp_t)  (bbmc_input_encoder_t volatile *state);

typedef void    (*bbmc_output_fp_t) (bbmc_output_motor_t volatile *output);

typedef void    (*bbmc_contrl_fp_t) (bbmc_input_encoder_t volatile *state, 
                                     bbmc_contrl_motor_t  volatile *contrl);


/** 
 *  
 */
typedef struct 
{
    bbmc_input_fp_t   input_func;
    bbmc_output_fp_t  output_func;
}
bbmc_io_funcs_t;


typedef struct 
{
    bbmc_contrl_fp_t traject_func;
    bbmc_contrl_fp_t contrl_func;
    bbmc_contrl_fp_t term_func;
}
bbmc_controller_t;




/** Configuration and Set-up functions.
 *  
 */

int 
io_func_setup (void);

int 
io_func_config (bbmc_io_funcs_t*   func_ptrs, 
                const char*        conf_mode, 
                const char*        io_mode);


/** Functions to handle module internal data.
 *  
 */
 
int 
qei_state_set (bbmc_input_encoder_t volatile *data, unsigned int value);



/** 
 *  Motor Input & Output functions
 *
 */

int input_qei_dual (bbmc_input_encoder_t volatile *data);

int input_qei_cap  (bbmc_input_encoder_t volatile *data);

int input_qei_std  (bbmc_input_encoder_t volatile *data);


void output_pwm_dif  (bbmc_output_motor_t volatile *data);

void output_gpio_dir (unsigned int dev_id, unsigned int pin_value);

void output_pwm_dir  (bbmc_output_motor_t volatile *data);






#ifdef __cplusplus
}
#endif


#endif /* _MOTOR_CONTROL_H_ */
