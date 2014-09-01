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
    unsigned int min;
    unsigned int max;
}
bbmc_motor_range_t;


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
    
    bbmc_output_motor_t   control;
    
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

int io_func_setup (void);

int io_func_config (bbmc_io_funcs_t*   func_ptrs, 
                    const char*        conf_mode, 
                    const char*        io_mode);


/** Encode Configuration & Managment Functions.
 *  
 */

int qei_data_init (bbmc_input_encoder_t volatile *data, unsigned int timer);

int qei_data_cpy (bbmc_input_encoder_t volatile *src,
                  bbmc_input_encoder_t volatile *dest);

int qei_position_set (bbmc_input_encoder_t volatile *data, unsigned int value);

int qei_switch_velocity (bbmc_input_encoder_t volatile *data, 
                         double switch_speed);

int qei_capture_config (bbmc_input_encoder_t volatile *data,
                        unsigned int unit_position,
                        unsigned int clk_prescaler);

int qei_motor (bbmc_input_encoder_t volatile *data, double max_motor_speed);

int qei_frequency_set (bbmc_input_encoder_t volatile *data, 
                       unsigned int frequency);

int qei_print (const char *format);


/** 
 *  Input Functions
 *
 */

int input_qei_dual (bbmc_input_encoder_t volatile *data);

int input_qei_cap  (bbmc_input_encoder_t volatile *data);

int input_qei_std  (bbmc_input_encoder_t volatile *data);


/** Primary Encoder Input Functions
 * 
 */

void input_encoder_1D (bbmc_input_encoder_t volatile *state);

void input_encoder_2D (bbmc_input_encoder_t volatile *state);



/** Output Configurations & Management Functions
 * 
 */

int pwm_enable  (unsigned int dev_id);

int pwm_disable (unsigned int dev_id);

int pwm_frequency_get (unsigned int dev_id, double *frequency);

int pwm_frequency_set (unsigned int dev_id, double frequency);

int pwm_print (const char *format);


/** Output Functions
 * 
 */

void output_pwm_dif  (bbmc_output_motor_t volatile *data);

void output_gpio_dir (unsigned int dev_id, unsigned int pin_value);

void output_pwm_dir  (bbmc_output_motor_t volatile *data);


/** Primary Encoder Input Functions
 * 
 */

void output_motor_1D (bbmc_contrl_motor_t volatile *data);

void output_motor_2D (bbmc_contrl_motor_t volatile *data);





#ifdef __cplusplus
}
#endif


#endif /* _MOTOR_CONTROL_H_ */
