#ifndef _IO_SUB_SYSTEM__H_
#define _IO_SUB_SYSTEM_H_


#ifdef __cplusplus
    extern "C" {
#endif


#include "control.h"



/** Output type configuration.
 * 
 */
#define OUTPUT_PWM_DIFF
//#define OUTPUT_PWM_DIR
//#define OUTPUT_SPI_DAC


/** Input type configuration.
 * 
 */
#define INPUT_QEP_DUAL
//#define INPUT_QEP_STD
//#define INPUT_QEP_CAP
//#define EQEP_ISR


/**
 * 
 */
#define BBMC_CONTROL_FUNC_NUM           (8)
#define CONTROLLER_ARG_DOUBLE_NUM       (8)
#define CONTROLLER_ARG_INT_NUM          (8)



/**
 * BBMC - DEVICE STRUCTS AND TYPES - (PORT) devconfig will define these
 */

/* hw data types */
typedef eqep_data_t             bbmc_input_qei_t;
typedef double                   bbmc_output_pwm_t;


/** Input & Output front-ends 
 * 
 */
typedef struct
{
    bbmc_output_pwm_t   value;    
    
    unsigned int         dof_id;
    
}
bbmc_dof_output_t;


typedef struct
{
    bbmc_input_qei_t   state;
    
    unsigned int        dof_id;
}
bbmc_dof_state_t;


typedef struct 
{
    unsigned int limval[2];
}
bbmc_dof_limits_t;


typedef struct 
{
    jointspace_state_t    state_desired;
    
    double volatile       arg_double[CONTROLLER_ARG_DOUBLE_NUM];
    
    bbmc_dof_output_t     output;
    
    int                     arg_int[CONTROLLER_ARG_INT_NUM];
    
    //! this may be redundant. dof_id already in output struct.
    unsigned int           dof_id;
}
bbmc_dof_contrl_t;



/** Prototypes for I/O function types
 *  
 */
typedef int     (*bbmc_input_fp_t) (bbmc_dof_state_t volatile *state);

typedef void    (*bbmc_output_fp_t) (bbmc_dof_output_t volatile *output);

typedef void    (*bbmc_contrl_fp_t) (bbmc_dof_state_t volatile *state, 
                                         bbmc_dof_contrl_t volatile *contrl);


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
    bbmc_input_fp_t         input_funcs[DEVICE_INPUT_FUNC_NUM];
    bbmc_output_fp_t        output_funcs[DEVICE_OUTPUT_FUNC_NUM];
}
bbmc_io_func_tbl_t;


typedef struct 
{
    bbmc_contrl_fp_t traject_func;
    bbmc_contrl_fp_t contrl_func;
    bbmc_contrl_fp_t term_func;
}
bbmc_contrl_funcs_t;



#ifdef __cplusplus
}
#endif


#endif /* _IO_SUB_SYSTEM_H_ */
