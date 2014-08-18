#ifndef _BBMC_SYSTEM_CONFIG_H_
#define _BBMC_SYSTEM_CONFIG_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *  
 *  
 */



/** BBMC INFRASTRUCTURE - DATA STRUCTS AND TYPEDEFS
 *  
 */

typedef struct
{
    double        arg_double[4];
    int            arg_int[4];
    unsigned int  arg_uint[4];
}
bbmc_cmd_args_t;


typedef struct 
{
    /** standard system flags **/
    int cmdln;
    int debug;
    
    int perf;
    int datalog;
    
    int exec_checkpoint;
    int contrl_run;
    
    /** ISR relative flags **/
    int volatile isr_return;
    int volatile stop_immediate;
    int volatile gpos_reset[BBMC_DOF_NUM];
}
bbmc_system_flags_t;


typedef struct
{
    volatile unsigned int termination_flag;
    volatile unsigned int iteration_counter;
    volatile unsigned int termination_counter;
}
bbmc_cisr_t;


/**
 * Experimental features 
 */
typedef struct
{
    unsigned int ticks;
    unsigned int flag;
}
systick_t;


/** 
 *  
 *  
 */




/** 
 *  
 *  
 */



/** 
 *  
 *  
 */




#ifdef __cplusplus
}
#endif


#endif /* _BBMC_SYSTEM_CONFIG_H_ */
