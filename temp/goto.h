#ifndef _GOTO__H_
#define _GOTO_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** 
 *  
 *  
 */



/** 
 *  
 *  
 */

/* Facility parameters */
#define HELP_GOTO                       "\r\n-GoTo-\r\n\nThis facility executes positioning maneuver."
#define RETURN_GOTO                     (700)

/* for voltage controlled motor drives */
#define GOTO_VC_FAST_MODE_1             (40)//!
#define GOTO_VC_FAST_MODE_2             (40)//!

#define GOTO_VC_SLOW_MODE_1             (20)//!
#define GOTO_VC_SLOW_MODE_2             (20)//!

/* for current controlled motor drives */
#define GOTO_CC_FAST_MODE_1             (80000)//!
#define GOTO_CC_FAST_MODE_2             (80000)//!

#define GOTO_CC_SLOW_MODE_1             (40000)//!
#define GOTO_CC_SLOW_MODE_2             (40000)//!

#define GOTO_CC_MODE_SWITCH_DISTANCE_1  (10000)//!
#define GOTO_CC_MODE_SWITCH_DISTANCE_2  (10000)//!

/* default goto controller gains */
#define GOTO_SPEED_GAIN_1               (0.01)//!
#define GOTO_SPEED_GAIN_2               (0.01)//!
#define GOTO_1_P_GAIN                   (0.05)
#define GOTO_2_P_GAIN                   (0.05)

/* number of total controller iterations to execute in DEBUG mode */
#define GOTO_DEBUG_STOP_COUNT           (2000)//!

/* number of controller iterations to verify termination conditions (msec)  */
#define GOTO_REST_COUNT                 (100)

/* total error margin for task termination (in encoder counts) */
#define GOTO_STOP_ERROR_1               (500)//!
#define GOTO_STOP_ERROR_2               (500)//!



/**
 * 
 * 
 */
int func_goto (void);


/**
 * 
 * 
 */
void isr_goto (void);


/** 
 *  
 *  
 */
void contrl_goto_v3 (bbmc_dof_state_t volatile *state, 
                     bbmc_dof_contrl_t volatile *controller);

/** 
 *  
 *  
 */
void goto_timer_interrupt_on (void);

void goto_timer_interrupt_off (void);

void goto_controller (void);

void goto_stop (void);

void goto_debug (void);

void goto_output (void);

void goto_datalogging (void);


/** 
 *  
 *  
 */
void traject_goto_v0 (bbmc_dof_state_t volatile *state, 
                      bbmc_dof_contrl_t volatile *controller);




/** 
 *  
 *  
 */



#ifdef __cplusplus
}
#endif


#endif /* _GOTO_H_ */

