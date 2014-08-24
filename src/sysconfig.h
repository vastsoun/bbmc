#ifndef _BBMC_SYSTEM_CONFIG_H_
#define _BBMC_SYSTEM_CONFIG_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *
 */







/* performance timer functions */
void bbmc_perf_init (void);

void bbmc_perf_print (void);


/* poslim functions */
int bbmc_poslim_print (const char *format);


/* dof state functions */
int bbmc_dof_state_print (unsigned int dof_id, const char *format);


/* system hardware state functions */
int bbmc_pwm_print (const char *format);

int bbmc_qei_print (const char *format);

int bbmc_timers_print (const char *format);


/* runtime functions */

int bbmc_goto_home (void);
                                  
void  bbmc_contrl_input (void);



/**
 *  Master functions
 */
void bbmc_setup (void);


void system_halt (void);






/** 
 *  BBMC System initialization & configuration functions.
 */

/* startup system configurations */
int sysconfig_dof_setup (void);

int sysconfig_logs_setup (void);

int sysconfig_poslim_setup (void);

int sysconfig_contrl_stop_setup (void);

int sysconfig_contrl_stop_init (void);

int sysconfig_io_func_setup (bbmc_io_func_tbl_t *func_table);

int sysconfig_io_func (unsigned int dof_id,
                       bbmc_io_funcs_t *func_ptrs, 
                       bbmc_io_func_tbl_t *func_table, 
                       const char *conf_mode, 
                       const char *io_mode);


/* online configurations */
int sysconfig_dof_state_set (bbmc_dof_state_t volatile *data, unsigned int value);


int sysconfig_pwm_frequency_get (unsigned int dof_id, double *ret_frequency);

int sysconfig_pwm_enable (unsigned int dof_id);

int sysconfig_pwm_disable (unsigned int dof_id);

int sysconfig_qei_capture (unsigned int dof_id,
                           unsigned int unit_position,
                           unsigned int clk_prescaler);

int sysconfig_qei_motor (unsigned int dof_id, double max_motor_speed);

int sysconfig_qei_speed_switch (unsigned int dof_id, double switch_speed);

int sysconfig_qei_data_init (unsigned int timer, unsigned int dof_id);

int sysconfig_timer_frequency_set (unsigned int timer, double frequency);

int sysconfig_timer_frequency_get (unsigned int timer, int *frequency);

int sysconfig_poslim_enable (unsigned int axis);

int sysconfig_position_reset (unsigned int dof_id);

int sysconfig_position_set (unsigned int dof_id, unsigned int position);

int sysconfig_killswitch_enable (void);

int sysconfig_poslim_disable (unsigned int axis);

int sysconfig_killswitch_disable (void);

int sysconfig_timer_enable (unsigned int timer);

int sysconfig_timer_disable (unsigned int timer);

int sysconfig_gpio_killswitch_get (unsigned int *pin_value);

int sysconfig_gpio_poslim_get (unsigned int  dof_id, unsigned int *pin_value);

int sysconfig_setup (void);



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
