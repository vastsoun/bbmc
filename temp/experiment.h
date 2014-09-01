#ifndef _EXPERIMENT__H_
#define _EXPERIMENT_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *  
 *  
 */
#define HELP_RUN                        "\r\n-Run-\r\n"
#define RETURN_RUN                      (100)


/** 
 *  
 *  
 */
#define RUN_POSINIT_Y                   (200000)
#define RUN_POSINIT_X                   (250000)


/**
 * 
 */
int func_run (void);


/**
 * 
 */
void isr_run (void);


/** 
 *  Experiment command functions
 *  
 */
int cmnd_run_position_init (unsigned int pos_y, unsigned int pos_x);

int cmnd_run_trapezoid_default (void);

int cmnd_run_sinusoid_default (void);

int cmnd_run_circle_default (void);

int cmnd_run_control_pid_default (void);

int cmnd_run_config_args (int argc, char *argv[]);

int cmnd_run_control_pid_config (void);

int cmnd_run_trapezoid_config (void);

int cmnd_run_sinusoid_config (void);

int cmnd_run_circle_config (void);

int cmnd_run_sinusoid_config (void);

int cmnd_run (int argc, char *argv[]);



/** 
 *  
 *  
 */
static inline void run_traject (bbmc_dof_state_t volatile *state, 
                                  bbmc_dof_contrl_t volatile *controller);

static inline void run_contrl (bbmc_dof_state_t volatile *state, 
                                 bbmc_dof_contrl_t volatile *controller);

static inline void run_term (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);

static inline void run_output (void);

static inline void run_controller (void);

static inline void run_datalogging (void);



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


#endif /* _EXPERIMENT_H_ */
