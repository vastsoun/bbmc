#ifndef _RMPI_H_
#define _RMPI_H_


#ifdef __cplusplus
    extern "C" {
#endif



/** 
 *  
 *  
 */

/* RMPI help message. */
#define HELP_RMPI                       "\r\n-Response Measurement Parameter Identification-\r\n"

/* RMPI return value. */
#define RETURN_RMPI                     (800)

/* RMPI functionality return values. */
#define RETURN_RMPI_BREAK               (10)
#define RETURN_RMPI_STEP                (20)
#define RETURN_RMPI_STEP2               (30)
#define RETURN_RMPI_SINE                (40)
#define RETURN_RMPI_SI                  (50)
#define RETURN_RMPI_PID_TUNE            (60)

/* Confguration parameters for the facilities services. */
#define RMPI_STEP2_MAX_DURATION          (7000)
#define RMPI_BREAKAWAY_STOP_SPEED        (1000)

#define RMPI_BREAKAWAY_STOP_POSITION_Y_P (1733000)
#define RMPI_BREAKAWAY_STOP_POSITION_X_P (450000)

#define RMPI_BREAKAWAY_STOP_POSITION_N   (1000)
#define RMPI_BREAKAWAY_STOP_POSITION_THR (500)//!

#define RMPI_BREAKAWAY_STEP_INCREM       (50)//!



/**
 * 
 * 
 */
int func_rmpi (unsigned int rmpi_dof);

/**
 * 
 * 
 */
void isr_rmpi (void);

/** 
 *  rmpi command functions
 *  
 */
int cmnd_rmpi_break(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_break_args(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_break_func(bbmc_cmd_args_t *args);

int cmnd_rmpi_step(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_step2(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_sine(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_si(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_si_args(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_si_func(bbmc_cmd_args_t *args);

int cmnd_rmpi_pid_tune(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_pid_tune_args(int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_rmpi_control_pid_config (unsigned int dof_id);

int cmnd_rmpi_pid_tune_func(bbmc_cmd_args_t *args);

int cmnd_rmpi (int argc, char *argv[]);


/* control functions */
static void contrl_rmpi_breakaway(bbmc_dof_state_t volatile *state, 
                                    bbmc_dof_contrl_t volatile *controller);

static void contrl_rmpi_step (bbmc_dof_state_t volatile *state, 
                                bbmc_dof_contrl_t volatile *controller);

static void contrl_rmpi_step2 (bbmc_dof_state_t volatile *state, 
                                 bbmc_dof_contrl_t volatile *controller);

static void contrl_rmpi_sine (bbmc_dof_state_t volatile *state, 
                                bbmc_dof_contrl_t volatile *controller);

static void contrl_rmpi_si (bbmc_dof_state_t volatile *state, 
                              bbmc_dof_contrl_t volatile *controller);

/* termination condition functions */
static void term_rmpi_breakaway(bbmc_dof_state_t volatile *state, 
                                  bbmc_dof_contrl_t volatile *controller);

static void term_rmpi_step (bbmc_dof_state_t volatile *state, 
                              bbmc_dof_contrl_t volatile *controller);
                                
static void term_rmpi_si (bbmc_dof_state_t volatile *state, 
                            bbmc_dof_contrl_t volatile *controller);


/* rmpi controller */
static inline void rmpi_timer_interrupt_on (void);

static inline void rmpi_timer_interrupt_off (void);

static inline void rmpi_controller (void);

static inline void rmpi_poslim_stop (void);

static inline void rmpi_datalogging (void);


/** 
 *  
 */
static void traject_rmpi_si (bbmc_dof_state_t volatile *state, 
                             bbmc_dof_contrl_t volatile *controller);




/** 
 *  
 *  
 */


#ifdef __cplusplus
}
#endif


#endif /* _RMPI_H_ */

