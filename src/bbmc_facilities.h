#ifndef _BBMC_FACILITIES_H_
#define _BBMC_FACILITIES_H_


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
 
/* Controller configuration parameters */
#define MAX_DURATION_COUNT              (131072)
#define MAX_DEBUG_COUNT                 (2000)
#define MAX_STOP_COUNT                  (1000)
#define MAX_STOP_DELAY_COUNT            (0xFFFu)


/* System data limit values and confs. */
#define GLOBAL_DATA_MAX                 (131072)
#define CONTROLLER_ITERATIONS_MAX       (GLOBAL_DATA_MAX)
#define MAX_DATA_INDEX                  (12)//!

/*  */
#define MAX_CMD_NAME_SIZE               (16)
#define MAX_NUM_COMMANDS                (13)
#define MAX_HELP_SIZE                   (256)

/*  */
#define HELP_DATALOG                    "\r\n-Datalog Servie-\r\n"
#define HELP_PERF                       "\r\n-Performance Counter-\r\n"
#define HELP_RESET                      "\r\n-System Reset-\r\n"
#define HELP_CONFIG                     "\r\n-Configuration Command-\r\n "
#define HELP_PATH                       "\r\n-Path and Trejectory Generator-\r\n"
#define HELP_TEST                       "\r\n-I/O Testing-\r\n"
#define HELP_DEBUG                      "\r\n-Debug-\r\n"
#define HELP_QUIT                       "\r\n-Quit-\r\n"
#define HELP_STATUS                     "\r\n-System Status-\r\n"

/*  */
#define ISR_RETURN_CLEAN                (0)
#define ISR_RETURN_KILLSW               (-100)
#define ISR_RETURN_GPIO_LIM             (-200)
#define ISR_RETURN_ERROR                (-300)
#define ISR_RETURN_DEBUG                (-400)

/* Primary return values */
#define RETURN_DATALOG                  (200)
#define RETURN_PERF                     (300)
#define RETURN_RESET                    (400)
#define RETURN_CONFIG                   (500)
#define RETURN_PATH                     (600)
#define RETURN_TEST                     (900)
#define RETURN_DEBUG                    (1000)
#define RETURN_STATUS                   (1100)
#define RETURN_QUIT                     (0)

/* Return values for each BBMC facility. */
#define RETURN_TEST_PWM                 (10)
#define RETURN_TEST_QEI                 (20)
#define RETURN_TEST_GPIO_HALL           (30)
#define RETURN_TEST_GPIO_KILLSW         (40)

#define RETURN_DATALOG_RESET            (1)
#define RETURN_DATALOG_PRINT            (2)
#define RETURN_DATALOG_ENABLE           (3)
#define RETURN_DATALOG_DISABLE          (4)

#define RETURN_DEBUG_ENABLE             (1)
#define RETURN_DEBUG_DISABLE            (2)

#define RETURN_PERF_RESET               (1)
#define RETURN_PERF_PRINT               (2)
#define RETURN_PERF_ENABLE              (3)
#define RETURN_PERF_DISABLE             (4)

#define RETURN_RESET_POSCALIB           (1)
#define RETURN_RESET_SYSSTATE           (2)

#define RETURN_CONFIG_GAINS             (1)
#define RETURN_CONFIG_QEI               (2)
#define RETURN_CONFIG_PWM               (3)
#define RETURN_CONFIG_TIMERS            (4)

#define RETURN_ERROR_UNKNOWN            (-10)
#define RETURN_ERROR_INVALID_ARG        (-11)
#define RETURN_ERROR_INVALID_SUBARG     (-12)
#define RETURN_ERROR_INVALID_OPT        (-13)
#define RETURN_ERROR_INVALID_OPT_VAL    (-14)
#define RETURN_ERROR_FEW_ARGS           (-15)
#define RETURN_ERROR_MANY_ARGS          (-16)
#define RETURN_ERROR_RUN_ABORT          (-17)
#define RETURN_ERROR_GPIO_LIM           (-18)
#define RETURN_ERROR_GPIO_KILL          (-19)

#define RETURN_SUCCESS                  (1729)
#define RETURN_STATUS_SYSDIAG           (10)//!

/* BBMC functionality macros. */
#define TEST_SPEED_LOG_SIZE             (16)
#define TEST_DSR_SPEED_REFRESH_COUNT    (1000)

#define POSCALIB_MAX_DUTY_Y             (20)//!
#define POSCALIB_SPEED_DEST_Y           (20000)//!
#define POSCALIB_SPEED_GAIN_Y           (0.01)//!

#define POSCALIB_MAX_DUTY_X             (20)//!
#define POSCALIB_SPEED_DEST_X           (20000)//!
#define POSCALIB_SPEED_GAIN_X           (0.01)//!

#define STOP_SPEED_GAIN_P_1               (0.01)
#define STOP_SPEED_GAIN_P_2               (0.01)
#define STOP_SPEED_GAIN_I_1               (0.01)
#define STOP_SPEED_GAIN_I_2               (0.01)

#define STOP_SPEED_Y                      (500)
#define STOP_SPEED_X                      (500)


/** 
 *  
 *  
 */




/** 
 *  
 *  
 */

/* datalog command functions */
int cmnd_datalog_args (int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_datalog (int argc, char *argv[]);


/* perf command functions */
int cmnd_perf (int argc, char*argv[]);


/* reset command functions */
int cmnd_reset_poscalib_func (int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_reset_poscalib_args (int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_reset (int argc, char *argv[]);


/* config command functions */
int cmnd_config (int argc, char *argv[]);


/* motion planner command functions */
int cmnd_path (int argc, char *argv[]);


/* goto command functions */
int cmnd_goto_args (int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_goto_return_val (int argc, char *argv[], unsigned int goto_ret);

int cmnd_goto (int argc, char *argv[]);



/* test command functions */
int cmnd_test_pwm_func (int argc, char *argv[], unsigned int dof_id);

int cmnd_test_qei_args(int argc, char *argv[], unsigned int dof_id);

int cmnd_test_qei_func(unsigned int dof_id);

int cmnd_test_gpio_func (int argc, char *argv[], unsigned int dof_id);

int cmnd_test (int argc, char *argv[]);


/* debug command functions */
int cmnd_debug (int argc, char *argv[]);


/* status command functions */
int cmnd_status (int argc, char *argv[]);


/* quit command functions */
int cmnd_quit (int argc, char *argv[]);



/**
 * 
 */

/* control functions */
void contrl_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                     bbmc_dof_contrl_t volatile *controller);

void contrl_test_dsr (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);

void contrl_stop_immediate (bbmc_dof_state_t volatile *state, 
                                            bbmc_dof_contrl_t volatile *controller);

/* termination functions */
void term_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                   bbmc_dof_contrl_t volatile *controller);

void term_test_dsr (bbmc_dof_state_t volatile *state, 
                             bbmc_dof_contrl_t volatile *controller);


/* trajectory generation functions */
void traject_null (bbmc_dof_state_t volatile *state, 
                   bbmc_dof_contrl_t volatile *controller);



/* safety controllers */
void isr_gpio_pos_limit (void);

void isr_gpio_killswitch (void);

void isr_systick (void);



#ifdef __cplusplus
}
#endif


#endif /* _BBMC_FACILITIES_H_ */

