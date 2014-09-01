#ifndef _BBMC_FACILITIES_H_
#define _BBMC_FACILITIES_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** Required Headers
 *  
 */
//TODO




/** BBMC general facility return value.
 * 
 */

#define RETURN_TEST                     (400)
#define RETURN_STATUS                   (500)
#define RETURN_PLANNER                  (600)
#define RETURN_DATALOG                  (700)
#define RETURN_PERF                     (800)
#define RETURN_CONFIG                   (900)
#define RETURN_DEBUG                    (1000)
#define RETURN_QUIT                     (1100)

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

#define RETURN_CONFIG_POSCALIB          (1)
#define RETURN_CONFIG_GSTATE            (2)
#define RETURN_CONFIG_QEI               (3)
#define RETURN_CONFIG_PWM               (4)
#define RETURN_CONFIG_TIMERS            (5)

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
#define RETURN_STATUS_SYSDIAG           (31459)



/** BBMC functionality macros.
 * 
 */

#define TEST_SPEED_LOG_SIZE             (16)
#define TEST_DSR_SPEED_REFRESH_COUNT    (1000)

#define POSCALIB_MAX_DUTY_Y             (20)//!
#define POSCALIB_SPEED_DEST_Y           (20000)//!
#define POSCALIB_SPEED_GAIN_Y           (0.01)//!

#define POSCALIB_MAX_DUTY_X             (20)//!
#define POSCALIB_SPEED_DEST_X           (20000)//!
#define POSCALIB_SPEED_GAIN_X           (0.01)//!


/** Data Types for configuring facility functionality
 * 
 */

typedef struct
{
    double        arg_double[4];
    int           arg_int[4];
    unsigned int  arg_uint[4];
}
cmd_args_t;



/** TEST
 *  
 */

int test (int argc, char *argv[]);

int test_args(int argc, char *argv[], cmd_args_t *args);

int test_pwm  (cmd_args_t *args);

int test_qei  (cmd_args_t *args);

int test_gpio (cmd_args_t *args);



/** STATUS
 *  
 */

int status (int argc, char *argv[]);



/** PLANNER
 *  
 */

int planner (int argc, char *argv[]);

int planner_args (int argc, char *argv[], cmd_args_t *args);



/** DATALOG
 *  
 */

int datalog (int argc, char *argv[]);

int datalog_args (int argc, char *argv[], cmd_args_t *args);



/** PERFLOG
 *  
 */

int perflog (int argc, char*argv[]);



/** CONFIG
 *  
 */

int config (int argc, char *argv[]);

int config_gstate_args (int argc, char *argv[], cmd_args_t *args);

int config_gstate (cmd_args_t *args);

int config_poscalib_args (int argc, char *argv[], cmd_args_t *args);

int config_poscalib (cmd_args_t *args);




/** DEBUG
 *  
 */

int debug (int argc, char *argv[]);



/** QUIT
 *  
 */

int quit (int argc, char *argv[]);




#ifdef __cplusplus
}
#endif


#endif /* _BBMC_FACILITIES_H_ */
