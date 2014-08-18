/**
 * \file   bbmc-0.6.h
 *
 * \brief  the bbmc-0.6.x application header
 */
 

#ifndef _BBMC_0_6_H_
#define _BBMC_0_6_H_

#ifdef __cplusplus
extern "C" {
#endif



/** Controller configuration parameters **/
#define MAX_DURATION_COUNT              (131072)
#define MAX_DEBUG_COUNT                 (2000)
#define MAX_STOP_COUNT                  (1000)
#define MAX_STOP_DELAY_COUNT            (0xFFFu)


/** System data limit values and confs. **/
#define GLOBAL_DATA_MAX                 (131072)
#define CONTROLLER_ITERATIONS_MAX       (GLOBAL_DATA_MAX)
#define MAX_DATA_INDEX                  (12)//!


/**
 * BBMC CLI COMMAND MACROS
 */
 
#define MAX_CMD_NAME_SIZE               (16)
#define MAX_NUM_COMMANDS                (13)
#define MAX_HELP_SIZE                   (256)


#define HELP_DATALOG                    "\r\n-Datalog Servie-\r\n"
#define HELP_PERF                       "\r\n-Performance Counter-\r\n"
#define HELP_RESET                      "\r\n-System Reset-\r\n"
#define HELP_CONFIG                     "\r\n-Configuration Command-\r\n "
#define HELP_PATH                       "\r\n-Path and Trejectory Generator-\r\n"
#define HELP_TEST                       "\r\n-I/O Testing-\r\n"
#define HELP_DEBUG                      "\r\n-Debug-\r\n"
#define HELP_QUIT                       "\r\n-Quit-\r\n"
#define HELP_STATUS                     "\r\n-System Status-\r\n"

#define ISR_RETURN_CLEAN                (0)
#define ISR_RETURN_KILLSW               (-100)
#define ISR_RETURN_GPIO_LIM             (-200)
#define ISR_RETURN_ERROR                (-300)
#define ISR_RETURN_DEBUG                (-400)

#define HELP_GOTO                       "\r\n-GoTo-\r\n"
#define RETURN_GOTO                     (700)
#define GOTO_VC_FAST_MODE_1             (40)//!
#define GOTO_VC_FAST_MODE_2             (40)//!
#define GOTO_VC_SLOW_MODE_1             (20)//!
#define GOTO_VC_SLOW_MODE_2             (20)//!
#define GOTO_CC_FAST_MODE_1             (80000)//!
#define GOTO_CC_FAST_MODE_2             (80000)//!
#define GOTO_CC_SLOW_MODE_1             (40000)//!
#define GOTO_CC_SLOW_MODE_2             (40000)//!
#define GOTO_SPEED_GAIN_1               (0.01)//!
#define GOTO_SPEED_GAIN_2               (0.01)//!
#define GOTO_DEBUG_STOP_COUNT           (2000)//!

/* (msec) home many control iterations to verify termination conditions */
#define GOTO_REST_COUNT                 (100)

#define GOTO_1_P_GAIN                   (0.05)
#define GOTO_2_P_GAIN                   (0.05)
#define GOTO_STOP_ERROR_1               (500)//!
#define GOTO_STOP_ERROR_2               (500)//!

#define GOTO_CC_MODE_SWITCH_DISTANCE_1  (10000)//!
#define GOTO_CC_MODE_SWITCH_DISTANCE_2  (10000)//!


#define HELP_RMPI                       "\r\n-Response Measurement Parameter Identification-\r\n"
#define RETURN_RMPI                     (800)
#define RETURN_RMPI_BREAK               (10)
#define RETURN_RMPI_STEP                (20)
#define RETURN_RMPI_STEP2               (30)
#define RETURN_RMPI_SINE                (40)
#define RETURN_RMPI_SI                  (50)
#define RETURN_RMPI_PID_TUNE            (60)

#define RMPI_STEP2_MAX_DURATION          (7000)
#define RMPI_BREAKAWAY_STOP_SPEED        (1000)
#define RMPI_BREAKAWAY_STOP_POSITION_Y_P (1733000)
#define RMPI_BREAKAWAY_STOP_POSITION_X_P (450000)
#define RMPI_BREAKAWAY_STOP_POSITION_N   (1000)
#define RMPI_BREAKAWAY_STOP_POSITION_THR (500)//!
#define RMPI_BREAKAWAY_STEP_INCREM       (50)//!


#define RETURN_DATALOG                  (200)
#define RETURN_PERF                     (300)
#define RETURN_RESET                    (400)
#define RETURN_CONFIG                   (500)
#define RETURN_PATH                     (600)
#define RETURN_TEST                     (900)
#define RETURN_DEBUG                    (1000)
#define RETURN_STATUS                   (1100)
#define RETURN_QUIT                     (0)




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


/**
 * BBMC FUNCTIONALITY MACROS
 */

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


#ifdef __cplusplus
}
#endif
#endif  /* _BBMC_0_6_H_ */
