/**
 * \file   bbmc-0.5.h
 *
 * \brief  the bbmc-0.5.x application header
 */
 

#ifndef _BBMC_0_5_H_
#define _BBMC_0_5_H_

#ifdef __cplusplus
extern "C" {
#endif



/* 
 * BBMC CONFIGURATION MACROS
 */
 
/* bbmc number of actuators to control */
#define BBMC_DOF_NUM                    (2)

/* Numerical Constants */
#define PI                              (3.14159265359)
#define PI_X2                           (6.283185307)

/* Motor driver parameters */
#define VOLT_AMPLITUDE_PWM              (12)

/* Controller configuration parameters */
#define MAX_DURATION_COUNT              (131072)
#define MAX_DEBUG_COUNT                 (2000)
#define MAX_STOP_COUNT                  (1000)
#define MAX_STOP_DELAY_COUNT            (0xFFFu)

#define GLOBAL_DATA_MAX                 (131072)

/* system data limit values and confs */
#define CONTROLLER_ITERATIONS_MAX       (GLOBAL_DATA_MAX)
#define DATALOG_STATIC_DATALEN          (GLOBAL_DATA_MAX)
#define DATALOG_STATIC_DATASIZE         (6)

#define MAX_DATA_INDEX                  (12)//!

/* TaskSpace parameters */

#define POSITION_Y_POS_INIT             (1740000)
#define POSITION_Y_NEG_INIT             (1000)
#define POSITION_Y_POS_THRESH           (1650000)
#define POSITION_Y_NEG_THRESH           (10000)
#define POSITION_Y_POS_HOME             (1600000)
#define POSITION_Y_NEG_HOME             (15000)

#define POSITION_X_POS_INIT             (500000)
#define POSITION_X_NEG_INIT             (1000)
#define POSITION_X_POS_THRESH           (450000)
#define POSITION_X_NEG_THRESH           (50000)
#define POSITION_X_POS_HOME             (400000)
#define POSITION_X_NEG_HOME             (100000)

/* Quadrature decoding parameters */
#define ENC_1_LIN_PER_ROT               (4000)
#define ENC_2_LIN_PER_ROT               (4000)

#define SPEED_MODE_THRESHOLD_Y          (10000)//!TODO
#define SPEED_MODE_THRESHOLD_X          (10000)//!TODO

/* Output type configuration */
#define OUTPUT_PWM_DIFF
//#define OUTPUT_PWM_DIR
//#define OUTPUT_SPI_DAC

/* Input type configuration */
#define INPUT_QEP_DUAL
//#define INPUT_QEP_STD
//#define INPUT_QEP_CAP
//#define EQEP_ISR

/* Motor Drier type configuration */
#define DRIVER_MODE_CURRENT
//#define DRIVER_MODE_VOLTAGE


/* Maximum Motor speed limitation - used in eQEP-capture algorithm
 *
 * Maximum Speed (counts/s) = (rpm*lines_per_rot)/60
 *
 */
#define MAX_SPEED_MOTOR_1               (155334)
#define MAX_SPEED_MOTOR_2               (318667)


/* 
 * BBMC HARDWARE MACROS
 */

#define DEVICE_INPUT_FUNC_NUM           (3)
#define DEVICE_OUTPUT_FUNC_NUM          (2)
#define BBMC_CONTROL_FUNC_NUM           (8)

#define TIMER_RUN                       2
#define TIMER_GOTO                      3
#define TIMER_RMPI                      4
#define TIMER_STOP                      5

#define DMTIMER_CONTROLLER              (SOC_DMTIMER_2_REGS)
#define DMTIMER_GOTO                    (SOC_DMTIMER_3_REGS)
#define DMTIMER_RMPI                    (SOC_DMTIMER_4_REGS)
#define DMTIMER_STOP                    (SOC_DMTIMER_5_REGS)

#define DIR_1_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_1_GPIO_PIN_ADDRESS          (GPIO_1_7)     
#define DIR_1_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_1_GPIO_PIN                  (7)
#define DIR_1_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define DIR_2_GPIO_ADDRESS              (SOC_GPIO_1_REGS)
#define DIR_2_GPIO_PIN_ADDRESS          (GPIO_1_3)     
#define DIR_2_GPIO_PIN_MODE             (CONTROL_CONF_MUXMODE(7))
#define DIR_2_GPIO_PIN                  (3)
#define DIR_2_GPIO_PIN_DIR              (GPIO_DIR_OUTPUT)

#define HALL_Y_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_Y_GPIO_PIN                 (6)
#define HALL_Y_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define HALL_X_GPIO_ADDRESS             (SOC_GPIO_1_REGS)
#define HALL_X_GPIO_PIN                 (2)
#define HALL_X_GPIO_INT_LINE            (GPIO_INT_LINE_1)

#define KILLSWITCH_GPIO_ADDRESS         (SOC_GPIO_1_REGS)
#define KILLSWITCH_GPIO_PIN             (14)
#define KILLSWITCH_GPIO_INT_LINE        (GPIO_INT_LINE_2)
#define KILLSWITCH_GPIO_DEBOUCE_TIME    (1000)

/* 
 * BBMC CLI COMMAND MACROS
 */
 
#define MAX_CMD_NAME_SIZE               (16)
#define MAX_NUM_COMMANDS                (13)
#define MAX_HELP_SIZE                   (256)

#define HELP_RUN                        "\r\n-Run-\r\n"
#define HELP_DATALOG                    "\r\n-Datalog Servie-\r\n"
#define HELP_PERF                       "\r\n-Performance Counter-\r\n"
#define HELP_RESET                      "\r\n-System Reset-\r\n"
#define HELP_CONFIG                     "\r\n-Configuration Command-\r\n "
#define HELP_PATH                       "\r\n-Path and Trejectory Generator-\r\n"
#define HELP_GOTO                       "\r\n-GoTo-\r\n"
#define HELP_RMPI                       "\r\n-Response Measurement Parameter Identification-\r\n"
#define HELP_TEST                       "\r\n-I/O Testing-\r\n"
#define HELP_DEBUG                      "\r\n-Debug-\r\n"
#define HELP_QUIT                       "\r\n-Quit-\r\n"
#define HELP_STATUS                     "\r\n-System Status-\r\n"

#define ISR_RETURN_CLEAN                (0)
#define ISR_RETURN_KILLSW               (-100)
#define ISR_RETURN_GPIO_LIM             (-200)
#define ISR_RETURN_ERROR                (-300)
#define ISR_RETURN_DEBUG                (-400)

#define RETURN_RUN                      (100)
#define RETURN_DATALOG                  (200)
#define RETURN_PERF                     (300)
#define RETURN_RESET                    (400)
#define RETURN_CONFIG                   (500)
#define RETURN_PATH                     (600)
#define RETURN_GOTO                     (700)
#define RETURN_RMPI                     (800)
#define RETURN_TEST                     (900)
#define RETURN_DEBUG                    (1000)
#define RETURN_STATUS                   (1100)
#define RETURN_QUIT                     (0)


#define RETURN_RMPI_BREAK               (10)
#define RETURN_RMPI_STEP                (20)
#define RETURN_RMPI_STEP2               (30)
#define RETURN_RMPI_SINE                (40)
#define RETURN_RMPI_SI                  (50)
#define RETURN_RMPI_PID_TUNE            (60)

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

#define RETURN_CONFIG_GAINS             1
#define RETURN_CONFIG_QEI               2
#define RETURN_CONFIG_PWM               3
#define RETURN_CONFIG_TIMERS            4

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

/* 
 * BBMC FUNCTIONALITY MACROS
 */

#define TEST_SPEED_LOG_SIZE             (16)
#define TEST_DSR_SPEED_REFRESH_COUNT    (1000)

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

#define RMPI_STEP2_MAX_DURATION          (7000)
#define RMPI_BREAKAWAY_STOP_SPEED        (1000)
#define RMPI_BREAKAWAY_STOP_POSITION_Y_P (1733000)
#define RMPI_BREAKAWAY_STOP_POSITION_X_P (450000)
#define RMPI_BREAKAWAY_STOP_POSITION_N   (1000)
#define RMPI_BREAKAWAY_STOP_POSITION_THR (500)//!
#define RMPI_BREAKAWAY_STEP_INCREM       (50)//!

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


#define CONTROLLER_ARG_DOUBLE_NUM       (8)
#define CONTROLLER_ARG_INT_NUM          (8)


/* 
 * BBMC UTILS MACROS
 */
 
#define RX_BUFF_SIZE                    (128)

/* 
 * BBMC TRAJECTORY MACROS
 */
 
#define TRAJECTORY_POINTS_NUM           (DATA_SIGLEN_MAX)

/* 
 * BBMC SIGNAL-GEN MACROS
 */

#define SIGNAL_PRBS_V1_SWITCH_NUM    590
#define SIGNAL_PRBS_V1_ARRAY_SIZE    1180
#define SIGNAL_PRBS_V1_SWITCH_INIT   {27,1,43,-1,66,1,82,-1,105,1,118,-1,145,1,157,-1,\
                                    165,1,187,-1,210,1,235,-1,248,1,271,-1,288,1,304,-1,\
                                    327,1,353,-1,361,1,382,-1,394,1,408,-1,420,1,444,-1,\
                                    470,1,490,-1,511,1,536,-1,556,1,574,-1,589,1,608,-1,\
                                    622,1,637,-1,651,1,667,-1,683,1,708,-1,719,1,733,-1,\
                                    751,1,767,-1,779,1,797,-1,822,1,829,-1,845,1,853,-1,\
                                    871,1,889,-1,911,1,922,-1,940,1,956,-1,980,1,1000,-1,\
                                    1014,1,1027,-1,1039,1,1065,-1,1078,1,1087,-1,1101,1,1123,-1,\
                                    1144,1,1164,-1,1171,1,1185,-1,1210,1,1223,-1,1241,1,1254,-1,\
                                    1263,1,1278,-1,1301,1,1327,-1,1351,1,1359,-1,1373,1,1384,-1,\
                                    1397,1,1423,-1,1441,1,1463,-1,1486,1,1496,-1,1521,1,1530,-1,\
                                    1551,1,1573,-1,1594,1,1603,-1,1618,1,1635,-1,1653,1,1676,-1,\
                                    1697,1,1719,-1,1737,1,1763,-1,1771,1,1788,-1,1800,1,1817,-1,\
                                    1837,1,1848,-1,1867,1,1880,-1,1904,1,1913,-1,1922,1,1948,-1,\
                                    1958,1,1967,-1,1977,1,1986,-1,2001,1,2022,-1,2045,1,2067,-1,\
                                    2088,1,2104,-1,2125,1,2151,-1,2168,1,2181,-1,2203,1,2214,-1,\
                                    2226,1,2242,-1,2261,1,2280,-1,2289,1,2298,-1,2310,1,2332,-1,\
                                    2347,1,2370,-1,2395,1,2410,-1,2418,1,2436,-1,2457,1,2470,-1,\
                                    2487,1,2498,-1,2523,1,2540,-1,2557,1,2568,-1,2577,1,2585,-1,\
                                    2608,1,2630,-1,2641,1,2665,-1,2685,1,2705,-1,2732,1,2739,-1,\
                                    2754,1,2771,-1,2789,1,2797,-1,2817,1,2825,-1,2843,1,2864,-1,\
                                    2880,1,2899,-1,2910,1,2930,-1,2944,1,2953,-1,2967,1,2979,-1,\
                                    2990,1,3009,-1,3027,1,3041,-1,3052,1,3076,-1,3100,1,3113,-1,\
                                    3132,1,3154,-1,3180,1,3205,-1,3219,1,3229,-1,3251,1,3260,-1,\
                                    3270,1,3284,-1,3308,1,3326,-1,3344,1,3370,-1,3388,1,3395,-1,\
                                    3417,1,3436,-1,3453,1,3465,-1,3476,1,3493,-1,3504,1,3512,-1,\
                                    3522,1,3533,-1,3544,1,3563,-1,3586,1,3609,-1,3632,1,3652,-1,\
                                    3676,1,3699,-1,3709,1,3736,-1,3744,1,3764,-1,3775,1,3787,-1,\
                                    3800,1,3811,-1,3827,1,3853,-1,3868,1,3886,-1,3905,1,3926,-1,\
                                    3940,1,3964,-1,3989,1,4011,-1,4023,1,4046,-1,4056,1,4072,-1,\
                                    4091,1,4112,-1,4126,1,4148,-1,4172,1,4180,-1,4189,1,4213,-1,\
                                    4228,1,4243,-1,4269,1,4291,-1,4317,1,4328,-1,4337,1,4351,-1,\
                                    4368,1,4386,-1,4412,1,4429,-1,4443,1,4470,-1,4482,1,4502,-1,\
                                    4528,1,4548,-1,4560,1,4578,-1,4584,1,4609,-1,4624,1,4636,-1,\
                                    4662,1,4678,-1,4690,1,4699,-1,4717,1,4727,-1,4735,1,4753,-1,\
                                    4765,1,4779,-1,4800,1,4824,-1,4838,1,4864,-1,4890,1,4900,-1,\
                                    4922,1,4941,-1,4966,1,4985,-1,5006,1,5027,-1,5042,1,5053,-1,\
                                    5061,1,5077,-1,5097,1,5109,-1,5126,1,5137,-1,5150,1,5170,-1,\
                                    5194,1,5218,-1,5229,1,5244,-1,5268,1,5280,-1,5306,1,5325,-1,\
                                    5335,1,5358,-1,5378,1,5395,-1,5407,1,5415,-1,5426,1,5440,-1,\
                                    5459,1,5486,-1,5496,1,5518,-1,5543,1,5559,-1,5568,1,5591,-1,\
                                    5608,1,5617,-1,5641,1,5660,-1,5669,1,5694,-1,5701,1,5708,-1,\
                                    5735,1,5755,-1,5769,1,5786,-1,5808,1,5816,-1,5837,1,5858,-1,\
                                    5873,1,5892,-1,5905,1,5915,-1,5930,1,5955,-1,5966,1,5980,-1,\
                                    5993,1,6002,-1,6019,1,6042,-1,6067,1,6088,-1,6102,1,6115,-1,\
                                    6135,1,6159,-1,6185,1,6193,-1,6209,1,6218,-1,6230,1,6255,-1,\
                                    6265,1,6274,-1,6292,1,6305,-1,6319,1,6342,-1,6365,1,6385,-1,\
                                    6400,1,6420,-1,6431,1,6450,-1,6470,1,6489,-1,6502,1,6516,-1,\
                                    6526,1,6549,-1,6565,1,6579,-1,6601,1,6613,-1,6636,1,6659,-1,\
                                    6683,1,6697,-1,6711,1,6735,-1,6751,1,6769,-1,6790,1,6816,-1,\
                                    6833,1,6853,-1,6871,1,6896,-1,6920,1,6930,-1,6940,1,6952,-1,\
                                    6973,1,6984,-1,7010,1,7031,-1,7041,1,7065,-1,7090,1,7116,-1,\
                                    7134,1,7152,-1,7162,1,7179,-1,7197,1,7207,-1,7223,1,7241,-1,\
                                    7251,1,7270,-1,7278,1,7291,-1,7308,1,7321,-1,7340,1,7354,-1,\
                                    7363,1,7388,-1,7408,1,7427,-1,7448,1,7469,-1,7493,1,7510,-1,\
                                    7519,1,7534,-1,7557,1,7567,-1,7580,1,7606,-1,7629,1,7640,-1,\
                                    7667,1,7675,-1,7690,1,7705,-1,7719,1,7728,-1,7743,1,7762,-1,\
                                    7789,1,7800,-1,7813,1,7825,-1,7838,1,7848,-1,7855,1,7871,-1,\
                                    7883,1,7907,-1,7924,1,7949,-1,7975,1,7993,-1,8002,1,8028,-1,\
                                    8046,1,8071,-1,8086,1,8104,-1,8113,1,8138,-1,8157,1,8166,-1,\
                                    8187,1,8201,-1,8210,1,8221,-1,8232,1,8243,-1,8264,1,8286,-1,\
                                    8304,1,8321,-1,8341,1,8367,-1,8386,1,8405,-1,8430,1,8448,-1,\
                                    8461,1,8487,-1,8503,1,8521,-1,8542,1,8563,-1,8573,1,8582,-1,\
                                    8594,1,8618,-1,8636,1,8652,-1,8667,1,8689,-1,8709,1,8729,-1,\
                                    8739,1,8754,-1,8771,1,8785,-1,8801,1,8815,-1,8837,1,8851,-1,\
                                    8872,1,8888,-1,8910,1,8935,-1,8952,1,8977,-1,8988,1,9012,-1,\
                                    9020,1,9036,-1,9043,1,9066,-1,9076,1,9086,-1,9097,1,9117,-1,\
                                    9128,1,9141,-1,9163,1,9180,-1,9204,1,9212,-1,9228,1,9246,-1,\
                                    9264,1,9287,-1,9296,1,9309,-1,9315,1,9341,-1,9363,1,9385,-1,\
                                    9394,1,9408,-1,9418,1,9434,-1,9457,1,9476,-1,9497,1,9516,-1,\
                                    9537,1,9561,-1,9581,1,9591,-1,9609,1,9619,-1,9644,1,9652,-1,\
                                    9665,1,9684,-1,9700,1,9723,-1,9740,1,9751,-1,9772,1,9782,-1,\
                                    9801,1,9821,-1,9832,1,9842,-1,9852,1,9862,-1,9873,1,9898,-1,\
                                    9905,1,9931,-1,9938,1,9964,-1,9977,1,9994,0}
                                    
#define SIGNAL_PRBS_V2_SWITCH_NUM   590
#define SIGNAL_PRBS_V2_ARRAY_SIZE   1180
#define SIGNAL_PRBS_V2_SWITCH_INIT  {27,1,43,-1,66,1,82,-1,105,1,118,-1,145,1,157,-1,\
                                    165,1,187,-1,210,1,235,-1,248,1,271,-1,288,1,304,-1,\
                                    327,1,353,-1,361,1,382,-1,394,1,408,-1,420,1,444,-1,\
                                    470,1,490,-1,511,1,536,-1,556,1,574,-1,589,1,608,-1,\
                                    622,1,637,-1,651,1,667,-1,683,1,708,-1,719,1,733,-1,\
                                    751,1,767,-1,779,1,797,-1,822,1,829,-1,845,1,853,-1,\
                                    871,1,889,-1,911,1,922,-1,940,1,956,-1,980,1,1000,-1,\
                                    1014,1,1027,-1,1039,1,1065,-1,1078,1,1087,-1,1101,1,1123,-1,\
                                    1144,1,1164,-1,1171,1,1185,-1,1210,1,1223,-1,1241,1,1254,-1,\
                                    1263,1,1278,-1,1301,1,1327,-1,1351,1,1359,-1,1373,1,1384,-1,\
                                    1397,1,1423,-1,1441,1,1463,-1,1486,1,1496,-1,1521,1,1530,-1,\
                                    1551,1,1573,-1,1594,1,1603,-1,1618,1,1635,-1,1653,1,1676,-1,\
                                    1697,1,1719,-1,1737,1,1763,-1,1771,1,1788,-1,1800,1,1817,-1,\
                                    1837,1,1848,-1,1867,1,1880,-1,1904,1,1913,-1,1922,1,1948,-1,\
                                    1958,1,1967,-1,1977,1,1986,-1,2001,1,2022,-1,2045,1,2067,-1,\
                                    2088,1,2104,-1,2125,1,2151,-1,2168,1,2181,-1,2203,1,2214,-1,\
                                    2226,1,2242,-1,2261,1,2280,-1,2289,1,2298,-1,2310,1,2332,-1,\
                                    2347,1,2370,-1,2395,1,2410,-1,2418,1,2436,-1,2457,1,2470,-1,\
                                    2487,1,2498,-1,2523,1,2540,-1,2557,1,2568,-1,2577,1,2585,-1,\
                                    2608,1,2630,-1,2641,1,2665,-1,2685,1,2705,-1,2732,1,2739,-1,\
                                    2754,1,2771,-1,2789,1,2797,-1,2817,1,2825,-1,2843,1,2864,-1,\
                                    2880,1,2899,-1,2910,1,2930,-1,2944,1,2953,-1,2967,1,2979,-1,\
                                    2990,1,3009,-1,3027,1,3041,-1,3052,1,3076,-1,3100,1,3113,-1,\
                                    3132,1,3154,-1,3180,1,3205,-1,3219,1,3229,-1,3251,1,3260,-1,\
                                    3270,1,3284,-1,3308,1,3326,-1,3344,1,3370,-1,3388,1,3395,-1,\
                                    3417,1,3436,-1,3453,1,3465,-1,3476,1,3493,-1,3504,1,3512,-1,\
                                    3522,1,3533,-1,3544,1,3563,-1,3586,1,3609,-1,3632,1,3652,-1,\
                                    3676,1,3699,-1,3709,1,3736,-1,3744,1,3764,-1,3775,1,3787,-1,\
                                    3800,1,3811,-1,3827,1,3853,-1,3868,1,3886,-1,3905,1,3926,-1,\
                                    3940,1,3964,-1,3989,1,4011,-1,4023,1,4046,-1,4056,1,4072,-1,\
                                    4091,1,4112,-1,4126,1,4148,-1,4172,1,4180,-1,4189,1,4213,-1,\
                                    4228,1,4243,-1,4269,1,4291,-1,4317,1,4328,-1,4337,1,4351,-1,\
                                    4368,1,4386,-1,4412,1,4429,-1,4443,1,4470,-1,4482,1,4502,-1,\
                                    4528,1,4548,-1,4560,1,4578,-1,4584,1,4609,-1,4624,1,4636,-1,\
                                    4662,1,4678,-1,4690,1,4699,-1,4717,1,4727,-1,4735,1,4753,-1,\
                                    4765,1,4779,-1,4800,1,4824,-1,4838,1,4864,-1,4890,1,4900,-1,\
                                    4922,1,4941,-1,4966,1,4985,-1,5006,1,5027,-1,5042,1,5053,-1,\
                                    5061,1,5077,-1,5097,1,5109,-1,5126,1,5137,-1,5150,1,5170,-1,\
                                    5194,1,5218,-1,5229,1,5244,-1,5268,1,5280,-1,5306,1,5325,-1,\
                                    5335,1,5358,-1,5378,1,5395,-1,5407,1,5415,-1,5426,1,5440,-1,\
                                    5459,1,5486,-1,5496,1,5518,-1,5543,1,5559,-1,5568,1,5591,-1,\
                                    5608,1,5617,-1,5641,1,5660,-1,5669,1,5694,-1,5701,1,5708,-1,\
                                    5735,1,5755,-1,5769,1,5786,-1,5808,1,5816,-1,5837,1,5858,-1,\
                                    5873,1,5892,-1,5905,1,5915,-1,5930,1,5955,-1,5966,1,5980,-1,\
                                    5993,1,6002,-1,6019,1,6042,-1,6067,1,6088,-1,6102,1,6115,-1,\
                                    6135,1,6159,-1,6185,1,6193,-1,6209,1,6218,-1,6230,1,6255,-1,\
                                    6265,1,6274,-1,6292,1,6305,-1,6319,1,6342,-1,6365,1,6385,-1,\
                                    6400,1,6420,-1,6431,1,6450,-1,6470,1,6489,-1,6502,1,6516,-1,\
                                    6526,1,6549,-1,6565,1,6579,-1,6601,1,6613,-1,6636,1,6659,-1,\
                                    6683,1,6697,-1,6711,1,6735,-1,6751,1,6769,-1,6790,1,6816,-1,\
                                    6833,1,6853,-1,6871,1,6896,-1,6920,1,6930,-1,6940,1,6952,-1,\
                                    6973,1,6984,-1,7010,1,7031,-1,7041,1,7065,-1,7090,1,7116,-1,\
                                    7134,1,7152,-1,7162,1,7179,-1,7197,1,7207,-1,7223,1,7241,-1,\
                                    7251,1,7270,-1,7278,1,7291,-1,7308,1,7321,-1,7340,1,7354,-1,\
                                    7363,1,7388,-1,7408,1,7427,-1,7448,1,7469,-1,7493,1,7510,-1,\
                                    7519,1,7534,-1,7557,1,7567,-1,7580,1,7606,-1,7629,1,7640,-1,\
                                    7667,1,7675,-1,7690,1,7705,-1,7719,1,7728,-1,7743,1,7762,-1,\
                                    7789,1,7800,-1,7813,1,7825,-1,7838,1,7848,-1,7855,1,7871,-1,\
                                    7883,1,7907,-1,7924,1,7949,-1,7975,1,7993,-1,8002,1,8028,-1,\
                                    8046,1,8071,-1,8086,1,8104,-1,8113,1,8138,-1,8157,1,8166,-1,\
                                    8187,1,8201,-1,8210,1,8221,-1,8232,1,8243,-1,8264,1,8286,-1,\
                                    8304,1,8321,-1,8341,1,8367,-1,8386,1,8405,-1,8430,1,8448,-1,\
                                    8461,1,8487,-1,8503,1,8521,-1,8542,1,8563,-1,8573,1,8582,-1,\
                                    8594,1,8618,-1,8636,1,8652,-1,8667,1,8689,-1,8709,1,8729,-1,\
                                    8739,1,8754,-1,8771,1,8785,-1,8801,1,8815,-1,8837,1,8851,-1,\
                                    8872,1,8888,-1,8910,1,8935,-1,8952,1,8977,-1,8988,1,9012,-1,\
                                    9020,1,9036,-1,9043,1,9066,-1,9076,1,9086,-1,9097,1,9117,-1,\
                                    9128,1,9141,-1,9163,1,9180,-1,9204,1,9212,-1,9228,1,9246,-1,\
                                    9264,1,9287,-1,9296,1,9309,-1,9315,1,9341,-1,9363,1,9385,-1,\
                                    9394,1,9408,-1,9418,1,9434,-1,9457,1,9476,-1,9497,1,9516,-1,\
                                    9537,1,9561,-1,9581,1,9591,-1,9609,1,9619,-1,9644,1,9652,-1,\
                                    9665,1,9684,-1,9700,1,9723,-1,9740,1,9751,-1,9772,1,9782,-1,\
                                    9801,1,9821,-1,9832,1,9842,-1,9852,1,9862,-1,9873,1,9898,-1,\
                                    9905,1,9931,-1,9938,1,9964,-1,9977,1,9994,0}
                                    
#define SIGNAL_PULSE_SWITCH_NUM       33

#define SIGNAL_PULSE_ARRAY_SIZE       66

#define SIGNAL_PULSE_SWITCH_INIT      {301,0,601,1,901,0,1201,1,1501,0,1800,1,2101,0,2401,1,\
                                      2701,0,3001,1,3301,0,3601,1,3901,0,4201,1,4501,0,4801,1,\
                                      5101,0,5401,1,5700,0,6001,1,6301,0,6601,1,6901,0,7201,1,\
                                      7501,0,7801,1,8101,0,8401,1,8701,0,9001,1,9301,0,9601,1,\
                                      9901,0}

#define SIGNAL_PULSE_TIME_ON          200

#define ATAN2_ZERO_ERROR              100

#define PGH_REDUCTION_1_NOM           1539
#define PGH_REDUCTION_1_DENOM         44

#define FLYWHEEL_RADIUS_1             5E-2

#define PGH_REDUCTION_2_NOM           1539
#define PGH_REDUCTION_2_DENOM         44

#define FLYWHEEL_RADIUS_2             3E-2

#define RUN_POSINIT_Y                 200000
#define RUN_POSINIT_X                 250000

/*
 * DATA DEFINITIONS
 *
 * Description: 
 *   
 */

typedef double data_t;

/* 
 * DATALOG STRUCTURES - STATIC 
 * 
 * NOTE: use datalog.h when bbmc-0.5 will be refactored (possibly bbmc-0.5)
 * 
 */
typedef struct
{
    data_t data[DATALOG_STATIC_DATASIZE];
}
dataset_s_t;

typedef struct
{
    dataset_s_t log[DATALOG_STATIC_DATALEN];
    
    int d_size;
    int l_size;
    
    int d_index;
    int l_index;
}
datalog_s_t;


/*
 * BBMC - DEVICE STRUCTS AND TYPES - (PORT) devconfig will define these
 */

/* hw data types */
typedef eqep_data_t             bbmc_input_qei_t;
typedef double                   bbmc_output_pwm_t;

/* hw device types */
typedef dmtimer_handle_t       bbmc_dev_timer_t;
typedef eqep_handle_t           bbmc_dev_encoder_t;
typedef ehrpwm_handle_t         bbmc_dev_pwm_t;
typedef gpio_device_pin_t      bbmc_dev_gpio_pin_t;


/* define the input & output structs */

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


/*
 * BBMC ARLGORITHM STRUCTS
 */
 
typedef struct 
{
    double x_0;
    double y_0;
    
    double x_f;
    double y_f;
    
    double time_a;
    double time_d;
    double time_ss;
    
    double time_s;
    double time_f;
    
    double speed_ss;
    
    double acc_a;
    double acc_d;
    
    double sampling_period;
    
    double volatile time_current;
}
sg_trapez_t;


typedef struct
{
    double sampling_period;
    
    double frequency;
    double phase;
    
    double volatile offset;
    double volatile amplitude;
}
sg_sine_t;


typedef struct
{
    sg_sine_t sine;
    
    double x_0;
    double y_0;
    
    double x_f;
    double y_f;
    
    double duration;
}
sg_sinusoid_t;


typedef struct
{
    sg_sine_t sine;
    
    double x_0;
    double y_0;
    
    double x_c;
    double y_c;
    
    double radius;
}
sg_circle_t;


typedef struct 
{
    double  q;
    double  q_dot;
    double  q_ddot;
}
jointspace_state_t;


typedef struct 
{
    double  x;
    double  x_dot;
    double  x_ddot;
}
taskspace_state_t;


typedef struct
{
    double beta_y;
    double beta_x;
}
csl_carriage_t;


/*
 * BBMC INFRASTRUCTURE - DATA STRUCTS AND TYPEDEFS
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
    unsigned int limval[2];
}
bbmc_dof_limits_t;


typedef struct 
{
    /* standard system flags */
    int cmdln;
    int debug;
    
    int perf;
    int datalog;
    
    int exec_checkpoint;
    int contrl_run;
    
    /* ISR relative flags */
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


/*
 * BBMC INFRASTRUCTURE - FUNCTION TYPE-DEFS
 */

/* this is the prototype/template for I/O function types */

typedef int     (*bbmc_input_fp_t) (bbmc_dof_state_t volatile *state);

typedef void    (*bbmc_output_fp_t) (bbmc_dof_output_t volatile *output);

/* system controller function types */

typedef void    (*bbmc_contrl_fp_t) (bbmc_dof_state_t volatile *state, 
                                         bbmc_dof_contrl_t volatile *contrl);


/*
 * BBMC INFRASTRUCTURE - FUNCTION TYPE-DEFS
 * 
 * Note: these are only used for the test
 * 
 */

typedef struct 
{
    bbmc_contrl_fp_t traject_func;
    bbmc_contrl_fp_t contrl_func;
    bbmc_contrl_fp_t term_func;
}
bbmc_contrl_funcs_t;


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



/* 
 * Experimental features 
 */
 
typedef struct
{
    unsigned int ticks;
    unsigned int flag;
}
systick_t;




#ifdef __cplusplus
}
#endif
#endif  /* _BBMC_0_5_H_ */
