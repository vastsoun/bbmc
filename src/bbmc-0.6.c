/**
 *  \file   bbmc-0.5.c
 *
 *  \brief  BBMC (Beagle Bone Motor Control) firmware package for Vassilis Tsounis' 
 *           diploma thesis at CSL-EP NTUA.
 *
 *   This is the primary source file that runs the bbmc infrastructure. TODO
 */


static char *g_bbmc_greeting = "\r\n"
"                                                                             \r\n"
"                                                                             \r\n"
"      ____                   _        ____                                   \r\n"
"     |  _ \\                 | |      |  _ \\                                \r\n"
"     | |_) | ___  __ _  __ _| | ___  | |_) | ___  _ __   ___                 \r\n"
"     |  _ < / _ \\/ _` |/ _` | |/ _ \\ |  _ < / _ \\| '_ \\ / _ \\           \r\n"
"     | |_) |  __/ (_| | (_| | |  __/ | |_) | (_) | | | |  __/                \r\n"
"     |____/_\\___|\\__,_|\\__, |_|\\___|_|____/ \\___/|_| |_|\\___|      _   \r\n"
"     |  \\/  |     | |   __/ |      / ____|          | |           | |       \r\n"
"     | \\  / | ___ | |_ |___/_ __  | |     ___  _ __ | |_ _ __ ___ | |       \r\n"
"     | |\\/| |/ _ \\| __/ _ \\| '__| | |    / _ \\| '_ \\| __| '__/ _ \\| |  \r\n"
"     | |  | | (_) | || (_) | |    | |___| (_) | | | | |_| | | (_) | |        \r\n"
"     |_|  |_|\\___/_\\__\\___/|_|     \\_____\\___/|_| |_|\\__|_|  \\___/|_| \r\n"
"            / _ \\                                                           \r\n"
"     __   _| | | |                                                           \r\n"
"     \\ \\ / / | |                  by Vassilis Tsounis                      \r\n"
"      \\ V /| |_| |                                                          \r\n"
"       \\_/  \\___/                  vastsoun@gmail.com                      \r\n"
"                                                                             \r\n"
"                                                                             \r\n";

/* standard C headers */
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <float.h>

/* Primary Hardware headers */
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "pin_mux.h"
#include "interrupt.h"
#include "cache.h"

/* Device headers */
#include "hw_ehrpwm.h"
#include "hw_eqep.h"

/* Drivers */
#include "uartStdio.h"
#include "dmtimer.h"
#include "pwmss.h"
#include "ehrpwm.h"
#include "eqep.h"
#include "gpio_v2.h"

/* Utilities */
#include "perf.h"
#include "cmdline.h"
#include "bbmc-0.5.h"



/*
 * BBMC Device initialization & configuration functions.
 */

int devconfig_stdio_setup(void);

int devconfig_mpucache_setup (unsigned int cache_mode);

int devconfig_interrupts_setup (void);

int devconfig_peripheral_setup (void);

int devconfig_gpio_setup (void);

int devconfig_timer_setup (void);

int devconfig_pwm_setup (void);

int devconfig_qei_setup (void);

int devconfig_position_set(unsigned int dof_id, unsigned int position);

int devconfig_setup(void);


/* online configurations */

int devconfig_pwm_frequency_set (unsigned int dof_id, 
                                  double frequency, 
                                  unsigned int resolution);
                             
int devconfig_pwm_frequency_get (unsigned int dof_id, 
                                  double *ret_frequency);

int devconfig_pwm_enable (unsigned int dof_id);

int devconfig_pwm_disable (unsigned int dof_id);

int devconfig_qei_capture (unsigned int dof_id,
                           unsigned int unit_position,
                           unsigned int clk_prescaler);

int devconfig_qei_data_init (bbmc_dof_state_t volatile *state);

int devconfig_timer_frequency_set (unsigned int timer, unsigned int count);

int devconfig_timer_frequency_get (unsigned int timer, int *frequency);


int devconfig_io_func (bbmc_io_funcs_t *func_ptrs, 
                       bbmc_io_func_tbl_t *func_table, 
                       const char *conf_mode, 
                       const char *io_mode);

int devconfig_dof_state_set (bbmc_dof_state_t volatile *data, unsigned int value);

int devconfig_io_func_setup (bbmc_io_func_tbl_t *func_table);

int devconfig_poslim_disable (unsigned int axis);

int devconfig_poslim_enable (unsigned int axis);

int devconfig_killswitch_enable (void);

int devconfig_killswitch_disable (void);

int devconfig_timer_run_enable (void);

int devconfig_timer_run_disable (void);

int devconfig_timer_goto_enable (void);

int devconfig_timer_goto_disable (void);

int devconfig_timer_rmpi_enable (void);

int devconfig_timer_rmpi_disable (void);

int devconfig_timer_stop_enable (void);

int devconfig_timer_stop_disable (void);

unsigned int devconfig_gpio_poslim_get (unsigned int  dof_id);

unsigned int devconfig_gpio_killswitch_get (void);

int devconfig_intc_master_disable (void);

int devconfig_intc_master_enable (void);




/*
 * BBMC Input & Output functions - (PORT)
 */

/* input functions */

int input_qei_dual (bbmc_dof_state_t volatile *data);

int input_qei_cap (bbmc_dof_state_t volatile *data);

int input_qei_std (bbmc_dof_state_t volatile *data);


/* output functions */

void output_pwm_dif (bbmc_dof_output_t volatile *output);

void output_gpio_dir (unsigned int dof_id, unsigned int pin_value);

void output_pwm_dir (bbmc_dof_output_t volatile *output);



/*
 * BBMC System initialization & configuration functions.
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

int sysconfig_intc_master_enable (void);

int sysconfig_intc_master_disable (void);

int sysconfig_setup (void);


/* 
 * Master Initialization 
 */

void bbmc_setup (void);

int bbmc_greeting (void);

void system_halt (void);


/*
 * BBMC Controller Infrastructure Functions
 */

/* ISR - interrupt service routines */

/* run controller */

static inline void run_traject (bbmc_dof_state_t volatile *state, 
                                  bbmc_dof_contrl_t volatile *controller);

static inline void run_contrl (bbmc_dof_state_t volatile *state, 
                                 bbmc_dof_contrl_t volatile *controller);

static inline void run_term (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);

static inline void run_output (void);

static inline void run_controller (void);

static inline void run_datalogging (void);

void isr_run (void);

/* goto controller */

static inline void goto_timer_interrupt_on (void);

static inline void goto_timer_interrupt_off (void);

static inline void goto_controller (void);

static inline void goto_stop (void);

static inline void goto_debug (void);

static inline void goto_output (void);

static inline void goto_datalogging (void);

void isr_goto (void);

/* rmpi controller */

static inline void rmpi_timer_interrupt_on (void);

static inline void rmpi_timer_interrupt_off (void);

static inline void rmpi_controller (void);

static inline void rmpi_poslim_stop (void);

static inline void rmpi_datalogging (void);

void isr_rmpi (void);

/* other controllers */

void isr_gpio_pos_limit (void);

void isr_gpio_killswitch (void);

void isr_systick (void);

/* ISR handler functions */

int func_run (void);

int func_goto (void);

int func_rmpi (unsigned int rmpi_dof);

/* CLI functions */

/* run command functions */

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

/* goto command functions */

int cmnd_path (int argc, char *argv[]);

/* goto command functions */

int cmnd_goto_args (int argc, char *argv[], bbmc_cmd_args_t *args);

int cmnd_goto_return_val (int argc, char *argv[], unsigned int goto_ret);

int cmnd_goto (int argc, char *argv[]);

/* rmpi command functions */

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


/* datalog functions */

int datalog_s_setup (datalog_s_t volatile *datalog_ptr);

int datalog_s_init (datalog_s_t volatile *datalog_ptr, data_t init_val);

int datalog_s_print (datalog_s_t volatile *datalog_ptr, int range_indeces[4]);

inline void bbmc_datalog_write (unsigned int index,
                                  datalog_s_t volatile *datalog, 
                                  bbmc_dof_state_t volatile *state,
                                  bbmc_dof_contrl_t volatile *contrl);

/*
 * BBMC system functions
 */

/* performance timer functions */

void bbmc_perf_init (void);

void bbmc_perf_print (void);

/* system flag handler functions */

int bbmc_sysflags_clear  (bbmc_system_flags_t *sysflags, const char *init_mode);

int bbmc_sysflags_print (bbmc_system_flags_t *sysflags, const char *format);

int bbmc_sysflags_set (bbmc_system_flags_t * sysflags, const char *init_mode);
                             
int bbmc_sysflags_get (bbmc_system_flags_t *sysflags, 
                        const char *flag);

int bbmc_sysflags_gpreset_set (bbmc_system_flags_t *sysflags, 
                                int dof_id, 
                                int set_val);
                                     
int bbmc_sysflags_gpreset_get (bbmc_system_flags_t *sysflags, int dof_id);


/* controller state/flag handler fucntions */

int bbmc_cisr_init (bbmc_cisr_t *contrl_state);

int bbmc_cisr_print (bbmc_cisr_t *contrl_state, const char *format);

int bbmc_cisr_get (bbmc_cisr_t *contrl_state, 
                    const char *get_mode);

int bbmc_cisr_set  (bbmc_cisr_t *contrl_state, 
                    const char *set_mode, 
                    int set_val);
                              
/* status-command functions */

int bbmc_cli_clear (void);

int bbmc_cli_newlin (unsigned int lines);

int bbmc_cli_cursor_mv_top (void);

int bbmc_cli_cursor_mv_bottom (void);

/* poslim functions */

int bbmc_poslim_print (const char *format);

/* dof state functions */

int bbmc_dof_state_print (unsigned int dof_id, const char *format);

/* system hardware state functions */

int bbmc_pwm_print (const char *format);

int bbmc_qei_print (const char *format);

int bbmc_timers_print (const char *format);

/* runtime functions */

int bbmc_isr_return_value(unsigned int cmnd_ret, int ret);

int bbmc_goto_home (void);

/* trajecotry generators */

int signalgen_prbs_v0 (datalog_s_t *datalog, int data_index);

int signalgen_prbs_v1 (datalog_s_t *datalog, int *prbs_table, 
                                    int duration, int idle_time, int data_index);
int 
signalgen_prbs_v2 (datalog_s_t *datalog, 
                   int *prbs_table, 
                   int duration, 
                   int idle_time, 
                   int data_index);

int 
signalgen_pulse_v1 (datalog_s_t *datalog, 
                   int *pulse_table, 
                   int duration, 
                   int on_time,
                   int off_time,
                   int idle_time,
                   int data_index);

inline double signalgen_sine_generic (unsigned int index, sg_sine_t *parameters);

inline double signalgen_cosine_generic (unsigned int index, sg_sine_t *parameters);

int signalgen_circle_generic_setup (sg_circle_t *circle);

inline void signalgen_circle_poscontrl_online (sg_circle_t *circle,
                                                csl_carriage_t *inv_kinematics);

inline void signalgen_circle_speedcontrl_online (unsigned int counter, sg_circle_t *circle);

inline void signalgen_circle_trajectcontrl_online (sg_circle_t *circle,
                                                    csl_carriage_t *inv_kinematics);

inline void signalgen_trapezoid_speedcontrl_online (sg_trapez_t *trapez,
                                                      csl_carriage_t *inv_kinematics);

inline void signalgen_trapezoid_trajectcontrl_online (sg_trapez_t *trapez,
                                                        csl_carriage_t *inv_kinematics);

int 
bbmc_kinematics_csl_carriage_setup (void);


/*
 * Controller functions
 */

static inline void  bbmc_contrl_input (void);


/* trajectory generation functions */

static void traject_null (bbmc_dof_state_t volatile *state, 
                            bbmc_dof_contrl_t volatile *controller);
                                 
static void traject_rmpi_si (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);
                                        
static inline void traject_goto_v0 (bbmc_dof_state_t volatile *state, 
                                       bbmc_dof_contrl_t volatile *controller);

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

static void contrl_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                     bbmc_dof_contrl_t volatile *controller);

static void contrl_test_dsr (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);

static inline void contrl_goto_v3 (bbmc_dof_state_t volatile *state, 
                                     bbmc_dof_contrl_t volatile *controller);

static inline void contrl_stop_immediate (bbmc_dof_state_t volatile *state, 
                                            bbmc_dof_contrl_t volatile *controller);


/* termination condition functions */

static void term_rmpi_breakaway(bbmc_dof_state_t volatile *state, 
                                  bbmc_dof_contrl_t volatile *controller);

static void term_rmpi_step (bbmc_dof_state_t volatile *state, 
                              bbmc_dof_contrl_t volatile *controller);
                                
static void term_rmpi_si (bbmc_dof_state_t volatile *state, 
                            bbmc_dof_contrl_t volatile *controller);

static void term_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                   bbmc_dof_contrl_t volatile *controller);

static void term_test_dsr (bbmc_dof_state_t volatile *state, 
                             bbmc_dof_contrl_t volatile *controller);

/* Utility Functions */

static double  util_strtod(const char *str, unsigned char **endptr);

static void    util_dtoa(double f, char *buf);

static int     util_checkpoint_yn(char *format, char *ptr);

static void    util_delay(int count);


/* 
 * GLOBAL DATA
 * 
 */

/* commands table */
 
static const char cmd_names[MAX_NUM_COMMANDS][MAX_CMD_NAME_SIZE] = {
    
    "run","datalog","perf","reset","config",
    "path","goto","rmpi","test","debug",
    "status","quit"
    
};

static const char cmd_help[MAX_NUM_COMMANDS][MAX_HELP_SIZE] = {
    
    HELP_RUN,HELP_DATALOG,HELP_PERF,HELP_RESET,
    HELP_CONFIG,HELP_PATH,HELP_GOTO,HELP_RMPI,
    HELP_TEST,HELP_DEBUG,HELP_STATUS,HELP_QUIT
    
};

tCmdLineEntry g_cmd_table[MAX_NUM_COMMANDS] = {
    
    {cmd_names[0],cmnd_run,cmd_help[0]},{cmd_names[1],cmnd_datalog,cmd_help[1]},
    {cmd_names[2],cmnd_perf,cmd_help[2]},{cmd_names[3],cmnd_reset,cmd_help[3]},
    {cmd_names[4],cmnd_config,cmd_help[4]},{cmd_names[5],cmnd_path,cmd_help[5]},
    {cmd_names[6],cmnd_goto,cmd_help[6]},{cmd_names[7],cmnd_rmpi,cmd_help[7]},
    {cmd_names[8],cmnd_test,cmd_help[8]},{cmd_names[9],cmnd_debug,cmd_help[9]},
    {cmd_names[10],cmnd_status,cmd_help[10]},{cmd_names[11],cmnd_quit,cmd_help[11]},
    {0,0,0}
    
};

/* Device Drivers Data - (PORT) - only devconfig handles these */
static dmtimer_handle_t             bbmc_timers[4];
static ehrpwm_handle_t              bbmc_pwm[2];
static eqep_handle_t                bbmc_qei[2];


/* ISR data */
static bbmc_cisr_t                   g_isr_state;


/* system flags */
static bbmc_system_flags_t          g_flags;

//!
/* Signal Generator Data */
static int                         g_prbs_v1_table[SIGNAL_PRBS_V1_ARRAY_SIZE] = 
                                        SIGNAL_PRBS_V1_SWITCH_INIT;
                                        
static int                         g_prbs_v2_table[SIGNAL_PRBS_V2_ARRAY_SIZE] = 
                                        SIGNAL_PRBS_V2_SWITCH_INIT;

static int                         g_pulse_v1_table[SIGNAL_PULSE_ARRAY_SIZE] = 
                                        SIGNAL_PULSE_SWITCH_INIT;

/* ISR controller runtime logs */
static unsigned int volatile        g_log_perf[CONTROLLER_ITERATIONS_MAX];
static datalog_s_t volatile         g_datalog[BBMC_DOF_NUM];
static datalog_s_t                   g_log_signalgen;

/* machine state data */
static bbmc_dof_state_t volatile   g_dof_state[BBMC_DOF_NUM];

static bbmc_dof_limits_t            g_position_limits[BBMC_DOF_NUM];
static bbmc_dof_limits_t            g_position_init[BBMC_DOF_NUM];
static bbmc_dof_limits_t            g_position_home[BBMC_DOF_NUM];

/* machine controller data */
static bbmc_dof_contrl_t volatile  g_args_contrl[BBMC_DOF_NUM];
static bbmc_dof_contrl_t volatile  g_args_goto[BBMC_DOF_NUM];
static bbmc_dof_contrl_t volatile  g_args_stop[BBMC_DOF_NUM];

/* global controller function table */
static bbmc_io_func_tbl_t           g_func_tbl[BBMC_DOF_NUM];

/* rmpi data */
static bbmc_io_funcs_t              rmpi_io;
static bbmc_contrl_funcs_t         rmpi_contrl;

static datalog_s_t volatile        *rmpi_log;
static bbmc_dof_limits_t           *rmpi_limits;
static bbmc_dof_state_t  volatile *rmpi_state;
static bbmc_dof_contrl_t volatile *rmpi_args;

/* algorithm data */
static sg_trapez_t                  g_signalgen_trapezoid; 
static sg_sinusoid_t                    g_signalgen_sinusoid; 
static sg_circle_t                  g_signalgen_circle; 
static csl_carriage_t              g_carriage_kinematics;


//! typedef into a single unit struct
/* systick timer data */ 
static volatile int                 timer_ticks   = 0;
static volatile int                 timer_ticking = 0;


/*
 * 
 * MAIN
 * 
 */

int 
main(void)
{
    static char cmd_rx_buff[RX_BUFF_SIZE] = {0};
    static int  cmd_ret;
    
    /* 
     * BBMC boot:
     *
     * - all timers have 1ms default period 
     * - pwm frequency defaults to 50 kHz
     * - qei capture defaults to upps=1, ccps=64.
     * - by default, all arm-core caches are enabled. 
     * -
     * -
     * -
     * 
     */
    
    /* Device and System Initializations */
    bbmc_setup();
    
    /* BBMC greeting message to stdio channel */
    bbmc_greeting();

    /* Initialize System Flags*/
    bbmc_sysflags_clear (&g_flags, "-all");
    
    /* command loop */
    for (;;)
    {
        
        if (g_flags.cmdln == 1)
        {
            continue;
        }
        
        if (g_flags.cmdln == -1)
        {
            break;
        }
        
        //UARTprintf("@%d\r\n", __LINE__);
        
        UARTPuts("\r\nbbmc:~$ ", -1);
        UARTGets(cmd_rx_buff, RX_BUFF_SIZE);
        cmd_ret = CmdLineProcess(cmd_rx_buff, g_cmd_table);
        
        /* Debug */
        if (g_flags.debug == 1)
        {
            UARTprintf("\r\nDEBUG: %d\r\n", cmd_ret);
        }
    }
    
    /* HALT the system - power down and loop forever */
    system_halt();
    
    return 0;
}



/* system halt function */
void
system_halt (void)
{
    /* Add power down and cleanup functions */
    
    
    UARTPuts("\r\nSystem Halt: System is shuting down. Goodbye!\r\n", -1);

    for (;;)
    {
        ;
    }
}

/*
 * BBMC device initialization & configuration functions.
 */

/* default/initial configurations */

int 
devconfig_stdio_setup(void)
{
    UARTStdioInit();
    
    return 0;
}

int 
devconfig_mpucache_setup (unsigned int cache_mode)
{
    if (cache_mode == CACHE_ALL)
    {
        CacheEnable(CACHE_ALL);
        UARTPuts("\r\ndevconfig: mpu cache: all enabled", -1);
    }
    
    else if (cache_mode == CACHE_ICACHE)
    {
        CacheEnable(CACHE_ICACHE);
        UARTPuts("\r\ndevconfig: mpu cache: data cache enabled", -1);
    }
    
    else if (cache_mode == CACHE_DCACHE)
    {
        CacheEnable(CACHE_DCACHE);
        UARTPuts("\r\ndevconfig: mpu cache: instruction cache enabled", -1);
    }
    
    else
    {
        UARTPuts("\r\nerror: devconfig: mpu cache: invalid cache_mode argumemnt", -1);
    }
    
    return 0;
}

int 
devconfig_interrupts_setup (void)
{
    IntAINTCInit();
    
    /* Register the func pntr for every ISR */
    IntRegister(SYS_INT_TINT2, isr_run);
    IntRegister(SYS_INT_TINT3, isr_goto);
    IntRegister(SYS_INT_TINT4, isr_rmpi);
    IntRegister(SYS_INT_TINT5, isr_systick);
    IntRegister(SYS_INT_GPIOINT1A , isr_gpio_pos_limit);
    IntRegister(SYS_INT_GPIOINT1B , isr_gpio_killswitch);
    
    /* Set priority for each ISR */
    IntPrioritySet(SYS_INT_TINT2, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT3, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT4, 3, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_TINT5, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_GPIOINT1A, 1, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_GPIOINT1B, 2, AINTC_HOSTINT_ROUTE_IRQ);
    
    /* Enable Interrupt recognition in AINTC */
    IntSystemEnable(SYS_INT_TINT2);
    IntSystemEnable(SYS_INT_TINT3);
    IntSystemEnable(SYS_INT_TINT4);
    IntSystemEnable(SYS_INT_TINT5);
    IntSystemEnable(SYS_INT_GPIOINT1A);
    IntSystemEnable(SYS_INT_GPIOINT1B);
    
    return 0;
}

int 
devconfig_gpio_setup (void)
{
    //! this is temporary until the gpio driver is updated to new prog. model.
    GPIO1ModuleClkConfig();
    GpioModuleEnable(SOC_GPIO_1_REGS);
    
    gpio_dir_1_setup();
    gpio_dir_2_setup();
    
    gpio_hall_1_setup();
    gpio_hall_2_setup();
    
    gpio_killswitch_setup (KILLSWITCH_GPIO_DEBOUCE_TIME);
    
    return 0;
}

int 
devconfig_timer_setup (void)
{
    dmtimer_handle_init(2, &bbmc_timers[0]);
    dmtimer_handle_init(3, &bbmc_timers[1]);
    dmtimer_handle_init(4, &bbmc_timers[2]);
    dmtimer_handle_init(5, &bbmc_timers[3]);
    
    dmtimer_open(2);
    dmtimer_open(3);
    dmtimer_open(4);
    dmtimer_open(5);
    
    dmtimer_init(2);
    dmtimer_init(3);
    dmtimer_init(4);
    dmtimer_init(5);
    
    return 0;
}

int 
devconfig_peripheral_setup (void)
{
    L3L4_driver_init();
    pwmss_driver_init();
    
    return 0;
}

int 
devconfig_pwm_setup (void)
{
    ehrpwm_handle_init(1, &bbmc_pwm[0]);
    ehrpwm_handle_init(2, &bbmc_pwm[1]);
    
    ehrpwm_open(1);
    ehrpwm_open(2);
    
    ehrpwm_init(1);
    ehrpwm_init(2);
    
    ehrpwm_1_pinmux_setup();
    ehrpwm_2_pinmux_setup();
    
    return 0;
}

int 
devconfig_qei_setup (void)
{
    eqep_handle_init(1, &bbmc_qei[0]);
    eqep_handle_init(2, &bbmc_qei[1]);
    
    eqep_open(1);
    eqep_open(2);
    
    eqep_init(1);
    eqep_init(2);
    
    eqep_1_pinmux_setup();
    eqep_2_pinmux_setup();
    
    eqep_data_init(&(g_dof_state[0].state));
    eqep_data_init(&(g_dof_state[1].state));
    
    return 0;
}

int 
devconfig_setup(void)
{
    
    devconfig_stdio_setup();
    UARTprintf("\033[2J\033[1;1HBBMC-v0.5 is now initializing...\r\n");
    UARTPuts("\r\nUART_0 stdio channel has opened.", -1);
    
    UARTPuts("\r\nInitializing Peripheral Drivers: ", -1);
    devconfig_peripheral_setup();
    UARTPuts("\tDONE", -1);
    
    UARTPuts("\r\nConfiguring MPU cache: ", -1);
    devconfig_mpucache_setup(CACHE_ALL);
    UARTPuts("\tDONE", -1);
    
    UARTPuts("\r\nConfiguring Interrupt Controller: ", -1);
    devconfig_interrupts_setup();
    UARTPuts("\tDONE", -1);
    
    UARTPuts("\r\nConfiguring System Timers: ", -1);
    devconfig_timer_setup();
    UARTPuts("\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring Performance Timer: ", -1);
    PerfTimerSetup();
    UARTPuts("\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring GPIO: ", -1);
    devconfig_gpio_setup();
    UARTPuts("\t\t\tDONE", -1);
    
    UARTPuts("\r\nConfiguring PWM and QEI: ", -1);
    devconfig_pwm_setup();
    devconfig_qei_setup();
    UARTPuts("\t\tDONE", -1);
    
    return 0;
}

/* online configurations */

unsigned int 
devconfig_gpio_poslim_get (unsigned int  dof_id)
{
    unsigned int ret;
    
    if (dof_id == 1)
    {
        ret = GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN;
    }
    
    else if (dof_id == 2)
    {
        ret = GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN;
    }
    
    else
    {
        ret = 0;
    }
    
    return ret;
}

int 
devconfig_intc_master_disable (void)
{
    IntMasterIRQDisable();
    
    return 0;
}

int 
devconfig_intc_master_enable (void)
{
    IntMasterIRQEnable();
    
    return 0;
}

int 
devconfig_pwm_frequency_set (unsigned int dof_id, 
                             double frequency, 
                             unsigned int resolution)
{
    return ehrpwm_config_frequency_set(dof_id, frequency, resolution);
}

int
devconfig_pwm_frequency_get (unsigned int dof_id, 
                             double *ret_frequency)
{
    return ehrpwm_config_frequency_get(dof_id, ret_frequency);
}

int 
devconfig_pwm_enable (unsigned int dof_id)
{
    return ehrpwm_enable(dof_id);
}

int 
devconfig_pwm_disable (unsigned int dof_id)
{
    return ehrpwm_disable(dof_id);
}

int 
devconfig_qei_capture (unsigned int dof_id,
                       unsigned int unit_position,
                       unsigned int clk_prescaler)
{
    return eqep_caputure_config(dof_id, unit_position, clk_prescaler);
}

int 
devconfig_qei_data_init (bbmc_dof_state_t volatile *state)
{
    return eqep_data_init(&(state->state));
}

int 
devconfig_position_set(unsigned int dof_id, unsigned int position)
{
    return eqep_write(dof_id, position);
}

int 
devconfig_timer_frequency_set (unsigned int timer, unsigned int count)
{
    return dmtimer_tldr_config_set(timer, count);
}

int 
devconfig_timer_frequency_get (unsigned int timer, int *frequency)
{
    if (frequency < 0)
    {
        return -1;
    }
    
    unsigned int temp;
    int freq;
    
    temp = dmtimer_tldr_config_get(timer);
    temp = DMTIMER_COUNT_MAX - temp;
    
    freq = DMTIMER_SYSTEM_CLK_DEFAULT / temp;
    
    *frequency = freq;
    
    return 0;
}

int 
devconfig_io_func (bbmc_io_funcs_t *func_ptrs, 
                   bbmc_io_func_tbl_t *func_table, 
                   const char *conf_mode, 
                   const char *io_mode)
{
    int ret;
    
    if (!strcmp((const char *)io_mode, "-i"))
    {
        if (!strcmp((const char *)conf_mode, "-std"))
        {
            func_ptrs->input_func = func_table->input_funcs[0];
        }
        
        else if (!strcmp((const char *)conf_mode, "-cap"))
        {
            func_ptrs->input_func = func_table->input_funcs[1];
        }
        
        else if (!strcmp((const char *)conf_mode, "-dual"))
        {
            func_ptrs->input_func = func_table->input_funcs[2];
        }
        
        else
        {
            return -2;
        }
        
        ret = 0;
    }
    
    else if (!strcmp((const char *)io_mode, "-o"))
    {
        if (!strcmp((const char *)conf_mode, "-dir"))
        {
            func_ptrs->output_func = func_table->output_funcs[0];
        }
        
        else if (!strcmp((const char *)conf_mode, "-dif"))
        {
             func_ptrs->output_func = func_table->output_funcs[1];
        }
        
        else
        {
            return -2;
        }
        
        ret = 1;
    }
    
    else
    {
        return -1;
    }
    
    return ret;
}

int 
devconfig_dof_state_set (bbmc_dof_state_t volatile *data, unsigned int value)
{
    data->state.count[1] = value;
    
    return eqep_write (data->dof_id, value);
}

int 
devconfig_io_func_setup (bbmc_io_func_tbl_t *func_table)
{
    if (func_table == NULL)
    {
        return -1;
    }
    
    func_table->input_funcs[0] = input_qei_std;
    func_table->input_funcs[1] = input_qei_cap;
    func_table->input_funcs[2] = input_qei_dual;
    
    func_table->output_funcs[0] = output_pwm_dir;
    func_table->output_funcs[1] = output_pwm_dif;
    
    return 0;
}

int 
devconfig_poslim_disable (unsigned int axis)
{
    if (axis == 0)
    {
        GPIOPinIntDisable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    else if (axis == 1)
    {
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        GPIOPinIntDisable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else if (axis == BBMC_DOF_NUM)
    {
        GPIOPinIntDisable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntDisable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else
    {
        return -1;
    }
    
    return 0;
}

int 
devconfig_poslim_enable (unsigned int axis)
{
    
    if (axis == 0)
    {
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntEnable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    else if (axis == 1)
    {
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        GPIOPinIntEnable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else if (axis == BBMC_DOF_NUM)
    {
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
        
        GPIOPinIntEnable(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
        GPIOPinIntEnable(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    else
    {
        return -1;
    }
    
    return 0;
}

int
devconfig_killswitch_enable (void)
{
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    GPIOPinIntEnable(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    
    return 0;
}

int
devconfig_killswitch_disable (void)
{
    GPIOPinIntDisable(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
    
    return 0;
}

int
devconfig_timer_run_enable (void)
{
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(SOC_DMTIMER_2_REGS);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
devconfig_timer_run_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(SOC_DMTIMER_2_REGS);
    
    return 0;
}

int
devconfig_timer_goto_enable (void)
{
    DMTimerIntEnable(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(SOC_DMTIMER_3_REGS);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
devconfig_timer_goto_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(SOC_DMTIMER_3_REGS);
    
    return 0;
}

int
devconfig_timer_rmpi_enable (void)
{
    DMTimerIntEnable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(SOC_DMTIMER_4_REGS);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
devconfig_timer_rmpi_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(SOC_DMTIMER_4_REGS);
    
    return 0;
}

int
devconfig_timer_stop_enable (void)
{
    DMTimerIntEnable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerEnable(SOC_DMTIMER_5_REGS);
    
    IntMasterIRQEnable();
    
    return 0;
}

int
devconfig_timer_stop_disable (void)
{
    IntMasterIRQDisable();
    
    DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
    DMTimerDisable(SOC_DMTIMER_5_REGS);
    
    return 0;
}

unsigned int
devconfig_gpio_killswitch_get (void)
{
    return (GPIOPinRead(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_PIN) >> KILLSWITCH_GPIO_PIN);
}



/*
 * BBMC Input & Output functions - (PORT)
 */

/* input functions */

int 
input_qei_dual (bbmc_dof_state_t volatile *data)
{
    eqep_read (data->dof_id, EQEP_DUAL, &(data->state));
    
    if (data->state.speed_mode == 1)
    {
        data->state.speed = data->state.speed_std;
        
        if (fabs(data->state.speed_std) <= data->state.speed_thr)
        {
             data->state.speed_mode = 0;
        }
    }
    
    else
    {
        data->state.speed = data->state.speed_cap;
        
        if (fabs(data->state.speed_cap) > data->state.speed_thr)
        {
             data->state.speed_mode = 1;
        }
    }
    
    return 0;
}

int 
input_qei_cap (bbmc_dof_state_t volatile *data)
{
    eqep_read (data->dof_id, EQEP_CAP, &(data->state));
    
    data->state.speed = data->state.speed_cap;
    
    return 0;
}

int 
input_qei_std (bbmc_dof_state_t volatile *data)
{
    eqep_read (data->dof_id, EQEP_STD, &(data->state));
    
    data->state.speed = data->state.speed_std;
    
    return 0;
}

/* output functions */

void 
output_pwm_dif (bbmc_dof_output_t volatile *output)
{
    ehrpwm_write(output->dof_id, EHRPWM_WRITE_DIFF, output->value);
}

//! this must be updated when gpio driver is updated.
void
output_gpio_dir (unsigned int dof_id, unsigned int pin_value)
{
    if (dof_id == 1)
    {
        GPIOPinWrite(DIR_1_GPIO_ADDRESS, DIR_1_GPIO_PIN, pin_value);
    }
    
    else if (dof_id == 2)
    {
        GPIOPinWrite(DIR_2_GPIO_ADDRESS, DIR_2_GPIO_PIN, pin_value);
    }
    
    else
    {
        ;
    }
}

void 
output_pwm_dir (bbmc_dof_output_t volatile *output)
{
    if (output->value >= 0)
    {
        output_gpio_dir(output->dof_id, GPIO_PIN_HIGH);
    }
    
    if (output->value < 0)
    {
        output->value = -1 * output->value;
        output_gpio_dir(output->dof_id, GPIO_PIN_LOW);
    }
    
    if (output->value > 100)
    {
        output->value = 100;
    }
    
    ehrpwm_write(output->dof_id, EHRPWM_WRITE_A, output->value);
}


/*
 * BBMC system initialization & configuration functions.
 */

/* startup system configurations */

int
sysconfig_dof_setup (void)
{
    int i;
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        g_dof_state[i].dof_id = (i + 1);
        
        g_dof_state[i].state.speed = 0;
        g_dof_state[i].state.count[0] = 0;
        g_dof_state[i].state.count[1] = 0;
        
        g_args_contrl[i].dof_id = (i + 1);
        g_args_contrl[i].output.dof_id = (i + 1);
        
        g_args_goto[i].dof_id = (i + 1);
        g_args_goto[i].output.dof_id = (i + 1);
        
        g_args_stop[i].dof_id = (i + 1);
        g_args_stop[i].output.dof_id = (i + 1);
    }
    
    //!
    sysconfig_qei_capture(1, 1, 64);
    sysconfig_qei_capture(2, 1, 64);
    
    sysconfig_qei_motor(1, MAX_SPEED_MOTOR_1);
    sysconfig_qei_motor(2, MAX_SPEED_MOTOR_2);
    
    sysconfig_qei_speed_switch(1, SPEED_MODE_THRESHOLD_Y);
    sysconfig_qei_speed_switch(2, SPEED_MODE_THRESHOLD_X);
    
    return 0;
}

int 
sysconfig_logs_setup (void)
{
    int i;
    
    UARTPuts("\r\n\r\nConfiguring <datalog> and <perf> handlers: ", -1);
    datalog_s_setup(&g_log_signalgen);
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        datalog_s_setup(&g_datalog[i]);
    }
    
    bbmc_perf_init();
    UARTPuts("\tDONE\r\n", -1);
    
    UARTPuts("\r\nInitializing <datalog> and <perf> data arrays: ", -1);
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        datalog_s_init(&g_datalog[i], 0);
    }
    
    UARTPuts("\tDONE\r\n", -1);
    
    return 0;
}

int
sysconfig_poslim_setup (void)
{
    g_position_limits[0].limval[0] = POSITION_Y_NEG_THRESH;
    g_position_limits[0].limval[1] = POSITION_Y_POS_THRESH;
    g_position_limits[1].limval[0] = POSITION_X_NEG_THRESH;
    g_position_limits[1].limval[1] = POSITION_X_POS_THRESH;
    
    g_position_home[0].limval[0] = POSITION_Y_NEG_HOME;
    g_position_home[0].limval[1] = POSITION_Y_POS_HOME;
    g_position_home[1].limval[0] = POSITION_X_NEG_HOME;
    g_position_home[1].limval[1] = POSITION_X_POS_HOME;
    
    g_position_init[0].limval[0] = POSITION_Y_NEG_INIT;
    g_position_init[0].limval[1] = POSITION_Y_POS_INIT;
    g_position_init[1].limval[0] = POSITION_X_NEG_INIT;
    g_position_init[1].limval[1] = POSITION_X_POS_INIT;
    
    return 0;
}

int 
sysconfig_contrl_stop_setup (void)
{
    /* argument map: STOP_ARGS::contrl_stop_immediate
     * 
     *  i0: -                         , d0: P gain
     *  i1: -                         , d1: I gain
     *  i2: -                         , d2: Integral sum
     *  i3: -                         , d3: -
     *  i4: -                         , d4: -
     *  i5: -                         , d5: -
     *  i6: -                         , d6: -
     *  i7: -                         , d7: -
     */
    
    g_args_stop[0].arg_double[0] = STOP_SPEED_GAIN_P_1;
    g_args_stop[1].arg_double[0] = STOP_SPEED_GAIN_P_2;
    
    g_args_stop[0].arg_double[1] = STOP_SPEED_GAIN_I_1;
    g_args_stop[1].arg_double[1] = STOP_SPEED_GAIN_I_2;
    
    g_args_stop[0].arg_double[2] = 0;
    g_args_stop[1].arg_double[2] = 0;
    
    return 0;
}

int
sysconfig_contrl_stop_init (void)
{
    /* reset integral sum */
    g_args_stop[0].arg_double[2] = 0;
    g_args_stop[1].arg_double[2] = 0;
    
    return 0;
}

int 
sysconfig_io_func_setup (bbmc_io_func_tbl_t *func_table)
{
    if (func_table == NULL)
    {
        UARTPuts("error: sysconfig_func_table_setup: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    return devconfig_io_func_setup(func_table);
}

int
sysconfig_io_func (unsigned int dof_id,
                   bbmc_io_funcs_t *func_ptrs, 
                   bbmc_io_func_tbl_t *func_table, 
                   const char *conf_mode, 
                   const char *io_mode)
{
    int ret;
    
    if (func_ptrs == NULL)
    {
         UARTPuts("\r\nerror: sysconfig_io_func: func_ptr argument is NULL", -1);
        return -1;
    }
    
    if (func_table == NULL)
    {
         UARTPuts("\r\nerror: sysconfig_io_func:  func_table argument is NULL", -1);
        return -2;
    }
    
    ret = devconfig_io_func (func_ptrs, func_table, conf_mode, io_mode);
    
    if (ret == -1)
    {
        UARTPuts("\r\nerror: sysconfig_io_func: invalid input/output mode argument", -1);
    }
    
    else if (ret == -2)
    {
        UARTPuts("\r\nerror: sysconfig_io_func: invalid config mode argument", -1);
    }
    
    else if (ret == 0)
    {
        UARTprintf("\r\nsysconfig_io_func: axis-%d input mode is %s", dof_id, conf_mode);
    }
    
    else if (ret == 1)
    {
        UARTprintf("\r\nsysconfig_io_func: axis-%d output mode is %s", dof_id, conf_mode);
    }
    
    else
    {
         UARTPuts("\r\nerror: sysconfig_io_func: unknown error event.", -1);
    }
    
    return 0;
}

/* online configurations */

int 
sysconfig_dof_state_set (bbmc_dof_state_t volatile *data, unsigned int value)
{
    if (data == NULL)
    {
        UARTPuts("\r\nerror: sysconfig_dof_state_set: invalid data pointer argument", -1);
        return -1;
    }

    return devconfig_dof_state_set (data, value);
}

int
sysconfig_qei_capture (unsigned int dof_id,
                       unsigned int unit_position,
                       unsigned int clk_prescaler)
{
    unsigned int prescaler;
    
    prescaler = EQEP_SYSCLK / clk_prescaler;
    prescaler = prescaler * unit_position;
    
    g_dof_state[dof_id-1].state.cap_prescaler = prescaler;
    
    return devconfig_qei_capture (dof_id, unit_position, clk_prescaler);
}

int 
sysconfig_qei_speed_switch (unsigned int dof_id, double switch_speed)
{
    if (dof_id > BBMC_DOF_NUM)
    {
        return -1;
    }
    
    if (switch_speed < 0)
    {
        return -2;
    }
    
    g_dof_state[dof_id-1].state.speed_thr = switch_speed;
    
    return 0;
}

int 
sysconfig_qei_motor (unsigned int dof_id, double max_motor_speed)
{
    if (dof_id > BBMC_DOF_NUM)
    {
        return -1;
    }
    
    max_motor_speed = (g_dof_state[dof_id-1].state.cap_prescaler) / max_motor_speed;
    
    g_dof_state[dof_id-1].state.cprd_min = (unsigned int)max_motor_speed;
    
    return 0;
}

int 
sysconfig_qei_data_init (unsigned int timer, unsigned int dof_id)
{
    int freq;
    
    sysconfig_timer_frequency_get(timer, &freq);
    
    g_dof_state[dof_id-1].state.sampling_freq = freq;
    
    return devconfig_qei_data_init(&(g_dof_state[dof_id-1]));
}

int 
sysconfig_position_reset (unsigned int dof_id)
{
    if (g_flags.gpos_reset[dof_id-1] == 0)
    {
        g_dof_state[dof_id-1].state.count[1] = g_position_init[dof_id-1].limval[1];
        
        devconfig_position_set(dof_id, g_position_init[dof_id-1].limval[1]);
    }
    
    if (g_flags.gpos_reset[dof_id-1] == 1)
    {
        g_dof_state[dof_id-1].state.count[1] = g_position_init[dof_id-1].limval[0];
        
        devconfig_position_set(dof_id, g_position_init[dof_id-1].limval[0]);
    }
    
    return 0;
}

int 
sysconfig_position_set (unsigned int dof_id, unsigned int position)
{
    g_dof_state[dof_id-1].state.count[1] = position;
        
    devconfig_position_set(dof_id, position);
    
    return 0;
}

int
sysconfig_pwm_frequency_get (unsigned int dof_id, 
                             double *ret_frequency)
{
    return devconfig_pwm_frequency_get(dof_id, ret_frequency);
}

int 
sysconfig_pwm_enable (unsigned int dof_id)
{
    return devconfig_pwm_enable(dof_id);
}

int 
sysconfig_pwm_disable (unsigned int dof_id)
{
    return devconfig_pwm_disable(dof_id);
}

int 
sysconfig_timer_frequency_set (unsigned int timer, double frequency)
{
    unsigned int count;
    
    if (frequency < 0)
    {
        return -1;
    }
    
    g_dof_state[0].state.sampling_freq = (unsigned int)frequency;
    g_dof_state[1].state.sampling_freq = (unsigned int)frequency;
    
    frequency = DMTIMER_SYSTEM_CLK_DEFAULT / frequency;
    
    count = DMTIMER_COUNT_MAX - (unsigned int)frequency;
    
    return devconfig_timer_frequency_set (timer, count);
}

int 
sysconfig_timer_frequency_get (unsigned int timer, int *frequency)
{
    return devconfig_timer_frequency_get (timer, frequency);
}

int 
sysconfig_poslim_enable (unsigned int axis)
{
    return devconfig_poslim_enable(axis);
}

int 
sysconfig_killswitch_enable (void)
{
    return devconfig_killswitch_enable();
}

int 
sysconfig_poslim_disable (unsigned int axis)
{
    return devconfig_poslim_disable(axis);
}

int 
sysconfig_killswitch_disable (void)
{
    return devconfig_killswitch_disable();
}

int
sysconfig_timer_enable (unsigned int timer)
{
    int ret;
    
    if (timer == TIMER_RUN)
    {
        devconfig_timer_run_enable();
        ret = 0;
    }
    
    else if (timer == TIMER_GOTO)
    {
        devconfig_timer_goto_enable();
        ret = 1;
    }
    
    else if (timer == TIMER_RMPI)
    {
        devconfig_timer_rmpi_enable();
        ret = 2;
    }
    
    else if (timer == TIMER_STOP)
    {
        devconfig_timer_stop_enable();
        ret = 3;
    }
    
    else
    {
        ret = -1;
    }
    
    return ret;
}

int
sysconfig_timer_disable (unsigned int timer)
{
    int ret;
    
    if (timer == TIMER_RUN)
    {
        devconfig_timer_run_disable();
        ret = 0;
    }
    
    else if (timer == TIMER_GOTO)
    {
        devconfig_timer_goto_disable();
        ret = 1;
    }
    
    else if (timer == TIMER_RMPI)
    {
        devconfig_timer_rmpi_disable();
        ret = 2;
    }
    
    else if (timer == TIMER_STOP)
    {
        devconfig_timer_stop_disable();
        ret = 3;
    }
    
    else
    {
        ret = -1;
    }
    
    return ret;
}

int
sysconfig_setup (void)
{
    sysconfig_dof_setup();
    
    sysconfig_logs_setup();
    
    sysconfig_poslim_setup();
    
    sysconfig_contrl_stop_setup();
    
    sysconfig_io_func_setup(g_func_tbl);
    
    return 0;
}

int
sysconfig_gpio_killswitch_get (unsigned int *pin_value)
{
    *pin_value = devconfig_gpio_killswitch_get();
    
    return 0;
}

int
sysconfig_gpio_poslim_get (unsigned int  dof_id, unsigned int *pin_value)
{
    *pin_value = devconfig_gpio_poslim_get(dof_id);
    
    return 0;
}

int
sysconfig_intc_master_enable (void)
{
    return devconfig_intc_master_enable();
}

int
sysconfig_intc_master_disable (void)
{
    return devconfig_intc_master_disable();
}


/* master initialization */

void 
bbmc_setup (void)
{
    /* hardware configurations */
    devconfig_setup();
    
    /* system configurations */
    sysconfig_setup();
    
    /* other configurations */
    bbmc_kinematics_csl_carriage_setup();
}

int
bbmc_greeting (void)
{
    UARTPuts(g_bbmc_greeting, -1);
    return 0;
}

/*
 * BBMC Infrastructure functions.
 */

/* TRAJECTORY functions */
static inline void 
traject_goto_v0 (bbmc_dof_state_t volatile *state, 
                 bbmc_dof_contrl_t volatile *controller)
{
    
    if (controller->arg_int[1] >= 0)
    {
        controller->state_desired.q_dot = controller->arg_double[2];
    }
    
    else
    {
        controller->state_desired.q_dot = -controller->arg_double[2];
    }
}

static void 
traject_null (bbmc_dof_state_t volatile *state, 
              bbmc_dof_contrl_t volatile *controller)
{
    ;
}

static void 
traject_rmpi_si (bbmc_dof_state_t volatile *state, 
                 bbmc_dof_contrl_t volatile *controller)
{
    int dof_id = controller->dof_id - 1;
    
    if (controller->arg_int[2] == 0)
    {
        int i = g_isr_state.iteration_counter;
        
        controller->arg_double[1] = g_log_signalgen.log[i].data[dof_id];
    }
    
    else
    {
        controller->arg_double[1] = 0;
    }
}

static void 
traject_rmpi_pid_tune (bbmc_dof_state_t volatile *state, 
                       bbmc_dof_contrl_t volatile *controller)
{
    ;
}

/* CONTROLLER functions */

static void 
contrl_rmpi_breakaway (bbmc_dof_state_t volatile *state, 
                       bbmc_dof_contrl_t volatile *controller)
{
    
    if ((g_isr_state.iteration_counter%(controller->arg_int[3])) == 0)
    {
        controller->output.value += (controller->arg_double[0] * controller->arg_int[0]);
        
    }
}

static void 
contrl_rmpi_step (bbmc_dof_state_t volatile *state, 
                  bbmc_dof_contrl_t volatile *controller)
{
    
    if (g_isr_state.iteration_counter > controller->arg_int[0])
    {
        controller->output.value = controller->arg_double[0];
    }
    else
    {
        controller->output.value = 0;
    }
}

static void 
contrl_rmpi_step2 (bbmc_dof_state_t volatile *state, 
                   bbmc_dof_contrl_t volatile *controller)
{
    
    if (g_isr_state.iteration_counter >= controller->arg_int[0])
    {
        
        controller->output.value = controller->arg_double[0];
    }
    else if (g_isr_state.iteration_counter >= controller->arg_int[2])
    {
        
        controller->output.value = controller->arg_double[1];
    }
    else
    {
        controller->output.value = 0;
    }
}

static void 
contrl_rmpi_si (bbmc_dof_state_t volatile *state, 
                  bbmc_dof_contrl_t volatile *controller)
{
    controller->output.value = controller->arg_double[0] * controller->arg_double[1];
}

static void 
contrl_rmpi_sine (bbmc_dof_state_t volatile *state, 
                  bbmc_dof_contrl_t volatile *controller)
{
    if (g_isr_state.iteration_counter >= controller->arg_int[0])
    {
        
        controller->output.value = (((controller->arg_double[0])
        
        * sin(PI_X2*((g_isr_state.iteration_counter*(controller->arg_double[1]))
         
        + controller->arg_double[2]))) + controller->arg_double[3]);
    }
}

inline double
signalgen_sine_generic (unsigned int index, sg_sine_t *parameters)
{
    double out;
    
    out = PI_X2 * index * parameters->frequency * parameters->sampling_period;
    
    out += parameters->phase;
    
    out = sin(out);
    
    out *= parameters->amplitude;
    
    out += parameters->offset;
    
    return out;
}

inline double
signalgen_cosine_generic (unsigned int index, sg_sine_t *parameters)
{
    double out;
    
    out = PI_X2 * index * parameters->frequency * parameters->sampling_period;
    
    out += parameters->phase;
    
    out = cos(out);
    
    out *= parameters->amplitude;
    
    out += parameters->offset;
    
    return out;
}

int 
signalgen_circle_generic_setup (sg_circle_t *circle)
{
    if (circle == NULL)
    {
        return -1;
    }
    
    double tmp[2];
    
    tmp[0] = circle->y_0 - circle->y_c;
    tmp[1] = circle->x_0 - circle->x_c;
    
    circle->sine.phase = atan2(tmp[0], tmp[1]);
    
    return 0;
}

inline void
signalgen_circle_poscontrl_online (sg_circle_t *circle,
                                 csl_carriage_t *inv_kinematics)
{
    /* motor 1 */
    
    circle->sine.offset = circle->y_c;
    
    circle->sine.amplitude = circle->radius;
    
    g_args_contrl[0].state_desired.q = signalgen_sine_generic(
                                     g_isr_state.iteration_counter,
                                     &(circle->sine));
    
    /* motor 2 */
    
    circle->sine.offset = circle->x_c;
    
    circle->sine.amplitude = circle->radius;
    
    g_args_contrl[1].state_desired.q = signalgen_cosine_generic(
                                     g_isr_state.iteration_counter,
                                     &(circle->sine));
}

inline void
signalgen_circle_speedcontrl_online (unsigned int counter, sg_circle_t *circle)
{
    circle->sine.offset = 0;
    
    /* motor 1 */
    
    circle->sine.amplitude = circle->radius * PI_X2 * circle->sine.frequency;
    
    g_args_contrl[0].state_desired.q_dot = signalgen_cosine_generic(counter, &(circle->sine));
    
    /* motor 2 */
    
    circle->sine.amplitude = circle->radius * (-1) * PI_X2 * circle->sine.frequency;
    
    g_args_contrl[1].state_desired.q_dot = signalgen_sine_generic(counter, &(circle->sine));
}

inline void
signalgen_circle_trajectcontrl_online (sg_circle_t *circle,
                                 csl_carriage_t *inv_kinematics)
{
    /* motor 1 */
    
    circle->sine.offset = circle->y_c;
    
    circle->sine.amplitude = circle->radius * inv_kinematics->beta_y;
    
    g_args_contrl[0].state_desired.q = signalgen_sine_generic(
                                        g_isr_state.iteration_counter,
                                        &(circle->sine));
                                     
    circle->sine.offset = 0;
    
    circle->sine.amplitude *= PI_X2 * circle->sine.frequency;
    
    g_args_contrl[0].state_desired.q_dot = signalgen_cosine_generic(
                                           g_isr_state.iteration_counter,
                                           &(circle->sine));
    
    /* motor 2 */
    
    circle->sine.offset = circle->x_c;
    
    circle->sine.amplitude = circle->radius * inv_kinematics->beta_x;
    
    g_args_contrl[1].state_desired.q = signalgen_cosine_generic(
                                       g_isr_state.iteration_counter,
                                       &(circle->sine));
                                     
    circle->sine.offset = 0;
    
    circle->sine.amplitude *= (-1) * PI_X2 * circle->sine.frequency;
    
    g_args_contrl[1].state_desired.q_dot = signalgen_sine_generic(
                                           g_isr_state.iteration_counter,
                                           &(circle->sine));
}

inline void
signalgen_trapezoid_speedcontrl_online (sg_trapez_t *trapez,
                                        csl_carriage_t *inv_kinematics)
{
    double temp;
    
    unsigned int counter = g_isr_state.iteration_counter;
    
    /* motor 1 */
    
    if (counter <= g_args_contrl[0].arg_int[2])
    {
        g_args_contrl[0].state_desired.q_dot = 0;
        g_args_contrl[0].state_desired.q = trapez->y_0;
    }
    
    else if ((counter > g_args_contrl[0].arg_int[2]) && (counter <= g_args_contrl[0].arg_int[3]))
    {
        temp = counter - g_args_contrl[0].arg_int[2];
        temp = temp * trapez->sampling_period;
        g_args_contrl[0].state_desired.q_dot = trapez->acc_a * temp;
        
        temp = 0.5 * trapez->acc_a * temp * temp;
        g_args_contrl[0].state_desired.q = temp + trapez->y_0;
    }
    
    else if ((counter > g_args_contrl[0].arg_int[3]) && (counter <= g_args_contrl[0].arg_int[4]))
    {
        g_args_contrl[0].state_desired.q_dot = trapez->speed_ss;
        
        temp = counter - g_args_contrl[0].arg_int[2];
        temp = ((temp * trapez->sampling_period) - trapez->time_a);
        temp *= (trapez->time_a * trapez->acc_a);
        temp += (trapez->time_a * trapez->time_a * trapez->acc_a * 0.5) + trapez->y_0;
        g_args_contrl[0].state_desired.q = temp;
    }
    
    else if ((counter > g_args_contrl[0].arg_int[4]) && (counter <= g_args_contrl[0].arg_int[1]))
    {
        temp = counter - g_args_contrl[0].arg_int[4];
        temp = temp * trapez->sampling_period;
        g_args_contrl[0].state_desired.q_dot = trapez->speed_ss - (trapez->acc_d * temp);
        
        temp = (trapez->acc_a * trapez->time_a * temp) - (0.5 * trapez->acc_a * temp * temp);
        temp += (trapez->acc_a * trapez->time_a * trapez->time_ss);
        temp += (0.5 * trapez->acc_a * trapez->time_a * trapez->time_a);
        g_args_contrl[0].state_desired.q = temp + trapez->y_0;
    }
    
    else
    {
        g_args_contrl[0].state_desired.q_dot = 0;
        g_args_contrl[0].state_desired.q = trapez->y_f;
    }
    
    
    /* motor 2 */
    
    g_args_contrl[1].state_desired.q_dot = 0;
    
    g_args_contrl[1].state_desired.q = trapez->x_f;
}

inline void
signalgen_trapezoid_trajectcontrl_online (sg_trapez_t *trapez,
                                          csl_carriage_t *inv_kinematics)
{
    /* motor 1 */
    
    g_args_contrl[0].state_desired.q = 0;
    
    g_args_contrl[0].state_desired.q_dot = 0;
    
    /* motor 2 */
    
    g_args_contrl[1].state_desired.q = 0;
    
    g_args_contrl[1].state_desired.q_dot = 0;
}

/* TODO */
int 
bbmc_kinematics_csl_carriage_setup (void)
{
    double tmp;
    
    tmp = ENC_1_LIN_PER_ROT;
    tmp = tmp * PGH_REDUCTION_1_NOM;
    tmp = tmp / PGH_REDUCTION_1_DENOM; 
    tmp = tmp / FLYWHEEL_RADIUS_1; 
    tmp = tmp / PI_X2;
    
    g_carriage_kinematics.beta_y = tmp;
    
    tmp = ENC_2_LIN_PER_ROT;
    tmp = tmp * PGH_REDUCTION_2_NOM;
    tmp = tmp / PGH_REDUCTION_2_DENOM; 
    tmp = tmp / FLYWHEEL_RADIUS_2; 
    tmp = tmp / PI_X2;
    
    g_carriage_kinematics.beta_x = tmp;
    
    return 0;
}

static void 
contrl_reset_poscalib (bbmc_dof_state_t volatile *state, 
                       bbmc_dof_contrl_t volatile *controller)
{
    #ifdef DRIVER_MODE_VOLTAGE
    
    controller->output.value = (controller->arg_double[0]);
    
    #endif
    
    #ifdef DRIVER_MODE_CURRENT
    
    controller->output.value = (controller->arg_double[1]) 
                               * (controller->state_desired.q_dot - state->state.speed);
    
    #endif
}

static void 
contrl_test_dsr (bbmc_dof_state_t volatile *state, 
                 bbmc_dof_contrl_t volatile *controller)
{
    if (((g_isr_state.iteration_counter)%TEST_DSR_SPEED_REFRESH_COUNT) == 0)
    {
        UARTPuts("\e[2A\r\e[7C          \e[10D", -1);
        UARTPutNum((int)(state->state.speed));
        UARTPuts("\e[1B\r\e[7C          \e[10D", -1);
        UARTPutNum(state->state.count[1]);
        UARTPuts("\e[2B\r\e[6C", -1);
    }
    
    controller->output.value = controller->arg_double[0];
}

static inline void 
contrl_goto_v3 (bbmc_dof_state_t volatile *state, 
                bbmc_dof_contrl_t volatile *controller)
{
    
    /* argument map:
     * 
     *  i0: -                         , d0: position - gain
     *  i1: position - error          , d1: output   - limit (max) value
     *  i2: position - final bount    , d2: speed    - limit (max)
     *  i3: position - CC mode switch , d3: speed    - gain
     *  i4: direction flag            , d4: -
     *  i5: -                         , d5: -
     *  i6: -                         , d6: -
     *  i7: -                         , d7: -
     */
     
    #ifdef DRIVER_MODE_VOLTAGE
    
    controller->output.value = (controller->arg_double[0])*(controller->arg_int[1]);
    
    if (controller->output.value < -controller->arg_double[1])
    {
        controller->output.value = -controller->arg_double[1];
    }
    
    else if (controller->output.value > controller->arg_double[1])
    {
        controller->output.value = controller->arg_double[1];
    }
    
    else
    {
        ;
    }
    
    #endif
    
    #ifdef DRIVER_MODE_CURRENT
    
    /* CRUISE mode : distance-to-target is greater than first bound */
    if (abs(controller->arg_int[1]) > controller->arg_int[3])
    {
        controller->output.value = (controller->arg_double[3]) *
                               (controller->state_desired.q_dot - state->state.speed);
    }
    /* GOTO-POSITION mode : distance-to-target is within the first bound */
    else
    {
        controller->output.value = (controller->arg_double[0])*(controller->arg_int[1]);
    }
    
    #endif
}

static inline void
contrl_stop_immediate (bbmc_dof_state_t volatile *state, 
                       bbmc_dof_contrl_t volatile *controller)
{
    #ifdef DRIVER_MODE_VOLTAGE
    
    controller->output.value = 0;
    
    #endif
    
    #ifdef DRIVER_MODE_CURRENT
    
    int dof_id = controller->dof_id;
    
    controller->arg_double[2] += (- state->state.speed);
    
    controller->output.value = g_args_stop[dof_id].arg_double[0] * (- state->state.speed);
    
    controller->output.value += g_args_stop[dof_id].arg_double[1] * g_args_stop[dof_id].arg_double[2];
    
    #endif
}


/* TERM functions */


static void 
term_rmpi_breakaway (bbmc_dof_state_t volatile *state, 
                     bbmc_dof_contrl_t volatile *controller)
{
    if (fabs(state->state.speed) > controller->arg_int[2])
    {
        g_isr_state.termination_flag = 1;
        g_flags.isr_return = ISR_RETURN_CLEAN;
        controller->output.value = 0;
    }
}

static void 
term_rmpi_step (bbmc_dof_state_t volatile *state, 
                bbmc_dof_contrl_t volatile *controller)
{
    if ((g_isr_state.iteration_counter >= controller->arg_int[1]))
    {
        contrl_stop_immediate(state, controller);
        
        controller->arg_int[0]++;
        
        if ((controller->arg_int[0] >= MAX_STOP_COUNT) && (state->state.speed == 0))
        {
            controller->output.value = 0;
            
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_CLEAN;
        }
    }
}

static void 
term_rmpi_si (bbmc_dof_state_t volatile *state, 
                bbmc_dof_contrl_t volatile *controller)
{
    if (g_isr_state.iteration_counter >= controller->arg_int[1])
    {
        controller->arg_int[2] = 1;
        
        //contrl_stop_immediate(state, controller);
        controller->output.value = 0;
        
        controller->arg_int[0]++;
        
        if (controller->arg_int[0] >= MAX_STOP_COUNT)
        {
            controller->output.value = 0;
            
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_CLEAN;
        }
    }
}

static void 
term_reset_poscalib (bbmc_dof_state_t volatile *state, 
                     bbmc_dof_contrl_t volatile *controller)
{
    ;
}

static void 
term_test_dsr (bbmc_dof_state_t volatile *state, 
               bbmc_dof_contrl_t volatile *controller)
{
    ;
}


/* UTILS functions */
 
double 
util_strtod (const char *str, unsigned char **endptr)
{
    double number;
    int exponent;
    int negative;
    unsigned char *p = (unsigned char *) str;
    double p10;
    int n;
    int num_digits;
    int num_decimals;
    
    // Skip leading whitespace
    while (isspace(*p)) p++;
    
    // Handle optional sign
    negative = 0;
    
    switch (*p)
    {
        case '-': negative = 1; // Fall through to increment position
        case '+': p++;
    }
    
    number = 0.;
    exponent = 0;
    num_digits = 0;
    num_decimals = 0;
    
    // Process string of digits
    while (isdigit(*p))
    {
        number = number * 10. + (*p - '0');
        p++;
        num_digits++;
    }
    
    // Process decimal part
    if (*p == '.')
    {
        p++;
        
        while (isdigit(*p))
        {
            number = number * 10. + (*p - '0');
            p++;
            num_digits++;
            num_decimals++;
        }
        
        exponent -= num_decimals;
    }
    
    if (num_digits == 0)
    {
        errno = ERANGE;
        return 0.0;
    }
    
    // Correct for sign
    if (negative)
    {
        number = -number;
    }
    
    // Process an exponent string
    if (*p == 'e' || *p == 'E')
    {
        // Handle optional sign
        negative = 0;
        switch (*++p)
        {
          case '-': negative = 1;   // Fall through to increment pos
          case '+': p++;
        }
        
        // Process string of digits
        n = 0;
        while (isdigit(*p))
        {
          n = n * 10 + (*p - '0');
          p++;
        }
        
        if (negative)
        {
            exponent -= n;
        }
        else
        {
            exponent += n;
        }
    }
    
    if (exponent < DBL_MIN_EXP  || exponent > DBL_MAX_EXP)
    {
        errno = ERANGE;
        return HUGE_VAL;
    }
    
    // Scale the result
    p10 = 10.;
    n = exponent;
    
    if (n < 0)
    {
        n = -n;
    }
    
    while (n)
    {
        if (n & 1)
        {
          if (exponent < 0)
          {
            number /= p10;
          }
          
          else
          {
            number *= p10;
          }
        }
        
        n >>= 1;
        p10 *= p10;
    }
    
    if (number == HUGE_VAL)
    {
      errno = ERANGE;
    }
    
    if (endptr)
    {
        *endptr = p;
    }
    
    return number;
}

int 
util_checkpoint_yn (char *format, char *ptr)
{
    static int flag=-1;
    static int t1=-1;
    static int t2=-1;
    
    do
    {
        UARTPuts(format, -1);
        UARTGets(ptr, RX_BUFF_SIZE);
        
        t1=strcmp((const char *)ptr, "Y");
        t2=strcmp((const char *)ptr, "n");
        
    }
    while((t1 != 0) && (t2 != 0));
            
    if (t1 == 0)
    { 
        flag = 1;
    }
    
    if (t2 == 0)
    { 
        flag = 0;
    }
    
    return flag;
}

void 
util_dtoa (double f, char *buf)
{
    
    int pos=0, ix=0, dp=0, num=0, ip=0;
    
    pos = 0;
    
    if (f < 0)
    {
        buf[pos++]='-';
        f = -f;
    }
    
    dp=0;
    
    while (f >= 10.0)
    {
        f = f/10.0;
        dp++;
    }
    
    ip = dp;
    
    for (ix=1; ix < (ip + MAX_DECIMAL_DIGITS); ix++)
    {
        
            num = (int)f;
            f = f - num;
            
            if (num > 9)
            {
                buf[pos++] = '#';
            }
            
            else
            {
                /*if ((ix == (ip + MAX_DECIMAL_DIGITS - 1)) && (num >= 5)){
                    num++;
                }*/
                buf[pos++] = '0' + num;
            }
            
            if (dp == 0)
            {
                buf[pos++] = '.';
            }
            
            f = f*10.0;
            dp--;
    }
}

void 
util_delay (int count)
{
    
    static int i=0;
    
    for (i=0; i < count; i++);
    
}

/* SIGNAL GENERATORS */
 
int 
signalgen_prbs_v0 (datalog_s_t *datalog, int data_index)
{
    int temp;
    
    if (datalog == NULL)
    {
        UARTprintf("\r\nerror: signalgen_prbs_v0:datalog_ptr is NULL\r\n");
        return -1;
    }
    
    int i = 0;
    
    for (i=0; i < 20000; i++)
    {
        datalog->log[data_index].data[i] = 0;
    }
    
    /* set the switch points */
    datalog->log[data_index].data[715] = 1;
    datalog->log[data_index].data[1951] = -1;
    datalog->log[data_index].data[2540] = 1;
    datalog->log[data_index].data[2838] = -1;
    datalog->log[data_index].data[3153] = 1;
    datalog->log[data_index].data[5570] = -1;
    datalog->log[data_index].data[8436] = 1;
    datalog->log[data_index].data[9708] = -1;
    datalog->log[data_index].data[10938] = 1;
    datalog->log[data_index].data[12648] = -1;
    datalog->log[data_index].data[13115] = 1;
    datalog->log[data_index].data[13575] = -1;
    datalog->log[data_index].data[15845] = 1;
    datalog->log[data_index].data[16006] = -1;
    datalog->log[data_index].data[16983] = 1;
    datalog->log[data_index].data[18116] = -1;
    datalog->log[data_index].data[18268] = 1;
    datalog->log[data_index].data[18315] = -1;
    datalog->log[data_index].data[18680] = 1;
    datalog->log[data_index].data[19144] = -1;
    datalog->log[data_index].data[19151] = 1;
    datalog->log[data_index].data[19190] = -1;
    datalog->log[data_index].data[19298] = 1;
    datalog->log[data_index].data[19412] = 0;
    
    /* fill the rest of the signal */
    temp = 1;
    
    for (i=714; i < 19412; i++)
    {
        if (datalog->log[data_index].data[i+1] == 1)
        {
            temp = 1;
        }
        
        if (datalog->log[data_index].data[i+1] == -1)
        {
            temp = -1;
        }
        
        datalog->log[data_index].data[i] = temp;
    }
    
    return 0;
}


int 
signalgen_prbs_v1 (datalog_s_t *datalog, 
                   int *prbs_table, 
                   int duration, 
                   int idle_time, 
                   int data_index)
{
    int switch_times[SIGNAL_PRBS_V1_SWITCH_NUM];
    int switch_values[SIGNAL_PRBS_V1_SWITCH_NUM];
    
    int start_point;
    int end_point;
    int index_max;
    
    int temp;
    int i, j;
    
    if (duration > DATALOG_STATIC_DATALEN)
    {
        UARTprintf("error: signalgen_prbs_v1: prbs-v1: duration=%d exceeds system limit of %d", 
                    duration, DATALOG_STATIC_DATALEN);
        return -1;
    }
    
    /* get switch times from reference prbs_v1 table */
    for (i = 0; i < SIGNAL_PRBS_V1_SWITCH_NUM; i++)
    {
        switch_times[i] = prbs_table[2*i] * duration;
        switch_times[i] = switch_times[i] / 10000; 
        switch_times[i] = switch_times[i] + idle_time; 
        
        switch_values[i] = prbs_table[2*i+1];
    }
    
    start_point = switch_times[0];
    end_point = switch_times[SIGNAL_PRBS_V1_SWITCH_NUM-1];
    index_max = duration + idle_time;
    
    j = 0;
    temp = switch_times[0];
    
    /* prepare (initialize) signal-log */
    for (i = 0; i < index_max; i++)
    {
        if (i == temp)
        {
            datalog->log[i].data[data_index] = switch_values[j];
            j++;
            temp = switch_times[j];
        }
        
        else
        {
            datalog->log[i].data[data_index] = 0;
        }
    }
    
    temp = 1;
    
    /* create PRBS signal */
    for (i = start_point; i < end_point; i++)
    {
        if (datalog->log[i+1].data[data_index] == 1)
        {
            temp = 1;
        }
        
        if (datalog->log[i+1].data[data_index] == -1)
        {
            temp = -1;
        }
        
        datalog->log[i].data[data_index] = temp;
    }
    
     return 0;
}

int 
signalgen_prbs_v2 (datalog_s_t *datalog, 
                   int *prbs_table, 
                   int duration, 
                   int idle_time, 
                   int data_index)
{
    int switch_times[SIGNAL_PRBS_V2_SWITCH_NUM];
    int switch_values[SIGNAL_PRBS_V2_SWITCH_NUM];
    
    int start_point;
    int end_point;
    int index_max;
    
    int temp;
    int i, j;
    
    if (duration > DATALOG_STATIC_DATALEN)
    {
        UARTprintf("error: signalgen_prbs_v2: prbs-v2: duration=%d exceeds system limit of %d", 
                    duration, DATALOG_STATIC_DATALEN);
        return -1;
    }
    
    /* get switch times from reference prbs_v1 table */
    for (i = 0; i < SIGNAL_PRBS_V2_SWITCH_NUM; i++)
    {
        switch_times[i] = prbs_table[2*i] * duration;
        switch_times[i] = switch_times[i] / 10000; 
        switch_times[i] = switch_times[i] + idle_time; 
        
        switch_values[i] = prbs_table[2*i+1];
    }
    
    start_point = switch_times[0];
    end_point = switch_times[SIGNAL_PRBS_V2_SWITCH_NUM-1];
    index_max = duration + idle_time;
    
    j = 0;
    temp = switch_times[0];
    
    /* prepare (initialize) signal-log */
    for (i = 0; i < index_max; i++)
    {
        if (i == temp)
        {
            datalog->log[i].data[data_index] = switch_values[j];
            j++;
            temp = switch_times[j];
        }
        
        else
        {
            datalog->log[i].data[data_index] = 0;
        }
    }
    
    temp = 1;
    
    /* create PRBS signal */
    for (i = start_point; i < end_point; i++)
    {
        if (datalog->log[i+1].data[data_index] == 1)
        {
            temp = 1;
        }
        
        if (datalog->log[i+1].data[data_index] == -1)
        {
            temp = -1;
        }
        
        datalog->log[i].data[data_index] = temp;
    }
    
     return 0;
}

int 
signalgen_pulse_v1 (datalog_s_t *datalog, 
                   int *pulse_table, 
                   int duration, 
                   int on_time,
                   int off_time,
                   int idle_time,
                   int data_index)
{
    
    int start_point;
    int end_point;
    int pulse_num;
    int i;
    
    if (duration > DATALOG_STATIC_DATALEN)
    {
        UARTprintf("error: signalgen_prbs_v2: prbs-v2: duration=%d exceeds system limit of %d", 
                    duration, DATALOG_STATIC_DATALEN);
        return -1;
    }
    
    end_point = duration + idle_time;
    
    int temp_time = idle_time;
    
    for (i = 0; i < end_point; i++)
    {
        if ((i >= temp_time) && (i < (temp_time + on_time)))
        {
            datalog->log[i].data[data_index] = 1;
        }
        
        else
        {
            datalog->log[i].data[data_index] = 0;
        }
        
        if (i == (temp_time + on_time))
        {
            temp_time += (on_time + off_time);
        }
    }
    
     return 0;
}


/* SYSTEM FLAG FUNCTIONS */

int 
bbmc_sysflags_clear  (bbmc_system_flags_t *sysflags, const char *init_mode)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: bbmc_sysflags_clear : pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    int i = 0;
    
    /* init appropriately */
    if(!strcmp((const char *)init_mode, "-all"))
    {
        sysflags->cmdln = 0;
        sysflags->debug = 0;
        
        sysflags->perf = 0;
        sysflags->datalog = 0;
        
        sysflags->exec_checkpoint = 0;
        sysflags->contrl_run = 0;
        
        sysflags->isr_return = 0;
        sysflags->stop_immediate = 0;
    
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            sysflags->gpos_reset[i] = 0;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-isr"))
    {
        sysflags->isr_return = 0;
        sysflags->stop_immediate = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-preset"))
    {
        for (i = 0; i< BBMC_DOF_NUM; i++)
        {
            sysflags->gpos_reset[i] = 0;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-log"))
    {
        sysflags->perf = 0;
        sysflags->datalog = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-cli"))
    {
        sysflags->cmdln = 0;
        sysflags->debug = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-cmd"))
    {
        sysflags->exec_checkpoint = 0;
        sysflags->contrl_run = 0;
    }
    
    else
    {
        UARTPuts("error: sytemflags_init: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
bbmc_sysflags_set (bbmc_system_flags_t * sysflags, const char *init_mode)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("\r\nerror: bbmc_sysflags_set: pointer argument is NULL", -1);
        return -1;
    }
    
    int i = 0;
    
    /* init appropriately */
    if(!strcmp((const char *)init_mode, "-all"))
    {
        sysflags->cmdln = 1;
        sysflags->debug = 1;
        
        sysflags->perf = 1;
        sysflags->datalog = 1;
        
        sysflags->exec_checkpoint = 1;
        sysflags->contrl_run = 1;
        
        sysflags->isr_return = 1;
        sysflags->stop_immediate = 1;
    
        for (i = 1; i < BBMC_DOF_NUM; i++)
        {
            sysflags->gpos_reset[i] = 1;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-isr"))
    {
        sysflags->isr_return = 1;
        sysflags->stop_immediate = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-preset"))
    {
        for (i = 0; i< BBMC_DOF_NUM; i++)
        {
            sysflags->gpos_reset[i] = 1;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-log"))
    {
        sysflags->perf = 1;
        sysflags->datalog = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-cli"))
    {
        sysflags->cmdln = 1;
        sysflags->debug = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-cmd"))
    {
        sysflags->exec_checkpoint = 1;
        sysflags->contrl_run = 1;
    }
    
    else
    {
        UARTPuts("error: sytemflags_init: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int
bbmc_sysflags_gpreset_set (bbmc_system_flags_t * sysflags, int dof_id, int set_val)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: bbmc_sysflags_gpreset_set: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    if ((set_val != 0) || (set_val != 1))
    {
        UARTPuts("\r\nerror: bbmc_sysflags_gpreset_set: invalid set value", -1);
        return -2;
    }
    
    sysflags->gpos_reset[dof_id-1] = set_val;
    
    return 0;
}

int
bbmc_sysflags_gpreset_get (bbmc_system_flags_t * sysflags, int dof_id)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: bbmc_sysflags_gpreset_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    return sysflags->gpos_reset[dof_id-1];
}

int 
bbmc_sysflags_get (bbmc_system_flags_t * sysflags, const char * flag)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: bbmc_sysflags_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    int ret_val;
    
    if(!strcmp((const char *)flag, "cmdln"))
    {
        ret_val = sysflags->cmdln;
    }
    
    else if(!strcmp((const char *)flag, "debug"))
    {
        ret_val = sysflags->debug;
    }
    
    else if(!strcmp((const char *)flag, "dlog"))
    {
        ret_val = sysflags->datalog;
    }
    
    else if(!strcmp((const char *)flag, "checkp"))
    {
        ret_val = sysflags->exec_checkpoint;
    }
    
    else if(!strcmp((const char *)flag, "run"))
    {
        ret_val = sysflags->contrl_run;
    }
    
    else if(!strcmp((const char *)flag, "ret"))
    {
        ret_val = sysflags->isr_return;
    }
    
    else if(!strcmp((const char *)flag, "stop"))
    {
        ret_val = sysflags->stop_immediate;
    }
    
    else
    {
        UARTPuts("error: bbmc_sysflags_get: flag argument is invalid\r\n", -1);
        return -1;
    }
    
    return ret_val;
}

int 
bbmc_sysflags_print (bbmc_system_flags_t *sysflags, const char *format)
{
    if (sysflags == NULL)
    {
        UARTPuts("\r\nerror: bbmc_sysflags_print: pointer argument is NULL", -1);
        return -1;
    }
    
    UARTprintf("\r\n%sSystem Flags:\r\n", format);
    UARTprintf("\r\n%sflag        value", format);
    UARTprintf("\r\n%scmdln:         %d", format, sysflags->cmdln);
    UARTprintf("\r\n%sdebug:         %d", format, sysflags->debug);
    UARTprintf("\r\n%sperf:          %d", format, sysflags->perf);
    UARTprintf("\r\n%sdatalog:       %d", format, sysflags->datalog);
    UARTprintf("\r\n%scheckpoint:    %d", format, sysflags->exec_checkpoint);
    UARTprintf("\r\n%srun:           %d", format, sysflags->contrl_run);
    UARTprintf("\r\n%sisr_ret:       %d", format, sysflags->isr_return);
    UARTprintf("\r\n%sstop:          %d", format, sysflags->stop_immediate);
    
    int i = 0;
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        UARTprintf("\r\n%spos_reset-%d:   %d", format,(i+1), sysflags->gpos_reset[i]);
    }
    
    bbmc_cli_newlin(2);
    
    return 0;
}


/* Controller state flags */

int 
bbmc_cisr_init (bbmc_cisr_t * contrl_state)
{
    if (contrl_state == NULL)
    {
        UARTPuts("error: contrl_state_init: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    contrl_state->termination_flag = 0;
    contrl_state->iteration_counter = 0;
    contrl_state->termination_counter = 0;
    
    return 0;
}

int 
bbmc_cisr_set (bbmc_cisr_t * contrl_state, const char * set_mode, int set_val)
{
    if (contrl_state == NULL)
    {
        UARTPuts("error: contrl_state_set: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    if(!strcmp((const char *)set_mode, "termc"))
    {
        contrl_state->termination_counter = set_val;
    }
    
    else if(!strcmp((const char *)set_mode, "termf"))
    {
        contrl_state->termination_flag = set_val;
    }
    
    else if(!strcmp((const char *)set_mode, "iterc"))
    {
        contrl_state->iteration_counter = set_val;
    }
    
    else
    {
        UARTPuts("error: contrl_state_set: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
bbmc_cisr_get (bbmc_cisr_t * contrl_state, const char * get_mode)
{
    int ret_val = 0;
    
    if (contrl_state == NULL)
    {
        UARTPuts("error: contrl_state_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    if(!strcmp((const char *)get_mode, "termc"))
    {
        ret_val = contrl_state->termination_counter;
    }
    
    else if(!strcmp((const char *)get_mode, "termf"))
    {
        ret_val = contrl_state->termination_flag;
    }
    
    else if(!strcmp((const char *)get_mode, "iterc"))
    {
        ret_val = contrl_state->iteration_counter;
    }
    
    else
    {
        UARTPuts("error: contrl_state_get: get_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return ret_val;
}

int 
bbmc_cisr_print (bbmc_cisr_t *contrl_state, const char *format)
{
    if (contrl_state == NULL)
    {
        UARTPuts("error: contrl_state_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    UARTprintf("\r\n%sController State is:\r\n", format);
    
    UARTprintf("\r\n%siteration counter is at:    %d", format, contrl_state->iteration_counter);
    UARTprintf("\r\n%stermination counter is at:  %d", format, contrl_state->termination_counter);
    UARTprintf("\r\n%stermination flag is :       %d", format, contrl_state->termination_flag);
    
    bbmc_cli_newlin(2);
    
    return 0;
}

int
bbmc_cli_clear (void)
{
    UARTPuts("\033[2J\033[1;1H", -1);
    
    return 0;
}

int
bbmc_cli_newlin (unsigned int lines)
{
    int i;
    
    for (i = 0; i < lines; i++)
    {
        UARTPuts("\r\n", -1);
    }
    
    return 0;
}

int 
bbmc_cli_cursor_mv_top (void)
{
    UARTPuts("\r\n\e[H", -1);
    
    return 0;
}

int 
bbmc_cli_cursor_mv_bottom (void)
{
    bbmc_cli_cursor_mv_top();
    
    UARTPuts("\e[30B",-1);
    
    return 0;
}

/* TODO */
int 
bbmc_dof_state_print (unsigned int dof_id, const char *format)
{
    bbmc_contrl_input();
    
    UARTprintf("\r\n%sSystem State: \r\n", format);
    
    if (dof_id > BBMC_DOF_NUM)
    {
        int i;
        
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            UARTprintf("\r\n%saxis:     %d", format, (i+1));
            UARTprintf("\r\n%sposition: %d", format, (int)g_dof_state[i].state.count[1]);
            UARTprintf("\r\n%sspeed:    %d", format, (int)g_dof_state[i].state.speed);
            UARTprintf("\r\n%s", format);
        }
    }
    
    else
    {
        UARTprintf("\r\n%saxis:     %d", format, dof_id);
        UARTprintf("\r\n%sposition: %d", format, (int)g_dof_state[dof_id-1].state.count[1]);
        UARTprintf("\r\n%sspeed:    %d", format, (int)g_dof_state[dof_id-1].state.speed);
        UARTprintf("\r\n%s", format);
    }
    
    bbmc_cli_newlin(2);
    
    return 0;
}

int 
bbmc_poslim_print (const char *format)
{
    UARTprintf("\r\n%sPosition Limit Configurations: \r\n", format);

    UARTprintf("\r\n%saxis 1: global max    = %d", format, g_position_init[0].limval[1]);
    UARTprintf("\r\n%s        global min    = %d", format, g_position_init[0].limval[0]);
    UARTprintf("\r\n%saxis 2: global max    = %d", format, g_position_init[1].limval[1]);
    UARTprintf("\r\n%s        global min    = %d", format, g_position_init[1].limval[0]);
    
    UARTprintf("\r\n%saxis 1: active max    = %d", format, g_position_limits[0].limval[1]);
    UARTprintf("\r\n%s        active min    = %d", format, g_position_limits[0].limval[0]);
    UARTprintf("\r\n%saxis 2: active max    = %d", format, g_position_limits[1].limval[1]);
    UARTprintf("\r\n%s        active min    = %d", format, g_position_limits[1].limval[0]);
    
    UARTprintf("\r\n%saxis 1: home position = %d", format, g_position_home[0].limval[0]);
    UARTprintf("\r\n%saxis 2: home position = %d", format, g_position_home[1].limval[0]);
    
    bbmc_cli_newlin(2);
    
    return 0;
}

int 
bbmc_pwm_print (const char *format)
{
    int i;
    double freq;

    UARTprintf("\r\n%sSystem Output Configurations: \r\n", format);
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        sysconfig_pwm_frequency_get((i+1), &freq);
        UARTprintf("\r\n%sdof-%d pwm frequncy = %d Hz", format, (i+1), (int)freq);
    }
    
    bbmc_cli_newlin(2);
    
    return 0;
}

int 
bbmc_qei_print (const char *format)
{
    UARTprintf("\r\n%sInput System: \r\n", format);
    UARTprintf("\r\n%sQEI configurations: \r\n", format);
    
    //! must do this properly
    UARTprintf("\r\n%seQEP::1::unit_position := %d", format, 1);
    UARTprintf("\r\n%seQEP::1::clk_prescaler := %d", format, 64);
    UARTprintf("\r\n%seQEP::2::unit_position := %d", format, 1);
    UARTprintf("\r\n%seQEP::2::clk_prescaler := %d", format, 64);
    
    bbmc_cli_newlin(2);
    
    return 0;
}

int 
bbmc_timers_print (const char *format)
{
    UARTprintf("\r\n%sControl Timer Configurations: \r\n", format);
    
    int freq;
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    UARTprintf("\r\n%stimer::run  := %d Hz", format, freq);
    
    sysconfig_timer_frequency_get(TIMER_GOTO, &freq);
    UARTprintf("\r\n%stimer::goto := %d Hz", format, freq);
    
    sysconfig_timer_frequency_get(TIMER_RMPI, &freq);
    UARTprintf("\r\n%stimer::rmpi := %d Hz", format, freq);
    
    sysconfig_timer_frequency_get(TIMER_STOP, &freq);
    UARTprintf("\r\n%stimer::stop := %d Hz", format, freq);
    
    
    bbmc_cli_newlin(2);
    
    return 0;
}

/* TODO */
int 
bbmc_isr_return_value(unsigned int cmnd_ret, int ret)
{
    if (ret == (cmnd_ret + ISR_RETURN_CLEAN))
    {
        UARTPuts("\r\nController has executed succesfully.\r\n", -1);
        return (RETURN_SUCCESS);
    }
    
    else if (ret == (cmnd_ret + ISR_RETURN_GPIO_LIM))
    {
        UARTPuts("\r\nWARNING: Controller has been terminated by position limit.\r\n", -1);
        return (RETURN_ERROR_GPIO_LIM);
    }
    
    else if (ret == (cmnd_ret + ISR_RETURN_KILLSW))
    {
        UARTPuts("\r\nWARNING: Controller has been terminated by SW killswitch.\r\n", -1);
        return (RETURN_ERROR_GPIO_KILL);
    }
    
    else if (ret == (cmnd_ret + ISR_RETURN_DEBUG))
    {
        UARTPuts("\r\nController has executed DEBUG (NULL) procedure.\r\n", -1);
        return (RETURN_DEBUG);
    }
    
    else
    {
        UARTPuts("\r\nwarning: execution has been terminated by unknown event", -1);
        UARTprintf("\r\nreturn value is: %d\r\n", ret);
        return (RETURN_ERROR_UNKNOWN);
    }
    
    return 0;
}

int 
bbmc_goto_home (void)
{
    int ret;
    
    //! put these so the caller determines the destination
    g_args_goto[0].state_desired.q = g_position_home[0].limval[0];
    g_args_goto[1].state_desired.q = g_position_home[1].limval[0];
    
    bbmc_sysflags_set(&g_flags, "-cmd");
    
    UARTprintf("\r\nReseting System to starting position\r\n");
    
    ret = cmnd_goto(-1, NULL);
    
    bbmc_sysflags_clear(&g_flags, "-cmd");
    
    return bbmc_isr_return_value(RETURN_GOTO, ret);
}

//!


/* 
 * BBMC Primary Controller
 */

inline void 
run_traject (bbmc_dof_state_t volatile *state, 
             bbmc_dof_contrl_t volatile *controller)
{
    if (controller->arg_int[5] == 0)
    {
        signalgen_trapezoid_speedcontrl_online(&g_signalgen_trapezoid, &g_carriage_kinematics);
    }
    
    else
    {
        signalgen_circle_trajectcontrl_online(&g_signalgen_circle, &g_carriage_kinematics);
    }
}

inline void 
run_contrl (bbmc_dof_state_t volatile *state, 
            bbmc_dof_contrl_t volatile *controller)
{
    controller->arg_double[2] += (controller->state_desired.q - state->state.count[1]);
    
    controller->output.value = controller->arg_double[0] * 
                               (controller->state_desired.q - state->state.count[1])
                               
                             + controller->arg_double[1] * controller->arg_double[2]
                               
                             + controller->arg_double[3] * 
                               (controller->state_desired.q_dot - state->state.speed);
}

inline void 
run_term (bbmc_dof_state_t volatile *state, 
          bbmc_dof_contrl_t volatile *controller)
{
    if (g_isr_state.iteration_counter >= controller->arg_int[1])
    {
        //contrl_stop_immediate(state, controller);
        controller->output.value = 0;
        
        controller->arg_int[0]++;
        
        if (controller->arg_int[0] >= MAX_STOP_COUNT)
        {
            controller->output.value = 0;
            
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_CLEAN;
        }
    }
}

/* run controller */

static inline void 
run_timer_interrupt_off (void)
{
    DMTimerIntDisable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_IT_FLAG);
}

static inline void 
run_timer_interrupt_on (void)
{
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

static inline void 
run_poslim_stop (void)
{
    if ((g_dof_state[0].state.count[1] < g_position_limits[0].limval[0]) || 
        (g_dof_state[0].state.count[1] > g_position_limits[0].limval[1]) ||
        (g_dof_state[1].state.count[1] < g_position_limits[1].limval[0]) || 
        (g_dof_state[1].state.count[1] > g_position_limits[1].limval[1]))
    {
        
        //!TODO
        
        /*contrl_stop_immediate(&g_dof_state[0], &g_args_stop[0]);
        contrl_stop_immediate(&g_dof_state[1], &g_args_stop[1]);
        
        g_args_contrl[0].output.value = g_args_stop[0].output.value; 
        g_args_contrl[1].output.value = g_args_stop[1].output.value;*/
        
        g_args_contrl[0].output.value = 0; 
        g_args_contrl[1].output.value = 0;
        
        g_isr_state.termination_counter++;
        
        if (g_isr_state.termination_counter >= MAX_STOP_COUNT)
        {
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_GPIO_LIM;
            
            g_args_contrl[0].output.value = 0;
            g_args_contrl[1].output.value = 0;
        }
    }
}

/* TODO */
static inline void
bbmc_contrl_input (void)
{
    #ifdef INPUT_QEP_DUAL
        input_qei_dual(&g_dof_state[0]);
        input_qei_dual(&g_dof_state[1]);
    #endif
    #ifdef INPUT_QEP_STD
        input_qei_std(&g_dof_state[0]);
        input_qei_std(&g_dof_state[1]);
    #endif
    #ifdef INPUT_QEP_CAP
        input_qei_cap(&g_dof_state[0]);
        input_qei_cap(&g_dof_state[1]);
    #endif 
}

static inline void
run_output (void)
{
    if (g_flags.stop_immediate == 0)
    {
        #ifdef OUTPUT_PWM_DIFF
            output_pwm_dif(&(g_args_contrl[0].output));
            output_pwm_dif(&(g_args_contrl[1].output));
        #endif
        #ifdef OUTPUT_PWM_DIR
            output_pwm_dir(&(g_args_contrl[0].output));
            output_pwm_dir(&(g_args_contrl[1].output));
        #endif
    }
}

static inline void
run_controller (void)
{
    /* trajectory generation */
    run_traject(&g_dof_state[0], &g_args_contrl[0]);
    //run_traject(&g_dof_state[1], &g_args_contrl[1]);
    
    /* Control Algorithm  */
    run_contrl(&g_dof_state[0], &g_args_contrl[0]);
    run_contrl(&g_dof_state[1], &g_args_contrl[1]);
    
    /* algorithm termination condition */
    run_term(&g_dof_state[0], &g_args_contrl[0]);
    run_term(&g_dof_state[1], &g_args_contrl[1]);
}

static inline void 
run_datalogging (void)
{
    int counter = g_isr_state.iteration_counter;
        
    bbmc_datalog_write(counter, &g_datalog[0], &g_dof_state[0], &g_args_contrl[0]);
    bbmc_datalog_write(counter, &g_datalog[1], &g_dof_state[1], &g_args_contrl[1]);
}

void 
isr_run (void)
{
    run_timer_interrupt_off();
    
    if (g_flags.perf == 1)
    {
        PerfTimerStart();
    }
    
    /* state inputs */
    bbmc_contrl_input();
     
    /* ontroller */
    run_controller();
    
    /* global position thresholds - termination condition*/
    run_poslim_stop();
    
    /* controller output */
    run_output();
    
    /* data logging */
    run_datalogging();
    
    /* performance measure */
    if (g_flags.perf == 1)
    {
        g_log_perf[g_isr_state.iteration_counter] = PerfTimerStop();
    }
    
    g_isr_state.iteration_counter++;
    
    run_timer_interrupt_on();
}

/* goto controller */

static inline void
goto_timer_interrupt_on (void)
{
    DMTimerIntEnable(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

static inline void
goto_timer_interrupt_off (void)
{
    DMTimerIntDisable(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_3_REGS, DMTIMER_INT_OVF_IT_FLAG);
}

static inline void
goto_controller (void)
{
    /* position error */
    g_args_goto[0].arg_int[1] = 
    
    (g_args_goto[0].state_desired.q - g_dof_state[0].state.count[1]);
    
    g_args_goto[1].arg_int[1] = 
    
    (g_args_goto[1].state_desired.q - g_dof_state[1].state.count[1]);
    
    /* trajectory generator */
    traject_goto_v0(&g_dof_state[0], &g_args_goto[0]);
    traject_goto_v0(&g_dof_state[1], &g_args_goto[1]);
    
    /* Control Algorithm  */
    contrl_goto_v3(&g_dof_state[0], &g_args_goto[0]);
    contrl_goto_v3(&g_dof_state[1], &g_args_goto[1]);
}

static inline void
goto_stop (void)
{
    /*
    if ((fabs(g_args_goto[0].arg_int[1]) <= g_args_goto[0].arg_int[2]) && 
         (fabs(g_args_goto[1].arg_int[1]) <= g_args_goto[1].arg_int[2]) && 
         (g_dof_state[0].speed == 0) && 
         (g_dof_state[1].speed == 0))
    */
    
    if ((fabs(g_args_goto[0].arg_int[1]) <= g_args_goto[0].arg_int[2]))
    {
        
        contrl_stop_immediate(&g_dof_state[0], &g_args_stop[0]);
        contrl_stop_immediate(&g_dof_state[1], &g_args_stop[1]);
        
        g_args_goto[0].output.value = g_args_stop[0].output.value; 
        g_args_goto[1].output.value = g_args_stop[1].output.value;
        
        g_isr_state.termination_counter++;
        
        if (g_isr_state.termination_counter >= GOTO_REST_COUNT)
        {
            g_args_goto[0].output.value = 0;
            g_args_goto[1].output.value = 0;
            
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_CLEAN;
        }
    }
}

static inline void
goto_debug (void)
{
    if ((g_flags.debug == 1))
    {
        g_args_goto[0].output.value = 0;
        g_args_goto[1].output.value = 0;
            
        g_isr_state.termination_counter++;
        
        if (g_isr_state.termination_counter >= GOTO_DEBUG_STOP_COUNT)
        {
            g_isr_state.termination_flag = 1;
            g_flags.isr_return = ISR_RETURN_DEBUG;
        }
    }
}

static inline void
goto_output (void)
{
    if (g_flags.stop_immediate == 0)
    {
        #ifdef OUTPUT_PWM_DIFF
            output_pwm_dif(&(g_args_goto[0].output));
            output_pwm_dif(&(g_args_goto[1].output));
        #endif
        #ifdef OUTPUT_PWM_DIR
            output_pwm_dir(&(g_args_goto[0].output));
            output_pwm_dir(&(g_args_goto[1].output));
        #endif
    }
}

static inline void
goto_datalogging (void)
{
    if (g_flags.datalog == 1)
    {
        int counter = g_isr_state.iteration_counter;
        
        bbmc_datalog_write(counter, &g_datalog[0], &g_dof_state[0], &g_args_goto[0]);
        bbmc_datalog_write(counter, &g_datalog[1], &g_dof_state[1], &g_args_goto[1]);
    }
}

void 
isr_goto(void)
{
    goto_timer_interrupt_off();
    
    if (g_flags.perf == 1)
    {
        PerfTimerStart();
    }
    
    /* state inputs */
    bbmc_contrl_input();
    
    /* controller */
    goto_controller();
    
    /* position limiter */
    goto_stop();
    
    /* debug mode */
    goto_debug();
    
    /* output */
    goto_output();
    
    /* data logging */
    goto_datalogging();
    
    if (g_flags.perf == 1)
    {
        g_log_perf[g_isr_state.iteration_counter] = PerfTimerStop();
    }
    
    g_isr_state.iteration_counter++;
    
    goto_timer_interrupt_on();
}


/* rmpi controller */

static inline void
rmpi_timer_interrupt_on (void)
{
    DMTimerIntEnable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

static inline void
rmpi_timer_interrupt_off (void)
{
    DMTimerIntDisable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_IT_FLAG);
}

static inline void
rmpi_controller (void)
{
    /* trajectory generator */
    rmpi_contrl.traject_func(rmpi_state, rmpi_args);
    
    /* Control Algorithm  */
    rmpi_contrl.contrl_func(rmpi_state, rmpi_args);
    
    /* termination condition */
    rmpi_contrl.term_func(rmpi_state, rmpi_args);
}

static inline void
rmpi_poslim_stop (void)
{
    if ((rmpi_state->state.count[1] > rmpi_limits->limval[1]) || 
        (rmpi_state->state.count[1] < rmpi_limits->limval[0]))
    {
        
        contrl_stop_immediate(rmpi_state, &g_args_stop[rmpi_args->dof_id - 1]);
        
        rmpi_args->output.value = g_args_stop[rmpi_args->dof_id - 1].output.value; 
        
        g_isr_state.termination_counter++;
        
        if (g_isr_state.termination_counter >= MAX_STOP_COUNT)
        {
            g_isr_state.termination_flag = 1;
            
            rmpi_args->output.value = 0;
            g_flags.isr_return = ISR_RETURN_GPIO_LIM;
        }
    }
}

static inline void
rmpi_datalogging (void)
{
    if (g_flags.datalog == 1)
    {
        int counter = g_isr_state.iteration_counter;
        
        bbmc_datalog_write(counter, rmpi_log, rmpi_state, rmpi_args);
    }
}

void 
isr_rmpi(void)
{
    rmpi_timer_interrupt_off();
    
    if (g_flags.perf == 1)
    {
        PerfTimerStart();
    }
    
    /* state inputs - update position and speed */
    rmpi_io.input_func(rmpi_state);
    
    /* controller */
    rmpi_controller();
    
    /* position limit stop */
    if (g_flags.debug == 0)
    {
        rmpi_poslim_stop();
    }
    
    /* controller output */
    if (g_flags.stop_immediate == 0)
    {
        rmpi_io.output_func(&(rmpi_args->output));
    }
    
    /* data logging */
    rmpi_datalogging();
    
    if (g_flags.perf == 1)
    {
        g_log_perf[g_isr_state.iteration_counter] = PerfTimerStop();
    }
    
    g_isr_state.iteration_counter++;
    
    rmpi_timer_interrupt_on();
}

void 
isr_systick(void)
{
    //! this can remain like this for now, but must be updated
    //! new features will be added to systick.
    
    DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
    DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
    
    timer_ticks++;
    timer_ticking = 1;
    
    DMTimerIntEnable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

void 
isr_gpio_pos_limit(void)
{
    static int counter = 0;
    
    if ((GPIOPinIntStatus(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN) 
        >> HALL_Y_GPIO_PIN))
    {
        if (((GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN)) && 
            (g_flags.gpos_reset[0] == 0))
        {
            
            g_flags.gpos_reset[0] = 1;         /* next reset via the MAX position */
            g_flags.stop_immediate = 1;
            
            if (g_flags.debug == 1)
            {
                
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: minpos y\r\n", -1);
            }
        }
        if ((!(GPIOPinRead(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_PIN) >> HALL_Y_GPIO_PIN)) &&
            (g_flags.gpos_reset[0] == 1))
            {
            
            g_flags.gpos_reset[0] = 0;         /* next reset via the MIN position */
            g_flags.stop_immediate = 1;
            
            if (g_flags.debug == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: maxpos y\r\n", -1);
            }
        }
        
        /* clear status(masked) of interrupts */
        GPIOPinIntClear(HALL_Y_GPIO_ADDRESS, HALL_Y_GPIO_INT_LINE, HALL_Y_GPIO_PIN);
    }
    
    if ((GPIOPinIntStatus(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN)
            >> HALL_X_GPIO_PIN))
    {
        if (((GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN)) && 
            (g_flags.gpos_reset[1] == 0))
        {
            g_flags.gpos_reset[1] = 1;             /* next reset via the MAX position */
            g_flags.stop_immediate = 1;
            
            if (g_flags.debug == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: minpos x\r\n", -1);
            }
        }
        if ((!(GPIOPinRead(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_PIN) >> HALL_X_GPIO_PIN)) && 
            (g_flags.gpos_reset[1] == 1))
        {
            g_flags.gpos_reset[1] = 0;             /* next reset via the MIN position */
            g_flags.stop_immediate = 1;
            
            if (g_flags.debug == 1)
            {
                UARTPuts("\r\nDEBUG_MSG: GPIO INTERRUPT: maxpos x\r\n", -1);
            }
        }
        
        /* clear status(masked) of interrupts */
        GPIOPinIntClear(HALL_X_GPIO_ADDRESS, HALL_X_GPIO_INT_LINE, HALL_X_GPIO_PIN);
    }
    
    if (g_flags.stop_immediate == 1)
    {
        
        g_flags.stop_immediate = 1;
        g_flags.isr_return = ISR_RETURN_GPIO_LIM;
        counter= 0;
        timer_ticks = 0;
        
        DMTimerEnable(SOC_DMTIMER_5_REGS);
        
        for (;;)
        {
            
            /* state inputs */
            #ifdef INPUT_QEP_DUAL
                input_qei_dual(&g_dof_state[0]);
                input_qei_dual(&g_dof_state[1]);
            #endif
            #ifdef INPUT_QEP_STD
                input_qei_std(&g_dof_state[0]);
                input_qei_std(&g_dof_state[1]);
            #endif
            #ifdef INPUT_QEP_CAP
                input_qei_cap(&g_dof_state[0]);
                input_qei_cap(&g_dof_state[1]);
            #endif
            
            /* stopping algorithm */
            //contrl_stop_immediate(&g_dof_state[0], &g_args_stop[0]);
            //contrl_stop_immediate(&g_dof_state[1], &g_args_stop[1]);
            
            g_args_stop[0].output.value = 0;
            g_args_stop[1].output.value = 0;
            
            if ((fabs(g_dof_state[0].state.speed) <= STOP_SPEED_X) && 
                 (fabs(g_dof_state[1].state.speed) <= STOP_SPEED_X))
            {
                
                counter = timer_ticks;
                
                if (counter >= MAX_STOP_COUNT)
                {
                    g_args_stop[0].output.value = 0;
                    g_args_stop[1].output.value = 0;
                    
                    g_isr_state.termination_flag = 1;
                    break;
                }
            }
            
            /* controller output */
            #ifdef OUTPUT_PWM_DIFF
                output_pwm_dif(&(g_args_stop[0].output));
                output_pwm_dif(&(g_args_stop[1].output));
            #endif
            #ifdef OUTPUT_PWM_DIR
                output_pwm_dir(&(g_args_stop[0].output));
                output_pwm_dir(&(g_args_stop[1].output));
            #endif
            
            /* start tick timer */
            DMTimerIntEnable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
            
            while(timer_ticking == 0)
            {
                ;
            }
            
            /* stop tick timer */
            DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
            DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
            
        }
        
        /* stop tick timer */
        DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
        DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
        
        g_args_stop[0].output.value = 0;
        g_args_stop[1].output.value = 0;
        
        /* controller output */
        #ifdef OUTPUT_PWM_DIFF
            output_pwm_dif(&(g_args_stop[0].output));
            output_pwm_dif(&(g_args_stop[1].output));
        #endif
        #ifdef OUTPUT_PWM_DIR
            output_pwm_dir(&(g_args_stop[0].output));
            output_pwm_dir(&(g_args_stop[1].output));
        #endif
        
        sysconfig_pwm_disable(1);
        sysconfig_pwm_disable(2);
        
        /* stop tick timer */
        DMTimerDisable(SOC_DMTIMER_5_REGS);
    
        UARTPuts("\r\n\r\n\tGPIO pos-lim system has enacted Emergency Stop\r\n", -1);
    }
}

void 
isr_gpio_killswitch(void)
{
    static int counter = 0;
    
    if ((GPIOPinIntStatus(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, 
            KILLSWITCH_GPIO_PIN) >> KILLSWITCH_GPIO_PIN))
    {
        
        g_flags.stop_immediate = 1;
        g_flags.isr_return = ISR_RETURN_KILLSW;
        counter = 0;
        timer_ticks = 0;
        
        DMTimerEnable(SOC_DMTIMER_5_REGS);
        
        for (;;)
        {
            /* state inputs */
            #ifdef INPUT_QEP_DUAL
                input_qei_dual(&g_dof_state[0]);
                input_qei_dual(&g_dof_state[1]);
            #endif
            #ifdef INPUT_QEP_STD
                input_qei_std(&g_dof_state[0]);
                input_qei_std(&g_dof_state[1]);
            #endif
            #ifdef INPUT_QEP_CAP
                input_qei_cap(&g_dof_state[0]);
                input_qei_cap(&g_dof_state[1]);
            #endif
            
            /* stopping algorithm */
            //contrl_stop_immediate(&g_dof_state[0], &g_args_stop[0]);
            //contrl_stop_immediate(&g_dof_state[1], &g_args_stop[1]);
            
            g_args_stop[0].output.value = 0;
            g_args_stop[1].output.value = 0;
            
            if ((fabs(g_dof_state[0].state.speed) <= STOP_SPEED_X) && 
                 (fabs(g_dof_state[1].state.speed) <= STOP_SPEED_X))
            {
                
                counter = timer_ticks;
                
                if (counter >= MAX_STOP_COUNT)
                {
                    g_args_stop[0].output.value = 0;
                    g_args_stop[1].output.value = 0;
                    
                    g_isr_state.termination_flag = 1;
                    break;
                }
            }
            
            /* controller output */
            #ifdef OUTPUT_PWM_DIFF
                output_pwm_dif(&(g_args_stop[0].output));
                output_pwm_dif(&(g_args_stop[1].output));
            #endif
            #ifdef OUTPUT_PWM_DIR
                output_pwm_dir(&(g_args_stop[0].output));
                output_pwm_dir(&(g_args_stop[1].output));
            #endif
            
            /* start tick timer */
            DMTimerIntEnable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
            
            while(timer_ticking == 0)
            {
                ;
            }
            
            /* stop tick timer */
            DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
            DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
            
        }
        
        /* stop tick timer */
        DMTimerIntDisable(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_EN_FLAG);
        DMTimerIntStatusClear(SOC_DMTIMER_5_REGS, DMTIMER_INT_OVF_IT_FLAG);
        
        /* stop tick timer */
        DMTimerDisable(SOC_DMTIMER_5_REGS);
        
        g_args_stop[0].output.value = 0;
        g_args_stop[1].output.value = 0;
        
        /* controller output */
        #ifdef OUTPUT_PWM_DIFF
            output_pwm_dif(&(g_args_stop[0].output));
            output_pwm_dif(&(g_args_stop[1].output));
        #endif
        #ifdef OUTPUT_PWM_DIR
            output_pwm_dir(&(g_args_stop[0].output));
            output_pwm_dir(&(g_args_stop[1].output));
        #endif
        
        sysconfig_pwm_disable(1);
        sysconfig_pwm_disable(2);
        
        UARTPuts("\r\nWARNING: Killswitch has enacted Emergency Stop\r\n", -1);
        
        if (g_flags.debug == 1)
        {
            UARTPuts("DEBUG_MSG: Killswitch Engage!!\r\n", -1);
        }
    }
    
    /* clear status (masked) of interrupts */
    GPIOPinIntClear(KILLSWITCH_GPIO_ADDRESS, KILLSWITCH_GPIO_INT_LINE, KILLSWITCH_GPIO_PIN);
}



/* 
 * ISR Handler Functions 
 */

int 
func_run(void)
{
    if (g_flags.perf == 1)
    {
        bbmc_perf_init();
    }
    
    bbmc_cisr_init(&g_isr_state);
    sysconfig_contrl_stop_init();
    
    sysconfig_qei_data_init(TIMER_RUN, 1);
    sysconfig_qei_data_init(TIMER_RUN, 2);
    
    UARTPuts("\r\n\r\nEXECUTING..\r\n", -1);
    
    sysconfig_poslim_enable(BBMC_DOF_NUM + 1);
    
    sysconfig_killswitch_enable();
    
    sysconfig_pwm_enable(1);
    sysconfig_pwm_enable(2);
    
    sysconfig_timer_enable(TIMER_RUN);
    
    while(!g_isr_state.termination_flag)
    {
        ;
    }
    
    sysconfig_timer_disable(TIMER_RUN);
    
    sysconfig_pwm_disable(1);
    sysconfig_pwm_disable(2);
    
    sysconfig_killswitch_disable();
    
    sysconfig_poslim_disable(BBMC_DOF_NUM + 1);
    
    UARTPuts("\r\nEOR\r\n", -1);
    
    UARTprintf("\r\nTotal Control-Loop Iterations =  %d\r\n", (int)g_isr_state.iteration_counter);
    
    if (g_flags.perf == 1)
    {
        bbmc_perf_print();
    }
    
    
    
    if (g_flags.datalog == 1)
    {
        char buff[8];
        int test = util_checkpoint_yn("\r\nPrint datalog? [Y/n]: ", buff);
        
        if (test == 1)
        {
            int range[4];
            
            range[0] = 0;
            range[1] = g_isr_state.iteration_counter;
            range[2] = 0;
            range[3] = DATALOG_STATIC_DATASIZE;
            
            datalog_s_print(&g_datalog[0], range);
            datalog_s_print(&g_datalog[1], range);
        }
    }
    
    return (RETURN_RUN + g_flags.isr_return);    
}

int 
func_goto(void)
{
    if (g_flags.perf == 1)
    {
        bbmc_perf_init();
    }
        
    bbmc_cisr_init(&g_isr_state);
    sysconfig_contrl_stop_init();
    
    sysconfig_qei_data_init(TIMER_GOTO, 1);
    sysconfig_qei_data_init(TIMER_GOTO, 2);
    
    UARTPuts("\r\n\r\nEXECUTING..\r\n", -1);
    
    sysconfig_poslim_enable(BBMC_DOF_NUM + 1);
    
    sysconfig_killswitch_enable();
    
    sysconfig_pwm_enable(1);
    sysconfig_pwm_enable(2);
    
    sysconfig_timer_enable(TIMER_GOTO);
    
    while(!g_isr_state.termination_flag)
    {
        ;
    }
    
    sysconfig_timer_disable(TIMER_GOTO);
    
    sysconfig_pwm_disable(1);
    sysconfig_pwm_disable(2);
    
    sysconfig_killswitch_disable();
    
    sysconfig_poslim_disable(BBMC_DOF_NUM + 1);
    
    UARTPuts("\r\nEOR\r\n", -1);
    
    UARTprintf("\r\nTotal Control-Loop Iterations =  %d\r\n",
               (int)g_isr_state.iteration_counter);
    
    if (g_flags.perf == 1)
    {
        bbmc_perf_print();
    }
    
    if (g_flags.datalog == 1)
    {
        char buff[8];
        int test = util_checkpoint_yn("\r\nPrint datalog? [Y/n]: ", buff);
        
        if (test == 1)
        {
            int range[4];
            
            range[0] = 0;
            range[1] = g_isr_state.iteration_counter;
            range[2] = 0;
            range[3] = DATALOG_STATIC_DATASIZE;
            
            datalog_s_print(&g_datalog[0], range);
            datalog_s_print(&g_datalog[1], range);
        }
    }
    
    return (RETURN_GOTO + g_flags.isr_return);
}

int 
func_rmpi(unsigned int rmpi_dof)
{
    if (g_flags.perf == 1)
    {
        bbmc_perf_init();
    }
    
    bbmc_cisr_init(&g_isr_state);
    sysconfig_contrl_stop_init();
    
    sysconfig_qei_data_init(TIMER_RMPI, rmpi_dof);
    
    UARTPuts("\r\n\r\nEXECUTING..\r\n", -1);
    
    sysconfig_poslim_enable(rmpi_dof);
    
    sysconfig_killswitch_enable();
    
    sysconfig_pwm_enable(rmpi_dof);
    
    sysconfig_timer_enable(TIMER_RMPI);
    
    IntMasterIRQEnable();
      
    while(!g_isr_state.termination_flag)
    {
        ;
    }
    
    sysconfig_timer_disable(TIMER_RMPI);
    
    sysconfig_pwm_disable(rmpi_dof);
    
    sysconfig_killswitch_disable();
    
    sysconfig_poslim_disable(rmpi_dof);
    
    UARTPuts("\r\nEOR\r\n", -1);
    
    if (g_flags.perf == 1)
    {
        bbmc_perf_print();
    }
    
    if (g_flags.datalog == 1)
    {
        char buff[8];
        int test = util_checkpoint_yn("\r\nPrint datalog? [Y/n]: ", buff);
        
        if (test == 1)
        {
            int range[4];
            
            range[0] = 0;
            range[1] = g_isr_state.iteration_counter;
            range[2] = 0;
            range[3] = DATALOG_STATIC_DATASIZE;
            
            datalog_s_print(rmpi_log, range);
        }
    }
    
    return (RETURN_RMPI + g_flags.isr_return);   
}



/*
 * BBMC infrastructure function definitions.
 */

/* perf fucntions */

void 
bbmc_perf_init(void)
{
    
    int i = 0;
    
    for (i = 0; i < MAX_DURATION_COUNT; i++)
    {
        g_log_perf[i] = 0;
    }
}

void 
bbmc_perf_print(void)
{   
    int i = 0;
    
    UARTPuts("\r\nPerformance Log: \r\n", -1);
    
    for(i = 0; i < g_isr_state.iteration_counter; ++i){
        
        UARTprintf("%d\r\n", (int)(g_log_perf[i]));
    }
    
    UARTprintf("\r\ng_isr_state.iteration_counter = %d\r\n", (int)g_isr_state.iteration_counter);
}

/* datalog functions */

int
datalog_s_setup (datalog_s_t volatile *datalog_ptr)
{
    /* check for NULL pointer and return error if needed */
    if (datalog_ptr == NULL)
    {
        UARTPuts("error: datalog_s_setup: datalog_ptr is null\r\n", -1);
        return -1;
    }
    
    /* setup the datalog dimensions */
    datalog_ptr->d_size = DATALOG_STATIC_DATASIZE;
    datalog_ptr->l_size = DATALOG_STATIC_DATALEN;
    
    /* initialize indeces */
    datalog_ptr->d_index = 0;
    datalog_ptr->l_index = 0;
    
    return 0;
}

int 
datalog_s_init(datalog_s_t volatile *datalog_ptr, data_t init_val)
{
    int i = 0, j = 0;
    
    if(datalog_ptr == NULL)
    {
        UARTprintf("error: datalog_s_init: datalog_ptr is null\r\n");
        return -1;
    }
    
    int size = datalog_ptr->d_size;
    int len = datalog_ptr->l_size;
    
    for (j = 0; j < len; j++)
    {
        for (i=0; i < size; i++)
        {
            datalog_ptr->log[j].data[i] = init_val;
        }
    }
    
    return 0;
}


int 
datalog_s_print(datalog_s_t volatile *datalog_ptr, int range_indeces[4])
{
    /* check for NULL pointer and return error if needed */
    if (datalog_ptr == NULL)
    {
        UARTPuts("error: datalog_s_print: datalog_ptr is null\r\n", -1);
        return -1;
    }
    
    /* get the datalog dimensions */
    int size = datalog_ptr->d_size;
    int len = datalog_ptr->l_size;
    
    int l_index_s;
    int l_index_f;
    int d_index_s;
    int d_index_f;
    
    /* if range_indecs is given as NULL then print the entire datalog */
    if (range_indeces == NULL)
    {
        l_index_s = 0;
        l_index_f = len - 1;
        d_index_s = 0;
        d_index_f = size - 1;
    }
    
    else
    {
        /* check to see of the range indeces are give correctly - i.e. increasing*/
        if ((range_indeces[1] < range_indeces[0]) || (range_indeces[3] < range_indeces[2]))
        {
            UARTPuts("error: datalog_s_print: range_indeces: index_s > index_f\r\n", -1);
            return -1;
        }
        
        /* check if the range arguments are valid */
        if ((range_indeces[0] > len) || (range_indeces[1] > len) || 
            (range_indeces[2] > size) || (range_indeces[3] > size))
        {
            UARTPuts("\r\nerror: datalog_s_print: range_indeces: some indeces exceed datalog bounds.", -1);
            return -1;
        }
        
        l_index_s = range_indeces[0];
        l_index_f = range_indeces[1];
        d_index_s = range_indeces[2];
        d_index_f = range_indeces[3];
    }
    
    /* for-loop indeces: j for len, i for size */
    int i = 0;
    int j = 0;
    
    UARTPuts("\r\n\r\nDATALOG_BEGIN:\r\n\r\n", -1);
    
    /* print the data header */
    UARTPuts("index,", -1);
    
    for (i = d_index_s; i < d_index_f; i++)
    {
        UARTPuts("data", -1);
        UARTPutNum(i);
        
        if (i == (d_index_f - 1))
        {
            UARTPutc('\r');
            UARTPutc('\n');
        }
        else
        {
            UARTPutc(',');
        }       
    }
    
    /* print the datalog data elements */
    for (j = l_index_s; j < l_index_f; j++)
    {
        UARTPutNum(j);
        UARTPutc(',');
        
        for (i = d_index_s; i < d_index_f; i++)
        {
            if (i == (d_index_f - 1))
            {
                UARTPutDouble(datalog_ptr->log[j].data[i]);
                UARTPutc('\r');
                UARTPutc('\n');
            }
            else
            {
                UARTPutDouble(datalog_ptr->log[j].data[i]);
                UARTPutc(',');
            }
        }
    }
    
    UARTPuts("\r\nDATALOG_END\r\n\r\n", -1);
    
    return 0;
}

/* TODO */
inline void
bbmc_datalog_write (unsigned int index,
                      datalog_s_t volatile *datalog, 
                      bbmc_dof_state_t volatile *state,
                      bbmc_dof_contrl_t volatile *contrl)
{
    datalog->log[index].data[0] = state->state.status;
    datalog->log[index].data[1] = state->state.count[1];
    datalog->log[index].data[2] = state->state.speed;
    datalog->log[index].data[3] = contrl->state_desired.q;
    datalog->log[index].data[4] = contrl->state_desired.q_dot;
    datalog->log[index].data[5] = contrl->output.value;
}


/*
 * Command Line Functions
 */

int 
cmnd_run_position_init (unsigned int pos_y, unsigned int pos_x)
{
    
    if (g_flags.debug == 0)
    {
        if ((pos_y <= g_position_limits[0].limval[0]) || 
            (pos_y >= g_position_limits[0].limval[1]) || 
            (pos_x <= g_position_limits[1].limval[0]) || 
            (pos_x >= g_position_limits[1].limval[1]))
        {
            UARTPuts("\r\nerror: cmnd_run_position_init: invalid init coordinates", -1);
            return -1;
        }
    }
    
    sysconfig_position_set(1, pos_y);
    sysconfig_position_set(2, pos_x);
    
    return 0;
}

int
cmnd_run_trapezoid_default (void)
{
    double tmp;
    int freq;
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    
    tmp = freq;
    
    g_signalgen_trapezoid.sampling_period = 1 / tmp;
    
    /* TRAJECTORY SETUP */
    
    g_signalgen_trapezoid.time_a = 0.3;
    g_signalgen_trapezoid.time_d = g_signalgen_trapezoid.time_a;
    
    g_signalgen_trapezoid.y_0 = RUN_POSINIT_Y;
    g_signalgen_trapezoid.x_0 = RUN_POSINIT_X;
    
    g_signalgen_trapezoid.y_f = RUN_POSINIT_Y + 2 * g_carriage_kinematics.beta_y;
    g_signalgen_trapezoid.x_f = RUN_POSINIT_X;
    
    g_signalgen_trapezoid.speed_ss = 0.10 * g_carriage_kinematics.beta_y;
    
    g_signalgen_trapezoid.acc_a = g_signalgen_trapezoid.speed_ss / g_signalgen_trapezoid.time_a;
    g_signalgen_trapezoid.acc_d = g_signalgen_trapezoid.acc_a;
    
    g_signalgen_trapezoid.time_s = 0.5;
    g_signalgen_trapezoid.time_current = 0;
    
    g_signalgen_trapezoid.time_f = g_signalgen_trapezoid.time_s;
    g_signalgen_trapezoid.time_f += g_signalgen_trapezoid.time_a + g_signalgen_trapezoid.time_d; 
    
    tmp = g_signalgen_trapezoid.y_f - g_signalgen_trapezoid.y_0;
    tmp = tmp / g_signalgen_trapezoid.speed_ss;
    tmp = tmp - g_signalgen_trapezoid.time_a;
    
    g_signalgen_trapezoid.time_ss = tmp;
    
    g_signalgen_trapezoid.time_f += tmp;
    
    /* set duration counter */
    tmp = freq * g_signalgen_trapezoid.time_f;
    
    g_args_contrl[0].arg_int[1] = (int)tmp;
    g_args_contrl[1].arg_int[1] = (int)tmp;
    
    /* time checkpoints for traject-gen */
    g_args_contrl[0].arg_int[2] = g_signalgen_trapezoid.time_s * freq;
    
    g_args_contrl[0].arg_int[3] = (g_signalgen_trapezoid.time_a * freq) + g_args_contrl[0].arg_int[2];
    
    g_args_contrl[0].arg_int[4] = g_args_contrl[0].arg_int[3] + (g_signalgen_trapezoid.time_ss * freq);
    
    g_args_contrl[0].arg_int[5] = 0;
    g_args_contrl[1].arg_int[5] = 0;
    
    UARTPuts("\r\ncmnd_run: speed trajectory has been set to: default trapezoid", -1);
    
    //! TODO: create this
    //signalgen_trapezoid_generic_setup(&g_signalgen_trapezoid);
    
    return 0;
}

int
cmnd_run_sinusoid_default (void)
{
    double tmp;
    int freq;
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    
    tmp = freq;
    
    g_signalgen_sinusoid.sine.sampling_period = 1 / tmp;
    
    /* TRAJECTORY SETUP */
    
    
    //! TODO: fill this
    
    g_signalgen_sinusoid.sine.frequency = 1;
    
    tmp = freq / g_signalgen_sinusoid.sine.frequency;
    tmp *= 2;
    
    g_args_contrl[0].arg_int[1] = (int)tmp;
    g_args_contrl[1].arg_int[1] = (int)tmp;
    
    //! TODO: create this
    //signalgen_sinusoid_generic_setup (&g_signalgen_sinusoid);
    
    UARTPuts("\r\ncmnd_run: state trajectory has been set to: default sinusoid", -1);
    
    //! TODO: reset this to zero
    return -1;
}

int
cmnd_run_circle_default (void)
{
    double tmp;
    int freq;
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    
    tmp = freq;
    
    g_signalgen_circle.sine.sampling_period = 1 / tmp;
    
    /* TRAJECTORY SETUP */
    
    g_signalgen_circle.radius = 0.20;
    g_signalgen_circle.sine.frequency = 0.15;
    
    g_signalgen_circle.y_0 = RUN_POSINIT_Y;
    g_signalgen_circle.x_0 = RUN_POSINIT_X;
    
    g_signalgen_circle.y_c = g_signalgen_circle.y_0 + (g_signalgen_circle.radius * g_carriage_kinematics.beta_y);
    g_signalgen_circle.x_c = g_signalgen_circle.x_0;
    
    tmp = freq / g_signalgen_circle.sine.frequency;
    tmp *= 2;
    
    g_args_contrl[0].arg_int[1] = (int)tmp;
    g_args_contrl[1].arg_int[1] = (int)tmp;
    
    g_args_contrl[0].arg_int[5] = 1;
    g_args_contrl[1].arg_int[5] = 1;
    
    signalgen_circle_generic_setup (&g_signalgen_circle);
    
    UARTPuts("\r\ncmnd_run: state trajectory has been set to: default circle", -1);
    
    return 0;
}

int
cmnd_run_control_pid_default (void)
{
    /* init controller P-gains */
    g_args_contrl[0].arg_double[0] = 0.008;
    g_args_contrl[1].arg_double[0] = 0.01;
    
    /* init controller I-gains */
    g_args_contrl[0].arg_double[1] = 2E-6;
    g_args_contrl[1].arg_double[1] = 0;
    
    /* integral sum init */
    g_args_contrl[0].arg_double[2] = 0;
    g_args_contrl[1].arg_double[2] = 0;
    
    /* init controller D-gains */
    g_args_contrl[0].arg_double[3] = 0.005;
    g_args_contrl[1].arg_double[3] = 0.0001;
    
    /* init algorithm termination counter */
    g_args_contrl[0].arg_int[0] = 0;
    g_args_contrl[1].arg_int[0] = 0;
    
    return 0;
}

int 
cmnd_run_config_args (int argc, char *argv[])
{
    int ret = 0;
    int i;
    int contrl_config = 0;
    int traject_config = 0;
    int traject_type = 0;
    
    
    for(i = 1; i < argc; i++)
    {
        if (!strcmp((const char *)argv[i],"-trapez"))
        {
            traject_type = 0;
        }
        
        else if (!strcmp((const char *)argv[i],"-circle"))
        {
            traject_type = 1;
        }
        
        else if (!strcmp((const char *)argv[i],"-contrl"))
        {
            //! TODO: generalize this
            ret =  cmnd_run_control_pid_config();
            contrl_config = 1;
        }
        
        else if (!strcmp((const char *)argv[i],"-traject"))
        {
            //! TODO: generalize this
            
            if (traject_type == 0)
            {
                ret = cmnd_run_trapezoid_config();
            }
            
            /*else if (traject_type == 1)
            {
                ret = cmnd_run_sinusoid_config();
            }*/
            
            else
            {
                ret = cmnd_run_circle_config();
            }
        
            traject_config = 1;
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_run_config_args: invalid option argument.\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT);
        }
    }
    
    if (contrl_config == 0)
    {
        cmnd_run_control_pid_default();
    }
    
    if (traject_config == 0)
    {
        if (traject_type == 0)
        {
            cmnd_run_trapezoid_default();
        }
        
        /*else if (traject_type == 1)
        {
            cmnd_run_sinusoid_default();
        }*/
        
        else
        {
            cmnd_run_circle_default();
        }
    }
    
    return ret;
}

int 
cmnd_run_control_pid_config (void)
{
    char buff[RX_BUFF_SIZE];
    double tmp;
    
    /* CONTROLLER SETUP */
    UARTPuts("\r\n\r\n - PID Controller Gains - \r\n", -1);
    
    UARTPuts("\r\nAxis-Y: \r\n", -1);
    
    UARTPuts("\r\n  speed-P:  ", -1);
    UARTPuts("\r\n  speed-I:  ", -1);
    UARTPuts("\r\n  speed-D:  ", -1);
    
    UARTPuts("\r\e[2A\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\n\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[0].arg_double[0] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[0].arg_double[1] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
     g_args_contrl[0].arg_double[3] = tmp;
    
    UARTPuts("\r\n\r\nAxis-X: \r\n", -1);
    
    UARTPuts("\r\n  speed-P:  ", -1);
    UARTPuts("\r\n  speed-I:  ", -1);
    UARTPuts("\r\n  speed-D:  ", -1);
    
    UARTPuts("\r\e[2A\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\n\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[1].arg_double[0] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[1].arg_double[1] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\nerror: cmnd_run_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
     g_args_contrl[1].arg_double[3] = tmp;
    
    /* integral sum init */
    g_args_contrl[0].arg_double[2] = 0;
    g_args_contrl[1].arg_double[2] = 0;
    
    /* init algorithm duration counter */
    g_args_contrl[0].arg_int[0] = 0;
    g_args_contrl[1].arg_int[0] = 0;
    
    return 0;
}

int 
cmnd_run_circle_config (void)
{
    char buff[RX_BUFF_SIZE];
    double tmp;
    int freq;
    
    /* TRAJECTORY SETUP */
    UARTPuts("\r\nCircle Trajectory Parameters: \r\n", -1);
    
    UARTPuts("\r\n  Radius:     ", -1);
    UARTPuts("\r\n  Frequency:  ", -1);
    
    UARTPuts("\r\e[A\e[13C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 0.3))
    {
        UARTPuts("\r\n\n\n\nerror: cmnd_run_trajectory_config: acceptable radius range is [0,0.3]", -1);
        return -1;
    }
    
    g_signalgen_circle.radius = tmp;
    
    UARTPuts("\r\e[B\e[13C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0.01) || (tmp > 2))
    {
        UARTPuts("\r\n\n\nerror: cmnd_run_trajectory_config: acceptable frequency range is [0.01, 2]", -1);
        return -1;
    }
    
    g_signalgen_circle.sine.frequency = tmp;
    
    
    bbmc_cli_newlin(2);
    
    //!TODO
    g_signalgen_circle.y_0 = RUN_POSINIT_Y;
    g_signalgen_circle.x_0 = RUN_POSINIT_X;
    
    cmnd_run_position_init(RUN_POSINIT_Y, RUN_POSINIT_X);
        
    //!TODO
    g_signalgen_circle.y_c = g_signalgen_circle.y_0 + 
                             (g_signalgen_circle.radius * g_carriage_kinematics.beta_y);
                             
    g_signalgen_circle.x_c = g_signalgen_circle.x_0;
    
    
    /* Process Configurations */
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    tmp = freq;
    g_signalgen_circle.sine.sampling_period = 1 / tmp;
    
    tmp = freq / g_signalgen_circle.sine.frequency;
    tmp *= 2;
    
    g_args_contrl[0].arg_int[1] = (int)tmp;
    g_args_contrl[1].arg_int[1] = (int)tmp;
    
    g_args_contrl[0].arg_int[5] = 1;
    g_args_contrl[1].arg_int[5] = 1;
    
    signalgen_circle_generic_setup (&g_signalgen_circle);
    
    return 0;
}

int 
cmnd_run_sinusoid_config (void)
{
    char buff[RX_BUFF_SIZE];
    double tmp;
    int freq;
    
    /* TRAJECTORY SETUP */
    UARTPuts("\r\nSinusoid Trajectory Parameters: \r\n", -1);
    
    UARTPuts("\r\n  Heading   :  ", -1);
    UARTPuts("\r\n  Amplitude :  ", -1);
    UARTPuts("\r\n  Frequency :  ", -1);
    
    UARTPuts("\r\e[2A\e[15C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    //! TODO
    return -1;
}

int 
cmnd_run_trapezoid_config (void)
{
    char buff[RX_BUFF_SIZE];
    double tmp;
    int freq;
    
    /* TRAJECTORY SETUP */
    UARTPuts("\r\nSTrapezoid Trajectory Parameters: \r\n", -1);
    
    UARTPuts("\r\n  Acceleration time  :  ", -1);
    UARTPuts("\r\n  Total Distance     :  ", -1);
    UARTPuts("\r\n  Steady-State Speed :  ", -1);
    
    UARTPuts("\r\e[2A\e[24C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp <= 0) || (tmp > 5))
    {
        UARTPuts("\r\n\n\n\n\nerror: cmnd_run_trapezoid_config: acceleration time must be in range (0, 5]", -1);
        return -1;
    }
    
    g_signalgen_trapezoid.time_a = tmp;
    
    UARTPuts("\r\e[B\e[24C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp <= 0) || (tmp > 4))
    {
        UARTPuts("\r\n\n\n\nerror: cmnd_run_trapezoid_config: Total distance range is (0, 4]", -1);
        return -1;
    }
    
    g_signalgen_trapezoid.y_f = RUN_POSINIT_Y + tmp * g_carriage_kinematics.beta_y;
    
    UARTPuts("\r\e[B\e[24C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0.001) || (tmp > 0.70))
    {
        UARTPuts("\r\n\n\nerror: cmnd_run_trapezoid_config: acceptable speed range is [0.001, 0.70]", -1);
        return -1;
    }
    
    g_signalgen_trapezoid.speed_ss = tmp * g_carriage_kinematics.beta_y;
    
    bbmc_cli_newlin(2);
    
    /* Process Configurations */
    
    sysconfig_timer_frequency_get(TIMER_RUN, &freq);
    tmp = freq;
    g_signalgen_trapezoid.sampling_period = 1 / tmp;
    
    g_signalgen_trapezoid.time_d = g_signalgen_trapezoid.time_a;
    
    g_signalgen_trapezoid.y_0 = RUN_POSINIT_Y;
    g_signalgen_trapezoid.x_0 = RUN_POSINIT_X;
    
    g_signalgen_trapezoid.x_f = RUN_POSINIT_X;
    
    g_signalgen_trapezoid.acc_a = g_signalgen_trapezoid.speed_ss / g_signalgen_trapezoid.time_a;
    g_signalgen_trapezoid.acc_d = g_signalgen_trapezoid.acc_a;
    
    g_signalgen_trapezoid.time_s = 0.5;
    g_signalgen_trapezoid.time_current = 0;
    
    g_signalgen_trapezoid.time_f = g_signalgen_trapezoid.time_s;
    g_signalgen_trapezoid.time_f += g_signalgen_trapezoid.time_a + g_signalgen_trapezoid.time_d; 
    
    tmp = g_signalgen_trapezoid.y_f - g_signalgen_trapezoid.y_0;
    tmp = tmp / g_signalgen_trapezoid.speed_ss;
    tmp = tmp - g_signalgen_trapezoid.time_a;
    
    g_signalgen_trapezoid.time_ss = tmp;
    
    g_signalgen_trapezoid.time_f += tmp;
    
    /* set duration counter */
    tmp = freq * g_signalgen_trapezoid.time_f;
    
    g_args_contrl[0].arg_int[1] = (int)tmp;
    g_args_contrl[1].arg_int[1] = (int)tmp;
    
    /* time checkpoints for traject-gen */
    g_args_contrl[0].arg_int[2] = g_signalgen_trapezoid.time_s * freq;
    
    g_args_contrl[0].arg_int[3] = (g_signalgen_trapezoid.time_a * freq) + g_args_contrl[0].arg_int[2];
    
    g_args_contrl[0].arg_int[4] = g_args_contrl[0].arg_int[3] + (g_signalgen_trapezoid.time_ss * freq);
    
    g_args_contrl[0].arg_int[5] = 0;
    g_args_contrl[1].arg_int[5] = 0;
    
    return 0;
}

int 
cmnd_run(int argc, char *argv[])
{
    //UARTprintf("@%d\r\n", __LINE__);
    
    static char *run_format = "\r\nProceed with RUN? [Y/n]: ";
    
    char run_buff[RX_BUFF_SIZE];
    int run_ret = 0;
    
    /* Online Configurations Here */
    if (argc > 1)
    {
        run_ret = cmnd_run_config_args(argc, argv);
        
        if (run_ret != 0)
        {
            return run_ret;
        }
    }
    
    /* Place Default Configurations Here */
    else
    {
        run_ret = cmnd_run_trapezoid_default();
        
        if (run_ret != 0)
        {
            return run_ret;
        }
        
        cmnd_run_control_pid_default();
    }
    
    run_ret = cmnd_run_position_init(RUN_POSINIT_Y, RUN_POSINIT_X);
        
    if (run_ret != 0)
    {
        return run_ret;
    }
    
    /* Proceed with execution */
    
    g_flags.contrl_run = util_checkpoint_yn(run_format, run_buff);
    
    if (g_flags.contrl_run == 1)
    {
        if (g_flags.exec_checkpoint == 0)
        {
            UARTPuts("\r\nPress any key to start execution...", -1);
            UARTGets(run_buff,  2);
        }
        
        //!
        bbmc_sysflags_clear(&g_flags, "-isr");
        g_flags.datalog = 1;
        
        run_ret = func_run();
        
        return bbmc_isr_return_value(RETURN_RUN, run_ret);
    }
    
    else
    {
        UARTPuts("\r\nwarning: Execution has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}


/* cmnd_datalog functions */

int 
cmnd_datalog_args (int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int i;
    
    args->arg_int[2] = -1;
    
    if (argc > 2)
    {
        int dof_id = atoi(argv[2]);
        
        if ((dof_id > BBMC_DOF_NUM) || (dof_id <= 0))
        {
            UARTPuts("\r\nerror: cmnd_datalog: Invalid DOF-id argument.\r\n", -1);
            return (RETURN_DATALOG + RETURN_ERROR_INVALID_ARG);
        }
        
        args->arg_int[2] = dof_id;
    }
    
    args->arg_int[0] = 0;
    args->arg_int[1] = DATALOG_STATIC_DATASIZE - 1;
    
    for (i = 3; i < argc; i++)
    {
        if (!strcmp((const char *)argv[i],"-idx"))
        {
            args->arg_int[1] = atoi(argv[i+1]);
            args->arg_int[0] = args->arg_int[1];
            
            if (args->arg_int[1] >= DATALOG_STATIC_DATALEN)
            {
                UARTPuts("\r\nerror: cmnd_datalog: Invalid value specified for max index.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_datalog: Invalid argument option.\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT); 
        }
    }

    return 0;
}

int 
cmnd_datalog(int argc, char *argv[])
{
    if (argc > 10)
    {
        UARTPuts("\r\nerror: cmnd_datalog: Too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    
    else if (argc > 1)
    {
        int dl_ret;
        int print_range[4] = {0};
        bbmc_cmd_args_t args;
        
        dl_ret = cmnd_datalog_args(argc, argv, &args);
        
        if (dl_ret < 0)
        {
            return dl_ret;
        }
        
        if (!strcmp((const char *)argv[1],"reset"))
        {
            if (args.arg_int[2] < 0)
            {
                int i;
                
                for (i = 0; i < BBMC_DOF_NUM; i++)
                {
                    datalog_s_init(&g_datalog[i], 0);
                }
            }
            
            else
            {
                datalog_s_init(&g_datalog[args.arg_int[2]-1], 0);
            }
            
            UARTPuts("\r\nDatalog has been reset.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_RESET);
        }
        
        else if (!strcmp((const char *)argv[1],"print"))
        {
            if (g_isr_state.iteration_counter == 0)
            {
                UARTPuts("\r\nerror: cmnd_datalog: There is no data to transmit.\r\n", -1);
                return (RETURN_DATALOG + RETURN_ERROR_UNKNOWN);
            }
            
            print_range[0] = 0;
            print_range[1] = g_isr_state.iteration_counter;
            print_range[2] = args.arg_int[0];
            print_range[3] = args.arg_int[1];
            
            if (args.arg_int[2] < 0)
            {
                int i;
                
                for (i = 0; i < BBMC_DOF_NUM; i++)
                {
                    datalog_s_print(&g_datalog[i], print_range);
                }
            }
            
            else
            {
                datalog_s_print(&g_datalog[args.arg_int[2]-1], print_range);
            }
            
            UARTPuts("\r\nDatalog is printing on console...\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_PRINT);
        }
        
        else if (!strcmp((const char *)argv[1],"enable")){
            
            g_flags.datalog = 1;
            UARTPuts("\r\nDatalog has been enabled.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_ENABLE);
        }
        
        else if (!strcmp((const char *)argv[1],"disable"))
        {
            g_flags.datalog = 0;
            UARTPuts("\r\nDatalog has been disabled.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_DISABLE);
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_datalog: Invalid datalog function.\r\n", -1);
            return (RETURN_DATALOG + RETURN_ERROR_INVALID_ARG);
        }
    }
    
    else
    {
        UARTPuts("\r\nerror: cmnd_datalog: Not enough arguments.\r\n", -1);
        return (RETURN_ERROR_FEW_ARGS);
    }
}

int 
cmnd_perf(int argc, char *argv[])
{
    
    if (argc > 10)
    {
                
        UARTPuts("\r\nperf: error: too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    else if (argc > 1)
    {
        if (!strcmp((const char *)argv[1],"reset"))
        {
            bbmc_perf_init();
            UARTPuts("\r\nPerformance log has been reset.\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_RESET);
        }
        
        if (!strcmp((const char *)argv[1],"print"))
        {
            bbmc_perf_print();
            return (RETURN_PERF + RETURN_PERF_PRINT);
        }
        
        if (!strcmp((const char *)argv[1],"enable"))
        {
            g_flags.perf = 1;
            UARTPuts("\r\nperf_measure mode: on\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_ENABLE);
        }
        
        if (!strcmp((const char *)argv[1],"disable"))
        {
            g_flags.perf = 0;
            UARTPuts("\r\nperf_measure mode: off\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_DISABLE);
        }
    }
    
    else
    {
        UARTPuts("\r\nperf: error: not enough arguments.\r\n", -1);
        return (RETURN_ERROR_FEW_ARGS);
    }
    
    UARTPuts("\r\nperf: error: unknown execution event.\r\n", -1);
    return (RETURN_ERROR_UNKNOWN);
}


/* reset command */

int 
cmnd_reset_poscalib_args (int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int dof_id;
    
    /*  argument map:
     * 
     *  i0: -                         , d0: output - limit (max) value
     *  i1: -                         , d1: speed  - limit (max)
     *  i2: -                         , d2: -
     *  i3: -                         , d3: -
     *  i4: -                         , d4: -
     *  i5: -                         , d5: -
     *  i6: -                         , d6: -
     *  i7: -                         , d7: -
     */
    
    dof_id = atoi(argv[2]);
    
    if ((dof_id > BBMC_DOF_NUM) || (dof_id <= 0))
    {
        UARTPuts( "\r\nerror: cmnd_reset: invalid DOF-id", -1);
        return -1;
    }
    
    args->arg_uint[0] = (unsigned int)dof_id;
    
    /* if reset mode is manually determined */
    if (argc > 3)
    {
        int i;
        
        for(i = 3; i < argc; i++)
        {
            if (!strcmp((const char *)argv[i], "-min"))
            {
                g_flags.gpos_reset[dof_id-1] = 0;
            }
            
            else if (!strcmp((const char *)argv[i-1], "-max"))
            {
                g_flags.gpos_reset[dof_id-1] = 1;
            }
            /*else if (!strcmp((const char *)argv[i], "<HERE>"){
                
               <CODE HERE>; <ADD TO ARGS SET BELOW ALSO>;
            }*/
            
            else
            {
                UARTPuts("\r\nreset: error: invalid option argument; argument must be in {-min, -max}.\r\n", -1);
                return -1;
            }
        }
    }
    
    /* configure direction gain */
    if (g_flags.gpos_reset[dof_id-1] == 0)
    {
        args->arg_int[0] = -1;
    }
    
    if (g_flags.gpos_reset[dof_id-1] == 1)
    {
        args->arg_int[0] = 1;
    }
    
    /* setup control and pointers to global data */
    rmpi_state = &g_dof_state[dof_id-1];
    rmpi_args = &g_args_contrl[dof_id-1];
    rmpi_limits = &g_position_limits[dof_id-1];
    
    /* setup for calibration functionality */
    rmpi_contrl.traject_func = traject_null;
    rmpi_contrl.contrl_func = contrl_reset_poscalib;
    rmpi_contrl.term_func = term_reset_poscalib;
    
    /* setup i/o functions */
    #ifdef INPUT_QEP_DUAL
        rmpi_io.input_func = input_qei_dual;
    #endif
    #ifdef INPUT_QEP_STD
        rmpi_io.input_func = input_qei_std;
    #endif
    #ifdef INPUT_QEP_CAP
        rmpi_io.input_func = input_qei_cap;
    #endif  
    #ifdef OUTPUT_PWM_DIFF
        rmpi_io.output_func = output_pwm_dif;
    #endif
    #ifdef OUTPUT_PWM_DIR
        rmpi_io.output_func = output_pwm_dir;
    #endif
    
    return 0;
}

int 
cmnd_reset_poscalib_func (int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int ret;
    unsigned int dof_id;
    
    dof_id = args->arg_int[0];
    
    /* controller ouput limit values */
    if (dof_id == 1)
    {
        rmpi_args->arg_double[0] = args->arg_int[0] * POSCALIB_MAX_DUTY_Y;
        rmpi_args->arg_double[1] = args->arg_int[0] * POSCALIB_SPEED_GAIN_Y;
        rmpi_args->state_desired.q_dot = args->arg_int[0] * POSCALIB_SPEED_DEST_Y;
    }
    
    else if (dof_id == 2)
    {
        rmpi_args->arg_double[0] = args->arg_int[0] * POSCALIB_MAX_DUTY_X;
        rmpi_args->arg_double[1] = args->arg_int[0] * POSCALIB_SPEED_GAIN_X;
        rmpi_args->state_desired.q_dot = args->arg_int[0] * POSCALIB_SPEED_DEST_X;
    }
    
    else
    {
        ; /* this is here to support additional axes in the future */
    }
    
    /* reset required g_flags */
    g_flags.stop_immediate = 0;
    
    ret = func_rmpi(dof_id);
    
    if (ret < 0)
    {
        return ret;
    }
    
    sysconfig_position_reset(dof_id);
    
    UARTprintf("\r\nGlobal position calibration for axis-%d has completed.", 
               dof_id);
    
    return 0;
}

int 
cmnd_reset(int argc, char *argv[])
{
    static char *reset_calib_format = "\r\nProceed with Position Reset? [Y/n]: ";
    static char *reset_goto_format = "\r\nReset to HOME position? [Y/n]: ";
    
    char reset_buff[RX_BUFF_SIZE];
    int reset_ret=0;
    
    if (argc > 10)
    {
        UARTPuts("\r\nreset: error: Too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    
    else if (argc > 2)
    {
        bbmc_cmd_args_t args;
        
        /* Calibrate and Reset Position Counter - Absolute Global Position  */
        if (!strcmp((const char *)argv[1],"poscalib"))
        {
            /* Note on args: 
             * 
             * uint0 = dof_id
             * int0 = direction
             * 
             */
            
            reset_ret = cmnd_reset_poscalib_args(argc, argv, &args);
            
            if (reset_ret == -1)
            {
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            g_flags.contrl_run = util_checkpoint_yn(reset_calib_format, reset_buff);
            
            if (g_flags.contrl_run == 1)
            {
                UARTPuts("\r\nPress any key to start position calibration..", -1);
                UARTGets(reset_buff,  2);
                
                reset_ret = cmnd_reset_poscalib_func(argc, argv, &args);
                
                if (reset_ret < 0)
                {
                    return (reset_ret);
                }
            }
            
            else
            {
                UARTprintf("\r\n\tCalibration of global position has been aborted.\r\n");
                return (RETURN_ERROR_RUN_ABORT);
            }
                
            reset_ret = util_checkpoint_yn(reset_goto_format, reset_buff);
            
            if (reset_ret == 1)
            {
                bbmc_sysflags_clear (&g_flags, "-isr");
                bbmc_sysflags_clear (&g_flags, "-log");
                
                reset_ret = bbmc_goto_home();
                
                if (reset_ret != (RETURN_GOTO + ISR_RETURN_CLEAN))
                {
                    return reset_ret;
                }
            }
            
            UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
            return (RETURN_RESET + RETURN_RESET_POSCALIB);
        }
        /* Reset System State  */
        else if (!strcmp((const char *)argv[1],"systate"))
        {
            //!
            
            UARTPuts("\r\nSystem State has been reset.\r\n", -1);
            return (RETURN_RESET + RETURN_RESET_SYSSTATE);
        }
        /* < TITLE HERE > */
        /*if (!strcmp((const char *)argv[1],"")){
            ;
        }*/
        
        else
        {
            UARTPuts("\r\nreset: error:Invalid fucntion argument.\r\n", -1);
            return (RETURN_ERROR_INVALID_ARG);
        }
    }
    
    else
    {
        UARTPuts("\r\nreset: error: Not enough arguments specified.\r\n", -1);
        return (RETURN_ERROR_FEW_ARGS);
    }
}

/* goto functions */

int 
cmnd_goto_args (int argc, char *argv[], bbmc_cmd_args_t *args)
{
    /* argument map:
     * 
     *  i0: position - state_desired    , d0: stop-immediate - gain
     *  i1: position - error          , d1: output   - limit (max) value
     *  i2: position - final bound    , d2: speed    - limit (max)
     *  i3: position - CC mode switch , d3: speed    - gain
     *  i4: -                         , d4: position - gain
     *  i5: -                         , d5: -
     *  i6: -                         , d6: -
     *  i7: -                         , d7: -
     */
    
    /* setup default gains */
    g_args_goto[0].arg_double[0] = GOTO_1_P_GAIN;
    g_args_goto[1].arg_double[0] = GOTO_2_P_GAIN;

    if (argc >= 2)
    {
        /* default speed settings - low speed mode for all axes */
        g_args_goto[0].arg_double[1] = GOTO_VC_SLOW_MODE_1;
        g_args_goto[0].arg_double[2] = GOTO_CC_SLOW_MODE_1;
        
        g_args_goto[1].arg_double[1] = GOTO_VC_SLOW_MODE_2;
        g_args_goto[1].arg_double[2] = GOTO_CC_SLOW_MODE_2;
        
        g_args_goto[0].arg_double[3] = GOTO_SPEED_GAIN_1;
        g_args_goto[1].arg_double[3] = GOTO_SPEED_GAIN_2;
        
        g_args_goto[0].state_desired.q = -1;
        g_args_goto[1].state_desired.q = -1;
        
        g_args_goto[0].arg_int[2] = GOTO_STOP_ERROR_1;
        g_args_goto[1].arg_int[2] = GOTO_STOP_ERROR_2;
        
        g_args_goto[0].arg_int[3] = GOTO_CC_MODE_SWITCH_DISTANCE_1;
        g_args_goto[1].arg_int[3] = GOTO_CC_MODE_SWITCH_DISTANCE_2;
        
        /* argument parsing */
        if (!strcmp((const char *)argv[1],"home"))
        {
            g_args_goto[0].state_desired.q = g_position_home[0].limval[0];
            g_args_goto[1].state_desired.q = g_position_home[1].limval[0];
        }
        
        else
        {
            g_args_goto[0].state_desired.q = util_strtod(argv[1], NULL);
            g_args_goto[1].state_desired.q = util_strtod(argv[2], NULL);
            
            if ((g_args_goto[0].state_desired.q < g_position_limits[0].limval[0]) ||
                ((g_args_goto[0].state_desired.q > g_position_limits[0].limval[1])))
            {
                UARTPuts("\r\ngoto: error: Invalid value for state_desired Y\r\n",-1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            if ((g_args_goto[1].state_desired.q < g_position_limits[1].limval[0]) ||
                ((g_args_goto[1].state_desired.q > g_position_limits[1].limval[1])))
            {
                UARTPuts("\r\ngoto: error: Invalid value for state_desired X\r\n",-1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            if (g_args_goto[0].state_desired.q < g_dof_state[0].state.count[1])
            {
                g_args_goto[0].arg_int[4] = -1;
            }
            
            else
            {
                g_args_goto[0].arg_int[4] = 1;
            }
            
            if (g_args_goto[1].state_desired.q < g_dof_state[1].state.count[1])
            {
                g_args_goto[1].arg_int[4] = -1;
            }
            
            else
            {
                g_args_goto[1].arg_int[4] = 1;
            }
            
        }
        
        if (argc > 2)
        {
            int i;
            
            for (i = 3; i < argc; i++)
            {
                if (!strcmp((const char *)argv[i],"-f"))
                {
                    
                    /* setup for fast mode */
                    g_args_goto[0].arg_double[1] = GOTO_VC_FAST_MODE_1;
                    g_args_goto[0].arg_double[2] = GOTO_CC_FAST_MODE_1;
                    
                    g_args_goto[1].arg_double[1] = GOTO_VC_FAST_MODE_2;
                    g_args_goto[1].arg_double[2] = GOTO_CC_FAST_MODE_2;
                }
                
                else if (!strcmp((const char *)argv[i],"-pg1"))
                {
                    
                    g_args_goto[0].arg_double[0] = util_strtod((const char *)argv[i+1], NULL);
                    
                    if (g_args_goto[0].arg_double[0] <= 0)
                    {
                        
                        UARTPuts("\r\ngoto: error: Invalid Y axis position gain. \
                                    Value must be within (0,inf) range.\r\n",-1);
                        return (RETURN_ERROR_INVALID_OPT_VAL);
                    }
                    
                    i++;
                }
                
                else if (!strcmp((const char *)argv[i],"-pg2"))
                {
                    g_args_goto[1].arg_double[0] = util_strtod((const char *)argv[i+1], NULL);
                    
                    if (g_args_goto[1].arg_double[0] <= 0)
                    {
                        UARTPuts("\r\ngoto: error: Invalid X axis position gain. \
                                    Value must be within (0,inf) range.\r\n",-1);
                        return (RETURN_ERROR_INVALID_OPT_VAL);
                    }
                    
                    i++;
                }
                
                else if (!strcmp((const char *)argv[i],"-sg1"))
                {
                    
                    g_args_goto[0].arg_double[3] = util_strtod((const char *)argv[i+1], NULL);
                    
                    if (g_args_goto[0].arg_double[3] <= 0)
                    {
                        
                        UARTPuts("\r\ngoto: error: Invalid X axis speed gain. \
                                    Value must be within (0,inf) range.\r\n",-1);
                        return (RETURN_ERROR_INVALID_OPT_VAL);
                    }
                    
                    i++;
                }
                
                else if (!strcmp((const char *)argv[i],"-sg2"))
                {
                    g_args_goto[1].arg_double[3] = util_strtod((const char *)argv[i+1], NULL);
                    
                    if (g_args_goto[1].arg_double[3] <= 0)
                    {
                        UARTPuts("\r\ngoto: error: Invalid X axis speed gain. \
                                    Value must be within (0,inf) range.\r\n",-1);
                        return (RETURN_ERROR_INVALID_OPT_VAL);
                    }
                    
                    i++;
                }
                
                else
                {
                    UARTPuts("\r\ngoto: error: Invalid sub-argument for GOTO\r\n",-1);
                    return (RETURN_ERROR_INVALID_OPT);
                }
            }
        }
    }
    
    return 0;
}

int 
cmnd_goto_return_val (int argc, char *argv[], unsigned int goto_ret)
{
    if (goto_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM))
    {
        UARTPuts("\r\nWARNING! GOTO stopped due to Hall-Posiiton-Limiter.\r\n", -1);
        return (goto_ret);
    }
    
    else if (goto_ret == (RETURN_GOTO + ISR_RETURN_KILLSW))
    {
        UARTPuts("\r\nWARNING! GOTO stopped due to Killswitch.\r\n", -1);
        return (goto_ret);
    }
    
    else if (goto_ret == (RETURN_GOTO + ISR_RETURN_DEBUG)){
        
        UARTprintf("\r\nWARNING! GOTO stopped due to DEBUG-functionality: %d iterations.\r\n", GOTO_DEBUG_STOP_COUNT);
        return (goto_ret);
    }
    
    else if (goto_ret == (RETURN_GOTO + ISR_RETURN_CLEAN))
    {
        if (argc > 0)
        {
            if (!strcmp((const char *)argv[1],"home"))
            {
                UARTPuts("\r\n\r\nSystem has been reset to HOME position.\r\n", -1);
            }
            
            else
            {
                UARTprintf("\r\n\r\nSystem has been relocated to: (X,Y) = (%d , %d)\r\n", 
                            g_dof_state[1].state.count[1], g_dof_state[0].state.count[1]);
            }
        }
        
        return (RETURN_GOTO + ISR_RETURN_CLEAN);
    }
    
    else
    {
        UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event wth return value: %d.\r\n", goto_ret);
        return (RETURN_ERROR_UNKNOWN);
    }
    
    return 0;
}

int 
cmnd_goto (int argc, char *argv[])
{
    static char *goto_format = "\r\nProceed with GOTO? [Y/n]: ";
    
    bbmc_cmd_args_t args;
    
    char goto_buff[RX_BUFF_SIZE];
    int goto_ret = 0;
    
    if ((argc >= 2) || (argc == -1))
    {
        goto_ret = cmnd_goto_args(argc, argv, &args);
        
        if (goto_ret != 0)
        {
            return goto_ret;
        }
        
        if (g_flags.exec_checkpoint == 0)
        {
            g_flags.contrl_run = util_checkpoint_yn(goto_format, goto_buff);
        }
        
        if (g_flags.contrl_run == 1)
        {
            
            if (g_flags.exec_checkpoint == 0)
            {
                UARTPuts("\r\nPress any key to start GOTO...", -1);
                UARTGets(goto_buff,  2);
            }
            
            bbmc_sysflags_clear(&g_flags, "-isr");
            
            goto_ret = func_goto();
           
            return cmnd_goto_return_val(argc, argv, goto_ret);
        }
        
        else
        {
            UARTPuts("\r\n\tGOTO has been aborted.\r\n", -1);
            return (RETURN_ERROR_RUN_ABORT);
        }
    }
    
    UARTPuts("\r\nNo arguments where specified for goto. Retry..", -1);
    return -3;
}


int 
cmnd_path(int argc, char *argv[])
{
    
    UARTPuts("\r\n im the trajectory planner/designer!\r\n", -1);
    //return 5;
    //trajectory generator
    
    UARTPuts("\r\nInvalid arguments.\r\n", -1);
    return (RETURN_ERROR_INVALID_ARG);
}


int 
cmnd_config(int argc, char *argv[])
{
    static char *config_X_format = 
    "\r\nProceed with <>? [Y/n]: ";
    
    bbmc_cmd_args_t args;
    char config_buff[RX_BUFF_SIZE];
    int config_ret;
    
    if (argc > 2)
    {
        if (!strcmp((const char *)argv[1],"gains"))
        {
            //!
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_GAINS);
        }
        
        else if (!strcmp((const char *)argv[1],"qei"))
        {
            //!
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_QEI);
        }
        
        else if (!strcmp((const char *)argv[1],"pwm"))
        {
            //!
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_PWM);
        }
        
        else if (!strcmp((const char *)argv[1],"timers"))
        {
            //!
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_TIMERS);
        }
        
        /* < TITLE HERE > */
        /*else if (!strcmp((const char *)argv[1],"")){
            ;
        }*/
        
        else
        {
            UARTPuts("\r\nerror: cmnd_config: invalid argument.\r\n", -1);
            return 54;
        }
    }
    
    UARTPuts("\r\nerror: cmnd_config: not enough arguments specified\r\n", -1);
    return (RETURN_CONFIG + RETURN_ERROR_INVALID_ARG);
}

int 
cmnd_rmpi_break(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_break_format = 
    "\r\nProceed with Break-Away procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    
    args->arg_int[0] = atoi((const char*)argv[2]);
    
    if ((args->arg_int[0] < 0) || (args->arg_int[0] > BBMC_DOF_NUM))
    {
        UARTPuts("\r\nerror: cmnd_rmpi_break: invalid DOF-id.", -1);
        return RETURN_ERROR_INVALID_ARG;
    }
    
    int ret = cmnd_rmpi_break_args(argc, argv, args);
    
    if (ret < 0)
    {
        return ret;
    }
    
    g_flags.contrl_run = util_checkpoint_yn(rmpi_break_format, buff);
    
    if (g_flags.contrl_run == 1)
    {
        UARTPuts("\r\nPress any key to start Break-Away procedure...\r\n", -1);
        UARTGets(buff,  2);
        
        return cmnd_rmpi_break_func(args);
    }
    
    UARTPuts("\r\nrmpi: si: procedure has been aborted.\r\n", -1);
    return (RETURN_ERROR_RUN_ABORT);
    
    return 0;
}


int 
cmnd_rmpi_break_args(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int i;
    int ret;
    int dof_id = args->arg_int[0];
    
    int temp;
    int posinit = 1000;
    int posmax;
    
    cmnd_run_position_init(1000, 1000);
    
    if (dof_id == 1)
    {
        posmax = RMPI_BREAKAWAY_STOP_POSITION_Y_P;
    }
    
    else if (dof_id == 2)
    {
        posmax = RMPI_BREAKAWAY_STOP_POSITION_X_P;
    }
    
    else
    {
        return -1;
    }
    
    rmpi_state   = &g_dof_state[dof_id - 1];
    rmpi_args    = &g_args_contrl[dof_id - 1];
    rmpi_limits  = &g_position_limits[dof_id - 1];
    rmpi_log     = &g_datalog[dof_id - 1];
    
    /* control argument map:
     * 
     *  i0: direction      i4:                 d0: torque-increm    d4: 
     *  i1: pos-thresh     i5:                 d1:                  d5:
     *  i2: veloc-thresh   i6:                 d2:                  d6:
     *  i3:                i7:                 d3:                  d7: 
    */
    
    /* default setup */
    rmpi_args->arg_int[0]    = 1;
    rmpi_args->arg_int[1]    = posmax;
    rmpi_args->arg_int[2]    = RMPI_BREAKAWAY_STOP_SPEED;
    rmpi_args->arg_int[3]    = RMPI_BREAKAWAY_STEP_INCREM;
    rmpi_args->arg_double[0] = 0.05;
    
    rmpi_args->output.value = 0;
    
    /* check for extra(optional) arguments */
    if (argc > 3)
    {
        for (i = 3; i < argc; i++)
        {
            if (!strcmp((const char *)argv[i], "-si"))
            {
                rmpi_args->arg_int[3] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[3] > 1000 || rmpi_args->arg_int[3] < 0){
                    
                    UARTPuts("\r\ncmnd_rmpi_break_args: error: Invalid value for step increment. Value must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-st"))
            {
                rmpi_args->arg_int[2] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[2] > 10000 || rmpi_args->arg_int[2] < 0){
                    
                    UARTPuts("\r\ncmnd_rmpi_break_args: error: Invalid value for maximum velocity. Value must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-dt"))
            {
                rmpi_args->arg_int[1] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[1] > posmax || rmpi_args->arg_int[1] < 0){
                    
                    UARTPuts("\r\ncmnd_rmpi_break_args: error: Invalid value for maximum distance. Value must be in physical range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-d"))
            {
                if (!strcmp((const char *)argv[i+1], "+"))
                {
                    rmpi_args->arg_int[0] = 1;
                    cmnd_run_position_init(1000, 1000);
                }
                
                else if (!strcmp((const char *)argv[i+1], "-"))
                {
                    rmpi_args->arg_int[0] = -1;
                    cmnd_run_position_init(posmax, posmax);
                }
                
                else
                {
                    UARTPuts("\r\ncmnd_rmpi_break_args: error: Invalid direction argument.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else
            {
                UARTPuts("\r\ncmnd_rmpi_break_args: error: Invalid sub-argument has been entered.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT);
            }
        }
    }
    
    return 0;
}

int 
cmnd_rmpi_break_func(bbmc_cmd_args_t *args)
{
    int dof_id = args->arg_int[0];
    int ret;
    int i;
    
    rmpi_contrl.traject_func  = traject_null;
    rmpi_contrl.contrl_func   = contrl_rmpi_breakaway;
    rmpi_contrl.term_func     = term_rmpi_breakaway;
    
    #ifdef INPUT_QEP_DUAL
        rmpi_io.input_func = input_qei_dual;
    #endif
    #ifdef INPUT_QEP_STD
        rmpi_io.input_func = input_qei_std;
    #endif
    #ifdef INPUT_QEP_CAP
        rmpi_io.input_func = input_qei_cap;
    #endif  
    #ifdef OUTPUT_PWM_DIFF
        rmpi_io.output_func = output_pwm_dif;
    #endif
    #ifdef OUTPUT_PWM_DIR
        rmpi_io.output_func = output_pwm_dir;
    #endif
    
    //!
    g_flags.datalog = 1;
    g_flags.exec_checkpoint = 1;
    
    bbmc_sysflags_clear(&g_flags, "-isr");
    
    /*while (fabs(state->state.count[1] - controller->arg_int[1]) <= 
               RMPI_BREAKAWAY_STOP_POSITION_THR)
    {
        ret = func_rmpi(dof_id);
    }*/
    
    
    UARTPuts("\r\nRMPI::BREAK-Away has completed.\r\n", -1);
    
    UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
    return (RETURN_RMPI + RETURN_RMPI_PID_TUNE);
}


int 
cmnd_rmpi_step(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_step_format = 
    "\r\nProceed with Step-Response procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    
    
    return 0;
}

int 
cmnd_rmpi_step2(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_step2_format = 
    "\r\nProceed with 2-Step-Response procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    
    
    return 0;
}

int 
cmnd_rmpi_sine(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_sine_format = 
    "\r\nProceed with Sine-Response procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    
    
    return 0;
}

int 
cmnd_rmpi_pid_tune(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_pid_tune_format = 
    "\r\nProceed with PID-tune procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    
    args->arg_int[0] = atoi((const char*)argv[2]);
    
    if ((args->arg_int[0] < 0) || (args->arg_int[0] > BBMC_DOF_NUM))
    {
        UARTPuts("\r\nerror: cmnd_rmpi_pid_tune: invalid DOF-id.", -1);
        return RETURN_ERROR_INVALID_ARG;
    }
    
    int ret = cmnd_rmpi_pid_tune_args(argc, argv, args);
    
    if (ret < 0)
    {
        return ret;
    }
    
    ret = cmnd_run_position_init(RUN_POSINIT_Y, RUN_POSINIT_X);
        
    if (ret != 0)
    {
        return ret;
    }
    
    g_flags.contrl_run = util_checkpoint_yn(rmpi_pid_tune_format, buff);
    
    if (g_flags.contrl_run == 1)
    {
        UARTPuts("\r\nPress any key to start PID-Tune procedure...\r\n", -1);
        UARTGets(buff,  2);
        
        return cmnd_rmpi_pid_tune_func(args);
    }
    
    UARTPuts("\r\nrmpi: si: procedure has been aborted.\r\n", -1);
    return (RETURN_ERROR_RUN_ABORT);
}

int 
cmnd_rmpi_pid_tune_args(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int i;
    int ret;
    int dof_id = args->arg_int[0];
    
    int temp;
    
    int posinit;
    
    if (dof_id == 1)
    {
        posinit = RUN_POSINIT_Y;
    }
    
    else if (dof_id == 2)
    {
        posinit = RUN_POSINIT_X;
    }
    
    else
    {
        return -1;
    }
    
    rmpi_state   = &g_dof_state[dof_id - 1];
    rmpi_args    = &g_args_contrl[dof_id - 1];
    rmpi_limits  = &g_position_limits[dof_id - 1];
    rmpi_log     = &g_datalog[dof_id - 1];
    
    /* control argument map:
     * 
     *  i0: term_count     i4:                 d0:                  d4: 
     *  i1: duration       i5:                 d1:                  d5:
     *  i2:                i6:                 d2:                  d6:
     *  i3:                i7:                 d3:                  d7: 
    */
    
    /* default setup */
    rmpi_args->arg_int[0]    = 0;
    rmpi_args->arg_int[1]    = 100000;
    
    rmpi_args->arg_double[0] = 0.001;
    rmpi_args->arg_double[1] = 0;
    rmpi_args->arg_double[2] = 0;
    rmpi_args->arg_double[3] = 0;
    
    rmpi_args->state_desired.q = posinit + 100000;
    rmpi_args->state_desired.q_dot = 0;
    
    if (argc > 3)
    {
        if (!strcmp((const char *)argv[3],"-c"))
        {
            return cmnd_rmpi_control_pid_config (dof_id - 1);
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_pid_tune_args: invalid argument.\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT);
        }
    }
    
    return 0;
}

int 
cmnd_rmpi_control_pid_config (unsigned int dof_id)
{
    char buff[RX_BUFF_SIZE];
    double tmp;
    
    int posinit;
    double kinematics;
    
    if (dof_id == 0)
    {
        posinit = RUN_POSINIT_Y;
        kinematics = g_carriage_kinematics.beta_y;
    }
    
    else if (dof_id == 1)
    {
        posinit = RUN_POSINIT_X;
        kinematics = g_carriage_kinematics.beta_x;
    }
    
    else
    {
        return -1;
    }
    
    /* CONTROLLER SETUP */
    UARTPuts("\r\n\r\n - PID Controller Gains - \r\n", -1);
    
    UARTPuts("\r\n  speed-P:  ", -1);
    UARTPuts("\r\n  speed-I:  ", -1);
    UARTPuts("\r\n  speed-D:  ", -1);
    
    UARTPuts("\r\e[2A\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\n\nerror: cmnd_rmpi_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[dof_id].arg_double[0] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\nerror: cmnd_rmpi_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
    g_args_contrl[dof_id].arg_double[1] = tmp;
    
    UARTPuts("\r\e[B\e[11C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\nerror: cmnd_rmpi_control_pid_config: gains must be in range [0,1]", -1);
        return -1;
    }
    
     g_args_contrl[dof_id].arg_double[3] = tmp;
    
    /* integral sum init */
    g_args_contrl[dof_id].arg_double[2] = 0;
    
    /* init algorithm duration counter */
    g_args_contrl[dof_id].arg_int[0] = 0;
    
    /* TRAJECTORY SETUP */
    UARTPuts("\r\n\r\n - Desired State - \r\n", -1);
    
    UARTPuts("\r\n  desired-pos  :  ", -1);
    UARTPuts("\r\n  desired-speed:  ", -1);
    
    UARTPuts("\r\e[A\e[18C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\n\nerror: cmnd_rmpi_control_pid_config: desired position must be in range (0,2]", -1);
        return -1;
    }
    
    rmpi_args->state_desired.q = posinit + (tmp * kinematics);
    
    UARTPuts("\r\e[B\e[18C", -1);
    UARTGets(buff, RX_BUFF_SIZE);
    
    tmp = util_strtod(buff, NULL);
    
    if ((tmp < 0) || (tmp > 1))
    {
        UARTPuts("\r\n\nerror: cmnd_rmpi_control_pid_config: desired speed must be in range (0,0.5]", -1);
        return -1;
    }
    
    rmpi_args->state_desired.q_dot = tmp * kinematics;
    
    return 0;
}

int 
cmnd_rmpi_pid_tune_func(bbmc_cmd_args_t *args)
{
    int dof_id = args->arg_int[0];
    int ret;
    int i;
    
    rmpi_contrl.traject_func  = traject_rmpi_pid_tune;
    rmpi_contrl.contrl_func   = run_contrl;
    rmpi_contrl.term_func     = run_term;
    
    #ifdef INPUT_QEP_DUAL
        rmpi_io.input_func = input_qei_dual;
    #endif
    #ifdef INPUT_QEP_STD
        rmpi_io.input_func = input_qei_std;
    #endif
    #ifdef INPUT_QEP_CAP
        rmpi_io.input_func = input_qei_cap;
    #endif  
    #ifdef OUTPUT_PWM_DIFF
        rmpi_io.output_func = output_pwm_dif;
    #endif
    #ifdef OUTPUT_PWM_DIR
        rmpi_io.output_func = output_pwm_dir;
    #endif
    
    //!
    g_flags.datalog = 1;
    g_flags.exec_checkpoint = 1;
    
    bbmc_sysflags_clear(&g_flags, "-isr");
    
    ret = func_rmpi(dof_id);
        
    UARTPuts("\r\nRMPI::PID-TUNE has completed.\r\n", -1);
    
    UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
    return (RETURN_RMPI + RETURN_RMPI_PID_TUNE);
}

int 
cmnd_rmpi_si(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    static char *rmpi_goto_format = 
    "\r\nReset to starting position? [Y/n]: ";
    
    static char *rmpi_si_format = 
    "\r\nProceed with System Identification procedure? [Y/n]: ";
    
    char buff[RX_BUFF_SIZE];
    int ret;
    
    args->arg_int[0] = atoi(argv[2]);
    
    if ((args->arg_int[0] < 0) || (args->arg_int[0] > BBMC_DOF_NUM))
    {
        UARTPuts("\r\nerror: cmd_rmpi_si: invalid DOF-id.", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    ret = cmnd_rmpi_si_args(argc, argv, args);
    
    if (ret < 0)
    {
        return ret;
    }
    
    /*ret = util_checkpoint_yn(rmpi_goto_format, buff);
    
    if (ret == 1)
    {
        ret = bbmc_goto_home();
        
        if (ret != (RETURN_GOTO + ISR_RETURN_CLEAN))
        {
            return ret;
        }
    }*/
    
    g_flags.contrl_run = util_checkpoint_yn(rmpi_si_format, buff);
    
    if (g_flags.contrl_run == 1)
    {
        UARTPuts("\r\nPress any key to start SI procedure...\r\n", -1);
        UARTGets(buff,  2);
        
        return cmnd_rmpi_si_func(args);
    }
    
    UARTPuts("\r\nrmpi: si: procedure has been aborted.\r\n", -1);
    return (RETURN_ERROR_RUN_ABORT);
}

int 
cmnd_rmpi_si_args(int argc, char *argv[], bbmc_cmd_args_t *args)
{
    int i;
    int ret;
    int dof_id = args->arg_int[0];
    
    int idle_time;
    int on_time;
    int off_time;
    
    int temp;
    
    /* control argument map:
     * 
     *  i0: term_count     i4:                 d0: control_val      d4: 
     *  i1: duration       i5:                 d1: traject_val      d5:
     *  i2: traj_mode      i6:                 d2:                  d6:
     *  i3:                i7:                 d3:                  d7: 
     *
     * command argument map:
     * 
     *   i0: dof_id        d0: contrl_val     ui0: direction: fwr = 0; rev = 1;
     *   i1:               d1: duration (sec) ui1: number of iterations
     *   i2:               d2:                ui2: contrl iterations
     *   i3:               d3:                ui3:
     * 
    */
    
    rmpi_state   = &g_dof_state[dof_id - 1];
    rmpi_args    = &g_args_contrl[dof_id - 1];
    rmpi_limits  = &g_position_limits[dof_id - 1];
    rmpi_log     = &g_datalog[dof_id - 1];
    
    /* intialization */
    args->arg_uint[0]   = 0;
    args->arg_uint[1]   = 1;
    args->arg_uint[2]   = 5000;
    
    args->arg_double[0] = 10;
    args->arg_double[1] = 5;
    
    idle_time           = 500;
    on_time             = 200;
    off_time            = 300;
    
    /* inits */
    rmpi_args->arg_int[0]    = 0;
    rmpi_args->arg_int[1]    = 5000;
    rmpi_args->arg_int[2]    = 0;
    
    rmpi_args->arg_double[0] = 10;
    rmpi_args->arg_double[1] = 0;
    
    for(i=3; i < argc; i++)
    {
        if (!strcmp((const char *)argv[i], "-i"))
        {
            args->arg_uint[1] = (unsigned int)atoi(argv[i+1]);
            
            if ((args->arg_uint[1] > 1000))
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args:iInvalid number of iterations: "
                         "value must be in [1,1000] range.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            i++;
        }
        
        else if (!strcmp((const char *)argv[i], "-t"))
        {
            args->arg_double[0] = util_strtod(argv[i+1], NULL);
            
            int freq;
            
            sysconfig_timer_frequency_get(TIMER_RMPI, &freq);
            
            args->arg_double[0] = args->arg_double[0] * freq;
            
            args->arg_uint[2] = (unsigned int)args->arg_double[0];
            
            if ((args->arg_double[0] > GLOBAL_DATA_MAX) || (args->arg_double[0] < freq))
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid duration argument: " 
                         "value must be in [1,131] (sec) range.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            i++;
        }
        
        else if (!strcmp((const char *)argv[i], "-p"))
        {
            temp = atoi((const char*)argv[i+1]);
            
            if ((temp > 1000) || (temp < 5))
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid pulse-high time: " 
                         "value must be in [5,1000] (msec) range.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            on_time  = temp;
            
            temp = atoi((const char*)argv[i+2]);
            
            if ((temp > 1000) || (temp < 5))
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid pulse-low time: " 
                         "value must be in [5,1000] (msec) range.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            off_time = temp;
            
            i += 2;
        }
        
        else if (!strcmp((const char *)argv[i], "-a"))
        {
            args->arg_double[0] = util_strtod(argv[i+1], NULL);
            
            if ((args->arg_double[0] > 100) || (args->arg_double[0] <= 0))
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid amplitude value "
                         "(% of max). Value must be in (0,100] range.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            i++;
        }
        
        else if (!strcmp((const char *)argv[i], "-d"))
        {
            if (!strcmp((const char *)argv[i+1], "+"))
            {
                args->arg_uint[0] = 0;
            }
            
            else if (!strcmp((const char *)argv[i+1], "-"))
            {
                args->arg_uint[0] = 1;
                
                rmpi_args->arg_double[0] = (-1) * rmpi_args->arg_double[0];
            }
            
            else
            {
                UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid direction argument: "
                         " value must be in {+,-}.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
            
            i++;
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_rmpi_si_args: invalid option argument.\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT);
        }
    }
    
    ret = args->arg_uint[0];
    
    if (ret == 0)
    {
        g_args_goto[0].state_desired.q = g_position_home[0].limval[0];
        g_args_goto[1].state_desired.q = g_position_home[1].limval[0];
    }
    else if (ret == 1)
    {
        g_args_goto[0].state_desired.q = g_position_home[0].limval[1];
        g_args_goto[1].state_desired.q = g_position_home[1].limval[1];
    }
    else
    {
        UARTPuts("\r\nrmpi: error: unknown error has occured when evaluating 'rmpi_val' variable.\r\n", -1);
        return (RETURN_ERROR_UNKNOWN);
    }
    
    args->arg_int[3] = args->arg_uint[2];
    
    //signalgen_prbs_v1(&g_log_signalgen, g_prbs_v1_table, args->arg_int[3], idle_time, 0);
    //signalgen_prbs_v2(&g_log_signalgen, g_prbs_v2_table, args->arg_int[3], idle_time, 1);
    
    signalgen_pulse_v1(&g_log_signalgen, g_pulse_v1_table, args->arg_int[3], 
                       on_time, off_time, idle_time, (dof_id-1));
    
    /* finalize the arguments */
    rmpi_args->arg_int[1] = args->arg_uint[2] + idle_time;
    rmpi_args->arg_double[0] = args->arg_double[0];
    
    rmpi_args->state_desired.q = 0;
    rmpi_args->state_desired.q_dot = 0;
    
    return 0;
}

int 
cmnd_rmpi_si_func(bbmc_cmd_args_t *args)
{
    int dof_id = args->arg_int[0];
    int ret;
    int i;
    
    rmpi_contrl.traject_func  = traject_rmpi_si;
    rmpi_contrl.contrl_func   = contrl_rmpi_si;
    rmpi_contrl.term_func     = term_rmpi_si;
    
    #ifdef INPUT_QEP_DUAL
        rmpi_io.input_func = input_qei_dual;
    #endif
    #ifdef INPUT_QEP_STD
        rmpi_io.input_func = input_qei_std;
    #endif
    #ifdef INPUT_QEP_CAP
        rmpi_io.input_func = input_qei_cap;
    #endif  
    #ifdef OUTPUT_PWM_DIFF
        rmpi_io.output_func = output_pwm_dif;
    #endif
    #ifdef OUTPUT_PWM_DIR
        rmpi_io.output_func = output_pwm_dir;
    #endif
    
    for(i = 0; i < args->arg_uint[1]; i++)
    {
        //!
        g_flags.datalog = 1;
        g_flags.exec_checkpoint = 1;
        
        bbmc_sysflags_clear(&g_flags, "-isr");
        
        ret = func_rmpi(dof_id);
        
        //!
        g_flags.datalog = 0;
        
        ret = bbmc_goto_home();
        
        if (ret != (RETURN_GOTO + ISR_RETURN_CLEAN))
        {
            return ret;
        }
    }
    
    UARTPuts("\r\nRMPI::SI has completed.\r\n", -1);
    
    UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
    return (RETURN_RMPI + RETURN_RMPI_SI);
}

int 
cmnd_rmpi(int argc, char *argv[])
{
    bbmc_cmd_args_t args;
    
    if (argc > 2)
    {
        if (!strcmp((const char *)argv[1],"break"))
        {
            return cmnd_rmpi_break(argc, argv, &args);
        }
        
        else if (!strcmp((const char *)argv[1],"step"))
        {
            return cmnd_rmpi_step(argc, argv, &args);
        }
        
        else if (!strcmp((const char *)argv[1],"step2"))
        {
            return cmnd_rmpi_step2(argc, argv, &args);
        }
        
        else if (!strcmp((const char *)argv[1],"sine"))
        {
            return cmnd_rmpi_sine(argc, argv, &args);
        }
        
        else if (!strcmp((const char *)argv[1],"pid"))
        {
            return cmnd_rmpi_pid_tune(argc, argv, &args);
        }
        
        else if (!strcmp((const char *)argv[1],"si"))
        {
            return cmnd_rmpi_si(argc, argv, &args);
        }
        
        else
        {
            UARTPuts("\r\nerror: cmnd_rmpi: not enough arguments specified.\r\n", -1);
            return (RETURN_ERROR_INVALID_ARG);
        }
    }
    
    UARTPuts("\r\nerror: cmnd_rmpi: not enough arguments specified.\r\n", -1);
    return (RETURN_ERROR_FEW_ARGS);
}

/* test command functions */

int 
cmnd_test_pwm_func  (int argc, char *argv[], unsigned int dof_id)
{
    char buff[32];
    bbmc_dof_output_t volatile test_out;
    bbmc_io_funcs_t test_io;
    
    test_out.dof_id = dof_id;
    test_out.value = 0;
    test_io.output_func = output_pwm_dif;
    
    if (argc > 3)
    {
    
        if (!strcmp((const char *)argv[3],"-dir"))
        {
            test_io.output_func = output_pwm_dif;
            
            UARTprintf("\r\nOutput %d configured for PWM+DIR comands\r\n", dof_id);
        }
        
        else if (!strcmp((const char *)argv[3],"-dif"))
        {
            test_io.output_func = output_pwm_dif;
            
            UARTprintf("\r\nOutput %d configured for differential PWM comands\r\n", dof_id);
        }
        else
        {
            UARTPuts("\r\nerror: cmnd_test_pwm_func: output mode invalid. argv4 = -{dif,dir}\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT);
        }
    }
    
    sysconfig_poslim_enable(dof_id);
        
    sysconfig_killswitch_enable();
    
    sysconfig_pwm_enable(dof_id);
    
    for(;;)
    {
        UARTPuts("\r\n\t<test-pwm>: ", -1);
        UARTGets(buff, 32);
        
        if (!strcmp((const char *)buff,"end"))
        {
            test_out.value = 0;
            test_io.output_func(&test_out);
            break;
        }
        
        if (!strcmp((const char *)buff,"pos"))
        {
            bbmc_dof_state_print(dof_id, "\t");
            continue;
        }
        
        test_out.value = util_strtod(buff, NULL);
        test_io.output_func(&test_out);
        
    }
    
    sysconfig_pwm_disable(dof_id);
    
    sysconfig_killswitch_disable();
        
    sysconfig_poslim_disable(dof_id);
    
    return (RETURN_TEST + RETURN_TEST_PWM);
}

int 
cmnd_test_qei_args(int argc, char *argv[], unsigned int dof_id)
{
    rmpi_io.input_func = input_qei_dual;
    rmpi_io.output_func = output_pwm_dif;
    
    rmpi_contrl.traject_func = traject_null;
    rmpi_contrl.contrl_func  = contrl_test_dsr;
    rmpi_contrl.term_func    = term_test_dsr;
    
    rmpi_state  = &g_dof_state[dof_id-1];
    rmpi_limits = &g_position_limits[dof_id-1];
    rmpi_args   = &g_args_contrl[dof_id-1];
    
    rmpi_args->arg_double[0] = 0;
    rmpi_args->output.value = 0;
    
    if (argc > 3)
    {
        int i;
        
        for (i = 3; i < argc; i++)
        {
            if (!strcmp((const char *)argv[i], "-so"))
            {
                if (!strcmp((const char *)argv[i+1],"dir"))
                {
                    rmpi_io.output_func = output_pwm_dir;
                    
                    UARTprintf("\r\nOutput %d configured for PWM+DIR comands\r\n", dof_id);
                }
                else if (!strcmp((const char *)argv[i+1],"dif"))
                {
                    rmpi_io.output_func = output_pwm_dif;
                    
                    UARTprintf("\r\nOutput %d configured for Differential PWM comands\r\n", dof_id);
                }
                else
                {
                    UARTPuts("\r\nOutput mode invalid or not specified. argv4 = {pdif,pdir}\r\n", -1);
                    return (RETURN_TEST + RETURN_ERROR_INVALID_SUBARG);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-si"))
            {
                if (!strcmp((const char *)argv[i+1],"std"))
                {
                    rmpi_io.input_func = input_qei_std;
                        
                    UARTprintf("\r\nInput %d configured for Standard BDM mode\r\n", dof_id);
                }
                
                else if (!strcmp((const char *)argv[i+1],"cap"))
                {
                    rmpi_io.input_func = input_qei_cap;
                    
                    UARTprintf("\r\nInput %d configured for Capture mode\r\n", dof_id);
                }
                
                else if (!strcmp((const char *)argv[i+1],"dual"))
                {
                    rmpi_io.input_func = input_qei_dual;
                        
                    UARTprintf("\r\nInput %d configured for Dual speed mode\r\n", dof_id);
                }
                
                else
                {
                    UARTPuts("\r\nInput mode invalid or not specified. argv4 = {std,cap,dual}\r\n", -1);
                    return -3;
                }
                
                i++;
            }
            
            else{
                
                UARTPuts("\r\nrmpi: error: Invalid sub-argument has been entered.\r\n", -1);
                return (RETURN_TEST + RETURN_ERROR_INVALID_SUBARG);
            }
        }
    }
    
    return 0;
}

int 
cmnd_test_qei_func(unsigned int dof_id)
{
    char buff[32];
    
    
    UARTPuts("\r\nPress any key to start QEI...", -1);
    UARTGets(buff,  2);
    
    bbmc_cisr_init(&g_isr_state);
    
    sysconfig_qei_data_init(TIMER_RMPI, dof_id);
    
    UARTPuts("\r\n\r\nEXECUTING...\r\n", -1);
    
    UARTPuts("\r\nSpeed: \r\nPos: \r\nDuty: ", -1);
    
    sysconfig_poslim_enable(dof_id);
    
    sysconfig_killswitch_enable();
    
    sysconfig_pwm_enable(dof_id);
    
    sysconfig_timer_enable(TIMER_RMPI);
    
    for (;;)
    {
        //!
        DMTimerIntDisable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
        UARTPuts("        \e[8D", -1);
        DMTimerIntEnable(SOC_DMTIMER_4_REGS, DMTIMER_INT_OVF_EN_FLAG);
        
        UARTGets(buff, 8);
        
        if (!strcmp((const char *)buff,"end"))
        {   
            rmpi_args->arg_double[0] = 0;
            util_delay(MAX_STOP_DELAY_COUNT);
            
            break;
        }
        
        g_args_contrl[dof_id-1].arg_double[0] = util_strtod(buff, NULL);
    }
    
    sysconfig_timer_disable(TIMER_RMPI);
    
    sysconfig_pwm_disable(dof_id);
    
    sysconfig_killswitch_disable();
    
    sysconfig_poslim_disable(dof_id);
    
    UARTPuts("\r\nQEI has terminated.", -1);
    
    UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
    return (RETURN_TEST + RETURN_TEST_QEI);
}

int 
cmnd_test_gpio_func (int argc, char *argv[], unsigned int dof_id)
{
    char buff[32];
    unsigned int test_pin;
    int test_flag_debug;
    
    if (!strcmp((const char *)argv[2], "hall"))
    {
        test_flag_debug = g_flags.debug;
        g_flags.debug = 1;
        
        UARTprintf("\r\nInput for axis %d is reading Hall Limit Position Sensor\r\n", 
        dof_id);
        
        sysconfig_poslim_enable(dof_id);
        sysconfig_intc_master_enable();
        
        for(;;)
        {
            
            UARTPuts("\r\n\t<test-gpio-hall>: ", -1);
            UARTGets(buff, 32);
            
            if (!strcmp((const char *)buff,"end"))
            {
                break;
            }
            
            if (!strcmp((const char *)buff,"read"))
            {
                sysconfig_gpio_poslim_get(dof_id, &test_pin);
                
                UARTPuts("\r\n\tValue form pin is: ", -1);
                UARTPutNum(test_pin);
            }
            
        }
        
        sysconfig_intc_master_disable();
        sysconfig_poslim_disable(dof_id);
        
        g_flags.debug = test_flag_debug;
        
        UARTPuts("\r\nExiting isr_gpio_*() test.\r\n", -1);
        return (RETURN_TEST + RETURN_TEST_GPIO_HALL);
    }
    
    else if (!strcmp((const char *)argv[2], "kill"))
    {
        test_flag_debug = g_flags.debug;
        g_flags.debug = 1;
        
        UARTPuts("\r\nReading Killswitch Pushbutton...\r\n", -1);
        
        sysconfig_killswitch_enable();
        sysconfig_intc_master_enable();
        
        for(;;)
        {
            
            UARTPuts("\r\n\t<test-gpio-kill>: ", -1);
            UARTGets(buff, 32);
            
            if (!strcmp((const char *)buff,"end"))
            {
                break;
            }
            
            if (!strcmp((const char *)buff,"read"))
            {
                sysconfig_gpio_killswitch_get(&test_pin);
                
                UARTPuts("\r\n\tValue form pin is: ", -1);
                UARTPutNum(test_pin);
            }
            
        }
        
        sysconfig_intc_master_enable();
        sysconfig_killswitch_enable();
        
        g_flags.debug = test_flag_debug;
        
        UARTPuts("\r\nExiting isr_gpio_killswitch test.\r\n", -1);
        return (RETURN_TEST + RETURN_TEST_GPIO_HALL);
    }
    
    else
    {
        UARTPuts("\r\nInvalid fucntionality argument.\r\n", -1);
        return (RETURN_TEST + RETURN_ERROR_INVALID_SUBARG);
    }
    
    return 0;
}

int 
cmnd_test(int argc, char *argv[])
{
    static char *test_qei_format = "\r\nProceed with QEI test? [Y/n]: ";
    
    char test_buff[RX_BUFF_SIZE];
    
    unsigned int test_dof;
    
    if (argc > 1)
    {
        /** pwm output test - set desired duty to ouput **/
        if (!strcmp((const char *)argv[1],"pwm"))
        {
            if (argc > 2)
            {
                test_dof = (unsigned int)atoi(argv[2]);
    
                if (test_dof > BBMC_DOF_NUM)
                {
                    UARTPuts("\r\nerror: cmnd_test: pwm: invalid pwm channel number\r\n", -1);
                    return (RETURN_ERROR_INVALID_ARG);
                }
            }
            
            else
            {
                UARTPuts("\r\nerror: cmnd_test: pwm: no pwm channel specified\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            return cmnd_test_pwm_func(argc, argv, test_dof);
        }
        
        /** speed test - read observed speed from encoders **/
        else if (!strcmp((const char *)argv[1],"qei"))
        {
            test_dof = (unsigned int)atoi(argv[2]);
    
            if (test_dof > BBMC_DOF_NUM)
            {
                UARTPuts("\r\nInvalid Module number. Retry...\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            cmnd_test_qei_args(argc, argv, test_dof);
            
            g_flags.contrl_run = util_checkpoint_yn(test_qei_format, test_buff);
    
            if (g_flags.contrl_run == 1)
            {
                return cmnd_test_qei_func(test_dof);
            }
    
            else
            {
                UARTPuts("\r\n\tDSR has been aborted.\r\n", -1);
                return (RETURN_TEST + RETURN_ERROR_RUN_ABORT);
            }
        }
        
        /** test facility for hall maxi/min position sensors **/
        else if (!strcmp((const char *)argv[1], "gpio"))
        {
            test_dof = (unsigned int)atoi(argv[2]);
    
            if (test_dof > BBMC_DOF_NUM)
            {
                UARTPuts("\r\nInvalid Module number. Retry...\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            return cmnd_test_gpio_func(argc, argv, test_dof); 
        }
        
        else
        {
            UARTPuts("\r\nInvalid sub-argument. Retry...\r\n", -1);
            return (RETURN_TEST + RETURN_ERROR_INVALID_SUBARG);
        }
    }
    
    UARTPuts("\r\ntest: invalid function\r\n", -1);
    return (RETURN_TEST + RETURN_ERROR_INVALID_ARG);
}

int 
cmnd_debug(int argc, char *argv[])
{
    
    if (argc > 1)
    {
        if (!strcmp((const char *)argv[1],"enable"))
        {
            g_flags.debug = 1;
            UARTPuts("\r\ndebug mode: on\r\n", -1);
            return (RETURN_DEBUG + RETURN_DEBUG_ENABLE);
        }
        if (!strcmp((const char *)argv[1],"disable"))
        {
            g_flags.debug = 0;
            UARTPuts("\r\ndebug mode: off\r\n", -1);
            return (RETURN_DEBUG + RETURN_DEBUG_DISABLE) ;
        }
    }
    
    UARTPuts("\r\ndebug: invalid function\r\n", -1);
    return (RETURN_DEBUG + RETURN_ERROR_INVALID_ARG);
}



int 
cmnd_status (int argc, char *argv[])
{
    bbmc_cli_clear();
    
    UARTPuts("\r\n\e[60C-TOTAL SYSTEM DIAGNOSTIC-", -1);
    
    /* column one */
    
    bbmc_cli_newlin(2);
    
    bbmc_sysflags_print(&g_flags, "\e[10C");
    
    bbmc_dof_state_print(3, "\e[10C");
    
    /* column two */
    
    bbmc_cli_cursor_mv_top();
    
    bbmc_cli_newlin(2);
    
    bbmc_cisr_print(&g_isr_state, "\e[50C");
    
    bbmc_poslim_print("\e[50C");
    
    /* column three */
    
    bbmc_cli_cursor_mv_top();
    
    bbmc_cli_newlin(2);
    
    bbmc_pwm_print("\e[95C");
    
    bbmc_timers_print("\e[95C");
    
    bbmc_qei_print("\e[95C");
    
    /* reset cursor */
    
    bbmc_cli_cursor_mv_bottom();
    
    return (RETURN_STATUS + RETURN_STATUS_SYSDIAG);
}

int 
cmnd_quit(int argc, char *argv[])
{
    
    // TODO: either add shutdown functionality here or after primary while() loop. 
    
    g_flags.cmdln = -1;
    return (RETURN_QUIT);
}


/* 
 * EOF 
 */
