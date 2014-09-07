/** Module Header **/
#include "bbmc_facilities.h"

/** Std C Headers **/
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>

/** Firmware Headers **/
#include "uartStdio.h"
#include "cmdline.h"

/** BBMC Headers **/
#include "device_layer.h"
#include "system_timers.h"
#include "isr_manager.h"
#include "safety_ss.h"

#include "motor_control.h"
#include "global_state.h"
#include "global_flags.h"

#include "datalog.h"
//#include "signal_generator.h"

//#include "uretts_model.h"
//#include "uretts_control.h"

#include "util.h"


/** Internal Data Types
 * 
 */




/** Internal Data 
 * 
 */




/** Internal Functions 
 * 
 */




/** TEST
 *  
 */

int test (int argc, char *argv[])
{
    UARTPuts("\r\n This is BBMC/Test\r\n", -1);
    //TODO
    
    return 0;
}

int test_args(int argc, char *argv[], cmd_args_t *args)
{
    //TODO
    
    return 0;
}

int test_pwm  (cmd_args_t *args)
{
    //TODO
    
    return 0;
}

int test_qei  (cmd_args_t *args)
{
    //TODO
    
    return 0;
}

int test_gpio (cmd_args_t *args)
{
    //TODO
    
    return 0;
}



/** STATUS
 *  
 */

int 
status (int argc, char *argv[])
{
    CmdLineClear();
    
    UARTPuts("\r\n\e[60C-TOTAL SYSTEM DIAGNOSTIC-", -1);
    
    
    /** column one **/

    CmdLineNewline(2);
    
    global_flags_print ("\e[10C");
    global_state_print (3, "\e[10C");
    
    
    /** column two **/
    
    CmdLineCursorMoveTop();
    CmdLineNewline(2);
    
    isr_state_print ("\e[50C");
    global_positions_print ("\e[50C");
    
    
    /** column three **/
    
    CmdLineCursorMoveTop();
    CmdLineNewline(2);
    
    pwm_print ("\e[95C");
    timers_print ("\e[95C");
    qei_print ("\e[95C");
    
    
    /** reset cursor **/
    
   CmdLineCursorMoveBottom();
   
    
    return (RETURN_STATUS + RETURN_STATUS_SYSDIAG);
}


/** PLANNER
 *  
 */

int 
planner (int argc, char *argv[])
{
    UARTPuts("\r\n Im the trajectory planner!. Coming soon...\r\n", -1);
    //TODO
    
    UARTPuts("\r\nInvalid arguments.\r\n", -1);
    return (RETURN_ERROR_INVALID_ARG);
}

int 
planner_args (int argc, char *argv[], cmd_args_t *args)
{
    //TODO
    return (RETURN_ERROR_INVALID_ARG);
}



/** DATALOG
 *  
 */

int 
datalog (int argc, char *argv[])
{
    if (argc > 10)
    {
        UARTPuts("\r\nerror: datalog: Too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    
    else if (argc > 1)
    {
        int dl_ret;
        int print_range[4] = {0};
        cmd_args_t args;
        unsigned int count;
        
        dl_ret = datalog_args(argc, argv, &args);
        
        if (dl_ret < 0)
        {
            return dl_ret;
        }
        
        if (!strcmp((const char *)argv[1],"reset"))
        {
            datalog_s_init(0);
            
            UARTPuts("\r\nDatalogs have been reset.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_RESET);
        }
        
        else if (!strcmp((const char *)argv[1],"print"))
        {
            count = isr_state_get (TERM_COUNT);
            
            if (count == 0)
            {
                UARTPuts("\r\nerror: datalog: There is no data to transmit.\r\n", -1);
                return (RETURN_DATALOG + RETURN_ERROR_UNKNOWN);
            }
            
            print_range[0] = 0;
            print_range[1] = count;
            print_range[2] = args.arg_int[0];
            print_range[3] = args.arg_int[1];
            
            if (args.arg_int[2] < 0)
            {
                int i;
                
                for (i = 1; i <= BBMC_DOF_NUM; i++)
                {
                    datalog_s_print(i, print_range);
                }
            }
            
            else
            {
                datalog_s_print(args.arg_int[2], print_range);
            }
            
            UARTPuts("\r\nDatalog is printing on console...\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_PRINT);
        }
        
        else if (!strcmp((const char *)argv[1],"enable"))
        {
            global_flag_set (FLG_DATALOG);
            UARTPuts("\r\nDatalog has been enabled.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_ENABLE);
        }
        
        else if (!strcmp((const char *)argv[1],"disable"))
        {
            global_flag_clear (FLG_DATALOG);
            UARTPuts("\r\nDatalog has been disabled.\r\n", -1);
            return (RETURN_DATALOG + RETURN_DATALOG_DISABLE);
        }
        
        else
        {
            UARTPuts("\r\nerror: datalog: Invalid datalog function.\r\n", -1);
            return (RETURN_DATALOG + RETURN_ERROR_INVALID_ARG);
        }
    }
    
    else
    {
        UARTPuts("\r\nerror: datalog: Not enough arguments.\r\n", -1);
        return (RETURN_ERROR_FEW_ARGS);
    }
}

int 
datalog_args (int argc, char *argv[], cmd_args_t *args)
{
    int i;
    
    args->arg_int[2] = -1;
    
    if (argc > 2)
    {
        int dof_id = atoi(argv[2]);
        
        if ((dof_id > BBMC_DOF_NUM) || (dof_id <= 0))
        {
            UARTPuts("\r\nerror: datalog: Invalid DOF-id argument.\r\n", -1);
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
                UARTPuts("\r\nerror: datalog: Invalid value specified for max index.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT_VAL);
            }
        }
        
        else
        {
            UARTPuts("\r\nerror: datalog: Invalid argument option.\r\n", -1);
            return (RETURN_ERROR_INVALID_OPT); 
        }
    }

    return 0;
}



/** PERFLOG
 *  
 */

int 
perflog (int argc, char *argv[])
{
    
    if (argc > 10)
    {
                
        UARTPuts("\r\nperflog: error: too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    
    else if (argc > 1)
    {
        if (!strcmp((const char *)argv[1],"reset"))
        {
            performance_log_init ();
            UARTPuts("\r\nPerformance log has been reset.\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_RESET);
        }
        
        if (!strcmp((const char *)argv[1],"print"))
        {
            unsigned int count = isr_state_get (TERM_COUNT);
            
            performance_log_print (count);
            return (RETURN_PERF + RETURN_PERF_PRINT);
        }
        
        if (!strcmp((const char *)argv[1],"enable"))
        {
            global_flag_set (FLG_DATALOG);
            UARTPuts("\r\nperflog_measure mode: on\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_ENABLE);
        }
        
        if (!strcmp((const char *)argv[1],"disable"))
        {
            global_flag_clear (FLG_DATALOG);
            UARTPuts("\r\nperflog_measure mode: off\r\n", -1);
            return (RETURN_PERF + RETURN_PERF_DISABLE);
        }
    }
    
    else
    {
        UARTPuts("\r\nperflog: error: not enough arguments.\r\n", -1);
        return (RETURN_ERROR_FEW_ARGS);
    }
    
    UARTPuts("\r\nperflog: error: unknown execution event.\r\n", -1);
    return (RETURN_ERROR_UNKNOWN);
}



/** CONFIG
 *  
 */

int 
config (int argc, char *argv[])
{
    static char *X_format = 
    "\r\nProceed with <>? [Y/n]: ";
    
    static char *poscalib_format = 
    "\r\nProceed with position calibration? [Y/n]: ";
    
    static char *gstate_format = 
    "\r\nProceed with Global State Reset? [Y/n]: ";
    
    cmd_args_t args;
    char config_buff[RX_BUFF_SIZE];
    int config_ret = 0;
    
    
    if (argc > 10)
    {
        UARTPuts("\r\nconfig: error: Too many arguments.\r\n", -1);
        return (RETURN_ERROR_MANY_ARGS);
    }
    
    else if (argc > 2)
    {
        if (!strcmp((const char *)argv[1],"gstate"))
        {
            config_gstate_args(argc, argv, &args);
            
            config_ret = util_checkpoint_yn (gstate_format, config_buff);
            
            if (config_ret == 1)
            {
                config_gstate (&args);
                UARTPuts("\r\nGlobal State has been reset.\r\n", -1);
                return (RETURN_CONFIG + RETURN_CONFIG_GSTATE);
            }
            
            UARTPuts("\r\nGlobal State reset has been aborted.\r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_GSTATE);
        }
        
        else if (!strcmp((const char *)argv[1],"poscalib"))
        {
            //TODO
            
            UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_POSCALIB);
        }
        
        else if (!strcmp((const char *)argv[1],"qei"))
        {
            //TODO
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_QEI);
        }
        
        else if (!strcmp((const char *)argv[1],"pwm"))
        {
            //TODO
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_PWM);
        }
        
        else if (!strcmp((const char *)argv[1],"timers"))
        {
            //TODO
            
            UARTPuts("\r\n \r\n", -1);
            return (RETURN_CONFIG + RETURN_CONFIG_TIMERS);
        }
        
        else
        {
            UARTPuts("\r\nerror: config: invalid argument.\r\n", -1);
            return (RETURN_CONFIG + RETURN_ERROR_INVALID_ARG);
        }
    }
    
    else
    {
        ;
    }
    
    UARTPuts("\r\nerror: config: not enough arguments specified\r\n", -1);
    return (RETURN_CONFIG + RETURN_ERROR_INVALID_ARG);
}

int 
config_gstate_args (int argc, char *argv[], cmd_args_t *args)
{
    //TODO
    
    return 0;
}

int 
config_gstate (cmd_args_t *args)
{
    UARTPuts("\r\ni promise i will do this soon!..\r\n", -1);
    
    return 0;
}

int 
config_poscalib_args (int argc, char *argv[], cmd_args_t *args)
{
    //TODO
    
    return 0;
}

int 
config_poscalib (cmd_args_t *args)
{
    //TODO
    
    return 0;
}



/** DEBUG
 *  
 */

int 
debug (int argc, char *argv[])
{
    if (argc > 1)
    {
        if (!strcmp((const char *)argv[1],"enable"))
        {
            global_flag_set(FLG_DEBUG);
            UARTPuts("\r\ndebug mode: on\r\n", -1);
            return (RETURN_DEBUG + RETURN_DEBUG_ENABLE);
        }
        if (!strcmp((const char *)argv[1],"disable"))
        {
            global_flag_clear(FLG_DEBUG);
            UARTPuts("\r\ndebug mode: off\r\n", -1);
            return (RETURN_DEBUG + RETURN_DEBUG_DISABLE) ;
        }
    }
    
    UARTPuts("\r\ndebug: invalid function\r\n", -1);
    return (RETURN_DEBUG + RETURN_ERROR_INVALID_ARG);
}



/** QUIT
 *  
 */

int 
quit (int argc, char *argv[])
{
    // TODO: either add shutdown functionality here or after primary while() loop. 
    
    global_flag_set(FLG_CMDLINE);
    
    return (RETURN_QUIT);
}




/** ISR return value tester
 * 
 */

int 
isr_return_value(unsigned int cmnd_ret, int ret)
{
    if (ret == (cmnd_ret + ISR_RETURN_CLEAN))
    {
        UARTPuts("\r\nController has executed succesfully.\r\n", -1);
        return (RETURN_SUCCESS);
    }
    
    else if (ret == (cmnd_ret + ISR_RETURN_POSLIM))
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




/**
 * 
 */


/*

void contrl_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                     bbmc_dof_contrl_t volatile *controller);

void contrl_test_dsr (bbmc_dof_state_t volatile *state, 
                               bbmc_dof_contrl_t volatile *controller);

void term_reset_poscalib (bbmc_dof_state_t volatile *state, 
                                   bbmc_dof_contrl_t volatile *controller);

void term_test_dsr (bbmc_dof_state_t volatile *state, 
                             bbmc_dof_contrl_t volatile *controller);

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


static void 
traject_null (bbmc_dof_state_t volatile *state, 
              bbmc_dof_contrl_t volatile *controller)
{
    ;
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

int 
test_pwm_func  (int argc, char *argv[], unsigned int dof_id)
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
            UARTPuts("\r\nerror: test_pwm_func: output mode invalid. argv4 = -{dif,dir}\r\n", -1);
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
test_qei_args(int argc, char *argv[], unsigned int dof_id)
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
test_qei_func(unsigned int dof_id)
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
test_gpio_func (int argc, char *argv[], unsigned int dof_id)
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
test(int argc, char *argv[])
{
    static char *test_qei_format = "\r\nProceed with QEI test? [Y/n]: ";
    
    char test_buff[RX_BUFF_SIZE];
    
    unsigned int test_dof;
    
    if (argc > 1)
    {
        if (!strcmp((const char *)argv[1],"pwm"))
        {
            if (argc > 2)
            {
                test_dof = (unsigned int)atoi(argv[2]);
    
                if (test_dof > BBMC_DOF_NUM)
                {
                    UARTPuts("\r\nerror: test: pwm: invalid pwm channel number\r\n", -1);
                    return (RETURN_ERROR_INVALID_ARG);
                }
            }
            
            else
            {
                UARTPuts("\r\nerror: test: pwm: no pwm channel specified\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            return test_pwm_func(argc, argv, test_dof);
        }
        
        else if (!strcmp((const char *)argv[1],"qei"))
        {
            test_dof = (unsigned int)atoi(argv[2]);
    
            if (test_dof > BBMC_DOF_NUM)
            {
                UARTPuts("\r\nInvalid Module number. Retry...\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            test_qei_args(argc, argv, test_dof);
            
            g_flags.contrl_run = util_checkpoint_yn(test_qei_format, test_buff);
    
            if (g_flags.contrl_run == 1)
            {
                return test_qei_func(test_dof);
            }
    
            else
            {
                UARTPuts("\r\n\tDSR has been aborted.\r\n", -1);
                return (RETURN_TEST + RETURN_ERROR_RUN_ABORT);
            }
        }
        
        else if (!strcmp((const char *)argv[1], "gpio"))
        {
            test_dof = (unsigned int)atoi(argv[2]);
    
            if (test_dof > BBMC_DOF_NUM)
            {
                UARTPuts("\r\nInvalid Module number. Retry...\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
            
            return test_gpio_func(argc, argv, test_dof); 
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

*/
