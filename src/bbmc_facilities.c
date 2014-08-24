

//TODO
typedef struct
{
    double        arg_double[4];
    int            arg_int[4];
    unsigned int  arg_uint[4];
}
bbmc_cmd_args_t;


/* systick timer data */ 
static volatile int                 timer_ticks   = 0;
static volatile int                 timer_ticking = 0;




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
