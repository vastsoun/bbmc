
static bbmc_contrl_motor_t volatile  g_args_goto[BBMC_DOF_NUM];

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
bbmc_goto_home (void)
{
    int ret;
    
    //! put these so the caller determines the destination
    g_args_goto[0].state_desired.q = g_position_home[0].limval[0];
    g_args_goto[1].state_desired.q = g_position_home[1].limval[0];
    
    bbmc_sysflags_set(&g_flags, "-cmd");
    
    UARTprintf("\r\nReseting URETTS Carriage to home position\r\n");
    
    ret = cmnd_goto(-1, NULL);
    
    bbmc_sysflags_clear(&g_flags, "-cmd");
    
    return bbmc_isr_return_value(RETURN_GOTO, ret);
}

/* goto command functions */
int goto_args (int argc, char *argv[], bbmc_cmd_args_t *args);

int goto_return_val (int argc, char *argv[], unsigned int goto_ret);

int goto (int argc, char *argv[]);
