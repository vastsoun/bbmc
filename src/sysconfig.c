




/* system halt function */
void
bbmc_system_halt (void)
{
    /* Add power down and cleanup functions */
    
    UARTPuts("\r\nSystem Halt: System is shuting down. Goodbye!\r\n", -1);
    
    for (;;)
    {
        ;
    }
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


//TODO ?????
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

/* TODO */
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


