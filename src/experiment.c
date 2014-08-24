












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







