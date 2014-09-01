










/* rmpi data */
static bbmc_io_funcs_t             rmpi_io;
static bbmc_contrl_funcs_t         rmpi_contrl;

static datalog_s_t volatile        *rmpi_log;
static bbmc_dof_limits_t           *rmpi_limits;
static bbmc_dof_state_t  volatile  *rmpi_state;
static bbmc_dof_contrl_t volatile  *rmpi_args;













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



