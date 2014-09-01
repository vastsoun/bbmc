int rmpi_ret, rmpi_dof;
int rmpi_iter, rmpi_steps;

bbmc_cmd_args_t args;

double arg_output_max, arg_output_min, arg_output_incr;
double rmpi_val;

/** BREAKAWAY FRICTION TORQUE/FORCE experiment **/
if (!strcmp((const char *)argv[1],"breakaway"))
{
    /* init system g_flags */
    g_flags.exec_checkpoint = 1;
    g_flags.stop_immediate = 0;
    
    /* default RMPI values for breakaway test */
    rmpi_iter = 1;
    rmpi_steps = 0;
    
    /* init goto parameters */
    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    
    /* argument map:
     * 
     *  i0: increment interval  , d0: output increment
     *  i1: stop speed          , d1: -
     * 
    */
    
    rmpi_dof = atoi(argv[2]);
    
    if (rmpi_dof == 1)
    {
        rmpi_state = &g_dof_state[0];
        rmpi_args = &g_args_contrl[0];
        rmpi_limits = &g_position_limits[0];
        rmpi_val = ENC_1_LIN_PER_ROT;
    }
    
    else if (rmpi_dof == 2)
    {
        rmpi_state = &g_dof_state[1];
        rmpi_args = &g_args_contrl[1];
        rmpi_limits = &g_position_limits[1];
        rmpi_val = ENC_2_LIN_PER_ROT;
    }
    
    else
    {
        UARTPuts("\r\nInvalid value for device argument.\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    sysconfig_pwm_frequency_get(rmpi_dof, &(rmpi_args->arg_double[0]));
    
    rmpi_args->arg_double[0] = 100 / rmpi_args->arg_double[0];
    
    /* setup for breakaway functionality */
    rmpi_funcs.traject_func = &traject_null;
    rmpi_funcs.contrl_func = &contrl_rmpi_breakaway;
    rmpi_funcs.term_func = &term_rmpi_breakaway;
    
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
    
    
    /* check for extra(optional) arguments */
    if (argc > 3)
    {
        for(i=3; i < argc; i++)
        {
            if (!strcmp((const char *)argv[i], "-i"))
            {
                rmpi_iter = atoi(argv[i+1]);
                
                if (rmpi_iter > 200 || rmpi_iter < 0)
                {
                    UARTPuts("\r\nInvalid number of iterations. Iterations of tas must be in [1,200] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-si"))
            {
                rmpi_args->arg_int[0] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[0] > 10000 || rmpi_args->arg_int[0] < 0){
                    
                    UARTPuts("\r\nInvalid number of step incriments. Value must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-d"))
            {
                if (!strcmp((const char *)argv[i+1], "+"))
                {
                    rmpi_args->arg_double[0] = (1)*rmpi_args->arg_double[0];
                    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
                }
                
                else if (!strcmp((const char *)argv[i+1], "-"))
                {
                    rmpi_args->arg_double[0] = (-1)*(rmpi_args->arg_double[0]);
                    g_args_goto[0].state_desired.q = RMPI_START_NEG_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_NEG_X;
                }
                
                else
                {
                    UARTPuts("\r\nInvalid direction argument.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else
            {
                UARTPuts("\r\nrmpi: error: Invalid sub-argument has been entered.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT);
            }
        }
    }
    
    /* request repositioning to starting coordinates */
    rmpi_ret = util_checkpoint_yn(rmpi_goto_format, rmpi_ptr);
    
    /* goto - reset postion of carriage */
    if (rmpi_ret == 1)
    {
        bbmc_sysflags_set(&g_flags, "-cmd");
        
        UARTPuts("\r\nReseting to HOME position\r\n", -1);
        
        rmpi_ret = cmnd_goto(-1, NULL);
        
        bbmc_sysflags_clear(&g_flags, "-cmd");
        
        bbmc_isr_return_value(RETURN_GOTO, rmpi_ret);
        
        if (rmpi_ret < 0)
        {
            return rmpi_ret;
        }
    }
    
    /* Execution checkpoint: start initiated by user with verification */
    g_flags.contrl_run = util_checkpoint_yn(rmpi_break_format, rmpi_ptr);
    
    if (g_flags.contrl_run == 1)
    {
        /* final pre-execution checkpoint */
        UARTPuts("\r\nPress any key to start BREAKAWAY...", -1);
        UARTGets(rmpi_ptr,  2);
        //rmpi_ptr = UARTGets(rmpi_ptr,  2);
        
        for(i=0; i < rmpi_iter; i++)
        {
            g_flags.datalog = 1;
            rmpi_ret = func_rmpi(rmpi_dof);
            
            if (rmpi_state->qepCount_nxt >= rmpi_limits->limval[0] || 
                rmpi_state->qepCount_nxt <= rmpi_limits->limval[1])
            {
                g_flags.datalog = 0;
                g_flags.contrl_run = 1;
                g_flags.exec_checkpoint = 1;
                
                UARTPuts("\r\nReseting to HOME position\r\n", -1);
                
                rmpi_ret = cmnd_goto(-1, NULL);
                
                g_flags.contrl_run = 0;
                g_flags.exec_checkpoint = 0;
                
                if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM))
                {
                    UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW))
                {
                    UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN))
                {
                    UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else
                {
                    UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
                    return (RETURN_ERROR_UNKNOWN);
                }
            }
        }
        
        UARTPuts("\r\nBreakaway has completed its run.\r\n", -1);
        
        /* system flag cleanup */
        g_flags.contrl_run = 0;
        g_flags.isr_qep_enable = 0;
        g_flags.exec_checkpoint = 0;
        g_flags.datalog = 0;
        g_flags.stop_immediate = 0;
        
        /* rmpi service exit */
        UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
        return (RETURN_RMPI + RETURN_RMPI_BREAK);
    }
    else
    {
        UARTPuts("\r\n\tBreakaway has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}

/** STEP RESPONSE experiment **/
else if (!strcmp((const char *)argv[1],"step")){
    
    /* init system g_flags */
    g_flags.exec_checkpoint = 1;
    g_flags.isr_qep_enable = 0;
    g_flags.stop_immediate = 0;
    
    /* default RMPI values for step test */
    rmpi_dof = -1;             /* default axis is axis1: Y axis */
    rmpi_iter = 1;              /* number of times test will be executed. */
    rmpi_steps = 1;
    
    /* init goto parameters */
    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    
    /* argument map:
     * 
     *  i0: start interval      , d0: step value
     *  i1: stop interval       , d1: -
     * 
    */
    
    arg_output_min = -1;
    arg_output_max = 0;
    arg_output_incr = 0;
    
    /* check standard arguments */
    rmpi_dof = atoi(argv[2]);
    
    if (rmpi_dof == 1)
    {
        rmpi_state = &g_dof_state[0];
        rmpi_args = &g_args_contrl[0];
        rmpi_limits = &g_position_limits[0];
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_Y;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_Y;
        rmpi_pos_max_count = EQEP_1_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_1;
        
        /* setup for step functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_step;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP1_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP1_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP1_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_1_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_1_pwm_dir;
        #endif
    }
    
    else if (rmpi_dof == 2)
    {
        rmpi_state = &g_dof_state[1];
        rmpi_args = &g_args_contrl[1];
        rmpi_limits = &g_position_limits[1];
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_X;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_X;
        rmpi_pos_max_count = EQEP_2_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_2;
        
        /* setup for step functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_step;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP2_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP2_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP2_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_2_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_2_pwm_dir;
        #endif
    }
    
    else
    {
        UARTPuts("\r\nInvalid value for device argument.\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* check for extra(optional) arguments */
    if (argc > 4)
    {
        for(i=3; i < argc; i++)
        {
            if (!strcmp((const char *)argv[i], "-i"))
            {
                rmpi_iter = atoi(argv[i+1]);
                
                if (rmpi_iter > 1000 || rmpi_iter < 0)
                {
                    UARTPuts("\r\nInvalid number of iterations. Iterations of tas must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sr"))
            {
                arg_output_min = util_strtod(argv[i+1], NULL);
                arg_output_max = util_strtod(argv[i+2], NULL);
                arg_output_incr = util_strtod(argv[i+3], NULL);
                
                if ((arg_output_max - arg_output_min)>0)
                {
                    rmpi_steps = ((arg_output_max - arg_output_min)/arg_output_incr) + 1;
                }
                
                else if ((arg_output_max - arg_output_min) == 0)
                {
                    UARTPuts("\r\nDuty values range cannot be zero.Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                else
                {
                    UARTPuts("\r\nDuty values range negative. Perhaps values max,min have been inputed in reverse?.Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i += 3;
            }
            
            else if (!strcmp((const char *)argv[i], "-sd"))
            {
                rmpi_val = util_strtod(argv[i+1], NULL);
                
                if ((rmpi_val <= 0) || (rmpi_val > 10))
                {
                    UARTPuts("\r\nInvalid value for starting time. Value must be in range (0,10] seconds. Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_val = rmpi_val*(rmpi_state->qepSamplingFreq);
                rmpi_args->arg_int[0] = (int)rmpi_val;
                
                rmpi_val = util_strtod(argv[i+2], NULL);
                
                if ((rmpi_val <= 0) || (rmpi_val > 120))
                {
                    UARTPuts("\r\nInvalid value for stoping time. Value must be in range (0,120] seconds. Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_val = rmpi_val*(rmpi_state->qepSamplingFreq);
                rmpi_args->arg_int[1] = (int)rmpi_val;
                
                i+=2;
            }
            
            else if (!strcmp((const char *)argv[i], "-d"))
            {
                if (!strcmp((const char *)argv[i+1], "+"))
                {
                    rmpi_args->arg_double[0] = (1)*rmpi_args->arg_double[0];
                    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
                }
                
                else if (!strcmp((const char *)argv[i+1], "-"))
                {
                    rmpi_args->arg_double[0] = (-1)*(rmpi_args->arg_double[0]);
                    g_args_goto[0].state_desired.q = RMPI_START_NEG_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_NEG_X;
                }
                
                else
                {
                    UARTPuts("\r\nInvalid direction argument.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            else
            {
                if (i > 3)
                {
                    UARTPuts("\r\nrmpi: error: Invalid sub-argument has been entered.\r\n", -1);
                    return (RETURN_ERROR_INVALID_SUBARG);
                }
            }
        }
    }
    else
    {
        UARTPuts("\r\n\tWarning: Default Configurations will be set.\r\n", -1);
        
        if (arg_output_min == -1)
        {
            arg_output_min = util_strtod(argv[3], NULL);
            
            rmpi_args->arg_double[0] = arg_output_min;
            arg_output_incr = 0;
            
            if ((arg_output_min > 100) || (arg_output_min <= 0))
            {
                UARTPuts("\r\nrmpi: error: Values for Step input are invalid or have not been specified.\r\nValue must be in range (0,100].\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
        }
        
    }
    
    /* request repositioning to HOME coordinates */
    rmpi_ret = util_checkpoint_yn(rmpi_goto_format, rmpi_ptr);
    
    /* goto - reset postion of carriage */
    if (rmpi_ret == 1)
    {
        g_flags.contrl_run = 1;
        g_flags.exec_checkpoint = 1;
        
        UARTPuts("\r\nReseting to HOME position\r\n", -1);
        
        rmpi_ret = cmnd_goto(-1, NULL);
        
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        
        if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM))
        {
            UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
            return (rmpi_ret);
        }
        
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW))
        {
            UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
            return (rmpi_ret);
        }
        
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN))
        {
            UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
            return (rmpi_ret);
        }
        
        else
        {
            UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
            return (RETURN_ERROR_UNKNOWN);
        }
    }
    
    /* Execution checkpoint: start initiated by user with verification */
    g_flags.contrl_run = util_checkpoint_yn(rmpi_step_format, rmpi_ptr);
    
    if (g_flags.contrl_run == 1)
    {
        /* final pre-execution checkpoint */
        UARTPuts("\r\nPress any key to start STEP...", -1);
        UARTGets(rmpi_ptr,  2);
        //rmpi_ptr = UARTGets(rmpi_ptr,  2, stdin);
        
        for(j=0; j < rmpi_steps; j++)
        {
            for(i=0; i < rmpi_iter; i++)
            {
                g_flags.contrl_run = 1;
                g_flags.exec_checkpoint = 1;
                g_flags.datalog = 1;
                g_flags.stop_immediate = 0;
                
                rmpi_ret = func_rmpi(rmpi_dof);
                    
                g_flags.datalog = 0;
                
                UARTPuts("\r\nReseting to HOME position\r\n", -1);
                
                rmpi_ret = cmnd_goto(-1, NULL);
                
                g_flags.contrl_run = 0;
                g_flags.exec_checkpoint = 0;
                
                if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
            
                    UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW))
                {
                    UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN))
                {
                    UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
                    return (rmpi_ret);
                }
                
                else
                {
                    UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
                    return (RETURN_ERROR_UNKNOWN);
                }
                
            }
            
            rmpi_args->arg_double[0] += arg_output_incr;
        }
        
        UARTPuts("\r\nStep Response experiment has completed its run.\r\n", -1);
        
        /* system flag cleanup */
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        g_flags.datalog = 0;
        g_flags.stop_immediate = 0;
        g_flags.isr_qep_enable = 0;
        
        /* rmpi service exit */
        UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
        return (RETURN_SUCCESS);
    }
    
    else
    {
        UARTPuts("\r\n\tStep has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}

/** STEP2: Step-to-Step Respones experiment **/
else if (!strcmp((const char *)argv[1],"step2")){
    
    /* init system g_flags */
    g_flags.exec_checkpoint = 1;
    g_flags.isr_qep_enable = 0;
    g_flags.stop_immediate = 0;
    
    /* default RMPI values for step2 experiment */
    rmpi_iter = 1;              /* number of times test will be executed. */
    rmpi_steps = 1;
    
    /* init goto parameters */
    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    
    /* argument map:
     * 
     *  i0: start interval      , d0: first step value
     *  i1: stop interval       , d1: second step value
     *  i2: switch interval     , d2: -
     * 
    */
    
    arg_output_min = -1;
    arg_output_max = -1;
    arg_output_incr = -1;
    
    /* check standard arguments */
    rmpi_dof = atoi(argv[2]);
    
    if (rmpi_dof == 1){
        
        rmpi_state = &g_dof_state[0];
        rmpi_args = &g_args_contrl[0];
        rmpi_limits = &g_position_limits[0];
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_Y;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_Y;
        rmpi_pos_max_count = EQEP_1_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_1;
        
        /* setup for step2 functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_step2;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP1_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP1_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP1_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_1_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_1_pwm_dir;
        #endif
    }
    else if (rmpi_dof == 2){
        
        rmpi_state = &g_dof_state[1];
        rmpi_args = &g_args_contrl[1];
        rmpi_limits = &g_position_limits[1];
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_X;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_X;
        rmpi_pos_max_count = EQEP_2_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_2;
        
        /* setup for step2 functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_step2;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP2_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP2_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP2_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_2_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_2_pwm_dir;
        #endif
    }
    else{
        UARTPuts("\r\nInvalid value for device argument.\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* check for extra(optional) arguments */
    if (argc > 3){
    
        for(i=3; i < argc; i++){
            
            if (!strcmp((const char *)argv[i], "-i")){
                
                rmpi_iter = atoi(argv[i+1]);
                
                if (rmpi_iter > 1000 || rmpi_iter < 0){
                    
                    UARTPuts("\r\nInvalid number of iterations. Iterations of tas must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sv")){
                
                arg_output_min = util_strtod(argv[i+1], NULL);
                rmpi_args->arg_double[0] = arg_output_min;
                
                arg_output_max = util_strtod(argv[i+2], NULL);
                rmpi_args->arg_double[1] = arg_output_max;
                
                if ((arg_output_max < 0) || (arg_output_min < 0)){
                    
                    UARTPuts("\r\nDuty values cannot of different sign. Step-to-Step is always in one direction.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                if ((arg_output_max == 0) || (arg_output_min == 0)){
                    
                    UARTPuts("\r\nDuty values cannot be zero. If a from-zero step response is desired use STEP functionality.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i += 2;
            }
            
            else if (!strcmp((const char *)argv[i], "-sd")){
                
                rmpi_val = util_strtod(argv[i+1], NULL);
                
                if ((rmpi_val <= 0) || (rmpi_val > 10)){
                    
                    UARTPuts("\r\nInvalid value for starting time. Value must be in range (0,10] seconds. Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_val = rmpi_val*(rmpi_state->qepSamplingFreq);
                rmpi_args->arg_int[0] = (int)rmpi_val;
                
                rmpi_val = util_strtod(argv[i+2], NULL);
                
                if ((rmpi_val <= 0) || (rmpi_val > 120)){
                    
                    UARTPuts("\r\nInvalid value for stoping time. Value must be in range (0,120] seconds. Retry... \r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_val = rmpi_val*(rmpi_state->qepSamplingFreq);
                rmpi_args->arg_int[1] = (int)rmpi_val;
                
                i+=2;
            }
            
            else if (!strcmp((const char *)argv[i], "-sc")){
                
                rmpi_args->arg_int[2] = atoi(argv[i+1]);
                
                if ((rmpi_args->arg_int[2] <= 2000) || (rmpi_args->arg_int[2] > RMPI_STEP2_MAX_DURATION)){
                    
                    UARTPuts("\r\nInvalid value for Step change time.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-d")){
                
                if (!strcmp((const char *)argv[i+1], "+")){
                    
                    rmpi_args->arg_double[0] = (1)*rmpi_args->arg_double[0];
                    rmpi_args->arg_double[1] = (1)*rmpi_args->arg_double[1];
                    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
                }
                else if (!strcmp((const char *)argv[i+1], "-")){
                
                    rmpi_args->arg_double[0] = (-1)*(rmpi_args->arg_double[0]);
                    rmpi_args->arg_double[1] = (-1)*rmpi_args->arg_double[1];
                    g_args_goto[0].state_desired.q = RMPI_START_NEG_Y;
                    g_args_goto[1].state_desired.q = RMPI_START_NEG_X;
                }
                else{
                    
                    UARTPuts("\r\nInvalid direction argument.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else{
                
                UARTPuts("\r\nrmpi: error: Invalid sub-argument has been entered.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT);
            }
        }
    }
    else{
        
        UARTPuts("\r\n\tWarning: Default configurations will be set.\r\n", -1);
        
        if (arg_output_min == -1){
            
            arg_output_min = util_strtod(argv[3], NULL);
            arg_output_max = arg_output_min*1.3;                    /* default final step is 130% of initial input step value */
            
            if (arg_output_max > 100){
                
                arg_output_max = 100;
            }
            
            rmpi_args->arg_double[0] = arg_output_min;
            rmpi_args->arg_double[1] = arg_output_max;
            
            if ((arg_output_min > 80) || (arg_output_min <= 0)){
                
                UARTPuts("\r\nrmpi: error: Value for step input are invalid or have not been specified.\r\nValue must be in range (0,80].\r\n", -1);
                return (RETURN_ERROR_INVALID_ARG);
            }
        }
    }
    
    /* request repositioning to HOME coordinates */
    rmpi_ret = util_checkpoint_yn(rmpi_goto_format, rmpi_ptr);
    
    /* goto - reset postion of carriage */
    if (rmpi_ret == 1){
        
        g_flags.contrl_run = 1;
        g_flags.exec_checkpoint = 1;
        
        UARTPuts("\r\nReseting to HOME position\r\n", -1);
        
        rmpi_ret = cmnd_goto(-1, NULL);
        
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        
        if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
        
            UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                
            UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
               
            UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
            return (rmpi_ret);
        }
        else{
               
            UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
            return (RETURN_ERROR_UNKNOWN);
        }
    }
    
    /* Execution checkpoint: start initiated by user with verification */
    g_flags.contrl_run = util_checkpoint_yn(rmpi_step2_format, rmpi_ptr);
    
    if (g_flags.contrl_run == 1){
        
        /* final pre-execution checkpoint */
        UARTPuts("\r\nPress any key to start STEP2...", -1);
        UARTGets(rmpi_ptr,  2);
        //rmpi_ptr = UARTGets(rmpi_ptr,  2, stdin);
        
        for(i=0; i < rmpi_iter; i++){
            
            g_flags.contrl_run = 1;
            g_flags.exec_checkpoint = 1;
            g_flags.datalog = 1;
            g_flags.stop_immediate = 0;
            
            rmpi_ret = func_rmpi(rmpi_dof);
                
            g_flags.datalog = 0;
            
            UARTPuts("\r\nReseting to HOME position\r\n", -1);
            
            rmpi_ret = cmnd_goto(-1, NULL);
            
            g_flags.contrl_run = 0;
            g_flags.exec_checkpoint = 0;
            
            if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
            
                UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                    
                UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
                   
                UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
                return (rmpi_ret);
            }
            else{
                   
                UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
                return (RETURN_ERROR_UNKNOWN);
            }
        }
        
        UARTPuts("\r\nStep2 has completed its run.\r\n", -1);
        
        /* cleanup */
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        g_flags.datalog = 0;
        g_flags.stop_immediate = 0;
        g_flags.isr_qep_enable = 0;
        
        /* rmpi service exit */
        UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
        return (RETURN_SUCCESS);
    }
    else{
        
        UARTPuts("\r\n\tStep2 has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}

/** SINE response experiment **/
else if (!strcmp((const char *)argv[1],"sine")){
    
    /* init system g_flags */
    g_flags.exec_checkpoint = 1;
    g_flags.isr_qep_enable = 0;
    g_flags.stop_immediate = 0;
    
    /* default RMPI values for breakaway test */
    rmpi_dof = -1;                             /* default axis is axis1: Y axis */
    rmpi_qep_max_count = 0;
    rmpi_timer = RMPI_TIMER_DEFAULT;
    rmpi_iter = 1;                              /* number of times test will be executed. */
    rmpi_steps = 0;
    
    /* used as buffers */
    arg_output_min = 0;
    arg_output_max = 0;
    arg_output_incr = 0;
    
    /* init goto parameters */
    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    
    /* argument map:
     * 
     *  i0: stop interval       , d0: 
     *  i1: -                   , d1: 
     *  i2: -                   , d2: 
     *  i3: -                   , d3: 
     *  i4: -                   , d4: 
     *  i5: -                   , d5: 
     *  i6: -                   , d6: 
     *  i7: -                   , d7: 
    */
    
    /* configure selected axis */
    rmpi_dof = atoi(argv[2]);
        
    if (rmpi_dof == 1){
        
        rmpi_state = &g_dof_state[0];
        rmpi_args = &g_args_contrl[0];
        rmpi_limits = &g_position_limits[0];
        
        rmpi_val = 1;
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_Y;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_Y;
        rmpi_pos_max_count = EQEP_1_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_1;
        
        /* default setup for SINE functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_sine;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP1_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP1_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP1_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_1_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_1_pwm_dir;
        #endif
    }
    else if (rmpi_dof == 2){
        
        rmpi_state = &g_dof_state[1];
        rmpi_args = &g_args_contrl[1];
        rmpi_limits = &g_position_limits[1];
        
        rmpi_val = 1;
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_X;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_X;
        rmpi_pos_max_count = EQEP_2_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_2;
        
        /* default setup for SINE functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_sine;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP2_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP2_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP2_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_2_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_2_pwm_dir;
        #endif
    }
    else{
        UARTPuts("\r\nInvalid value for device argument.\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* limiting frequency */
    arg_output_max = ((rmpi_state->qepSamplingFreq))/2;
    
    /* check for extra(optional) arguments */
    if (argc > 3){
    
        for(i=3; i < argc; i++){
            
            if (!strcmp((const char *)argv[i], "-i")){
                
                rmpi_iter = atoi(argv[i+1]);
                
                if (rmpi_iter > 1000 || rmpi_iter < 0){
                    
                    UARTPuts("\r\nInvalid number of iterations. Iterations of tas must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-samp")){
                
                rmpi_args->arg_double[0] = util_strtod(argv[i+1], NULL);
                
                if (rmpi_args->arg_double[0] > 100 || rmpi_args->arg_double[0] < 0){
                    
                    UARTPuts("\r\nInvalid duty amplitude. Value must be in [0,100] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sprd")){
                
                arg_output_incr = util_strtod(argv[i+1], NULL);
                rmpi_args->arg_double[1] = 1/arg_output_incr;
                
                if ((rmpi_args->arg_double[1] > arg_output_max) || (rmpi_args->arg_double[1] < 0.0001)){
                    
                    UARTPuts("\r\nInvalid Period value. Value must be in [2000/(Fs),10000] (msec) range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sphs")){
                
                rmpi_args->arg_double[2] = util_strtod(argv[i+1], NULL);
                
                if (rmpi_args->arg_double[2] > 1 || rmpi_args->arg_double[2] < 0){
                    
                    UARTPuts("\r\nInvalid phase offset value. Value must be in [0,1] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-soff")){
                
                arg_output_min = util_strtod(argv[i+1], NULL);
                
                if (arg_output_min > 100 || arg_output_min < 0){
                    
                    UARTPuts("\r\nInvalid number of step incriment value. Value must be in [0,100] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sd")){
                
                /* get starting time */
                rmpi_args->arg_int[0] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[0] > 30 || rmpi_args->arg_int[0] < 0){
                    
                    UARTPuts("\r\nInvalid execution duration value. Value must be in [0,30] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_args->arg_int[0] *= (rmpi_state->qepSamplingFreq);
                
                /* get termination time */
                rmpi_args->arg_int[1] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[1] > 120 || rmpi_args->arg_int[1] < 0){
                    
                    UARTPuts("\r\nInvalid execution duration value. Value must be in [0,120] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_args->arg_int[1] *= (rmpi_state->qepSamplingFreq);
                
                i+=2;
            }

            else if (!strcmp((const char *)argv[i], "-d")){
                
                if (!strcmp((const char *)argv[i+1], "+")){
                    
                    rmpi_val = 1;
                }
                else if (!strcmp((const char *)argv[i+1], "-")){
                
                    rmpi_val = -1;
                }
                else{
                    UARTPuts("\r\nInvalid number of step incriments. Value must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            else{
                
                UARTPuts("\r\nrmpi: error: Invalid option has been entered.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT);
            }
        }
    }
    else{
        
        UARTPuts("\r\nrmpi: error: Not enough arguments have been specified.\r\n\td[k] = amp*sin(2pi(k/T + phase)) + offset\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* final configurations */
    if (rmpi_val == 1){
                    
        g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
        g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    }
    else if (rmpi_val == -1){
        
        g_args_goto[0].state_desired.q = RMPI_START_NEG_Y;
        g_args_goto[1].state_desired.q = RMPI_START_NEG_X;
    }
    else{
              
        UARTPuts("\r\nrmpi: error: unknown error has occured when evaluating 'rmpi_val' variable.\r\n", -1);
        return (RETURN_ERROR_UNKNOWN);
    }
    
    /* request repositioning to HOME coordinates */
    rmpi_ret = util_checkpoint_yn(rmpi_goto_format, rmpi_ptr);
    
    /* goto - reset postion of carriage */
    if (rmpi_ret == 1){
        
        g_flags.contrl_run = 1;
        g_flags.exec_checkpoint = 1;
        
        UARTPuts("\r\nReseting to HOME position\r\n", -1);
        
        rmpi_ret = cmnd_goto(-1, NULL);
        
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        
        if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
        
            UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                
            UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
               
            UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
            return (rmpi_ret);
        }
        else{
               
            UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
            return (RETURN_ERROR_UNKNOWN);
        }
    }
    
    /* Execution checkpoint: start initiated by user with verification */
    g_flags.contrl_run = util_checkpoint_yn(rmpi_sine_format, rmpi_ptr);
    
    if (g_flags.contrl_run == 1){
        
        /* save previous contexts/settings */
        //DMTimerContextSave(DMTIMER_RMPI, &rmpi_timer_context);
        //eQEPx_ContextSave(rmpi_dof, &rmpi_qep_context);
        
        /* setup special timer settings for accurate speed measurement */
        //setup_timer(DMTIMER_RMPI, rmpi_timer);
        
        /*#ifdef INPUT_QEP_DUAL
            eQEPx_Config(rmpi_dof, QDECCTL_DEFAULT, RMPI_QEPCTL, RMPI_QCAP, 
                        QPOSCTL_DEFAULT, rmpi_qep_max_count, RMPI_UTMR_PRD);
        #endif
        #ifdef INPUT_QEP_STD
            eQEPx_Config(rmpi_dof, QDECCTL_DEFAULT, RMPI_QEPCTL, CEN_DISABLE, 
                        QPOSCTL_DEFAULT, rmpi_qep_max_count, RMPI_UTMR_PRD);
        #endif
        #ifdef INPUT_QEP_CAP
            eQEPx_Config(rmpi_dof, QDECCTL_DEFAULT, RMPI_QEPCTL, RMPI_QCAP, 
                        QPOSCTL_DEFAULT, rmpi_qep_max_count, RMPI_UTMR_PRD);
        #endif
        
        system_state_config(rmpi_state, rmpi_timer, (int)(rmpi_val), SPEED_MODE_THRESHOLD, RMPI_UPS_DEFAULT, RMPI_CPS_DEFAULT, EQEP_FUNC_CLK, rmpi_max_motor_speed);
        */
        
        /* final pre-execution checkpoint */
        UARTPuts("\r\nPress any key to start Sine Response Experiment...", -1);
        UARTGets(rmpi_ptr,  2);
        //rmpi_ptr = UARTGets(rmpi_ptr,  2);
        
        for(i=0; i < rmpi_iter; i++){
            
            /* prepare necessary g_flags and parameters*/
            g_flags.contrl_run = 1;
            g_flags.exec_checkpoint = 1;
            g_flags.datalog = 1;
            g_flags.stop_immediate = 0;
            rmpi_state->flag = 0;
            
            /* rmpi execution */
            rmpi_ret = func_rmpi(rmpi_dof);
            
            /* reset to home position before next iteration */
            g_flags.datalog = 0;
            
            UARTPuts("\r\nReseting to HOME position\r\n", -1);
            
            rmpi_ret = cmnd_goto(-1, NULL);
            
            g_flags.contrl_run = 0;
            g_flags.exec_checkpoint = 0;
            
            if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
        
                UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                    
                UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
                   
                UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
                return (rmpi_ret);
            }
            else{
                   
                UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
                return (RETURN_ERROR_UNKNOWN);
            }
        }
        
        UARTPuts("\r\nSINE has completed its run.\r\n", -1);
        
        /* system flag cleanup */
        g_flags.contrl_run = 0;
        g_flags.isr_qep_enable = 0;
        g_flags.exec_checkpoint = 0;
        g_flags.datalog = 0;
        g_flags.stop_immediate = 0;
        
        /* restore previous contexts/settings/configurations */
        //DMTimerContextRestore(DMTIMER_RMPI, &rmpi_timer_context);
        //eQEPx_ContextRestore(rmpi_dof, rmpi_qep_context);
        
        /*if (rmpi_dof == 1){
            
            system_state_config(rmpi_state, timer_rld, ENC_1_LIN_PER_ROT, SPEED_MODE_THRESHOLD, EQEP_1_UPS_DEFAULT, EQEP_1_CPS_DEFAULT, EQEP_FUNC_CLK, MAX_SPEED_MOTOR_1);
        }
        if (rmpi_dof == 2){
            
            system_state_config(rmpi_state, timer_rld, ENC_2_LIN_PER_ROT, SPEED_MODE_THRESHOLD, EQEP_2_UPS_DEFAULT, EQEP_2_CPS_DEFAULT, EQEP_FUNC_CLK, MAX_SPEED_MOTOR_2);
        }*/
        
        /* rmpi service exit */
        UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
        return (RETURN_RMPI + RETURN_RMPI_SINE);
    }
    else{
        
        UARTPuts("\r\n\tSINE has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}       

/** PRBS response for system identification **/
else if (!strcmp((const char *)argv[1],"prbs")){
    
    // specify axis
    // specify v1 or v0
    // specify initial direction
    // other options?
    
    rmpi_ret = signalgen_prbs_v0(&g_signalgen_log, 0);
    rmpi_ret = signalgen_prbs_v1(&g_signalgen_log, g_prbs_v1_table, 20000, 200, 1);
    
    /* init system g_flags */
    g_flags.exec_checkpoint = 1;
    g_flags.isr_qep_enable = 0;
    g_flags.stop_immediate = 0;
    
    
    
    /* default RMPI values for breakaway test */
    rmpi_dof = -1;                             /* default axis is axis1: Y axis */
    rmpi_qep_max_count = 0;
    rmpi_timer = RMPI_TIMER_DEFAULT;
    rmpi_iter = 1;                              /* number of times test will be executed. */
    rmpi_steps = 0;
    
    /* used as buffers */
    arg_output_min = 0;
    arg_output_max = 0;
    arg_output_incr = 0;
    
    /* init goto parameters */
    g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
    g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    
    /* argument map:
     * 
     *  i0: stop interval       , d0: 
     *  i1: -                   , d1: 
     *  i2: -                   , d2: 
     *  i3: -                   , d3: 
     *  i4: -                   , d4: 
     *  i5: -                   , d5: 
     *  i6: -                   , d6: 
     *  i7: -                   , d7: 
    */
    
    /* configure selected axis */
    rmpi_dof = atoi(argv[2]);
        
    if (rmpi_dof == 1)
    {
        
        rmpi_state = &g_dof_state[0];
        rmpi_args = &g_args_contrl[0];
        rmpi_limits = &g_position_limits[0];
        
        rmpi_val = 1;
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_Y;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_Y;
        rmpi_pos_max_count = EQEP_1_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_1;
        
        /* default setup for SINE functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_sine;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP1_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP1_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP1_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_1_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_1_pwm_dir;
        #endif
    }
    else if (rmpi_dof == 2)
    {
        
        rmpi_state = &g_dof_state[1];
        rmpi_args = &g_args_contrl[1];
        rmpi_limits = &g_position_limits[1];
        
        rmpi_val = 1;
        
        //! limval_conf()
        
        rmpi_limits->limval[0] = POSITION_THRESH_POS_X;
        rmpi_limits->limval[1] = POSITION_THRESH_NEG_X;
        rmpi_pos_max_count = EQEP_2_MAX_COUNTS;
        rmpi_max_motor_speed = MAX_SPEED_MOTOR_2;
        
        /* default setup for SINE functionality */
        rmpi_funcs.traject_func = &traject_null;
        rmpi_funcs.contrl_func = &contrl_rmpi_sine;
        rmpi_funcs.term_func = &term_rmpi_step;
        
        #ifdef INPUT_QEP_DUAL
            rmpi_io.input_func = &eQEP2_Update_dual;
        #endif
        #ifdef INPUT_QEP_STD
            rmpi_io.input_func = &eQEP2_Update_std;
        #endif
        #ifdef INPUT_QEP_CAP
            rmpi_io.input_func = &eQEP2_Update_cap;
        #endif  
        #ifdef OUTPUT_PWM_DIFF
            rmpi_io.output_func = &output_2_pwm_ab;
        #endif
        #ifdef OUTPUT_PWM_DIR
            rmpi_io.output_func = &output_2_pwm_dir;
        #endif
    }
    else{
        UARTPuts("\r\nInvalid value for device argument.\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* limiting frequency */
    arg_output_max = ((rmpi_state->qepSamplingFreq))/2;
    
    /* check for extra(optional) arguments */
    if (argc > 3){
    
        for(i=3; i < argc; i++){
            
            if (!strcmp((const char *)argv[i], "-i")){
                
                rmpi_iter = atoi(argv[i+1]);
                
                if (rmpi_iter > 1000 || rmpi_iter < 0){
                    
                    UARTPuts("\r\nInvalid number of iterations. Iterations of tas must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-samp")){
                
                rmpi_args->arg_double[0] = util_strtod(argv[i+1], NULL);
                
                if (rmpi_args->arg_double[0] > 100 || rmpi_args->arg_double[0] < 0){
                    
                    UARTPuts("\r\nInvalid duty amplitude. Value must be in [0,100] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sprd")){
                
                arg_output_incr = util_strtod(argv[i+1], NULL);
                rmpi_args->arg_double[1] = 1/arg_output_incr;
                
                if ((rmpi_args->arg_double[1] > arg_output_max) || (rmpi_args->arg_double[1] < 0.0001)){
                    
                    UARTPuts("\r\nInvalid Period value. Value must be in [2000/(Fs),10000] (msec) range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sphs")){
                
                rmpi_args->arg_double[2] = util_strtod(argv[i+1], NULL);
                
                if (rmpi_args->arg_double[2] > 1 || rmpi_args->arg_double[2] < 0){
                    
                    UARTPuts("\r\nInvalid phase offset value. Value must be in [0,1] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-soff")){
                
                arg_output_min = util_strtod(argv[i+1], NULL);
                
                if (arg_output_min > 100 || arg_output_min < 0){
                    
                    UARTPuts("\r\nInvalid number of step incriment value. Value must be in [0,100] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            
            else if (!strcmp((const char *)argv[i], "-sd")){
                
                /* get starting time */
                rmpi_args->arg_int[0] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[0] > 30 || rmpi_args->arg_int[0] < 0){
                    
                    UARTPuts("\r\nInvalid execution duration value. Value must be in [0,30] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_args->arg_int[0] *= (rmpi_state->qepSamplingFreq);
                
                /* get termination time */
                rmpi_args->arg_int[1] = atoi(argv[i+1]);
                
                if (rmpi_args->arg_int[1] > 120 || rmpi_args->arg_int[1] < 0){
                    
                    UARTPuts("\r\nInvalid execution duration value. Value must be in [0,120] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                rmpi_args->arg_int[1] *= (rmpi_state->qepSamplingFreq);
                
                i+=2;
            }

            else if (!strcmp((const char *)argv[i], "-d")){
                
                if (!strcmp((const char *)argv[i+1], "+")){
                    
                    rmpi_val = 1;
                }
                else if (!strcmp((const char *)argv[i+1], "-")){
                
                    rmpi_val = -1;
                }
                else{
                    UARTPuts("\r\nInvalid number of step incriments. Value must be in [1,1000] range.\r\n", -1);
                    return (RETURN_ERROR_INVALID_OPT_VAL);
                }
                
                i++;
            }
            else{
                
                UARTPuts("\r\nrmpi: error: Invalid option has been entered.\r\n", -1);
                return (RETURN_ERROR_INVALID_OPT);
            }
        }
    }
    else{
        
        UARTPuts("\r\nrmpi: error: Not enough arguments have been specified.\r\n\td[k] = amp*sin(2pi(k/T + phase)) + offset\r\n", -1);
        return (RETURN_ERROR_INVALID_ARG);
    }
    
    /* final configurations */
    if (rmpi_val == 1){
                    
        g_args_goto[0].state_desired.q = RMPI_START_POS_Y;
        g_args_goto[1].state_desired.q = RMPI_START_POS_X;
    }
    else if (rmpi_val == -1){
        
        g_args_goto[0].state_desired.q = RMPI_START_NEG_Y;
        g_args_goto[1].state_desired.q = RMPI_START_NEG_X;
    }
    else{
              
        UARTPuts("\r\nrmpi: error: unknown error has occured when evaluating 'rmpi_val' variable.\r\n", -1);
        return (RETURN_ERROR_UNKNOWN);
    }
    
    /* request repositioning to HOME coordinates */
    rmpi_ret = util_checkpoint_yn(rmpi_goto_format, rmpi_ptr);
    
    /* goto - reset postion of carriage */
    if (rmpi_ret == 1){
        
        g_flags.contrl_run = 1;
        g_flags.exec_checkpoint = 1;
        
        UARTPuts("\r\nReseting to HOME position\r\n", -1);
        
        rmpi_ret = cmnd_goto(-1, NULL);
        
        g_flags.contrl_run = 0;
        g_flags.exec_checkpoint = 0;
        
        if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
        
            UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                
            UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
            return (rmpi_ret);
        }
        else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
               
            UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
            return (rmpi_ret);
        }
        else{
               
            UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
            return (RETURN_ERROR_UNKNOWN);
        }
    }
    
    /* Execution checkpoint: start initiated by user with verification */
    g_flags.contrl_run = util_checkpoint_yn(rmpi_sine_format, rmpi_ptr);
    
    if (g_flags.contrl_run == 1){
        
        /* final pre-execution checkpoint */
        UARTPuts("\r\nPress any key to start Sine Response Experiment...", -1);
        UARTGets(rmpi_ptr,  2);
        //rmpi_ptr = UARTGets(rmpi_ptr,  2);
        
        for(i=0; i < rmpi_iter; i++){
            
            /* prepare necessary g_flags and parameters*/
            g_flags.contrl_run = 1;
            g_flags.exec_checkpoint = 1;
            g_flags.datalog = 1;
            g_flags.stop_immediate = 0;
            rmpi_state->flag = 0;
            
            /* rmpi execution */
            rmpi_ret = func_rmpi(rmpi_dof);
            
            /* reset to home position before next iteration */
            g_flags.datalog = 0;
            
            UARTPuts("\r\nReseting to HOME position\r\n", -1);
            
            rmpi_ret = cmnd_goto(-1, NULL);
            
            g_flags.contrl_run = 0;
            g_flags.exec_checkpoint = 0;
            
            if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_GPIO_LIM)){
        
                UARTPuts("\r\nWARNING! GOTO stoped due to Hall-Posiiton-Limiter.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_KILLSW)){
                    
                UARTPuts("\r\nWARNING! GOTO stoped due to Killswitch.\r\n", -1);
                return (rmpi_ret);
            }
            else if (rmpi_ret == (RETURN_GOTO + ISR_RETURN_CLEAN)){
                   
                UARTPuts("\r\nSuccessfuly returned to starting position.\r\n", -1);
                return (rmpi_ret);
            }
            else{
                   
                UARTprintf("\r\nWarning!: GOTO has ended due to unrecognized event with return value: %d.\r\n", rmpi_ret);
                return (RETURN_ERROR_UNKNOWN);
            }
        }
        
        UARTPuts("\r\nSINE has completed its run.\r\n", -1);
        
        /* system flag cleanup */
        g_flags.contrl_run = 0;
        g_flags.isr_qep_enable = 0;
        g_flags.exec_checkpoint = 0;
        g_flags.datalog = 0;
        g_flags.stop_immediate = 0;
        
        /* rmpi service exit */
        UARTPuts("\r\nReturning to BBMC-CLI.\r\n", -1);
        return (RETURN_RMPI + RETURN_RMPI_PRBS);
    }
    else{
        
        UARTPuts("\r\n\tPRBS has been aborted.\r\n", -1);
        return (RETURN_ERROR_RUN_ABORT);
    }
}

/* escape condition for unrecognized rmpi facility*/
else{
    UARTPuts("\r\nInvalid arguments. Retry...\r\n", -1);
    return -3;
}



void 
m_trapezoid_setup(int plmode)
{
    state_d.step_num = (int)((state_d.t_f)/state_d.t_step);
    
    UARTPutNum(state_d.step_num);
    
    state_d.acc_d_2 = ((state_d.spd_ss)/(state_d.t_a))/2;
    state_d.pos_a = state_d.acc_d_2*state_d.t_a*state_d.t_a;
    state_d.pos_d = state_d.spd_ss*(state_d.t_f-1.5*state_d.t_a);
}

void 
m_trapezoid_online(void)
{
    double temp;
    
    state_d.timr_currunt = state_d.t_curr + state_d.t_step;
    
    if (state_d.t_curr >= state_d.t_f)
    {
        return 1;
    }
    
    if (state_d.t_curr <= state_d.t_a)
    {
        state_d.pos_curr = state_d.acc_d_2 * (state_d.t_curr * state_d.t_curr);
    }
    
    if ((state_d.t_curr > state_d.t_a) && (state_d.t_curr <= state_d.t_d))
    {
        state_d.pos_curr = state_d.spd_ss*state_d.t_curr - state_d.pos_a;
    }
    
    if ((state_d.t_curr > state_d.t_d) && (state_d.t_curr <= state_d.t_f))
    {
        temp = (state_d.t_f - state_d.t_curr);
        state_d.pos_curr = state_d.pos_d - state_d.acc_d_2 * temp * temp;
    }
    
    return 0;
}






