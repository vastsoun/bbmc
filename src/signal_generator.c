



//!
/* Signal Generator Data */
static int                         g_prbs_v1_table[SIGNAL_PRBS_V1_ARRAY_SIZE] = 
                                        SIGNAL_PRBS_V1_SWITCH_INIT;
                                        
static int                         g_prbs_v2_table[SIGNAL_PRBS_V2_ARRAY_SIZE] = 
                                        SIGNAL_PRBS_V2_SWITCH_INIT;

static int                         g_pulse_v1_table[SIGNAL_PULSE_ARRAY_SIZE] = 
                                        SIGNAL_PULSE_SWITCH_INIT;

static datalog_s_t                   g_log_signalgen;


static sg_trapez_t                  g_signalgen_trapezoid; 
static sg_sinusoid_t                g_signalgen_sinusoid; 
static sg_circle_t                  g_signalgen_circle; 





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
