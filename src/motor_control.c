/** Module Library Header **/
#include "motor_control.h"

/** Std C Headers **/
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>



/** Internal Data Types
 * 
 */
typedef struct
{
    bbmc_input_fp_t         input_funcs[3];
    bbmc_output_fp_t        output_funcs[2];
}
bbmc_io_func_tbl_t;


/** Internal Data
 *  
 *  
 */
static bbmc_io_func_tbl_t   g_func_tbl;


/** Configuration and Set-up functions.
 *  
 */

int 
io_func_setup (void)
{
    g_func_tbl.input_funcs[0] = input_qei_std;
    g_func_tbl.input_funcs[1] = input_qei_cap;
    g_func_tbl.input_funcs[2] = input_qei_dual;
    
    g_func_tbl.output_funcs[0] = output_pwm_dir;
    g_func_tbl.output_funcs[1] = output_pwm_dif;
    
    return 0;
}


int 
io_func_config (bbmc_io_funcs_t*    func_ptrs, 
                const char*         conf_mode, 
                const char*         io_mode)
{
    int ret;
    
    if (!strcmp((const char *)io_mode, "-i"))
    {
        if (!strcmp((const char *)conf_mode, "std"))
        {
            func_ptrs->input_func = g_func_tbl.input_funcs[0];
        }
        
        else if (!strcmp((const char *)conf_mode, "cap"))
        {
            func_ptrs->input_func = g_func_tbl.input_funcs[1];
        }
        
        else if (!strcmp((const char *)conf_mode, "dual"))
        {
            func_ptrs->input_func = g_func_tbl.input_funcs[2];
        }
        
        else
        {
            return -2;
        }
        
        ret = 0;
    }
    
    else if (!strcmp((const char *)io_mode, "-o"))
    {
        if (!strcmp((const char *)conf_mode, "dir"))
        {
            func_ptrs->output_func = g_func_tbl.output_funcs[0];
        }
        
        else if (!strcmp((const char *)conf_mode, "dif"))
        {
             func_ptrs->output_func = g_func_tbl.output_funcs[1];
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



/** Encode Configuration & Managment Functions.
 *  
 */
//TODO
int 
qei_data_init (bbmc_input_encoder_t volatile *data)
{
    if (data == NULL)
    {
        UARTPuts("\r\nerror: qei_data_init: data pointer is NULL\r\n", -1);
        return -1;
    }
    
    return dev_qei_data_init(data);
}

int 
qei_data_cpy (bbmc_input_encoder_t volatile *src,
              bbmc_input_encoder_t volatile *dest)
{
    if (src == NULL)
    {
        UARTPuts("\r\nerror: qei_data_cpy: src pointer is NULL\r\n", -1);
        return -1;
    }
    
    if (dest == NULL)
    {
        UARTPuts("\r\nerror: qei_data_cpy: dest pointer is NULL\r\n", -1);
        return -2;
    }
    
    return dev_qei_data_cpy  (src, dest);
}

int 
qei_position_set (bbmc_input_encoder_t volatile *data, unsigned int value)
{
    return dev_qei_count_set (data, value);
}

int
qei_capture_config (bbmc_input_encoder_t volatile *data,
                    unsigned int unit_position,
                    unsigned int clk_prescaler)
{
    if (data->dev_id > BBMC_DOF_NUM)
    {
        return -1;
    }
    
    unsigned int prescaler;
    
    prescaler = EQEP_SYSCLK / clk_prescaler;
    prescaler = prescaler * unit_position;
    
    data->input.cap_prescaler = prescaler;
    
    return dev_qei_cap_config (data->dev_id, unit_position, clk_prescaler);
}

int 
qei_switch_velocity (bbmc_input_encoder_t volatile *data, double switch_speed)
{
    if (data->dev_id > BBMC_DOF_NUM)
    {
        return -1;
    }
    
    if (switch_speed < 0)
    {
        return -2;
    }
    
    data->input.speed_thr = switch_speed;
    
    return 0;
}

int 
qei_motor (bbmc_input_encoder_t volatile *data, double max_motor_speed)
{
    if (data->dev_id > BBMC_DOF_NUM)
    {
        return -1;
    }
    
    max_motor_speed = (data->input.cap_prescaler) / max_motor_speed;
    
    data->input.cprd_min = (unsigned int)max_motor_speed;
    
    return 0;
}

int qei_frequency_set (bbmc_input_encoder_t volatile *data, 
                       unsigned int frequency)
{
    return dev_qei_frequency_set (data, frequency);
}

int 
qei_print (const char *format)
{
    UARTprintf("\r\n%sInput System: \r\n", format);
    UARTprintf("\r\n%sQEI configurations: \r\n", format);
    
    //TODO: must print actually values.
    UARTprintf("\r\n%seQEP::1::unit_position := %d", format, 1);
    UARTprintf("\r\n%seQEP::1::clk_prescaler := %d", format, 64);
    UARTprintf("\r\n%seQEP::2::unit_position := %d", format, 1);
    UARTprintf("\r\n%seQEP::2::clk_prescaler := %d", format, 64);
    
    return 0;
}


/** 
 *  Input Functions
 *
 */

int 
input_qei_dual (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_DUAL, &(data->input));
    
    if (data->input.speed_mode == 1)
    {
        data->input.speed = data->input.speed_std;
        
        if (fabs(data->input.speed_std) <= data->input.speed_thr)
        {
             data->input.speed_mode = 0;
        }
    }
    
    else
    {
        data->input.speed = data->input.speed_cap;
        
        if (fabs(data->input.speed_cap) > data->input.speed_thr)
        {
             data->input.speed_mode = 1;
        }
    }
    
    return 0;
}

int 
input_qei_cap (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_CAP, &(data->input));
    
    data->input.speed = data->input.speed_cap;
    
    return 0;
}

int 
input_qei_std (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_STD, &(data->input));
    
    data->input.speed = data->input.speed_std;
    
    return 0;
}


/** Primary Encoder Input Functions
 * 
 */

void 
input_encoder_1D (bbmc_input_encoder_t volatile *state)
{
    #ifdef INPUT_QEP_DUAL
        input_qei_dual(state);
    #endif
    #ifdef INPUT_QEP_STD
        input_qei_std(state);
    #endif
    #ifdef INPUT_QEP_CAP
        input_qei_cap(state);
    #endif 
}

void 
input_encoder_2D (bbmc_input_encoder_t volatile *state)
{
    #ifdef INPUT_QEP_DUAL
        input_qei_dual(&state[0]);
        input_qei_dual(&state[1]);
    #endif
    #ifdef INPUT_QEP_STD
        input_qei_std(&state[0]);
        input_qei_std(&state[1]);
    #endif
    #ifdef INPUT_QEP_CAP
        input_qei_cap(&state[0]);
        input_qei_cap(&state[1]);
    #endif 
}




/** Output Configurations & Management Functions
 * 
 */

int 
pwm_enable (unsigned int dev_id)
{
    return dev_pwm_enable(dev_id);
}

int 
pwm_disable (unsigned int dev_id)
{
    return dev_pwm_disable(dev_id);
}

int
pwm_frequency_get (unsigned int dev_id, double *frequency)
{
    return dev_pwm_frequency_get(dev_id, frequency);
}

int
pwm_frequency_set (unsigned int dev_id, double frequency, unsigned int resolution)
{
    return dev_pwm_frequency_set(dev_id, frequency, resolution);
}

int 
pwm_print (const char *format)
{
    int i;
    double freq;

    UARTprintf("\r\n%sSystem Output Configurations: \r\n", format);
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        pwm_frequency_get((i+1), &freq);
        UARTprintf("\r\n%sdof-%d pwm frequncy = %d Hz", format, (i+1), (int)freq);
    }
    
    return 0;
}




/** Output Functions 
 * 
 */

void 
output_pwm_dif (bbmc_output_motor_t volatile *data)
{
    ehrpwm_write(data->dev_id, EHRPWM_WRITE_DIFF, data->output);
}

//TODO: update the GPIO to the new driver model and change this appropriately.
void
output_gpio_dir (unsigned int dev_id, unsigned int pin_value)
{
    if (dev_id == 1)
    {
        GPIOPinWrite(DIR_1_GPIO_ADDRESS, DIR_1_GPIO_PIN, pin_value);
    }
    
    else if (dev_id == 2)
    {
        GPIOPinWrite(DIR_2_GPIO_ADDRESS, DIR_2_GPIO_PIN, pin_value);
    }
    
    else
    {
        ;
    }
}

void 
output_pwm_dir (bbmc_output_motor_t volatile *data)
{
    if (data->output >= 0)
    {
        output_gpio_dir(data->dev_id, GPIO_PIN_HIGH);
    }
    
    if (data->output < 0)
    {
        data->output = -1 * data->output;
        output_gpio_dir(data->dev_id, GPIO_PIN_LOW);
    }
    
    if (data->output > 100)
    {
        data->output = 100;
    }
    
    ehrpwm_write(data->dev_id, EHRPWM_WRITE_A, data->output);
}



/** Primary Encoder Input Functions
 * 
 */
void 
output_motor_1D (bbmc_contrl_motor_t volatile *data)
{
    #ifdef OUTPUT_PWM_DIFF
        output_pwm_dif(&data->control);
    #endif
    #ifdef OUTPUT_PWM_DIR
        output_pwm_dir(&data->control);
    #endif
}

void 
output_motor_2D (bbmc_contrl_motor_t volatile *data)
{
    #ifdef OUTPUT_PWM_DIFF
        output_pwm_dif(&data[0].control);
        output_pwm_dif(&data[1].control);
    #endif
    #ifdef OUTPUT_PWM_DIR
        output_pwm_dir(&data[0].control);
        output_pwm_dir(&data[1].control);
    #endif
}




/**
 *
 */
