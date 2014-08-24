/** Module Library Header **/
#include "motor_control.h"



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
    if (func_table == NULL)
    {
        return -1;
    }
    
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



/** Functions to handle module internal data.
 *  
 */
 
int 
qei_state_set (bbmc_input_encoder_t volatile *data, unsigned int value)
{
    data->input.count[1] = value;
    
    return eqep_write (data->dev_id, value);
}



/** 
 *  Motor Input & Output functions
 *
 */

/** Input Functions **/

int 
input_qei_dual (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_DUAL, &(data->state));
    
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
input_qei_cap (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_CAP, &(data->state));
    
    data->state.speed = data->state.speed_cap;
    
    return 0;
}

int 
input_qei_std (bbmc_input_encoder_t volatile *data)
{
    eqep_read (data->dev_id, EQEP_STD, &(data->state));
    
    data->state.speed = data->state.speed_std;
    
    return 0;
}



/** Output Functions **/

void 
output_pwm_dif (bbmc_output_motor_t volatile *data)
{
    ehrpwm_write(data->dev_id, EHRPWM_WRITE_DIFF, data->value);
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
        output_gpio_dir(output->dev_id, GPIO_PIN_HIGH);
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




 
