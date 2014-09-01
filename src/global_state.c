/** Module Header **/
#include "global_state.h"

/** Std C Headers **/
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <float.h>

/** Firmware Headers **/
#include "uartStdio.h"

/** BBMC Headers **/
#include "uretts_model.h"
#include "util.h"


/** Internal Data
 * 
 */

static bbmc_actuator_state_t volatile   g_state[BBMC_DOF_NUM];

static bbmc_actuator_range_t            g_position_limits[BBMC_DOF_NUM];

static bbmc_actuator_range_t            g_position_init[BBMC_DOF_NUM];

static bbmc_actuator_range_t            g_position_home[BBMC_DOF_NUM];




/** State Information API
 * 
 */

int 
global_state_setup (void)
{
    g_state[0].dev_id = 1;
    g_state[1].dev_id = 2;
    
    return 0;
}

int 
global_state_init (void)
{
    qei_data_init (&g_state[0]);
    qei_data_init (&g_state[1]);
    
    qei_capture_config (&g_state[0], 1, 64);
    qei_capture_config (&g_state[1], 1, 64);
    
    qei_motor (&g_state[0], MAX_SPEED_MOTOR_1);
    qei_motor (&g_state[1], MAX_SPEED_MOTOR_2);
    
    qei_switch_velocity (&g_state[0], SPEED_MODE_THRESHOLD_Y);
    qei_switch_velocity (&g_state[1], SPEED_MODE_THRESHOLD_X);
    
    global_position_set(1, g_position_home[0].min);
    global_position_set(2, g_position_home[1].min);
    
    return 0;
}

int 
global_state_read (bbmc_actuator_state_t volatile *local_state)
{
    if (local_state == NULL)
    {
        UARTPuts("error: global_state_read: pointer is NULL\r\n", -1);
        return -1;
    }
    
    if (local_state->dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_state_read: dev_id is invalid\r\n", -1);
        return -2;
    }
    
    return qei_data_cpy (&g_state[local_state->dev_id-1], local_state);
}

int 
global_state_write (bbmc_actuator_state_t volatile *local_state)
{
    if (local_state == NULL)
    {
        UARTPuts("error: global_state_write: pointer is NULL\r\n", -1);
        return -1;
    }
    
    if (local_state->dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_state_write: argument 1: dev_id is invalid\r\n", -1);
        return -2;
    }
    
    return qei_data_cpy (local_state, &g_state[local_state->dev_id-1]);
}

int 
global_position_reset (unsigned int dev_id, pos_reset value)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_position_reset: dev_id is invalid\r\n", -1);
        return -1;
    }
    
    if (value == MIN)
    {
        UARTprintf("global_position_reset: global position for axis %d has been reset to MIN value.\r\n", dev_id);
        return qei_position_set(&g_state[dev_id-1], g_position_init[dev_id-1].min);
    }
    
    else if (value == MAX)
    {
        UARTprintf("global_position_reset: global position for axis %d has been reset to MAX value.\r\n", dev_id);
        return qei_position_set(&g_state[dev_id-1], g_position_init[dev_id-1].max);
    }
    
    else
    {
        UARTPuts("global_position_reset: error: invalid pos_reset value\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
global_position_get (unsigned int dev_id)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_position_set: dev_id is invalid\r\n", -1);
        return -1;
    }
    
    input_encoder_1D(&g_state[dev_id-1]);
    
    return 0;
}

int 
global_position_set (unsigned int dev_id, unsigned int position)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_position_set: dev_id is invalid\r\n", -1);
        return -1;
    }
    
    if ( (position > g_position_limits[dev_id-1].max) || (position < g_position_limits[dev_id-1].min) )
    {
        UARTPuts("error: global_position_set: position is out of bounds\r\n", -1);
        return -2;
    }
    
    return qei_position_set(&g_state[dev_id-1], position);
}

int 
global_sampling_frequency_set (unsigned int dev_id, unsigned int frequency)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_sampling_frequency_set: dev_id is invalid\r\n", -1);
        return -1;
    }
    
    return qei_frequency_set (&g_state[dev_id-1], frequency);
}

int 
global_state_print (unsigned int dev_id, const char *format)
{
    UARTprintf("\r\n%sGlobal Mechanism State: \r\n", format);
    
    if (dev_id > BBMC_DOF_NUM)
    {
        int i;
        
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            UARTprintf("\r\n%saxis:     %d", format, (i+1));
            UARTprintf("\r\n%sposition: %d", format, (int)g_state[i].input.count[1]);
            UARTprintf("\r\n%sspeed:    %d", format, (int)g_state[i].input.speed);
            UARTprintf("\r\n%s", format);
        }
    }
    
    else
    {
        UARTprintf("\r\n%saxis:     %d", format, dev_id);
        UARTprintf("\r\n%sposition: %d", format, (int)g_state[dev_id-1].input.count[1]);
        UARTprintf("\r\n%sspeed:    %d", format, (int)g_state[dev_id-1].input.speed);
        UARTprintf("\r\n%s", format);
    }
    
    return 0;
}



/** Global Position Limit and Pre-set Configurations
 * 
 */

int 
global_presets_setup (void)
{
    global_home_setup ();
    
    global_limits_setup ();
    
    global_inits_setup ();
    
    return 0;
}


int 
global_limits_setup (void)
{
    g_position_limits[0].min = POSITION_Y_MIN_THRESH;
    g_position_limits[0].max = POSITION_Y_MAX_THRESH;
    g_position_limits[1].min = POSITION_X_MIN_THRESH;
    g_position_limits[1].max = POSITION_X_MAX_THRESH;
    
    return 0;
}

int 
global_limits_get (unsigned int dev_id, bbmc_actuator_range_t *local_limits)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_limits_get: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_limits == NULL)
    {
        UARTPuts("error: global_limits_get: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    local_limits->min = g_position_limits[dev_id-1].min;
    local_limits->max = g_position_limits[dev_id-1].max;
    
    return 0;
}

int 
global_limits_set (unsigned int dev_id, bbmc_actuator_range_t *local_limits)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_limits_set: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_limits == NULL)
    {
        UARTPuts("error: global_limits_set: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    g_position_limits[dev_id-1].min = local_limits->min;
    g_position_limits[dev_id-1].max = local_limits->max;
    
    return 0;
}


int 
global_inits_setup (void)
{
    g_position_init[0].min = POSITION_Y_MIN_INIT;
    g_position_init[0].max = POSITION_Y_MAX_INIT;
    g_position_init[1].min = POSITION_X_MIN_INIT;
    g_position_init[1].max = POSITION_X_MAX_INIT;
    
    return 0;
}

int 
global_inits_get (unsigned int dev_id, bbmc_actuator_range_t *local_inits)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_inits_get: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_inits == NULL)
    {
        UARTPuts("error: global_inits_get: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    local_inits->min = g_position_init[dev_id-1].min;
    local_inits->max = g_position_init[dev_id-1].max;
    
    return 0;
}

int 
global_inits_set (unsigned int dev_id, bbmc_actuator_range_t *local_inits)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_inits_set: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_inits == NULL)
    {
        UARTPuts("error: global_inits_set: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    g_position_init[dev_id-1].min = local_inits->min;
    g_position_init[dev_id-1].max = local_inits->max;
    
    return 0;
}


int 
global_home_setup (void)
{
    g_position_home[0].min = POSITION_Y_MIN_HOME;
    g_position_home[0].max = POSITION_Y_MAX_HOME;
    g_position_home[1].min = POSITION_X_MIN_HOME;
    g_position_home[1].max = POSITION_X_MAX_HOME;
    
    return 0;
}

int 
global_home_get (unsigned int dev_id, bbmc_actuator_range_t *local_home)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_home_get: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_home == NULL)
    {
        UARTPuts("error: global_home_get: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    local_home->min = g_position_home[dev_id-1].min;
    local_home->max = g_position_home[dev_id-1].max;
    
    return 0;
}

int 
global_home_set (unsigned int dev_id, bbmc_actuator_range_t *local_home)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_home_set: argument 1: dev_id is invalid\r\n", -1);
    }
    
    if (local_home == NULL)
    {
        UARTPuts("error: global_home_set: argument 2: pointer is NULL\r\n", -1);
        return -1;
    }
    
    g_position_home[dev_id-1].min = local_home->min;
    g_position_home[dev_id-1].max = local_home->max;
    
    return 0;
}

int 
global_positions_print (const char *format)
{
    UARTprintf("\r\n%sPredefined System Positions: \r\n", format);
    
    UARTprintf("\r\n%saxis 1: active max    = %d", format, g_position_limits[0].max);
    UARTprintf("\r\n%s        active min    = %d", format, g_position_limits[0].min);
    
    UARTprintf("\r\n%saxis 2: active max    = %d", format, g_position_limits[1].max);
    UARTprintf("\r\n%s        active min    = %d", format, g_position_limits[1].min);
    
    UARTprintf("\r\n%saxis 1: home position = %d", format, g_position_home[0].min);
    UARTprintf("\r\n%saxis 2: home position = %d", format, g_position_home[1].min);
    
    return 0;
}





/**
 * 
 */
