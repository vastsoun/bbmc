/** Datalog Module Header **/
#include "datalog.h"

/** Std C Headers **/
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

/** Firmware Headers **/
#include "uartStdio.h"

/** BBMC Headers **/
#include "util.h"



/** Internal Data Types 
 * 
 */


typedef struct
{
    data_t data[DATALOG_STATIC_DATASIZE];
}
dataset_s_t;


typedef struct
{
    dataset_s_t log[DATALOG_STATIC_DATALEN];
    
    int d_size;
    int l_size;
    
    int d_index;
    int l_index;
}
datalog_s_t;



/** Internal Data
 * 
 */
static datalog_s_t  volatile   g_datalog[BBMC_DOF_NUM];

static unsigned int volatile   g_perflog[GLOBAL_DATA_MAX];


/** Datalog API Functions
 * 
 */

int
datalog_s_setup (void)
{
    int i = 0;
    
    for (i = 0; i < BBMC_DOF_NUM; ++i)
    {
        /* setup the datalog dimensions */
        g_datalog[i].d_size = DATALOG_STATIC_DATASIZE;
        g_datalog[i].l_size = DATALOG_STATIC_DATALEN;
        
        /* initialize indeces */
        g_datalog[i].d_index = 0;
        g_datalog[i].l_index = 0;
    }
    
    return 0;
}

int 
datalog_s_init(data_t init_val)
{
    int i = 0, j = 0, k = 0;
    
    int size, len;
    
    for (k = 0; k < BBMC_DOF_NUM; ++k)
    {
        size = g_datalog[k].d_size;
        len  = g_datalog[k].l_size;
        
        for (j = 0; j < len; j++)
        {
            for (i=0; i < size; i++)
            {
                g_datalog[k].log[j].data[i] = init_val;
            }
        }
    }
    
    return 0;
}

void
datalog_s_write (unsigned int dev_id,
                 unsigned int index,
                 bbmc_input_encoder_t volatile *state,
                 bbmc_contrl_motor_t  volatile *contrl)
{
    g_datalog[dev_id-1].log[index].data[0] = state->input.status;
    g_datalog[dev_id-1].log[index].data[1] = state->input.count[1];
    g_datalog[dev_id-1].log[index].data[2] = state->input.speed;
    g_datalog[dev_id-1].log[index].data[3] = contrl->state_desired.q;
    g_datalog[dev_id-1].log[index].data[4] = contrl->state_desired.q_dot;
    g_datalog[dev_id-1].log[index].data[5] = contrl->control.output;
}

int datalog_s_single_write (unsigned int dev_id,
                            unsigned int log_index,
                            unsigned int datum_index,
                            data_t value)
{
    g_datalog[dev_id-1].log[datum_index].data[log_index] = value;
    
    return 0;
}

int 
datalog_s_print(unsigned int dev_id, int range_indeces[4])
{
    /* check for NULL pointer and return error if needed */
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: datalog_s_print: dev_id is invalid\r\n", -1);
        return -1;
    }
    
    /* get the datalog dimensions */
    int size = g_datalog[dev_id-1].d_size;
    int len = g_datalog[dev_id-1].l_size;
    
    int l_index_s;
    int l_index_f;
    int d_index_s;
    int d_index_f;
    
    /* if range_indecs is given as NULL then print the entire datalog */
    if (range_indeces == NULL)
    {
        l_index_s = 0;
        l_index_f = len - 1;
        d_index_s = 0;
        d_index_f = size - 1;
    }
    
    else
    {
        /* check to see of the range indeces are give correctly - i.e. increasing*/
        if ((range_indeces[1] < range_indeces[0]) || (range_indeces[3] < range_indeces[2]))
        {
            UARTPuts("error: datalog_s_print: range_indeces: index_s > index_f\r\n", -1);
            return -1;
        }
        
        /* check if the range arguments are valid */
        if ((range_indeces[0] > len) || (range_indeces[1] > len) || 
            (range_indeces[2] > size) || (range_indeces[3] > size))
        {
            UARTPuts("\r\nerror: datalog_s_print: range_indeces: some indeces exceed datalog bounds.", -1);
            return -1;
        }
        
        l_index_s = range_indeces[0];
        l_index_f = range_indeces[1];
        d_index_s = range_indeces[2];
        d_index_f = range_indeces[3];
    }
    
    /* for-loop indeces: j for len, i for size */
    int i = 0;
    int j = 0;
    
    UARTPuts("\r\n\r\nDATALOG_BEGIN:\r\n\r\n", -1);
    
    /* print the data header */
    UARTPuts("index,", -1);
    
    for (i = d_index_s; i < d_index_f; i++)
    {
        UARTPuts("data", -1);
        UARTPutNum(i);
        
        if (i == (d_index_f - 1))
        {
            UARTPutc('\r');
            UARTPutc('\n');
        }
        else
        {
            UARTPutc(',');
        }
    }
    
    /* print the datalog data elements */
    for (j = l_index_s; j < l_index_f; j++)
    {
        UARTPutNum(j);
        UARTPutc(',');
        
        for (i = d_index_s; i < d_index_f; i++)
        {
            if (i == (d_index_f - 1))
            {
                UARTPutDouble(g_datalog[dev_id-1].log[j].data[i]);
                UARTPutc('\r');
                UARTPutc('\n');
            }
            else
            {
                UARTPutDouble(g_datalog[dev_id-1].log[j].data[i]);
                UARTPutc(',');
            }
        }
    }
    
    UARTPuts("\r\nDATALOG_END\r\n\r\n", -1);
    
    return 0;
}



/** Performance Log API Functions
 * 
 */

int performance_log_init  (void)
{
    int i = 0;
    
    for (i = 0; i < GLOBAL_DATA_MAX; i++)
    {
        g_perflog[i] = 0;
    }
    
    return 0;
}

int performance_log_write (unsigned int index, unsigned int value)
{
    g_perflog[index] = value;
    
    return 0;
}

int performance_log_print (unsigned int iterations)
{
    int i = 0;
    
    UARTPuts("\r\nPerformance Log: \r\n\nLOG_START:\r\n Iteration, Ticks\r\n", -1);
    
    for(i = 0; i < iterations; ++i)
    {
        UARTprintf("%d, %d\r\n", i, (int)(g_perflog[i]));
    }
    
    UARTPuts("\r\nLOG_END\r\n", -1);
    
    return 0;
}



/**
 *
 */
