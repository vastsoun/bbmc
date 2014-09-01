/** Module Header **/
#include "global_flags.h"

/** Std C Headers **/


/** Firmware Headers **/


/** BBMC Headers **/





/** Internal Data Types
 *  
 */
typedef struct 
{
    int cmdln;
    int debug;
    
    int perf;
    int datalog;
    
    int exec_checkpoint;
    int contrl_run;
    
    int volatile isr_return;
    int volatile stop_immediate;
    int volatile gpos_reset[BBMC_DOF_NUM];
}
bbmc_system_flags_t;




/** Internal Data
 *  
 */
static bbmc_system_flags_t         g_sysflags;




/** System Flag handler functions
 *  
 */

int 
global_flag_get (system_flag flag)
{
    int ret_val;
    
    if(flag == FLG_CMDLINE)
    {
        ret_val = g_sysflags.cmdln;
    }
    
    else if(flag == FLG_DEBUG)
    {
        ret_val = g_sysflags.debug;
    }
    
    else if(flag == FLG_DATALOG)
    {
        ret_val = g_sysflags.datalog;
    }
    
    else if(flag == FLG_PERFLOG)
    {
        ret_val = g_sysflags.perf;
    }
    
    else if(flag == FLG_EXEC_CHK)
    {
        ret_val = g_sysflags.exec_checkpoint;
    }
    
    else if(flag == FLG_CONTRL_RUN)
    {
        ret_val = g_sysflags.contrl_run;
    }
    
    else if(flag == FLG_STOP_EMR)
    {
        ret_val = g_sysflags.stop_immediate;
    }
    
    else
    {
        UARTPuts("error: global_flags_get: flag argument is invalid\r\n", -1);
        return -1;
    }
    
    return ret_val;
}

int 
global_flag_set (system_flag flag)
{
    if(flag == FLG_CMDLINE)
    {
        g_sysflags.cmdln = 1;
    }
    
    else if(flag == FLG_DEBUG)
    {
        g_sysflags.debug = 1;
    }
    
    else if(flag == FLG_DATALOG)
    {
       g_sysflags.datalog = 1;
    }
    
    else if(flag == FLG_PERFLOG)
    {
        g_sysflags.perf = 1;
    }
    
    else if(flag == FLG_EXEC_CHK)
    {
        g_sysflags.exec_checkpoint = 1;
    }
    
    else if(flag == FLG_CONTRL_RUN)
    {
        g_sysflags.contrl_run = 1;
    }
    
    else if(flag == FLG_STOP_EMR)
    {
        g_sysflags.stop_immediate = 1;
    }
    
    else
    {
        UARTPuts("error: global_flags_get: flag argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
global_flag_clear (system_flag flag)
{
    if(flag == FLG_CMDLINE)
    {
        g_sysflags.cmdln = 0;
    }
    
    else if(flag == FLG_DEBUG)
    {
        g_sysflags.debug = 0;
    }
    
    else if(flag == FLG_DATALOG)
    {
       g_sysflags.datalog = 0;
    }
    
    else if(flag == FLG_PERFLOG)
    {
        g_sysflags.perf = 0;
    }
    
    else if(flag == FLG_EXEC_CHK)
    {
        g_sysflags.exec_checkpoint = 0;
    }
    
    else if(flag == FLG_CONTRL_RUN)
    {
        g_sysflags.contrl_run = 0;
    }
    
    else if(flag == FLG_ISR_RET)
    {
        g_sysflags.isr_return = 0;
    }
    
    else if(flag == FLG_STOP_EMR)
    {
        g_sysflags.stop_immediate = 0;
    }
    
    else
    {
        UARTPuts("error: global_flags_get: flag argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
global_flags_clear  (system_flag flag)
{
    int i = 0;
    
    /* init appropriately */
    if (flag == FLG_ALL)
    {
        g_sysflags.cmdln = 0;
        g_sysflags.debug = 0;
        
        g_sysflags.perf = 0;
        g_sysflags.datalog = 0;
        
        g_sysflags.exec_checkpoint = 0;
        g_sysflags.contrl_run = 0;
        
        g_sysflags.isr_return = 0;
        g_sysflags.stop_immediate = 0;
    
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 0;
        }
    }
    
    else if (flag == FLG_ISR)
    {
        g_sysflags.isr_return = 0;
        g_sysflags.stop_immediate = 0;
    }
    
    else if (flag == FLG_GPOS_RESET)
    {
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 0;
        }
    }
    
    else if (flag == FLG_LOGS)
    {
        g_sysflags.perf = 0;
        g_sysflags.datalog = 0;
    }
    
    else if (flag == FLG_CLI)
    {
        g_sysflags.cmdln = 0;
        g_sysflags.debug = 0;
    }
    
    else if (flag == FLG_CMD)
    {
        g_sysflags.exec_checkpoint = 0;
        g_sysflags.contrl_run = 0;
    }
    
    else
    {
        UARTPuts("error: sytemflags_clear: clear_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
global_flags_set (system_flag flag)
{
    int i = 0;
    
    /* init appropriately */
    if (flag == FLG_ALL)
    {
        g_sysflags.cmdln = 1;
        g_sysflags.debug = 1;
        
        g_sysflags.perf = 1;
        g_sysflags.datalog = 1;
        
        g_sysflags.exec_checkpoint = 1;
        g_sysflags.contrl_run = 1;
        
        g_sysflags.isr_return = 1;
        g_sysflags.stop_immediate = 1;
    
        for (i = 1; i < BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 1;
        }
    }
    
    else if (flag == FLG_ISR)
    {
        g_sysflags.isr_return = 1;
        g_sysflags.stop_immediate = 1;
    }
    
    else if (flag == FLG_GPOS_RESET)
    {
        for (i = 0; i < BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 1;
        }
    }
    
    else if (flag == FLG_CLI)
    {
        g_sysflags.cmdln = 1;
        g_sysflags.debug = 1;
    }
    
    else if (flag == FLG_CMD)
    {
        g_sysflags.exec_checkpoint = 1;
        g_sysflags.contrl_run = 1;
    }
    
    else
    {
        UARTPuts("error: sytemflags_set: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}


int
global_flags_gpreset_set (int dev_id, pos_reset value)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_flags_gpreset_set: dev_id argument is invalid\r\n", -1);
        return -1;
    }
    
    if ((value != 0) || (value != 1))
    {
        UARTPuts("\r\nerror: global_flags_gpreset_set: invalid set value", -1);
        return -2;
    }
    
    g_sysflags.gpos_reset[dev_id-1] = (int)value;
    
    return 0;
}

pos_reset
global_flags_gpreset_get (int dev_id)
{
    if (dev_id > BBMC_DOF_NUM)
    {
        UARTPuts("error: global_flags_gpreset_get: dev_id argument is invalid\r\n", -1);
        return -1;
    }
    
    return (pos_reset)g_sysflags.gpos_reset[dev_id-1];
}


int global_flag_isr_set (isr_ret_t value)
{
    g_sysflags.isr_return = value;
    
    return 0;
}

isr_ret_t global_flag_isr_get (void)
{
    return g_sysflags.isr_return;
}


int 
global_flags_print (const char *format)
{
    if (format == NULL)
    {
        UARTPuts("\r\nerror: global_flags_print: pointer argument is NULL", -1);
        return -1;
    }
    
    UARTprintf("\r\n%sSystem Flags:\r\n", format);
    UARTprintf("\r\n%sflag        value", format);
    UARTprintf("\r\n%scmdln:         %d", format, g_sysflags.cmdln);
    UARTprintf("\r\n%sdebug:         %d", format, g_sysflags.debug);
    UARTprintf("\r\n%sperf:          %d", format, g_sysflags.perf);
    UARTprintf("\r\n%sdatalog:       %d", format, g_sysflags.datalog);
    UARTprintf("\r\n%scheckpoint:    %d", format, g_sysflags.exec_checkpoint);
    UARTprintf("\r\n%srun:           %d", format, g_sysflags.contrl_run);
    UARTprintf("\r\n%sisr_ret:       %d", format, g_sysflags.isr_return);
    UARTprintf("\r\n%sstop:          %d", format, g_sysflags.stop_immediate);
    
    int i = 0;
    
    for (i = 0; i < BBMC_DOF_NUM; i++)
    {
        UARTprintf("\r\n%spos_reset-%d:   %d", format,(i+1), g_sysflags.gpos_reset[i]);
    }
    
    return 0;
}





/**
 * 
 */
