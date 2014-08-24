




/** Internal Data Types
 *  
 */
typedef struct 
{
    /** standard system flags **/
    int cmdln;
    int debug;
    
    int perf;
    int datalog;
    
    int exec_checkpoint;
    int contrl_run;
    
    /** ISR relative flags **/
    int volatile isr_return;
    int volatile stop_immediate;
    int volatile gpos_reset[BBMC_DOF_NUM];
}
bbmc_system_flags_t;




/** Internal Data
 *  
 */
static bbmc_system_flags_t         g_flags;




/** System Flag handler functions
 *  
 */

int 
sys_flags_clear  (const char *init_mode)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: sys_flags_clear : pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    int i = 0;
    
    /* init appropriately */
    if(!strcmp((const char *)init_mode, "-all"))
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
    
    else if(!strcmp((const char *)init_mode, "-isr"))
    {
        g_sysflags.isr_return = 0;
        g_sysflags.stop_immediate = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-preset"))
    {
        for (i = 0; i< BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 0;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-log"))
    {
        g_sysflags.perf = 0;
        g_sysflags.datalog = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-cli"))
    {
        g_sysflags.cmdln = 0;
        g_sysflags.debug = 0;
    }
    
    else if(!strcmp((const char *)init_mode, "-cmd"))
    {
        g_sysflags.exec_checkpoint = 0;
        g_sysflags.contrl_run = 0;
    }
    
    else
    {
        UARTPuts("error: sytemflags_init: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int 
sys_flags_set (const char *init_mode)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("\r\nerror: sys_flags_set: pointer argument is NULL", -1);
        return -1;
    }
    
    int i = 0;
    
    /* init appropriately */
    if(!strcmp((const char *)init_mode, "-all"))
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
    
    else if(!strcmp((const char *)init_mode, "-isr"))
    {
        g_sysflags.isr_return = 1;
        g_sysflags.stop_immediate = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-preset"))
    {
        for (i = 0; i< BBMC_DOF_NUM; i++)
        {
            g_sysflags.gpos_reset[i] = 1;
        }
    }
    
    else if(!strcmp((const char *)init_mode, "-log"))
    {
        g_sysflags.perf = 1;
        g_sysflags.datalog = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-cli"))
    {
        g_sysflags.cmdln = 1;
        g_sysflags.debug = 1;
    }
    
    else if(!strcmp((const char *)init_mode, "-cmd"))
    {
        g_sysflags.exec_checkpoint = 1;
        g_sysflags.contrl_run = 1;
    }
    
    else
    {
        UARTPuts("error: sytemflags_init: set_mode argument is invalid\r\n", -1);
        return -1;
    }
    
    return 0;
}

int
sys_flags_gpreset_set (int dof_id, int set_val)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: sys_flags_gpreset_set: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    if ((set_val != 0) || (set_val != 1))
    {
        UARTPuts("\r\nerror: sys_flags_gpreset_set: invalid set value", -1);
        return -2;
    }
    
    g_sysflags.gpos_reset[dof_id-1] = set_val;
    
    return 0;
}

int
sys_flags_gpreset_get (int dof_id)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: sys_flags_gpreset_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    return g_sysflags.gpos_reset[dof_id-1];
}

int 
sys_flags_get (const char * flag)
{
    /* check for NULL pointer */
    if (sysflags == NULL)
    {
        UARTPuts("error: sys_flags_get: pointer argument is NULL\r\n", -1);
        return -1;
    }
    
    int ret_val;
    
    if(!strcmp((const char *)flag, "cmdln"))
    {
        ret_val = g_sysflags.cmdln;
    }
    
    else if(!strcmp((const char *)flag, "debug"))
    {
        ret_val = g_sysflags.debug;
    }
    
    else if(!strcmp((const char *)flag, "dlog"))
    {
        ret_val = g_sysflags.datalog;
    }
    
    else if(!strcmp((const char *)flag, "checkp"))
    {
        ret_val = g_sysflags.exec_checkpoint;
    }
    
    else if(!strcmp((const char *)flag, "run"))
    {
        ret_val = g_sysflags.contrl_run;
    }
    
    else if(!strcmp((const char *)flag, "ret"))
    {
        ret_val = g_sysflags.isr_return;
    }
    
    else if(!strcmp((const char *)flag, "stop"))
    {
        ret_val = g_sysflags.stop_immediate;
    }
    
    else
    {
        UARTPuts("error: sys_flags_get: flag argument is invalid\r\n", -1);
        return -1;
    }
    
    return ret_val;
}

int 
sys_flags_print (const char *format)
{
    if (sysflags == NULL)
    {
        UARTPuts("\r\nerror: sys_flags_print: pointer argument is NULL", -1);
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
    
    bbmc_cli_newlin(2);
    
    return 0;
}
