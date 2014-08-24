/** Module Header **/
#include "commands.h"

/** Std C Headers **/
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

/** BBMC Headers **/
#include "uartStdio.h"
#include "cmdline.h"





/** Commands Table 
 * 
 */
 
static const char cmd_names[MAX_NUM_COMMANDS][MAX_CMD_NAME_SIZE] = {
    
    "run","datalog","perf","reset","config",
    "path","goto","rmpi","test","debug",
    "status","quit"
    
};

static const char cmd_help[MAX_NUM_COMMANDS][MAX_HELP_SIZE] = {
    
    HELP_RUN,HELP_DATALOG,HELP_PERF,HELP_RESET,
    HELP_CONFIG,HELP_PATH,HELP_GOTO,HELP_RMPI,
    HELP_TEST,HELP_DEBUG,HELP_STATUS,HELP_QUIT
    
};

tCmdLineEntry g_cmd_table[MAX_NUM_COMMANDS] = {
    
    {cmd_names[0],cmnd_run,cmd_help[0]},{cmd_names[1],cmnd_datalog,cmd_help[1]},
    {cmd_names[2],cmnd_perf,cmd_help[2]},{cmd_names[3],cmnd_reset,cmd_help[3]},
    {cmd_names[4],cmnd_config,cmd_help[4]},{cmd_names[5],cmnd_path,cmd_help[5]},
    {cmd_names[6],cmnd_goto,cmd_help[6]},{cmd_names[7],cmnd_rmpi,cmd_help[7]},
    {cmd_names[8],cmnd_test,cmd_help[8]},{cmd_names[9],cmnd_debug,cmd_help[9]},
    {cmd_names[10],cmnd_status,cmd_help[10]},{cmd_names[11],cmnd_quit,cmd_help[11]},
    {0,0,0}
    
};


int CommandLine (void)
{
    static char cmd_rx_buff[RX_BUFF_SIZE] = {0};
    static int  cmd_ret;
    
    for (;;)
    {
        if (g_flags.cmdln == 1)
        {
            continue;
        }
        
        if (g_flags.cmdln == -1)
        {
            break;
        }
        
        /** Get and execute the input CLI command **/
        cmnd_ret = UARTPuts("\r\nbbmc:~$ ", -1);
        UARTGets(cmd_rx_buff, RX_BUFF_SIZE);
        cmd_ret = CmdLineProcess(cmd_rx_buff, g_cmd_table);
        
        /** Debug verbatim **/
        if (g_flags.debug == 1)
        {
            UARTprintf("\r\nDEBUG: %d\r\n", cmd_ret);
        }
    }
    
    return 1;
}


/** BBMC generation 
 * 
 */
int Greeting (void)
{
    UARTPuts(g_bbmc_greeting, -1);
    return 0;
}

