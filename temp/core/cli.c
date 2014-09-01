/** Module Header **/
#include "cli.h"

/** Std C Headers **/
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

/** Firmware Headers **/
#include "uartStdio.h"
#include "cmdline.h"

/** BBMC Headers **/
#include "bbmc_facilities.h"

#include "util.h"



/** The BBMC Greeting on the CLI at startup.
 *  
 *  (Internal Data)
 */
const char *g_bbmc_greeting = "\r\n"
"                                                                             \r\n"
"                                                                             \r\n"
"      ____                   _        ____                                   \r\n"
"     |  _ \\                 | |      |  _ \\                                \r\n"
"     | |_) | ___  __ _  __ _| | ___  | |_) | ___  _ __   ___                 \r\n"
"     |  _ < / _ \\/ _` |/ _` | |/ _ \\ |  _ < / _ \\| '_ \\ / _ \\           \r\n"
"     | |_) |  __/ (_| | (_| | |  __/ | |_) | (_) | | | |  __/                \r\n"
"     |____/_\\___|\\__,_|\\__, |_|\\___|_|____/ \\___/|_| |_|\\___|      _   \r\n"
"     |  \\/  |     | |   __/ |      / ____|          | |           | |       \r\n"
"     | \\  / | ___ | |_ |___/_ __  | |     ___  _ __ | |_ _ __ ___ | |       \r\n"
"     | |\\/| |/ _ \\| __/ _ \\| '__| | |    / _ \\| '_ \\| __| '__/ _ \\| |  \r\n"
"     | |  | | (_) | || (_) | |    | |___| (_) | | | | |_| | | (_) | |        \r\n"
"     |_|  |_|\\___/_\\__\\___/|_|     \\_____\\___/|_| |_|\\__|_|  \\___/|_| \r\n"
"            / _ \\                                                           \r\n"
"     __   _| | | |                                                           \r\n"
"     \\ \\ / / | |                  by Vassilios Tsounis                     \r\n"
"      \\ V /| |_| |                                                          \r\n"
"       \\_/  \\___/                  vastsoun@gmail.com                      \r\n"
"                                                                             \r\n"
"                                                                             \r\n";




/** Commands Table 
 * 
 */
 
static const char cmd_names[MAX_NUM_COMMANDS][MAX_CMD_NAME_SIZE] = 
{
    "exp","rmpi","goto","test","status",
    "planner","datalog","perf", "config", 
    "debug","quit"
};

static const char cmd_help[MAX_NUM_COMMANDS][MAX_HELP_SIZE] = 
{
    HELP_EXP, HELP_RMPI, HELP_GOTO, HELP_TEST,
    HELP_STATUS, HELP_PLANNER, HELP_DATALOG, HELP_PERF,
    HELP_CONFIG, HELP_DEBUG, HELP_QUIT
};

tCmdLineEntry g_cmd_table[MAX_NUM_COMMANDS] = 
{
    {cmd_names[0], cmd_experiment, cmd_help[0]}, {cmd_names[1], cmd_rmpi, cmd_help[1]},
    {cmd_names[2], cmd_goto, cmd_help[2]}, {cmd_names[3], cmd_test, cmd_help[3]},
    {cmd_names[4], cmd_status, cmd_help[4]}, {cmd_names[5], cmd_planner, cmd_help[5]},
    {cmd_names[6], cmd_datalog, cmd_help[6]}, {cmd_names[7], cmd_perf, cmd_help[7]},
    {cmd_names[8], cmd_config, cmd_help[8]}, {cmd_names[9], cmd_debug, cmd_help[9]},
    {cmd_names[10], cmd_quit, cmd_help[10]}, {0,0,0}
};


int command_line (void)
{
    static char cmd_rx_buff[128] = {0};
    static int  cmd_ret;
    static int cmd_flag;
    static int debug_flag;
    
    for (;;)
    {
        cmd_flag = global_flag_get(CMD_LINE);
        debug_flag = global_flag_get(DEBUG);
        
        if (cmd_flag == 0)
        {
            continue;
        }
        
        if (cmd_flag == 1)
        {
            break;
        }
        
        /** Get and execute the input CLI command **/
        cmd_ret = UARTPuts("\r\nbbmc:~$ ", -1);
        UARTGets(cmd_rx_buff, RX_BUFF_SIZE);
        cmd_ret = CmdLineProcess(cmd_rx_buff, g_cmd_table);
        
        /** Debug verbatim **/
        if (debug_flag == 1)
        {
            UARTprintf("\r\nDEBUG: %d\r\n", cmd_ret);
        }
    }
    
    return 1;
}


/** BBMC generation 
 * 
 */
int greeting (void)
{
    UARTPuts(g_bbmc_greeting, -1);
    
    return 0;
}


/** Entry Points for Tasks Called by CLI Commands
 * 
 * 
 */

int cmd_experiment (int argc, char*argv[])
{
    UARTPuts("\r\n This is BBMC/Experiment\r\n", -1);
    
    return 0;
}

int cmd_rmpi (int argc, char*argv[])
{
    UARTPuts("\r\n This is BBMC/RMPI\r\n", -1);
    
    return 0;
}

int cmd_goto (int argc, char*argv[])
{
    UARTPuts("\r\n This is BBMC/Goto\r\n", -1);
    
    return 0;
}

int cmd_test (int argc, char*argv[])
{
    test (argc, argv);
    
    return 0;
}

int cmd_status (int argc, char*argv[])
{
    status (argc, argv);
    
    return 0;
}

int cmd_planner (int argc, char*argv[])
{
    planner (arc, argv);
    
    return 0;
}

int cmd_datalog (int argc, char*argv[])
{
    datalog (arc, argv);
    
    return 0;
}

int cmd_perflog (int argc, char*argv[])
{
    perflog (arc, argv);
    
    return 0;
}

int cmd_config (int argc, char*argv[])
{
    config (arc, argv);
    
    return 0;
}

int cmd_debug (int argc, char*argv[]);
{
    debug (arc, argv);
    
    return 0;
}

int cmd_quit(int argc, char*argv[])
{
    quit (arc, argv);
    
    return 0;
}




/**
 * 
 */
