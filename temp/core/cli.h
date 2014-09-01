#ifndef _BBMC_COMMAND_LINE_INTERFACE_H_
#define _BBMC_COMMAND_LINE_INTERFACE_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** CLI Macros
 * 
 */

#define MAX_CMD_NAME_SIZE               (16)
#define MAX_NUM_COMMANDS                (13)
#define MAX_HELP_SIZE                   (256)



#define HELP_TEST                       "\r\n-I/O Testing-\r\n"
#define HELP_STATUS                     "\r\n-System Status-\r\n"

#define HELP_CONFIG                     "\r\n-Configuration Command-\r\n "
#define HELP_PLANNER                    "\r\n-Path and Trejectory Generator-\r\n"

#define HELP_DATALOG                    "\r\n-Datalog Servie-\r\n"
#define HELP_PERF                       "\r\n-Performance Counter-\r\n"

#define HELP_DEBUG                      "\r\n-Debug-\r\n"
#define HELP_QUIT                       "\r\n-Quit-\r\n"





/** The CLI looping function.
 *  
 */
int command_line (void);



/** BBMC generation 
 * 
 */
int greeting (void);



/** Entry Points for Tasks Called by CLI Commands
 * 
 */

int cmnd_experiment (int argc, char*argv[]);

int cmnd_rmpi (int argc, char*argv[]);

int cmnd_goto (int argc, char*argv[]);

int cmnd_test (int argc, char*argv[]);

int cmnd_status (int argc, char*argv[]);

int cmnd_planner (int argc, char*argv[]);

int cmnd_datalog (int argc, char*argv[]);

int cmnd_perflog (int argc, char*argv[]);

int cmnd_config (int argc, char*argv[]);

int cmnd_debug (int argc, char*argv[]);

int cmnd_quit (int argc, char*argv[]);






#ifdef __cplusplus
}
#endif


#endif /* _BBMC_COMMAND_LINE_INTERFACE_H_ */

