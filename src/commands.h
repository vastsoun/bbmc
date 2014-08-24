#ifndef _BBMC_COMMANDS__H_
#define _BBMC_COMMANDS_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** The BBMC Greeting on the CLI at startup.
 *  
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



/** The CLI looping function.
 *  
 */
int CommandLine (void);

/** BBMC generation 
 * 
 */
int Greeting (void);



#ifdef __cplusplus
}
#endif


#endif /* _BBMC_COMMANDS_H_ */

