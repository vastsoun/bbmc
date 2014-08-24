//TODO: License


/** BBMC Headers **/
#include "bbmc-0.6.h"



/** main(): the entry point into the BBMC system.
 *  
 */

int 
main(void)
{
    /** BBMC startup:
     *
     * - all timers have 1ms default period 
     * - pwm frequency defaults to 50 kHz
     * - qei capture defaults to upps=1, ccps=64.
     * - by default, all arm-core caches are enabled.
     * 
     */
    
    bbmc_setup();
    
    bbmc_greeting();

    bbmc_sysflags_clear (&g_flags, "-all");
    
    CommandLine();
    
    bbmc_system_halt();
    
    return 0;
}


/** EOF 
 */
