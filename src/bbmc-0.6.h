/**
 * \file   bbmc-0.6.h
 *
 * \brief  the bbmc-0.6.x application header
 */
 

#ifndef _BBMC_H_
#define _BBMC_H_

#ifdef __cplusplus
extern "C" {
#endif


/** Firmware Headers **/
#include "cmdline.h"


/** BBMC Headers 
 * 
 */
#include "device_layer.h"
#include "isr_manager.h"
#include "system_timers.h"
#include "safety_ss.h"
#include "motion_control.h"

#include "global_flags.h"
#include "global_state.h"
#include "datalog.h"
//#include "signal_generator.h"

//#include "uretts_model.h"
//#include "uretts_control.h"

#include "bbmc_facilities.h"
//#include "goto.h"
//#include "rmpi.h"
//#include "experiement.h"

#include "cli.h"




#ifdef __cplusplus
}
#endif
#endif  /* _BBMC_H_ */
