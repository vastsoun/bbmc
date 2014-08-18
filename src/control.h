#ifndef _TEMPLATE__H_
#define _TEMPLATE_H_


#ifdef __cplusplus
    extern "C" {
#endif


#include <math.h>


/** 
 *  
 */


/** BBMC TRAJECTORY MACROS
 *  
 */
#define TRAJECTORY_POINTS_NUM         (DATA_SIGLEN_MAX)
#define ATAN2_ZERO_ERROR              100


/** 
 *  
 *  
 */
typedef struct 
{
    double  q;
    double  q_dot;
    double  q_ddot;
}
jointspace_state_t;


typedef struct 
{
    double  x;
    double  x_dot;
    double  x_ddot;
}
taskspace_state_t;







#ifdef __cplusplus
}
#endif


#endif /* _TEMPLATE_H_ */
