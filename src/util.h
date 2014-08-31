#ifndef _BBMC_UTIL_H_
#define _BBMC_UTIL_H_


#ifdef __cplusplus
    extern "C" {
#endif


/** 
 *  
 */
#define RX_BUFF_SIZE                    (128)
#define MAX_DECIMAL_DIGITS              (6)


/** 
 *  General Utility Functions 
 */


/** 
 *  
 */
double  util_strtod(const char *str, unsigned char **endptr);


/** 
 *  
 */
void    util_dtoa(double f, char *buf);


/** 
 *  
 */
int     util_checkpoint_yn(char *format, char *ptr);


/** 
 *  
 */
void    util_delay(int count);




#ifdef __cplusplus
}
#endif


#endif /* _BBMC_UTIL_H_ */
