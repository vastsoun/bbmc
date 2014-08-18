#include "utils.h"


/** Std C Headers **/




/** 
*  
*/
double 
util_strtod (const char *str, unsigned char **endptr)
{
    double number;
    int exponent;
    int negative;
    unsigned char *p = (unsigned char *) str;
    double p10;
    int n;
    int num_digits;
    int num_decimals;
    
    // Skip leading whitespace
    while (isspace(*p)) p++;
    
    // Handle optional sign
    negative = 0;
    
    switch (*p)
    {
        case '-': negative = 1; // Fall through to increment position
        case '+': p++;
    }
    
    number = 0.;
    exponent = 0;
    num_digits = 0;
    num_decimals = 0;
    
    // Process string of digits
    while (isdigit(*p))
    {
        number = number * 10. + (*p - '0');
        p++;
        num_digits++;
    }
    
    // Process decimal part
    if (*p == '.')
    {
        p++;
        
        while (isdigit(*p))
        {
            number = number * 10. + (*p - '0');
            p++;
            num_digits++;
            num_decimals++;
        }
        
        exponent -= num_decimals;
    }
    
    if (num_digits == 0)
    {
        errno = ERANGE;
        return 0.0;
    }
    
    // Correct for sign
    if (negative)
    {
        number = -number;
    }
    
    // Process an exponent string
    if (*p == 'e' || *p == 'E')
    {
        // Handle optional sign
        negative = 0;
        switch (*++p)
        {
          case '-': negative = 1;   // Fall through to increment pos
          case '+': p++;
        }
        
        // Process string of digits
        n = 0;
        while (isdigit(*p))
        {
          n = n * 10 + (*p - '0');
          p++;
        }
        
        if (negative)
        {
            exponent -= n;
        }
        else
        {
            exponent += n;
        }
    }
    
    if (exponent < DBL_MIN_EXP  || exponent > DBL_MAX_EXP)
    {
        errno = ERANGE;
        return HUGE_VAL;
    }
    
    // Scale the result
    p10 = 10.;
    n = exponent;
    
    if (n < 0)
    {
        n = -n;
    }
    
    while (n)
    {
        if (n & 1)
        {
          if (exponent < 0)
          {
            number /= p10;
          }
          
          else
          {
            number *= p10;
          }
        }
        
        n >>= 1;
        p10 *= p10;
    }
    
    if (number == HUGE_VAL)
    {
      errno = ERANGE;
    }
    
    if (endptr)
    {
        *endptr = p;
    }
    
    return number;
}


/** 
 *  
 */
int 
util_checkpoint_yn (char *format, char *ptr)
{
    static int flag=-1;
    static int t1=-1;
    static int t2=-1;
    
    do
    {
        UARTPuts(format, -1);
        UARTGets(ptr, RX_BUFF_SIZE);
        
        t1=strcmp((const char *)ptr, "Y");
        t2=strcmp((const char *)ptr, "n");
        
    }
    while((t1 != 0) && (t2 != 0));
            
    if (t1 == 0)
    { 
        flag = 1;
    }
    
    if (t2 == 0)
    { 
        flag = 0;
    }
    
    return flag;
}


/** 
 *  
 */
void 
util_dtoa (double f, char *buf)
{
    
    int pos=0, ix=0, dp=0, num=0, ip=0;
    
    pos = 0;
    
    if (f < 0)
    {
        buf[pos++]='-';
        f = -f;
    }
    
    dp=0;
    
    while (f >= 10.0)
    {
        f = f/10.0;
        dp++;
    }
    
    ip = dp;
    
    for (ix=1; ix < (ip + MAX_DECIMAL_DIGITS); ix++)
    {
        
            num = (int)f;
            f = f - num;
            
            if (num > 9)
            {
                buf[pos++] = '#';
            }
            
            else
            {
                /*if ((ix == (ip + MAX_DECIMAL_DIGITS - 1)) && (num >= 5)){
                    num++;
                }*/
                buf[pos++] = '0' + num;
            }
            
            if (dp == 0)
            {
                buf[pos++] = '.';
            }
            
            f = f*10.0;
            dp--;
    }
}


/** 
 *  
 */
void 
util_delay (int count)
{
    
    static int i=0;
    
    for (i=0; i < count; i++);
    
}
