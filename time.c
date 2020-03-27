#include <time.h>
#include <string.h>
#include <stdio.h>
#include "native_gecko.h"

static time_t offset_time_in_sec;             /* Time passed since Unix epoch */
static const char * _month[] =  {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
};

clock_t clock(void)
{
    /* Be carefull the overflow. It should only be used to profile. */
    return gecko_cmd_hardware_get_time()->ticks;
}

time_t time(time_t *p_time)
{
    time_t seconds;

    /* seconds = elapsed time since last reset of RTCC + initial offset time */
    seconds = gecko_cmd_hardware_get_time()->seconds;
    seconds += offset_time_in_sec;

    if ( p_time != NULL )
        *p_time = seconds;

    return seconds;
}

static int month2int( char * str )
{
    int i;
    for (i = 0; i < 12; i++)
        if (strcmp(_month[i], str) == 0)
            break;
    return i;
}

void time_init(struct tm * time_ptr)
{
    if ( time_ptr == NULL ) {
        /* Use Compiled time as system init time. */
        char month_name[5];
        struct tm compiled_tm = {0};
        sscanf(__DATE__, "%s %d %d", month_name, &compiled_tm.tm_mday, &compiled_tm.tm_year);
        sscanf(__TIME__, "%d:%d:%d", &compiled_tm.tm_hour, &compiled_tm.tm_min, &compiled_tm.tm_sec);

        compiled_tm.tm_mon = month2int(month_name);
        compiled_tm.tm_year -= 1900;

        offset_time_in_sec = mktime(&compiled_tm) - 8 * 3600;  // compiled time is UTC+8
    } else {
        time_t current_time = time(NULL);
        time_t new_time     = mktime(time_ptr);
        if ( new_time > current_time )
            offset_time_in_sec += new_time - current_time;
        else
            offset_time_in_sec -= current_time - new_time;
    }
}

