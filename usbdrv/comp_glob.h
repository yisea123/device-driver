#ifndef _COMPU_GLOB_H_
#define _COMPU_GLOB_H_            

//#include "product.h"

#ifndef __ASSEMBLER__

/* Interface types                                                          */
typedef unsigned long			status_type;/* arm flags					*/
typedef unsigned char           byte;       /* machine byte                 */
typedef signed char             schar;
typedef unsigned short          word;       /* machine word                 */
typedef byte					*address;   /* machine address              */
typedef unsigned long           memdim;     /* memory size specifications   */

typedef unsigned long           TIME;		/* time of day					*/

#define NULL_ADDRESS	((address) 0)

#include <linux/param.h>    /*for HZ */

#define _get_time()				(jiffies)
#define compute_ticks(time)	((time) / 1000 * HZ +  (time) % 1000 * HZ / 1000)

#define _0ms        compute_ticks((TIME)0)
#define _1ms        compute_ticks((TIME)1)
#define _2ms        compute_ticks((TIME)2)
#define _5ms        compute_ticks((TIME)5)
#define _10ms       compute_ticks((TIME)10)
#define _15ms       compute_ticks((TIME)15)
#define _20ms       compute_ticks((TIME)20)
#define _25ms       compute_ticks((TIME)25)
#define _30ms       compute_ticks((TIME)30)
#define _50ms       compute_ticks((TIME)50)
#define _60ms       compute_ticks((TIME)60)
#define _100ms      compute_ticks((TIME)100)
#define _200ms      compute_ticks((TIME)200)
#define _250ms      compute_ticks((TIME)250)
#define _500ms      compute_ticks((TIME)500)
#define _1000ms     compute_ticks((TIME)1000)
#define _1500ms     compute_ticks((TIME)1500)
#define _2000ms     compute_ticks((TIME)2000)
#define _1sec		compute_ticks((TIME)1000)
#define _2sec		compute_ticks((TIME)2000)
#define _3sec		compute_ticks((TIME)3000)
#define _5sec		compute_ticks((TIME)5000)
#define _10sec		compute_ticks((TIME)10000)
#define _20sec		compute_ticks((TIME)20000)

#define boolean	byte	// Gli short non convegono xch?si sprecano istruzioni
						// che vanno a occupare memoria ottenendo :
						// codice pi?lento e maggiore occupazione memoria
#define TRUE        (1)
#define FALSE       (0)          

#define     NO_ERROR                0		// -200
#define     SYSERR                  -195
#define     BAD_ARG_1               -190
#define     BAD_ARG_2               -185
#define     BAD_ARG_3               -180
#define     BAD_ARG_4               -175
#define     BAD_ARG_8               -170
#define     NO_MORE_PROCESSES       -165
#define     NO_MORE_TIMERS          -160
#define     NO_MORE_MEMORY          -155
#define     NO_MORE_SEMAPHORES      -150
#define     CLEARED_SEMAPHORE       -145
#define     NO_CLOCK                -140
#define     MESSAGE_OVERRUN         -138
#define     NO_MORE_PORTS           -135
#define     HAS_MESSAGES            -130
#define     LOCKED_PORT             -125
#define     NOT_LOCKED_PORT         -120
#define     NO_MORE_POOLS           -115
#define     NOT_ATTACHED            -110
#define     YET_ATTACHED            -105
#define     NOT_OPEN                -100
#define     BAD_OPERATION            -95
#define     BAD_FUNCTION             -90
#define     BAD_FUNCTION_ARG_1       -85
#define     BAD_FUNCTION_ARG_2       -80
#define     BAD_FUNCTION_ARG_3       -75
#define     BAD_FUNCTION_ARG_4       -70
#define     BAD_FUNCTION_ARG_5       -65
#define     BAD_FUNCTION_ARG_6       -60
#define     BAD_FUNCTION_ARG_7       -55
#define     BAD_FUNCTION_ARG_8       -50
#define     INVALID_INTERVAL_TIME    -49
#define     BAD_SEEK_POSITION        -45
//#define     EOF                      -40
#define     MEMORY_ACCESS_ERROR      -35
#define     DOE_BREAK                -30
#define     DOE_INITIALIZE_FAILURE   -25
#define     DOE_FULL                 -24
#define     NO_MORE_SPACE            -20
#define     CHECKSUM_ERROR           -15
#define     INTERFACE_NOT_AVAILABLE  -10
#define     NO_MORE_IRPT_TASK        -9
 
#define SEEK_INITIAL    0
#define SEEK_CURRENT    1
#define SEEK_END        2

#ifdef __KERNEL__
// redefine interrupt enable/disable functions for linux 2.6
#define save_flags_cli(x)  local_irq_save(x)
#define save_flags_FIQ_IRQ_cli(x) local_irq_save(x);local_fiq_disable()
#define restore_flags(x) local_irq_restore(x)
#else
#define save_flags_cli(x)
#define save_flags_FIQ_IRQ_cli(x)
#define restore_flags(x)
#endif

#endif      
#endif      
