#if CONSOLE_PRINTF
#include <stdio.h>
#endif


#if CONSOLE_PRINTF
#define PRINTF(x) printf(x)
#else
#define PRINTF(x) (void)0
#endif

#ifdef CONSOLE_PRINTF
  #define DBGPRINTF(...) printf(__VA_ARGS__)
#else
  #define DBGPRINTF(...) (void)0
#endif


//#define DBG_LEVEL_1
//#define DBG_LEVEL_2
//#define DBG_LEVEL_3
