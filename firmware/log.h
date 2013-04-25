#ifndef __LOG_H__
#define __LOG_H__

#include <cyu3system.h>

/**
 * Size of max buffer for debug statements
 * If you have a long string to print you might get the end
 * ignored.  Else increase this if it's a big deal.
 **/
#define NITRO_MAX_DEBUG_LEN 100

enum {
  LOG_ERROR = 0,
  LOG_WARN,
  LOG_INFO,
  LOG_DEBUG,
};

#define LOG_THREAD_ID 0
#define LOG_COMPLEX 0

#if LOG_COMPLEX
#if LOG_THREAD_ID
#define log_stmt(LEVEL,X, ...) do {\
    uint8_t *pName;\
    CyU3PThread *pThread = CyU3PThreadIdentify();\
    CyU3PThreadInfoGet(pThread, &pName, NULL, NULL, NULL );\
    uint16_t tId = *((uint16_t*)pName);\
    CyU3PDebugPrint(LEVEL, "%d:%s:%d " X, tId, __FILE__, __LINE__, ##__VA_ARGS__); /* CyU3PDebugLogFlush(); */ } while (0)
#else
 // no thread ids
#define log_stmt(LEVEL,X, ...) do {\
    CyU3PDebugPrint(LEVEL, "%s:%d " X, __FILE__, __LINE__, ##__VA_ARGS__); /* CyU3PDebugLogFlush(); */ } while (0)
#endif
#else 
 // more simple debug
#define log_stmt(LEVEL,X,...) do { CyU3PDebugPrint(LEVEL,X, ##__VA_ARGS__); } while (0)
#endif

void nitro_log_stmt(uint32_t level, char* format, ...);

#define log_debug(X, ...) log_stmt(LOG_DEBUG, X, ##__VA_ARGS__)
#define log_error(X, ...) log_stmt(LOG_ERROR, X, ##__VA_ARGS__)



#endif
