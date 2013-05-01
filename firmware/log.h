#ifndef __LOG_H__
#define __LOG_H__


/**
 * log_debug is defined to do nothing unless CCFLAGS has -DENABLE_LOGGING
 * You can also add
 * -D LOG_LOCATION=1 - add file:line to log statements
 * -D LOG_THREAD_ID=1 - add thread id to start of log
 *  (Note right now thread id requires location)
 **/

#if ENABLE_LOGGING

#include <cyu3system.h>

enum {
  LOG_ERROR = 0,
  LOG_WARN,
  LOG_INFO,
  LOG_DEBUG,
};


#ifndef LOG_THREAD_ID
#define LOG_THREAD_ID 0
#endif
#ifndef LOG_LOCATION
#define LOG_LOCATION 0
#endif

#if LOG_LOCATION
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

#define log_debug(X, ...) log_stmt(LOG_DEBUG, X, ##__VA_ARGS__)
#define log_info(X, ...) log_stmt(LOG_INFO, X, ##__VA_ARGS__)
#define log_warn(X, ...) log_stmt(LOG_WARN, X, ##__VA_ARGS__)
#define log_error(X, ...) log_stmt(LOG_ERROR, X, ##__VA_ARGS__)

#else
// logging not enabled
#define log_debug(...) do {} while (0)
#define log_info(...) do {} while(0)
#define log_warn(...) do {} while(0)
#define log_error(...) do {} while (0)

#endif


#endif
