#ifndef __LOG_H__
#define __LOG_H__


/**
 * log_debug is defined to do nothing unless CCFLAGS has -DENABLE_LOGGING
 * You can also add
 * -D LOG_LOCATION=1 - add file:line to log statements
 * -D LOG_THREAD_ID=1 - add thread id to start of log
 *  (Note right now thread id requires location)
 **/

 void init_uart_debug();

#ifdef ENABLE_LOGGING

#include <cyu3system.h>

enum {
  LOG_ERROR = 0,
  LOG_WARN,
  LOG_INFO,
  LOG_DEBUG,
};

void logging_boot();

#ifdef USB_LOGGING

// treat logging as usb pipe instead of over UART
#define LOG_BUFFER_SIZE 2048
#define LOG_STMT_MAX 100


#include "handlers.h"

uint16_t log_read(CyU3PDmaBuffer_t*);

void log_stmt2(unsigned LEVEL, const uint8_t* stmt);

#define log_stmt(LEVEL,X,...) do { \
  uint8_t tmp[LOG_STMT_MAX]; \
  if (!CyU3PDebugStringPrint(tmp,LOG_BUFFER_SIZE,X, ##__VA_ARGS__)) { \
    log_stmt2(LEVEL,tmp); \
  } \
} while (0)

//#define log_stmt(LEVEL,X,...) do {} while(0)

#define DECLARE_LOG_HANDLER(term) \
  DECLARE_HANDLER(&glCpuHandler,term,0,0,log_read,0,0,0,0,0)

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
