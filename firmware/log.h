#ifndef __LOG_H__
#define __LOG_H__

#include <cyu3system.h>

enum {
  LOG_ERROR = 0,
  LOG_WARN,
  LOG_INFO,
  LOG_DEBUG,
};


#define log_debug(X, ...) CyU3PDebugPrint(LOG_DEBUG, X, ##__VA_ARGS__)
#define log_error(X, ...) CyU3PDebugPrint(LOG_ERROR, X, ##__VA_ARGS__)



#endif
