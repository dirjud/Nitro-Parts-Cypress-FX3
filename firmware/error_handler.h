#ifndef __ERROR_HANDLER_H__
#define __ERROR_HANDLER_H__

#include <cyu3types.h>
#include "log.h"

void error_handler(CyU3PReturnStatus_t apiRetStatus);
//
// TODO replace error_handler calls with log_error and
// handle in app.  Unless we can find a good way to make a universal handler.

//#define error_handler(r) log_error( "Application Error: %d\n", r )


#endif
