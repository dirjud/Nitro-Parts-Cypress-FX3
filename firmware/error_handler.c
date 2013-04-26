/* Application Error Handler */

#include <cyu3os.h>
#include <cyu3error.h>
#include "error_handler.h"
#include "log.h"

void error_handler(CyU3PReturnStatus_t apiRetStatus) {
  /* Application failed with the error code apiRetStatus */
  /* Add custom debug or recovery actions here */
  /* Loop Indefinitely */
  for (;;) {
    /* Thread sleep : 100 ms */
    CyU3PThreadSleep (1000);
    log_error("The app is stuck in the error_handler: %d\n", apiRetStatus );
  }
}
