/* Application Error Handler */

#include <cyu3os.h>
#include "error_handler.h"

void error_handler(CyU3PReturnStatus_t apiRetStatus) {
  /* Application failed with the error code apiRetStatus */
  /* Add custom debug or recovery actions here */
  /* Loop Indefinitely */
  for (;;) {
    /* Thread sleep : 100 ms */
    CyU3PThreadSleep (100);
  }
}
