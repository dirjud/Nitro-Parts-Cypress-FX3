#include <cyu3system.h>
#include "handlers.h"
#include <m24xx.h>
#include "fx3_terminals.h"
#include <spartan.h>

m24xx_config_t m24_config = { .dev_addr = TERM_FX3_PROM,
			      .bit_rate = 400000,
			      .size = 17, // 128KB prom
};


io_handler_t io_handlers[] = {
  DECLARE_DUMMY_HANDLER(5),
  DECLARE_M24XX_HANDLER(TERM_FX3_PROM, &m24_config),
  DECLARE_SPARTAN_HANDLER(0),
  DECLARE_TERMINATOR
};
