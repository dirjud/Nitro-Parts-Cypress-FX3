#include <cyu3system.h>
#include "handlers.h"
#include <m24xx.h>
#include "fx3_terminals.h"
#include "fx3_term.h"

m24xx_config_t m24_config = { .dev_addr = TERM_FX3_PROM,
			      .bit_rate = 400000,
			      .size = 17, // 128KB prom
};

app_init_t app_init[] = {
    { APP_INIT_VALID, 0, 0, 0, 0 },
    { 0 }
};

io_handler_t io_handlers[] = {
  DECLARE_DUMMY_HANDLER(TERM_DUMMY_FX3),
  DECLARE_FX3_HANDLER(TERM_FX3),
  DECLARE_M24XX_HANDLER(TERM_FX3_PROM, &m24_config),
  DECLARE_TERMINATOR
};
