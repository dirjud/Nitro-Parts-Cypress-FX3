#ifndef FX3_TERM_H
#define FX3_TERM_H

#include "handlers.h"

uint16_t fx3_read(CyU3PDmaBuffer_t*);
uint16_t fx3_write(CyU3PDmaBuffer_t*);

typedef struct {
  int gpif_clk_div;
  CyBool_t gpif_clk_halfdiv;
  int gpif_clk_src;
  int gpif_drive_strength;
  CyBool_t enable;
} fx3_gpif_config_t;

extern fx3_gpif_config_t fx3_gpif_config;

#define DECLARE_FX3_HANDLER(term) \
    DECLARE_HANDLER(HANDLER_TYPE_CPU,term,0,0,fx3_read,fx3_write,0,0,0,0)


#endif


