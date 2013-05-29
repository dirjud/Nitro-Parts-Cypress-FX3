#ifndef FX3_TERM_H
#define FX3_TERM_H

#include "handlers.h"

uint16_t fx3_read(CyU3PDmaBuffer_t*);

#define DECLARE_FX3_HANDLER(term) \
    DECLARE_HANDLER(HANDLER_TYPE_CPU,term,0,0,fx3_read,0,0,0,0,0)


#endif


