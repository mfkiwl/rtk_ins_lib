#ifndef _PVT_ENG_H_
#define _PVT_ENG_H_

#include <stdio.h>
#include "rtcm.h"
#include "gnss_datatype.h"
#include "ephemeris.h"
#include "gnss_filter.h"


#ifdef __cplusplus
extern "C" {
#endif

/* algorithm */
extern int comp_dop(measure_t* meas, int nm, double* dop);

extern int spp_processor(epoch_t* epoch, rcv_rtk_t *rcv, char *gga, char *sol);

#ifdef __cplusplus
}
#endif
#endif
