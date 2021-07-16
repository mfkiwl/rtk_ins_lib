#ifndef _EPHEMERIS_H_
#define _EPHEMERIS_H_

#include "rtcm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* by Dr. Yudan Yi */

/*-----------------------------------------------------------*/

#define EPHOPT_BRDC    0                   /* ephemeris option: broadcast ephemeris */

/* compute satellit position */
void satposs(obs_t *obs, vec_t *vec, nav_t *nav, int ephopt);

int compute_vector_data(obs_t* obs, vec_t* vec);

int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
	const nav_t *nav, double *rs, double *dts, double *var,
	int *svh);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
