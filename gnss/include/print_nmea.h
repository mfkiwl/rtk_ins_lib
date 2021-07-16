#ifndef PRINT_NMEA_H
#define PRINT_NMEA_H

/*--------------------------------------------------------*/
/* code from rtklib to decode RTCM3 */
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "model.h"
#include "rtk_eng.h"

#ifdef __cplusplus
extern "C" {
#endif

	extern int print_pos_gga(gtime_t gtime, const double* pos, unsigned char num_of_sat,
		unsigned char fixID, double *dop, double geo_sep, double age, char* gga);

	extern int print_nmea_gst(const double *ep, const float *var_llh, char *buff);

	extern int print_rmc(gtime_t gtime, const double* ecef, int fixID, char* buff);

	extern int print_zda(gtime_t gtime, char* buff);

	extern int print_gsa(unsigned char* buff, int fixID, const vec_t* ssat, const obs_t* rov, double *dop);

	extern int print_gsv(unsigned char *buff, int fixID, const vec_t *ssat, const obs_t *rov);


#ifdef __cplusplus
}
#endif
#endif
