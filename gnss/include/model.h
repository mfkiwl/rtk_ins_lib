#ifndef _MODEL_H_
#define _MODEL_H_

#include <stdio.h>
#include "rtklib_core.h"

#ifdef __cplusplus
extern "C" {
#endif


extern void blh2C_en(const double *blh, double C_en[3][3]);

extern void ned2xyz(double C_en[3][3], double *ned, double *covNED, double *xyz, double *covXYZ);

extern void xyz2ned(double C_en[3][3], double *xyz, double *covXYZ, double *ned, double *covNED);

extern void blhdiff(double *blh, double *blh_ref, double *ned);

extern void ecef2enu(const double *pos, const double *r, double *e);

extern void xyz2enu_(const double *pos, double *E);

extern void enu2ecef(const double *pos, const double *e, double *r);

extern void blh2xyz(const double *blh, double *xyz);

extern void xyz2blh(const double *xyz, double *blh);

extern void covecef(const double *pos, const double *Q, double *P);

extern void xyz2enu(const double *pos, double *E);

extern double satazel(const double *pos, const double *e, double *azel);

extern double geodist(const double *rs, const double *rr, double *e);

extern double geovel(const double *rs, const double *rr, double *e);

extern double tropmodel(const double *blh, const double *azel, double humi);

extern double tropmapf(gtime_t time, const double pos[], const double azel[], double *mapfw);

void deg2dms(double deg, double *dms, int ndec);

extern void ned2_Clateral_lon(const double heading, double C_en[3][3]);

extern void ned2lld(double C_en[3][3], double *ned, double *covNED, double *lld, double *covLLD);



#ifdef __cplusplus
}
#endif
#endif
