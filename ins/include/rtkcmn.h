#ifndef RTKCMN_H
#define RTKCMN_H


#include <time.h>
#include <string.h>
#include "gnss_interface_API.h"
#ifdef WIN32
#include <windows.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define USE_CMN

/* time and string functions -------------------------------------------------*/
#ifdef USE_CMN
int     str2time(const char *s, int i, int n, gtime_t *t);
void    time2str(gtime_t t, char *str, int n);
gtime_t epoch2time(const double *ep);
void    time2epoch(gtime_t t, double *ep);
gtime_t gpst2time(int week, double sec);
double  time2gpst(gtime_t t, int *week);
gtime_t gst2time(int week, double sec);
double  time2gst(gtime_t t, int *week);
gtime_t bdt2time(int week, double sec);
double  time2bdt(gtime_t t, int *week);
char    *time_str(gtime_t t, int n);
gtime_t timeadd(gtime_t t, double sec);
double  timediff(gtime_t t1, gtime_t t2);
gtime_t gpst2utc(gtime_t t);
gtime_t utc2gpst(gtime_t t);
gtime_t gpst2bdt(gtime_t t);
gtime_t bdt2gpst(gtime_t t);
gtime_t timeget(void);
void    timeset(gtime_t t);
#else
/* time function */
gtime_t timeadd(gtime_t t, double sec);
double  timediff(gtime_t t1, gtime_t t2);
gtime_t epoch2time(const double *ep);
void    time2epoch(gtime_t t, double *ep);
gtime_t bdt2time(int week, double sec);
double  time2bdt(gtime_t t, int *week);
double  time2gpst(gtime_t t, int *week);
gtime_t utc2gpst(gtime_t t);
gtime_t gpst2utc(gtime_t t);
gtime_t gpst2time(int week, double sec);
gtime_t gpst2bdt(gtime_t t);
gtime_t bdt2gpst(gtime_t t);
void    time2str(gtime_t t, char *s, int n);
char   *time_str(gtime_t t, int n);
gtime_t timeget();
void    timeset(gtime_t t);
#endif // USE_CMN

int adjgpsweek(gtime_t * time, int week);
int adjbdtweek(gtime_t * time, int week);
void adjday_glot(gtime_t * time, double tod);
void adjweek(gtime_t *time, double tow);

unsigned int rtk_crc24q(const unsigned char *buff, int len);
/* satellites, systems, codes functions --------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
