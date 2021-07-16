#ifndef _RTCM_H_
#define _RTCM_H_

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "rtklib_core.h"
#include "rtkcmn.h"
#include "model.h"

#ifndef WIN32
#define ARM_MCU
#endif

	/*-----------------------------------------------------------*/
	/* from rtklib to decode RTCM3 */
#define RTCM2PREAMB 0x66 /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3 /* rtcm ver.3 frame preamble */

typedef struct {
    /* move the observation data struct out of rtcm definiton, to save more memory for PPP only mode */
    obs_t obs[MAXSTN];
    rtcm_t rcv[MAXSTN];
    nav_t  nav;
	double time;
} gnss_rtcm_t;

int decode_rtcm3(rtcm_t *rtcm, obs_t *obs, nav_t *nav);
int input_rtcm3_data(rtcm_t *rtcm, unsigned char data, obs_t *obs, nav_t *nav);

/* interface to GNSS db */
int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss);

unsigned int rtcm_getbitu(const unsigned char *buff, int pos, int len);

/* glo frquent number function */
extern void set_glo_frq(int prn, int frq);
extern int get_glo_frq(int prn);

 void set_week_number(int week);
 int  get_week_number();

int satno(int sys, int prn);

/* satellite function */
int  satsys(int sat, int *prn);
int  satidx(int sat, int *prn);
char satid (int sat, int *prn);
char sys2char(int sys);
extern double satwavelen(int sat, int code);

extern double satwavelenbyfreq(int sat, int frq);

unsigned char obs2code(int sys, const char * obs, int * freq);
unsigned char obs2coderinex(int sys, const char *obs, int *freq);

extern int code2frq(int sys, int code);

int getcodepri(int sys, unsigned char code, const char * opt);

void ecef2pos(const double *r, double *pos);
void pos2ecef(const double *pos, double *r);

void set_approximate_time(int year, int doy, rtcm_t *rtcm);

int add_obs(obsd_t* obsd, obs_t* obs);
int add_eph(eph_t* eph, nav_t* nav);
int add_geph(geph_t* eph, nav_t* nav);

int is_complete_rtcm(rtcm_t *rtcm, unsigned char data);
//int gen_rtcm3(rtcm_t* rtcm, obs_t *obs, int type, int sync);

extern void rtk_teseo(gnss_rtcm_t *gnss, unsigned int stnID, rcv_rtk_t *rtk);

#ifdef __cplusplus
}
#endif
#endif
