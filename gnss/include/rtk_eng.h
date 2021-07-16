#ifndef _RTK_ENG_H_
#define _RTK_ENG_H_

#include <stdio.h>
#include <stdlib.h>
#include "gnss_datatype.h"
#include "rtk_math.h"
#include "lambda.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SECOND_PER_HOUR 3600
/*-----------------------------------------------------------*/
#define MAX_HOR_VEL  60                 /* max horizon  velocity (m/sec)*/
#define MIN_HOR_VEL  10                 /* min horizon  velocity (m/sec)*/
#define MAX_VER_VEL  10                 /* max vertical velocity (m/sec)*/
#define MAX_HDOP    10.0                /* max horizon dop*/
#define MAX_VDOP    10.0                /* max vertical dop*/

//#define GLO_AR
/* epoch-by-epoch folat filter */
#ifndef NX_MAX
#define NX_MAX (MAXOBS+MAXOBS)
#endif

#define IX_PHB (10+NX_CLK)   /* state index of sd_refamb*/

#define _PVA_
#define MIN_AMBNUM     8
#define MIN_NLOCK1     1 
#define MIN_NLOCK2     1

#define DPCS_THRES1   1.5
#define DPCS_THRES2   0.3

typedef struct
{
	double x[4];
	double P[SMD(4)];
    double pos[3];
    double dop[5];
	int np;
    double phas_omg;
    double code_omg;
    int phas_num;
    int code_num;
    int cpID;
    int pfID;
	double time;
	double age;
}rcv_tdp_t;

/* algorithm */

int timediff_processor(epoch_t *rov_epoch, epoch_t *rov_lastepoch, rcv_tdp_t *rcv);

int rtk_processor(epoch_t *rov_epoch, epoch_t* ref_epoch, rcv_rtk_t *rcv);

int rtk_reinitialization(double last_cbr, rcv_rtk_t *rtk, rcv_tdp_t *tdp);

int reset_rcv(rcv_rtk_t *rcv);

int doppler_smoothed_code(epoch_t *rov_epoch, rcv_rtk_t *rcv);

void use_sm_codeobs(epoch_t *rov_epoch, rcv_rtk_t *rcv);

int check_epoch_data_quality(obs_t* obs_rov, vec_t *vec, rcv_rtk_t* rtk);

int vel_processor(epoch_t *rov_epoch);

int rtk_pva_smoothing(double cur_time, rcv_pva_t *rtk_pva, rcv_rtk_t *rtk);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
