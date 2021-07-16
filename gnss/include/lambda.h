#ifndef _LAMBDA_H_
#define _LAMBDA_H_

#include "rtklib_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------*/

#define NUM_INT_AMB_CANDIDATE (2)
#define MAXNUM_INT_AMB_CANDIDATE (100)
#define MAX_NON_AMB_STATE (15)

extern int lambda(const double *a, const double *Q,const int n, const int m,
	              double *a_chk, double *ss_err, double *suc_rate);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
