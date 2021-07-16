#ifndef _GNSS_FILTER_H_
#define _GNSS_FILTER_H_

#include <stdio.h>
#include "gnss_datatype.h"
#ifdef __cplusplus
extern "C" {
#endif

double median_measure(measure_t* dat, int n);

void ekf_measurement_update(double* x, double* P, const double* H, const int* L, 
	double inov, double P_inov, int n, int m, double* PHt);

void ekf_measurement_predict(double* x, double* P, const double* H, const int* L, 
	double* z_, double* R_, int n, int m, double* PHt);

/* x_, P_ => input 
   x , P  => output
   PHt => temp matrix , to save memory space
*/
extern int measure_update_kalman_filter(double time, measure_t* meas, int nm, 
	double* x_, double* P_, double* x, double* P, double* PHt, int max_st, char type);

extern void compute_Qvv(double *P, const double *H, const int *L, double *R_, int n, int m, double *PHt);

extern int week_number(double sec);

extern double week_second(double sec);

extern int find_next_state_index(state_tag_t* tag, int np, int ns, int max_st);

extern int find_state_index(state_tag_t* tag, int ns, int s1, int s2, int f);

#ifdef __cplusplus
}
#endif
#endif
