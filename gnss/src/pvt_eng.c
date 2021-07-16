#include "pvt_eng.h"

/*--------------------------------------------------------*/

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "rtk_math.h"
#include "model.h"
#include "rtklib_core.h"

#define MEO_GPS 20200000       /* standard pseudorange (m) of GPS MEO satellite */
#define MEO_GLO 19100000       /* standard pseudorange (m) of GLO MEO satellite */
#define MEO_GAL 26900000       /* standard pseudorange (m) of GAL MEO satellite */
#define MEO_BDS 21500000       /* standard pseudorange (m) of BDS MEO satellite */
#define GEO_BDS 35800000       /* standard pseudorange (m) of GEO/IGSO satellite */

/* Pseudorange measurement error variance -----------------------------*/
RTK_RAM_CODE static void codevare(int sat, int sys, double el, double snr, double *codeCOV)
{
	double a = 0.3, b = 0.3, sys_scale = 1.0;
	double sinel = sin(el);

	if (snr < 30.0)
	{
		*codeCOV = 5000 * 5000;
	}
	else if(snr < 35.0)
	{
		*codeCOV = 500 * 500;
	}
	else
	{
		*codeCOV = a*a + b * b *(1.0 / sinel / sinel);
	}

	if (sys == _SYS_GLO_) sys_scale = 2.0;
	if (sys == _SYS_BDS_) sys_scale = 1.5;
	if (sys == _SYS_GAL_) sys_scale = 1.0;
	
	*codeCOV *= sys_scale * sys_scale;

}

// added by Xuanxuan HU, 05/06/2020
/* LU decomposition ----------------------------------------------------------*/
RTK_RAM_CODE static int ludcmp(double *A, int n, int *indx, double *d)
{
	double big, s, tmp, *vv = mat(n, 1);
	int i, imax = 0, j, k;

	*d = 1.0;
	for (i = 0; i < n; i++) {
		big = 0.0; for (j = 0; j < n; j++) if ((tmp = fabs(A[i + j * n])) > big) big = tmp;
		if (big > 0.0) vv[i] = 1.0 / big; else { free(vv); return -1; }
	}
	for (j = 0; j < n; j++) {
		for (i = 0; i < j; i++) {
			s = A[i + j * n]; for (k = 0; k < i; k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
		}
		big = 0.0;
		for (i = j; i < n; i++) {
			s = A[i + j * n]; for (k = 0; k < j; k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
			if ((tmp = vv[i] * fabs(s)) >= big) { big = tmp; imax = i; }
		}
		if (j != imax) {
			for (k = 0; k < n; k++) {
				tmp = A[imax + k * n]; A[imax + k * n] = A[j + k * n]; A[j + k * n] = tmp;
			}
			*d = -(*d); vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (A[j + j * n] == 0.0) { free(vv); return -1; }
		if (j != n - 1) {
			tmp = 1.0 / A[j + j * n]; for (i = j + 1; i < n; i++) A[i + j * n] *= tmp;
		}
	}
	free(vv);
	return 0;
}
/* LU back-substitution ------------------------------------------------------*/
RTK_RAM_CODE static void lubksb(const double *A, int n, const int *indx, double *b)
{
	double s;
	int i, ii = -1, ip, j;

	for (i = 0; i < n; i++) {
		ip = indx[i]; s = b[ip]; b[ip] = b[i];
		if (ii >= 0) for (j = ii; j < i; j++) s -= A[i + j * n] * b[j]; else if (s) ii = i;
		b[i] = s;
	}
	for (i = n - 1; i >= 0; i--) {
		s = b[i]; 
		for (j = i + 1; j < n; j++) s -= A[i + j * n] * b[j]; b[i] = s / A[i + i * n];
	}
}
/* inverse of matrix ---------------------------------------------------------*/
RTK_RAM_CODE static int matinv(double *A, int n)
{
	double d, *B;
	int i, j, *indx;

	indx = imat(n, 1); 
	B = mat(n, n);
	matcpy(B, A, n, n);
	if (ludcmp(B, n, indx, &d)) { free(indx); free(B); return -1; }
	for (j = 0; j < n; j++) {
		for (i = 0; i < n; i++) A[i + j * n] = 0.0;
		A[j + j * n] = 1.0;
		lubksb(B, n, indx, A + j * n);
	}
	free(indx); free(B);
	return 0;
}

/* least square estimation 
 * return : status (0:ok,0>:error) */
RTK_RAM_CODE static int lsq_estimate(const double *H, const double *P, const double *L, 
	const int n, const int m, double *N, double *W, double *X)
{
	double HtP[NX*MAXOBS*NFREQ] = { 0 };
	int i, j, k, info;

	/* H--n*m, P--n*n,HtP--m*n */
	for (i = 0; i < m; ++i)
	{
		for (j = 0; j < n; ++j)
		{
			HtP[i*n+j] = H[j*m+i]*P[j];
		}
	}

	/* PHt = P*H' */
	if (N != NULL && W != NULL)
	{
		for (i = 0; i < m; ++i)
		{	/* N=H'PH */
			for (j = 0; j < m; ++j)
			{
				N[i*m + j] = 0.0;
				for (k = 0; k < n; ++k)
				{
					N[i*m+j] += HtP[i*n + k] * H[k*m+j];
				}				
			}
			/* W=H'PL */
			W[i] = 0.0;
			for (k = 0; k < n; ++k)
			{
				W[i] += HtP[i*n + k] * L[k];
			}
		}
	}

	info = matinv(N, m);
	if (info) return 1;

	for (i = 0; i < m; ++i)
	{
		/* X=inv(N)*W */
		X[i] = 0.0;
		for (k = 0; k < m; ++k)
		{
			X[i] += N[i*m + k] * W[k];
		}
	}
	return 0;
}

/* validate solution ---------------------------------------------------------*/
RTK_RAM_CODE static int valsol(int n, const double *v, const double *P, int nv, int nx)
{
	double chisqr[100] = {      /* chi-sqr(n) (alpha=0.001) */
		10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
		31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
		46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
		61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
		74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
		88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
		101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
		113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
		126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
		138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
	};
	double vpv = 0.0;
	int n_para;

	n_para = nv - n + 3;
	if(nv < n_para + 1) return 0;

	/* chi-square validation of residuals */
	for (int i = 0; i < nv; i++)
	{
		vpv += v[i] * P[i] * v[i];
	}
	vpv /= (nv - n_para);

	if (vpv < 0.0001)
		return 0;

	if (nv > nx && vpv > 3 * chisqr[nv - n_para]) {
		return 0; /* threshold too strict for all use cases, report error but continue on */
	}

	return 1;
}

RTK_RAM_CODE extern int comp_dop(measure_t* meas, int nm, double* dop)
{
	/* compute dops */
	int k, i;
	int satIDs[MAXOBS] = { 0 };
	double HH[4 * MAXOBS] = { 0.0 }, Q[16];
	int n = 0;

	for (k = 0; k < nm; k++)
	{
		if (meas[k].flag == 1 && nm > 15)  continue;

		/* book-keeping the used satellite IDs */
		for (i = 0; i < n; ++i)
		{
			if (satIDs[i] == meas[k].sat)
			{
				break;
			}
		}
		if (i < n) continue; /* this satellite already used */
		satIDs[n] = meas[k].sat;

		double azi = meas[k].azim;
		double elev = meas[k].elev;
		double cosel = cos(elev);
		double sinel = sin(elev);
		HH[0 + 4 * n] = cosel * sin(azi);
		HH[1 + 4 * n] = cosel * cos(azi);
		HH[2 + 4 * n] = sinel;
		HH[3 + 4 * n] = 1.0;
		++n;
	}

	if (n >= 5)
	{
		matmul("NT", 4, 4, n, 1.0, HH, HH, 0.0, Q);
		double Q_[16] = { 0.0 };
		if (!inv4(Q, Q_))
		{
			dop[0] = sqrt(Q_[0] + Q_[5] + Q_[10] + Q_[15]); /* GDOP */
			dop[1] = sqrt(Q_[0] + Q_[5] + Q_[10]);          /* PDOP */
			dop[2] = sqrt(Q_[0] + Q_[5]);                   /* HDOP */
			dop[3] = sqrt(Q_[10]);                          /* VDOP */
		}
	}
	else
	{
		dop[2] = 99.0;
		return 0;
	}

	return n;
}

RTK_RAM_CODE static int cmpres(const void *p1, const void *p2)
{
	double *q1 = (double *)p1, *q2 = (double *)p2;
	int ret;

	ret = (int)((*q1)*1000 - (*q2)*1000);
	
	return ret;
}

RTK_RAM_CODE static void resdel(double *V, double *P, int num)
{
	int i;
	double mean_value;
	double data[MAXOBS*NFREQ] = { 0 };

	if (num < 4)	return;

	for (i = 0; i < num; i++)
	{
		data[i] = fabs(V[i]);
	}

	qsort(data, num, sizeof(double), cmpres);

	i = (int)(num / 2);
	mean_value = data[i];

	if (mean_value < 100 && mean_value > 10)
	{
		for (i = 0; i < num; i++)
		{
			if (fabs(V[i]) > mean_value * 50)
				P[i] = 1E-6 * 1E-6;
		}
	}
	else
	{
		for (i = 0; i < num; i++)
		{
			if (fabs(V[i]) > mean_value * 10)
				P[i] = 1E-6 * 1E-6;
		}
	}

}

RTK_RAM_CODE static void creobs(obs_t* obs)
{
	int i, f, sys, prn;
	double range1, range2;

	for (f = 0; f < NFREQ; f++)
	{
		for (i = 0; i < (int)obs->n && i < MAXOBS; i++) 
		{
			if (obs->data[i].sat <= 0 || obs->data[i].P[f] == 0.0)
			{
				continue;
			}

			sys = satsys(obs->data[i].sat, &prn);
			if (sys == _SYS_GPS_ && prn < NSATGP0)
			{
				range1 = fabs(obs->data[i].P[f] - MEO_GPS);
				if (range1 > RE_WGS84)
				{
					obs->data[i].qualP[f] = 4;
					obs->data[i].P[f] = 0.0;
					obs->data[i].L[f] = 0.0;
					obs->data[i].D[f] = 0.0;
				}
			}
			else if (sys == _SYS_GLO_) 
			{
				range1 = fabs(obs->data[i].P[f] - MEO_GLO);
				if (range1 > RE_WGS84)
				{
					obs->data[i].qualP[f] = 4;
					obs->data[i].P[f] = 0.0;
					obs->data[i].L[f] = 0.0;
					obs->data[i].D[f] = 0.0;
				}
			}
			else if (sys == _SYS_GAL_) 
			{
				range1 = fabs(obs->data[i].P[f] - MEO_GAL);
				if (range1 > RE_WGS84)
				{
					obs->data[i].qualP[f] = 4;
					obs->data[i].P[f] = 0.0;
					obs->data[i].L[f] = 0.0;
					obs->data[i].D[f] = 0.0;
				}
			}
			else if (sys == _SYS_BDS_) 
			{
				range1 = fabs(obs->data[i].P[f] - GEO_BDS);
				range2 = fabs(obs->data[i].P[f] - MEO_BDS);
				if (range1 > RE_WGS84 && range2 > RE_WGS84)
				{
					obs->data[i].qualP[f] = 4;
					obs->data[i].P[f] = 0.0;
					obs->data[i].L[f] = 0.0;
					obs->data[i].D[f] = 0.0;					
				}								
			}
			else 
			{
				continue;
			}
		}
	}

}

RTK_RAM_CODE static int init_spp_iter(epoch_t* epoch, rcv_spp_t *rcv, double *dop)
{
	obs_t* obs = &epoch->obs;
	vec_t* vec = epoch->vec + 0;
	obsd_t *pObs = NULL;
	vec_t *pVec = NULL;
	int i, j, s, f, sat, sys, n, n_last, info, n_loop, n_obsfrq, nsat;
	int max_st = NX, week = 0, prn = 0;
	double elev;
	double pos[3] = { 0 }, dx[NX] = { 0 }, cdt[NSYS*NFREQ] = { 0 };
	double H[(MAXOBS*NFREQ + NFREQ * NSYS)*NX];
	double P[MAXOBS*NFREQ + NFREQ * NSYS], L[MAXOBS*NFREQ + NFREQ * NSYS];
	double N[NX*NX], W[NX];
	double cur_time, last_time, clk_err, codeCOV, maskElev = 10.0 *D2R;
	int mask[NSYS*NFREQ] = {0}, loop = 0, flag_covg = 0;
	double tmp_data, ep[6] = { 0 };
	measure_t meas[MAXOBS * NFREQ] = { 0 };

	time2epoch(obs->time, ep);

	creobs(obs);
	//memset(obs->pos, 0, sizeof(double) * 3);
	last_time = rcv->time;
	cur_time = time2gpst(obs->time, &week);
	for (i = 0; i < 3; ++i)
	{
		obs->pos[i] = rcv->x[i];
	}
	rcv->time = cur_time;

	n_loop = obs->n * NFREQ * 0.35 < 10 ? 10 : obs->n * NFREQ * 0.35;
	if (n_loop > 30) n_loop = 30;

	while (++loop <= n_loop)
	{
		compute_vector_data(obs, vec);		
		n = n_last = n_obsfrq = nsat = 0;
		memset(H, 0, sizeof(H));
		for (i = 0; i < (int)obs->n; ++i)
		{
			meas[i].flag = 1;
		}
		for (s = 0; s < NSYS; ++s)
		{
			for (i = 0; i < (int)obs->n; ++i)
			{
				pObs = obs->data + i;
				pVec = vec + i;
				sat = pObs->sat;
				sys = satidx(sat, &prn);
				/* process data by satellite system index */
				elev = pVec->azel[1];
				if (sys != s || s_norm(pVec->rs, 3) < 1.0 || (flag_covg && elev < maskElev) )
				{
					continue;
				}
				for (f = 0; f < NFREQ; ++f)
				{
					if (pObs->P[f] < RE_WGS84 || pObs->qualP[f] == 4)
					{
						continue;
					}

					if (flag_covg > 0)
					{
						if (pObs->qualP[f] == 5) 
						{ 
							pObs->qualP[f] = 4; n_last++; 
							continue; 
						}
						else if (pObs->qualP[f] == 3) { tmp_data = P[n_last]; P[n] = tmp_data * 0.15; }
						else if (pObs->qualP[f] == 2) { tmp_data = P[n_last]; P[n] = tmp_data * 1.0; }
						else if (pObs->qualP[f] == 1) { tmp_data = P[n_last]; P[n] = tmp_data * 1.0; }
						else { tmp_data = P[n_last]; P[n] = tmp_data; }
					}
					else
					{
						codevare(prn, satsys(pObs->sat, &prn), elev, pObs->SNR[f], &codeCOV);
						P[n] = 1.0/codeCOV;
					}								
					
					H[n*NX + 0] = -pVec->e[0];
					H[n*NX + 1] = -pVec->e[1];
					H[n*NX + 2] = -pVec->e[2];
					H[n*NX + 3] = 1.0;
					clk_err = cdt[0];
					if (!(s == 0 && f == 0))
					{
						H[n*NX + 3 + MI(s, f, NFREQ)] = 1.0;
						clk_err += cdt[MI(s, f, NFREQ)];
					}
					L[n] = pObs->P[f] - (pVec->r + clk_err + pVec->tro);
					if (flag_covg > 0)
					{
						if (fabs(L[n]) > 20.0) { pObs->qualP[f] = 5;}
						else if (fabs(L[n]) > 15.0) { pObs->qualP[f] = 3; }
						else if (fabs(L[n]) > 10.0) { pObs->qualP[f] = 2; }
						else if (fabs(L[n]) > 5.0)  { pObs->qualP[f] = 1; }
						else { pObs->qualP[f] = 0;}
					}
					mask[MI(s, f, NFREQ)] = 1;
					meas[i].sat = sat;
					meas[i].elev = pVec->azel[1];
					meas[i].azim = pVec->azel[0];
					meas[i].flag = 0;
					++n; 
					++n_last;
					++n_obsfrq;
				}
				++nsat;
			}
		}
		resdel(L, P, n);
		/* constraint to avoid rank-deficient */
		for (i = 0; i < NSYS*NFREQ; i++) {
			if (mask[i]) continue;
			L[n] = 0.0;
			for (j = 0; j < NX; j++) H[j + n * NX] = j == i + 3 ? 1.0 : 0.0;
			P[n++] = 100.0;
		}

		if (info = lsq_estimate(H, P, L, n, NX, N, W, dx))  return 0;
		if  (n < 4  )  return 0;
		if  (s_norm(dx, 3) < 10.0)  flag_covg++;
		for (i = 0; i < 3; ++i)  obs->pos[i] += dx[i];
		for (i = 0; i < NSYS*NFREQ; ++i)   cdt[i] += dx[3+i];
		if (s_norm(dx, NX) < 1E-4) {
			rcv->time = cur_time - cdt[0] / CLIGHT;
			comp_dop(meas, obs->n, dop);
			if (dop[0] > 3.0 || dop[0] < 0.01 || !valsol(n_last, L, P, n, NX))
			{
				rcv->n_used = 0;
				rcv->solType = 0;
				n = 0;
				memset(rcv->x, 0, sizeof(double)*NX_SPP);
				return 0;
			}
			for (i = 0; i < 3; ++i)
			{
				rcv->x[i] = obs->pos[i];
				//obs->pos[i] = pos[i];
			}
			for (i = 0; i < NSYS*NFREQ; ++i)   rcv->x[i + 3] = cdt[i];

			rcv->n_used = nsat;
			rcv->solType = 1;	// SPP Solution Type.
			return n;			
		}
	}

	rcv->n_used = 0;
	rcv->solType = 0;
	n = 0;
	memset(rcv->x, 0, sizeof(double)*NX_SPP);
	return 0;
}

RTK_RAM_CODE extern int spp_processor(epoch_t* epoch, rcv_rtk_t *rcv, char *gga, char *sol)
{
	obs_t* obs = &epoch->obs;
	int i, week, num_of_obs = 0;
	double cur_time, tt;
	rcv_spp_t *spp = &rcv->spp;
	double dop[5] = { 0 }, rtk_pos[6];

	cur_time = time2gpst(obs->time, &week);
	cur_time = week * 86400 * 7 + cur_time;
	spp->n_used  = 0;
	spp->solType = 0;

	if (obs->n < 4)
	{
		/* do not have enough satellites */
		memset(obs->pos, 0.0, sizeof(double) * 6);
		return 0;
	}

	if (obs->rtcmtype == 99)
	{
		if (s_norm(obs->pos, 3) > RE_WGS84 *0.5)
		{
			for (i = 0; i < 6; i++)
			{
				spp->x[i] = obs->pos[i];
			}
			spp->n_used = obs->n;
			spp->solType = 1;
			spp->time = cur_time;
			//rcv->dop[0] = rcv->dop[1] = rcv->dop[2] = rcv->dop[3] = 1.0;
			//rcv->geo_sep = obs->geo_sep;
			memcpy(obs->pos, spp->x, sizeof(double) * 6);
			return 1;
		}
		else
		{
			memset(spp->x, 0.0, sizeof(double) * 6);
			return 0;
		}
	}

	tt = cur_time - rcv->time;
	if(rcv->fixType >= 2 && tt < 10.0)
	{
		
		for (i = 0; i < 3; i++)
		{
			spp->x[i] = rcv->x[i] + rcv->x[i + 3] * tt;
			rtk_pos[i] = spp->x[i];
		}
	}
	else if (spp->solType == 0)
	{
		memcpy(spp->x, obs->pos, sizeof(double) * 6);
	}

	if ((num_of_obs = init_spp_iter(epoch, spp, dop)) == 0)
	{
		//memset(obs->pos, 0, sizeof(double)*6);
		return 0;
	}
		
	if (rcv->fixType >= 2 && tt < 10.0)
	{
		memcpy(spp->x, rtk_pos, sizeof(double) * 6);
	}
	memcpy(obs->pos, spp->x, sizeof(double) * 6);
	memcpy(rcv->dop, dop, sizeof(double) * 5);
	return 1;
}
