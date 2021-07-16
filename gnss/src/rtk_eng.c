#include "pvt_eng.h"
#include "rtk_eng.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "rtcm.h"
#include "rtk_math.h"
#include "model.h"
#include "rtkcmn.h"
#include "ephemeris.h"
#include "gnss_filter.h"
#include "rtklib_core.h"

#define LN_063   (-0.46203546)
#define SNR_SCA      0.9
#define MAX_REFAGE   300.0
#define MIN_OBS      4
#define MIN_SD_OBS   6

#include "print_nmea.h"
#ifdef ARM_MCU
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

RTK_RAM_CODE extern int check_coordinate(const double* pos)
{
    if (s_norm(pos, 3) < 0.01 || pos[0] == 0.0 || pos[1] == 0.0 || pos[2] == 0.0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

RTK_RAM_CODE static int get_match_epoch(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref
	, vec_t *vec_rov, int *iref, int *irov)
{
    /* get the matched satellite index from the base and rover recivers, and sort by the elevation */
    int n = 0, i = 0, j = 0, k = 0;
    double elev[MAXOBS] = { 0.0 };
    obsd_t *pObsRov = NULL;
    obsd_t *pObsRef = NULL;
    vec_t  *pVecRov = NULL;
    vec_t  *pVecRef = NULL;
    for (i = 0; i < (int)obs_rov->n; ++i)
    {
        pObsRov = obs_rov->data + i;
        pVecRov = vec_rov + i;
        if (s_norm(pVecRov->rs, 3) < 0.1) continue;
        for (j = 0; j < (int)obs_ref->n; ++j)
        {
            pObsRef = obs_ref->data + j;
            pVecRef = vec_ref + j;
            if (s_norm(pVecRef->rs, 3) < 0.1) continue;
            if (pObsRov->sat != pObsRef->sat) continue;
            if (fabs(pObsRov->P[0] - pVecRov->r - (pObsRef->P[0] - pVecRef->r)) > 10000.0)
            {
                for (k = 0; k < 6; k++) pVecRov->rs[k] = 0.0;
                for (k = 0; k < NFREQ; k++)
                {
                    pObsRov->SNR[k]  = 0.0;
                    pObsRov->P[k]    = 0.0;
                    pObsRov->L[k]    = 0.0;
                    pObsRov->D[k]    = 0.0;
                    pVecRov->azel[1] = 0.0;
                }
                continue;
            }
            iref[n] = j;
            irov[n] = i;
            elev[n] = pVecRov->azel[1]; 
            ++n;
        }
    }
    for (i = 0; i < n; ++i)
    {
        for (j = i + 1; j < n; ++j)
        {
            if (elev[i] < elev[j])
            {
                int temp1 = iref[i];
                int temp2 = irov[i];
                double v = elev[i];
                iref[i] = iref[j];
                irov[i] = irov[j];
                elev[i] = elev[j];
                iref[j] = temp1;
                irov[j] = temp2;
                elev[j] = v;
            }
        }
    }
    return n;
}

RTK_RAM_CODE extern int code_filter_preprocessing(const obs_t *obs, int *ncode,double *ave_snr, double *ave_elev)
{
    int		 i,j;
    double	 sum = 0.0;
    *ncode = 0;
    if (obs== NULL) return 0;
    /* sort */
    for (i = 0; i < (int)obs->n; ++i)
    {
        for (j = 0; j < NFREQ; j++)
        {
            if (obs->data[i].P[j] != 0.0 && obs->data[i].SNR[j] > 100.0)
            {
                sum += obs->data[i].SNR[j] / 4.0;
                (*ncode)++;
            }
        }
    }
    if (*ncode > 0.0) *ave_snr = sum / (double)(*ncode);
    return 1;
}

RTK_DTCM_DATA static double normal_snr[19] = 
{
	34.75f, 36.84f, 38.12f, 38.84f, 40.12f, 41.87f,43.38f, 44.42f, 
	45.93f, 47.41f, 48.78f, 49.98f,50.34f, 50.64f, 50.65f, 50.36f, 
	50.46f, 50.29f,50.71f 
};

RTK_RAM_CODE static double get_snr_from_model(int sys, double el)
{
    double snr_mod = 0.0;
    int ele_index = floor(el / 5.0);
    double a = (el - ele_index * 5.0) / 5.0;
	if (sys == _SYS_BDS_)
	{
		snr_mod = (normal_snr[ele_index] + a * (normal_snr[ele_index + 1] - normal_snr[ele_index]))*SNR_SCA;
	}
	else
	{
		snr_mod = (normal_snr[ele_index] + a * (normal_snr[ele_index + 1] - normal_snr[ele_index]))*SNR_SCA;
	}

    return snr_mod;
}

RTK_RAM_CODE static void sd_varerr_ele_snrdiff(int sys, double el, double snr, double *codeCOV, double *phaseCOV)
{
    double a = 0.002;
    double b = 0.002;
    double fact = 100.0;
    double sys_scale = 1.0;
    double sinel = sin(el);
    double snr_mod = get_snr_from_model(sys, el*R2D);
    double snd_dif = snr - snr_mod;
    double snr_scale = (snr - snr_mod < -3.0) ? exp(snd_dif*LN_063) : 1.0;

    if (sys == _SYS_GLO_)
    {
        sys_scale = 2.0;
    }
    if (sys == _SYS_BDS_)
    {
        sys_scale = 1.5;
    }
    if (sys == _SYS_GAL_)
    {
        sys_scale = 1.0;
    }
    if (el < ((40.0 * PI) / 180.0))
    {
        (*phaseCOV) = 2.0 * ((a * a) + ((b * b) / (sinel * sinel))) * (sys_scale * sys_scale);
        (*codeCOV) = fact * fact * 2.0 * ((a * a)*snr_scale + ((b * b) / (sinel * sinel))) * (sys_scale * sys_scale);
    }
    else
    {
        (*phaseCOV) = (2.0) * ((a * a) + (b * b)) * (sys_scale * sys_scale);
        (*codeCOV) = (fact * fact) * (2.0) * ((a * a)*snr_scale + (b * b)) * (sys_scale * sys_scale);
    }
}

RTK_RAM_CODE extern int check_epoch_data_quality(obs_t* obs_rov, vec_t *vec, rcv_rtk_t* rtk)
{
    int i=0,f=0, nc=0, nobs=0, ret=0;
    double elev[60] = { 0.0 };
    double snr[60]  = { 0.0 };
    double sum1=0.0, sum2 = 0.0, sum3=0.0;
    double mean_snr = 0.0, mean_elev = 0.0;
    double coef = 0.0;
    for (i = 0; i < obs_rov->n; i++)
    {
        if (vec[i].azel[1] * R2D > 15.0)
        {
            for (f = 0; f < NFREQ; f++)
            {
                if (obs_rov->data[i].SNR[f] > 0)
                {
                    elev[nc] = vec[i].azel[1] * R2D;
                    snr[nc] = obs_rov->data[i].SNR[f] / 4;
                    mean_snr += snr[nc];
                    mean_elev+= elev[nc];
                    nc++; 
                }
                if (obs_rov->data[i].SNR[f] > 80.0)
                {
                    nobs++;
                }
            }
        }
    }

    if (nc > 0)
    {
        mean_snr /= nc;
        mean_elev /= nc;
        for (i = 0; i < nc; i++)
        {
            sum1 += (elev[i] - mean_elev)*(snr[i] - mean_snr);
            sum2 += SQR(elev[i] - mean_elev);
            sum3 += SQR(snr[i] - mean_snr);
        }
        coef = sum1 / sqrt(sum2*sum3);
        rtk->coreff = coef;

        if (coef >0.5 && rtk->num_unfixepoch > 10)
        {
            rtk->num_goodepoch++;
            ret = 1;
        }
        else if (coef < -0.2)
        {
            rtk->num_unfixepoch++;
        }
        else if (rtk->num_unfixepoch > 10)
        {
            rtk->num_goodepoch = 0;
        }
    }
    else
    {
        coef = 0.0;
        ret = 0;
    }

    if (rtk->num_goodepoch > 3)
    {
        rtk->num_unfixepoch = 0;
        rtk->epoch_quality_flag = 1;
        //printf("epoch_quality_flag,%10.2f,%3i\n", fmod(rtk->time, 86400.0) - 18.0, rtk->epoch_quality_flag);
    }
    else
    {
        rtk->epoch_quality_flag = 0;
    }
    
    return ret;
}

/* single-differenced measurement error variance -----------------------------*/
RTK_RAM_CODE static void sdvarerr(int sat, int sys, double el, double snr, double varfactor,
 double *codeCOV, double *phaseCOV)
{
    double a = 0.002, b = 0.002, fact = 100.0, sys_scale = 1.0, snr_scale=1.0;
    double sinel = sin(el);
	int prn;

	satsys(sat, &prn);

    if (sys == _SYS_GLO_) sys_scale = 2.0;
	if (sys == _SYS_BDS_)
	{
		sys_scale = 1.5;
	}
    if (sys == _SYS_GAL_) sys_scale = 1.0;

    if (el < 40.0 * PI / 180.0)
    {
        *phaseCOV = 2.0*(a*a + b * b *(1.0/ sinel / sinel) )*sys_scale*sys_scale;//+ snr_scale* snr_scale)/2.0
        *codeCOV = fact * fact*(*phaseCOV);
    }
    else
    {
        *phaseCOV = 2.0*(a*a + b * b )*sys_scale*sys_scale;   //*snr_scale*snr_scale
        *codeCOV = fact * fact*(*phaseCOV);
    }

    return;
}

/* geometry-free phase measurement -------------------------------------------*/
RTK_RAM_CODE extern double gfmeas(const obsd_t *obs)
{
    double lam[2];    
    int i = 1; 

	lam[0] = satwavelen(obs->sat, obs->code[0]);
	lam[1] = satwavelen(obs->sat, obs->code[1]);

	if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0)
	{
		return 0.0;
	}

    return lam[0] * obs->L[0] - lam[i] * obs->L[i];
}


RTK_RAM_CODE static int find_sat_index_from_obs(int sat, slipset_t *ssat, obsd_t *data, int n)
{
    int i = 0, j = 0;
    int satid = -1;
    int ns = 0;
    for (i = 0; i < MAXOBS; ++i)
    {	/* find empty ssar or find the right sat in ssat*/
        if (ssat[i].sat == 0 || ssat[i].sat == sat) 
        {
            satid = i;
            break;
        }
    }

    /*if not find one ssat which is not used*/
    if (satid == -1)
    {
        for (j = 0; j < MAXOBS; ++j)
        {
            ns = -1;
            for (i = 0; i < n; i++)
            {
                /*find sat in ssat[j]*/
                if (ssat[j].sat == data[i].sat)
                {
                    ns = j;
                    break;
                }
            }
            if (ns == -1)   break;
        }
        if (ns == -1)
        {
            satid = j;
            for (int f = 0; f < NFREQ; f++)
            {
                ssat[satid].slip[f] = 1;
                ssat[satid].nlock[f] = 0;
                ssat[satid].gf     = 0.0;
                ssat[satid].gf2    = 0.0;
                ssat[satid].D[f]   = 0.0;
                ssat[satid].L[f]   = 0.0;
                ssat[satid].dph[f] = 0.0;
            }
        }
    }

    return satid;
}
/* detect cycle slip by LLI --------------------------------------------------*/
RTK_RAM_CODE static void detslp_ll(obs_t *obs, slipset_t *ssat)
{
    int i, j, satid;
    obsd_t *pObs = NULL;
    for (i = 0; i < (int)obs->n &&i < MAXOBS; i++)
    {
        pObs = obs->data + i;
        satid = find_sat_index_from_obs(pObs->sat,ssat, obs->data, obs->n);
        if (satid == -1) continue;

        for (j = 0; j < NFREQ; j++)
        {
            if (pObs->L[j] == 0.0 || (pObs->LLI[j] &3) ) 
               ssat[satid].slip[j] = 1;
        }
    }
}

/* detect cycle slip by geometry free phase jump -----------------------------*/
RTK_RAM_CODE void detslp_ref(const obs_t *obs, slipset_t *ssat)
{
    double g0, g1;
    int i, j, sat_id;
    const obsd_t *pObs = NULL;
    int week;
    double time = time2gpst(obs->time, &week);
    time = time - floor(time / 86400.0) * 86400.0;

    for (i = 0; i < (int)obs->n && i < MAXOBS; i++)
    {
        pObs = obs->data + i;

        if ((g1 = gfmeas(pObs)) == 0.0) continue;
        sat_id = find_sat_index(pObs->sat, ssat);
        if (sat_id == -1) continue;
        ssat[sat_id].sat = pObs->sat;
        g0 = ssat[sat_id].gf;
        ssat[sat_id].gf = g1;
        if (g0 != 0.0 && fabs(g1 - g0) > 0.03)
        {
            for (j = 0; j < NFREQ; j++) ssat[sat_id].slip[j] = 1;
        } 
    }
}

/* estimate rcv bias -----------------------------*/
RTK_RAM_CODE void rcvbias_estimate(double *delph, const int nc, double *rcvbias, double *rcvstd, 
	double max_diff)
{
    int i, ns = 0;
    int nmid = ROUND(nc / 2.0);
    int nbound = nc - ROUND(nc / 2.0);
    *rcvstd = 0.0;
    *rcvbias = delph[nmid];
    double v = 0.0;
    for (i = 1; i <= nbound; i++)
    {
        if (nmid + i < nc)
        {
            v = fabs(delph[nmid] - delph[nmid + i]);
            if (v < max_diff)
            {
                *rcvbias += delph[nmid + i];
                *rcvstd += v * v;
                ns++;
            }
        }
        if (nmid - i >= 0)
        {
            v = fabs(delph[nmid] - delph[nmid - i]);
            if (v < max_diff)
            {
                *rcvbias += delph[nmid - i];
                *rcvstd += v * v;
                ns++;
            }
        }
    }

    if (ns == 0)
        *rcvstd = 99.0;
    else
    {
        *rcvbias = delph[nmid];
        *rcvstd = sqrt(*rcvstd / (ns + 1));
    }
}

RTK_RAM_CODE double detslp_doppler(const obsd_t *pRovObs, slipset_t *ssat,const int f, const double tt,
	obs_t *obs_rov)
{
    double d1, d0, l1, l0, dave, delph = 0.0;;
    int sat_id;
    sat_id = find_sat_index_from_obs(pRovObs->sat, ssat, obs_rov->data, obs_rov->n);
    l1 = pRovObs->L[f];
    d1 = pRovObs->D[f];

    ssat[sat_id].sat = pRovObs->sat;
    d0 = ssat[sat_id].D[f];
    l0 = ssat[sat_id].L[f];
    ssat[sat_id].L[f] = l1;
    ssat[sat_id].D[f] = d1;

    if (l0 == 0.0 || l1 == 0.0)
    {
        ssat[sat_id].dph[f] = 0.0;
        return ssat[sat_id].dph[f];
    }

    if (d0==0.0)
        dave = d1;
    else if (d1==0.0)
        dave = d0;
    else
        dave = (d0 + d1) / 2.0;

    ssat[sat_id].dph[f] = l1 - l0 + dave * tt;

    return ssat[sat_id].dph[f];
}

/* detect cycle slip by phase + doppler  -----------------------------*/
RTK_RAM_CODE void detslp_rov(obs_t *obs_rov, slipset_t *ssat, const double tt)
{
    int f, i, s, sat, prn, sys, idx, sat_id, ncount = 0, nslp = 0;
    obsd_t *pObsRov = NULL;
    double delph[NFREQ*MAXOBS] = { 0.0 },rcvbias[NFREQ*NSYS] = { 0.0 };
    double dph = 0.0, rcvstd[NFREQ*NSYS] = { 0.0 };
    double cs_thres = (tt<1.0)? 0.01 : 0.03;  /* temporal used*/
    int week;
    double time = time2gpst(obs_rov->time, &week);
    time = time - floor(time / 86400.0) * 86400.0;

    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            ncount = 0;
            for (i = 0; i < (int)obs_rov->n; ++i)
            {
                if (obs_rov->data[i].code[f] == 0) continue;
                pObsRov = obs_rov->data + i;
                sat = pObsRov->sat;
                sys = satsys(sat, NULL);
                if (satidx(sat, &prn) != s) continue;
                dph = detslp_doppler(pObsRov, ssat, f, tt, obs_rov);
                idx = -1;
                if (dph != 0.0)
                {
                    if (ncount > 0)
                    {
                        for (int j = 0; j < ncount; j++)
                        {
                            if (dph < delph[j])
                            {
                                idx = j;
                                for (int k = ncount - 1; k >= j; k--)
                                {
                                    delph[k + 1] = delph[k];
                                }
                                delph[j] = dph;
                                break;
                            }
                        }
                        if (idx == -1)
                        {
                            delph[ncount] = dph;
                        }
                    }
                    else
                    {
                        delph[ncount] = dph;
                    }
                    ncount++;
                }
            }

            if (ncount != 0)
            {
                rcvbias_estimate(delph, ncount, &rcvbias[s*NFREQ + f], &rcvstd[s*NFREQ + f], 10.0);
            }
        }
    }

    for (i = 0; i < (int)obs_rov->n; i++)
    {
        pObsRov = obs_rov->data + i;
        sat = pObsRov->sat;
        sat_id = find_sat_index(sat, ssat);
        if (ssat[sat_id].sat == 0 || sat_id == -1)  continue;
        for (f = 0; f < NFREQ; f++)
        {
            if (ssat[sat_id].dph[f] == 0.0)
            {
                ssat[sat_id].slip[f]  = 1;
                ssat[sat_id].nlock[f] = 0;
                continue;
            }
            ssat[sat_id].dph[f] -= rcvbias[satidx(sat, &prn)*NFREQ + f];
            if (fabs(ssat[sat_id].dph[f]) > DPCS_THRES1 || pObsRov->LLI[f] ==2)
            {
                int is_cs = 1;
                double g0, g1;
                if ((g1 = gfmeas(pObsRov)) != 0.0 ) 
                {
                    sat_id = find_sat_index(pObsRov->sat, ssat);
                    if (sat_id != -1 && sat_id<MAXOBS)
                    {
                        g0 = ssat[sat_id].gf2;
                        ssat[sat_id].gf2 = g1;
                        if (g0 != 0.0 && fabs(g1 - g0) < cs_thres && 
							fabs(g1-g0)>1.0e-4 && fabs(ssat[sat_id].dph[f]) < DPCS_THRES2) 
                        {
                            ssat[sat_id].slip[f]  = 0;
                            ssat[sat_id].nlock[f]++;
							if(ssat[sat_id].nlock[f] > 128)
							{
								ssat[sat_id].nlock[f] = 128;
							}
                            is_cs = 0;
                        }
                    }
                }  

                if (is_cs)
                {
                    ssat[sat_id].slip[f] = 1;
                    ssat[sat_id].nlock[f] = 0;
                }
            }
            else
            {
                ssat[sat_id].slip[f] = 0;
                ssat[sat_id].nlock[f]++;
				if (ssat[sat_id].nlock[f] > 128)
				{
					ssat[sat_id].nlock[f] = 128;
				}
            }
        }
    }

    return;
}

RTK_RAM_CODE static void get_clock_bias_rate(obs_t *rov, vec_t *vec_rov, obs_t *ref, vec_t *vec_ref, 
	double *x, double tt, double *cdt_bias, double *cdt_drift, int *n_bias, int *n_drift)
{
    int i, j, f, s;
    double cur_res = 0.0, wave = 0.0;
    /* get the clock bias*/
    for (i = 0; i < (int)rov->n; ++i)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            if (!(fabs(rov->data[i].D[f]) < 0.01 ||
                fabs(vec_rov[i].rate) < 0.01))
            {
                wave = satwavelen(rov->data[i].sat, rov->data[i].code[f]);
                cdt_drift[*n_drift] = -wave * rov->data[i].D[f] -
                    (vec_rov[i].rate + x[9] - CLIGHT * vec_rov[i].dts[1]);
                ++(*n_drift);
            }
        }
        for (j = 0; j < (int)ref->n; ++j)
        {
            if (rov->data[i].sat == ref->data[j].sat)
            {
                break;
            }
        }

        if (j == ref->n)
            continue;

        s = satidx(rov->data[i].sat, NULL);

        for (f = 0; f < NFREQ; ++f)
        {
            if (!(fabs(rov->data[i].P[f]) < 0.01 || fabs(ref->data[j].P[f]) < 0.01 ||
                fabs(vec_rov[i].r) < 0.01 || fabs(vec_ref[j].r) < 0.01))
            {
                cur_res = 0.0;
                cur_res += rov->data[i].P[f] - (vec_rov[i].r + vec_rov[i].tro + x[10]);  
                cur_res -= ref->data[j].P[f] - (vec_ref[j].r + vec_ref[j].tro);
                cdt_bias[*n_bias] = cur_res;
                ++(*n_bias);
            }
        }
    }
}

RTK_RAM_CODE extern int rtk_reinitialization(double last_cbr, rcv_rtk_t *rtk, rcv_tdp_t *tdp)
{
    int ret = 0;
    double cur_cbr = rtk->code_blunder_rate;
    double med_res = fabs(rtk->scales.medres[0]);
    double sig_res = fabs(rtk->scales.sigma0[0]);
    double ave_snr = rtk->scales.ave_snr[0];
    double ave_ele = rtk->scales.ave_elev;
    double cbr_dif = fabs(rtk->code_blunder_rate - last_cbr);
    double vel = sqrt(SQR(rtk->x[3])+ SQR(rtk->x[4])+ SQR(rtk->x[5]));
    if (rtk->fixType == 4)
    {
        return ret;
    }

    if (cur_cbr < 0.28 && ave_ele > 40.0 && med_res > 3.0 && ave_snr > 35.0 && rtk->coreff > 0.2 ) 
    {
       ret = 1; 
    } 
    if (cbr_dif > 0.40 && ave_ele > 40.0 && med_res > 5.0 && sig_res >  2.0 && rtk->coreff > 0.2 )  
    {
        ret = 1;
    }
    if (med_res  > 7.0 && sig_res > 7.0  && ave_snr >38.0 && ave_ele > 40.0 && rtk->coreff > 0.2 ) 
    {
         ret = 1;
    }
    if (rtk->num_floatpoch > 10 && tdp->code_num > 15 && tdp->phas_num > 12 && 
        tdp->phas_omg < 0.01 && tdp->code_omg < 0.4 && rtk->coreff > 0.20)
    {
         ret = 1;
    }

    if (rtk->epoch_quality_flag == 1)
    {
        rtk->num_unfixepoch = 0;
        rtk->num_goodepoch = 0;
        ret = 1;
    }
    
    return ret;
}

RTK_RAM_CODE static void filter_RTD_update(obs_t* rov, vec_t* vec_rov, obs_t* ref, vec_t* vec_ref, 
	double* x, double* P, double* time, int max_st)
{
    int week = 0, i, j, k;
    double cur_time = time2gpst(rov->time, &week);
    double cdt_bias[MAXOBS*NFREQ] = { 0 }, cdt_drift[MAXOBS*NFREQ] = { 0 };
    int n_bias = 0;
    int n_drift = 0;
    double cur_res = 0.0, wave = 0.0;
    double tt, tt2, tt3, tt4, tt5;
    double prnNoise[3] = { 0.5, 0.5, 0.5 };
    double qk;

    cur_time += week * 7 * 24 * 3600.0;

    tt = *time > 0.0 ? (cur_time - *time) : (0.0);
    tt2 = tt * tt;
    tt3 = tt * tt2;
    tt4 = tt * tt3;
    tt5 = tt * tt4;

    if (*time == 0.0 || s_norm(x,3) < 1.0 || fabs(cur_time - *time) > 120.0) /* init or large gap */
    {
        memset(x, 0, sizeof(double) * NX_RTK);
        memset(P, 0, sizeof(double) * SMD(NX_RTK));
        /* xyz */
        for (j = 0; j < 3; ++j)
        {
            P[SMI(j, j)] = 1000.0 * 1000.0;
        }
        /* vel */
        for (j = 0; j < 3; ++j)
        {
            P[SMI(j + 3, j + 3)] = 100.0 * 100.0;
        }
        /* acc */
        for (j = 0; j < 3; ++j)
        {
            P[SMI(j + 6, j + 6)] = 10.0 * 10.0;
        }
        /* get the clock bias*/
        get_clock_bias_rate(rov, vec_rov, ref, vec_ref, x, tt, cdt_bias, cdt_drift, &n_bias, &n_drift);
        x[9] = median_dat(cdt_drift, n_drift);
        x[10] = median_dat(cdt_bias, n_bias);
        for (j = 9; j < NX_RTK; ++j)
        {
            P[SMI(j, j)] = 10000.0 * 10000.0;
        }
    }
    else if (*time != cur_time)
    {

        double F[9][9] = { 0 };
        double x_[9] = { 0 };
        double xp[9] = { 0 };
        double P_[9][9] = { 0 };
        double PP[9][9] = { 0 };
        double FP[9][9] = { 0 };

        for (i = 0; i < 9; ++i) F[i][i] = 1.0;
        for (i = 0; i < 6; i++)
        {
            F[i][i + 3] = tt;
        }

        for (i = 0; i < 3; i++)
        {
            F[i][i + 6] = tt * tt / 2.0;
        }
        for (i = 0; i < 9; i++)
        {
            x_[i] = x[i];
            for (j = 0; j < 9; j++)
            {
                P_[i][j] = P[SMI(i, j)];
            }
        }
        /* x=F*x, P=F*P*F+Q */
        for (i = 0; i < 9; ++i)
        {
            xp[i] = 0.0;
            for (j = 0; j < 9; ++j)
            {
                xp[i] += F[i][j] * x_[j];
            }
        }
        /* FP */

        for (i = 0; i < 9; ++i)
        {
            P_[i][i] += 1.0e-6;
        }

        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                FP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {   
                    FP[i][j] += F[i][k] * P_[k][j];
                }
            }
        }
        /* F*P*F' */
        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                PP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {
                    PP[i][j] += FP[i][k] * F[j][k];
                }
            }
        }
        for (i = 0; i < 9; ++i)
        {
            x[i] = xp[i];
            for (j = i; j < 9; ++j)
            {
                P[SMI(i, j)] = PP[i][j];
            }
        }

       
        for (i = 0; i < 3; i++)
        {
            qk = prnNoise[i] * prnNoise[i];
            PP[i + 0][i + 0] += qk * tt5 / 20.0;
            PP[i + 0][i + 3] += qk * tt4 / 8.0;
            PP[i + 0][i + 6] += qk * tt3 / 6.0;
            PP[i + 3][i + 0] += qk * tt4 / 8.0;
            PP[i + 3][i + 3] += qk * tt3 / 3.0;
            PP[i + 3][i + 6] += qk * tt2 / 2.0;
            PP[i + 6][i + 0] += qk * tt3 / 6.0;
            PP[i + 6][i + 3] += qk * tt2 / 2.0;
            PP[i + 6][i + 6] += qk * tt;
        }

        for (i = 0; i < 9; ++i)
        {
            x[i] = xp[i];
            for (j = i; j < 9; ++j)
            {
                P[SMI(i, j)] = PP[i][j];
            }
        }

        /* store the initial position and velocity, only estimate the difference w.r.t initial position and velocity */
        for (i = 0; i < 6; ++i)
        {
            rov->pos[i] = x[i];
            x[i] = 0.0;         
        }
        compute_vector_data(rov, vec_rov);
        /* get the clock bias*/
        get_clock_bias_rate(rov, vec_rov, ref, vec_ref, x, tt, cdt_bias, cdt_drift, &n_bias, &n_drift);
        x[9] += median_dat(cdt_drift, n_drift);       P[SMI(9, 9)]   += 1000.0*1000.0;
        x[10] += median_dat(cdt_bias, n_bias);        P[SMI(10, 10)] += 10000.0 * 10000.0;
    }
    *time = cur_time;
    return;
}

RTK_RAM_CODE static int compute_sdrcvbias(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, 
	int *iref, int *irov, int nsd, double maskElev, double *rcvbias, double *rcvstd)
{
    int s,f,isd,i,j,k,m,sat,prn,sys;
    double elev, z;
    double sdobs[MAXOBS] = { 0.0 };
    int isfind;
    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;
    int nc;
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            nc = 0;
            /* reset the clock state vector */
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                sys = satsys(sat, &prn);
                if (satidx(sat, &prn) != s) continue;

                elev = pVecRov->azel[1];

                if (elev < (maskElev*PI / 180.0)) continue;

                if (fabs(pObsRov->P[f]) < 0.001 || fabs(pObsRef->P[f]) < 0.001) continue;

                z= (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro));

                isfind = 0;
                if (nc == 0)
                    sdobs[nc] = z;
                else
                {
                    for (k = 0; k < nc; k++)
                    {
                        if (z < sdobs[k])
                        {
                            for (m = nc - 1; m >= k; m--)
                                sdobs[m + 1] = sdobs[m];

                            isfind = 1;
                            sdobs[k] = z;
                            break;
                        }
                    }
                    if (isfind==0)
                    sdobs[nc] = z;
                }
                nc++;
            }

            if (nc > 0)
            {
                rcvbias_estimate(sdobs, nc, &rcvbias[s*NFREQ + f], &rcvstd[s*NFREQ + f],10.0);

                for (k = 0; k < nc; k++) sdobs[k] = 0.0;
            }
        }
    }

   return 1;
}

RTK_RAM_CODE static int compute_rcvbias_rate(obs_t *obs_rov, vec_t *vec_rov,double maskElev, double *rcvbrate,
	double *rcvbstd)
{
    int s, f, j, k, m, sat, prn, sys;
    double elev, z;
    double rcvbias_rate[MAXOBS*NFREQ] = { 0.0 };
    int isfind;
    obsd_t *pObsRov = NULL;
    vec_t  *pVecRov = NULL;
    int  nc;
    /* doppler measuremnt update, then the cdt state become as the cdt bias */
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            nc = 0;
            for (j = 0; j < (int)obs_rov->n; ++j)
            {
                pObsRov = obs_rov->data + j;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                sys = satsys(sat, &prn);
                if (satidx(sat, &prn) != s) continue;
                elev = pVecRov->azel[1];
                if (elev < (maskElev*PI / 180.0)) continue;
                /* measurement */
                double w = satwavelen(sat, pObsRov->code[f]);
                if (fabs(pObsRov->D[f]) < 0.001 || fabs(pVecRov->rate) < 0.001) continue;
                z = -w * pObsRov->D[f] - (pVecRov->rate - CLIGHT * pVecRov->dts[1]);
                isfind = 0;
                if (nc == 0)
                    rcvbias_rate[nc] = z;
                else
                {
                    for (k = 0; k < nc; k++)
                    {
                        if (z < rcvbias_rate[k])
                        {
                            for (m = nc - 1; m >= k; m--)
                                rcvbias_rate[m + 1] = rcvbias_rate[m];

                            isfind = 1;
                            rcvbias_rate[k] = z;
                            break;
                        }
                    }
                    if (isfind == 0)
                        rcvbias_rate[nc] = z;
                }
                nc++;
            }
            if (nc > 0)
            {
                rcvbias_estimate(rcvbias_rate, nc, &rcvbrate[s*NFREQ + f], &rcvbstd[s*NFREQ + f],1.0);
                for (k = 0; k < nc; k++) rcvbias_rate[k] = 0.0;
            }
        }
    }
    return 1;
}

RTK_RAM_CODE int compute_measure_scale_factor(double cur_time, int *code_nm, int *dop_nm, measure_t *code_meas, measure_t * dopp_meas, double *x, meas_scale_t *scale)
{
    int i, s, f, prn, nP = 0, nD = 0;
    double inov, sigmaP0 = 0.0, sigmaD0 = 0.0, ave_elev = 0.0, max_elev = -99.0;
    for (i = 0; i < (*code_nm); i++)
    {
        char syschar = satid(code_meas[i].sat, &prn);
        inov = code_meas[i].z - code_meas[i].z_ - scale->medres[0];
        //printf("%10.1f,%c%02d,P%i,%6.2f,%10.3f,%10.3f,%4c\n", fmod(cur_time, 86400.0), syschar, prn, code2frq(code_meas[i].sys, code_meas[i].code) + 1, code_meas[i].elev*R2D, code_meas[i].z, inov, code_meas[i].flag == 0 ? 'P' : 'F');
        if (code_meas[i].flag == 1) continue;
        sigmaP0 += SQR(inov);
        ave_elev += code_meas[i].elev*R2D;
        if (code_meas[i].elev*R2D > max_elev) max_elev = code_meas[i].elev*R2D;
        nP++;
    }
    sigmaP0 = sqrt(sigmaP0 / nP);
    scale->ave_elev = ave_elev / nP;
    scale->max_elev = max_elev;

    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            for (i = 0; i < (*dop_nm); i++)
            {
                int sat = dopp_meas[i].sat;
                int sys = satidx(sat, &prn);
                if (sys != s) continue;
                if (code2frq(dopp_meas[i].sys, dopp_meas[i].code) != f) continue;
                char syschar = satid(dopp_meas[i].sat, &prn);
                inov = dopp_meas[i].z - dopp_meas[i].z_ - scale->medres[1];
                if (dopp_meas[i].flag == 1) continue;
                sigmaD0 += SQR(inov);
                nD++;
                /*   printf("%10.1f,%c%02d,D%i,%6.2f,%10.3f,%10.3f,%4c\n", cur_time, syschar, prn, code2frq(meas[i].sys, meas[i].code) + 1, meas[i].elev*R2D, meas[i].z, inov, meas[i].flag == 0 ? 'P' : 'F');*/
            }
        }
    }
    sigmaD0 = sqrt(sigmaD0 / nD);

    if (sigmaP0 > 0.0 && sigmaP0 < 50.0)
    {
        scale->nep[0]++;
        if (scale->nep[0] == 1)
            scale->sigma0[0] = sigmaP0;
        else
            scale->sigma0[0] = 0.1*scale->sigma0[0] + 0.9*sigmaP0;

        scale->scale[0] = (scale->sigma0[0] / 0.01) / 100.0;
        scale->scale[0] = 1.0;
    }

    if (sigmaD0 > 0.0 && sigmaD0 < 1.0)
    {
        scale->nep[1]++;
        if (scale->nep[1] == 1)
            scale->sigma0[1] = sigmaD0;
        else
            scale->sigma0[1] = 0.5*scale->sigma0[1] + 0.5*sigmaD0;

        scale->scale[1] = (scale->sigma0[1] / 0.01);
        if (scale->scale[1] > 15.0)     scale->scale[1] = 15.0;
        else if (scale->scale[1] < 5.0) scale->scale[1] = 5.0;
    }
  /*  double av1 = (double)nP / (*code_nm);
    double av2 = (double)nD / (*dop_nm);
    //printf("SIGMA0:%10.1f,%10.3f,%10.3f,%10.3f,%6.2f,%6.2f,%10.3f,%10.3f,%10.3f,%10.3f,%3d,%3d,%10.3f\n",
    //    fmod(cur_time, 86400.0)-18.0, scale->medres[0], scale->sigma0[0], scale->ave_snr[0], scale->max_elev, scale->ave_elev, av1, scale->medres[1], sigmaD0, scale->sigma0[1], nD, *code_nm, av2);*/
    return nP;
}

RTK_RAM_CODE int is_doppfilter_update(double *dNED, double head_change, double *dop)
{
    int ret = 0;
    if (fabs(dNED[2]) < 5.5 && fabs(head_change) < 45.0 && dop[2] < MAX_HDOP && dop[3] < MAX_VDOP)
        ret = 1;
    return ret;
}

RTK_RAM_CODE int is_codefilter_update(double pos_det, double pos_std, double *dop)
{
    int ret = 0;
    if (pos_det < 10.0 && dop[2] < MAX_HDOP && dop[3] < MAX_VDOP) ret = 1;
    if (pos_std > 30.0 && dop[2] < MAX_HDOP && dop[3] < MAX_VDOP) ret = 1;
    return ret;
}

static int robust_filter_RTD(obs_t* obs_ref, obs_t* obs_rov, vec_t* vec_ref,
    vec_t* vec_rov, int* iref, int* irov, int nsd, double* x, double* P,
    int np, int ns, int* num_of_obs, int* num_of_out, double* dop,
    meas_scale_t* scale, double maskElev, double coeff)
{
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0, week = 0, prn = 0;
    double z = 0.0, R = 0.0, z_ = 0.0, inov = 0.0, elev = 0.0;
    double codeCOV = 0.0, phaseCOV = 0.0, varfactor = 0.0;
    double PHt[NX_RTK] = { 0.0 };
    double x_[NX_RTK] = { 0.0 };
    double P_[SMD(NX_RTK)] = { 0.0 };
    double* H = NULL;
    int* L = NULL;
    int max_st = NX_RTK;
    double cur_time = time2gpst(obs_rov->time, &week);
    obsd_t* pObsRef = NULL;
    obsd_t* pObsRov = NULL;
    vec_t* pVecRef = NULL;
    vec_t* pVecRov = NULL;
    measure_t code_meas[MAXOBS * NFREQ] = { 0 };
    measure_t dop_meas[MAXOBS * NFREQ] = { 0 };
    int code_nm = 0;
    int dop_nm = 0;
    double HH[4 * MAXOBS] = { 0.0 };
    int n = 0;
    int numP_sys[NSYS*NFREQ] = { 0 };
    int numD_sys[NSYS*NFREQ] = { 0 };
    double pos_std = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);
    double dNED[3] = { 0 }, dXYZ[3] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    ecef2pos(obs_rov->pos, blh);
    blh2C_en(blh, C_en);
    for (i = 0; i < 3; i++) dXYZ[i] = obs_rov->pos[i + 3];
    xyz2ned(C_en, dXYZ, NULL, dNED, NULL);
    double pre_heading = atan2(dNED[0], dNED[1])*R2D;
	double mean_snr = 0.0;
	int    nm_snr = 0;
    /* the bias vector can be used as the clock for all the system, need to reset before use it */
    *num_of_obs = 0;
    *num_of_out = 0;
    for (isd = 0; isd < nsd; ++isd)
    {
        j = irov[isd];
        i = iref[isd];
        pObsRef = obs_ref->data + i;
        pObsRov = obs_rov->data + j;
        pVecRef = vec_ref + i;
        pVecRov = vec_rov + j;
        sat = pObsRov->sat;
        sys = satsys(sat, &prn);
        s = satidx(sat, &prn);
        elev = pVecRov->azel[1];
        if (elev < (maskElev * PI / 180.0)) continue;
        for (f = 0; f < NFREQ; ++f)
        {
            if (fabs(pObsRov->P[f]) > 0.001 && fabs(pObsRef->P[f]) > 0.001)
                numP_sys[MI(s, f, NFREQ)]++;

            if (fabs(pObsRov->D[f]) > 0.001 > 0.001)
                numD_sys[MI(s, f, NFREQ)]++;
        }
    }

    //while (1)
    {
        code_nm = 0;
        for (isd = 0; isd < nsd; ++isd)
        {
            j = irov[isd];
            i = iref[isd];
            pObsRef = obs_ref->data + i;
            pObsRov = obs_rov->data + j;
            pVecRef = vec_ref + i;
            pVecRov = vec_rov + j;
            sat = pObsRov->sat;
            sys = satsys(sat, &prn);
            s = satidx(sat, &prn);
            elev = pVecRov->azel[1];
            if (elev < (maskElev * PI / 180.0)) continue;
			for (f = 0; f < NFREQ; ++f)
			{
				if (pObsRov->code[f] == 0)                                        continue;
				if (fabs(pObsRov->P[f]) < 0.001 || fabs(pObsRef->P[f]) < 0.001) continue;
				if (numP_sys[MI(s, f, NFREQ)] < 2)   continue;
				/* measurement */
				if (coeff >= 0.2)
					sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, varfactor, &codeCOV, &phaseCOV);
				else
					sd_varerr_ele_snrdiff(satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, &codeCOV, &phaseCOV);

				z = (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro));
				R = codeCOV;
				memset(code_meas + code_nm, 0, sizeof(measure_t));
				H = code_meas[code_nm].H + 0;
				L = code_meas[code_nm].L + 0;
				numl = 0;
				H[numl] = -pVecRov->e[0]; L[numl] = 0; ++numl;
				H[numl] = -pVecRov->e[1]; L[numl] = 1; ++numl;
				H[numl] = -pVecRov->e[2]; L[numl] = 2; ++numl;
				H[numl] = 1.0;            L[numl] = 10; ++numl; /* clock bias */
				int curLOC = MI(s, f, NFREQ);
				if (curLOC > 0)
				{
					H[numl] = 1.0;        L[numl] = 10 + curLOC; ++numl;
				}
				code_meas[code_nm].numl = numl;
				code_meas[code_nm].z = z;
				code_meas[code_nm].R = R;
				code_meas[code_nm].sat = sat;
				code_meas[code_nm].prn = prn;
				code_meas[code_nm].code = pObsRov->code[f];
				code_meas[code_nm].sys = sys;
				code_meas[code_nm].elev = elev;
				code_meas[code_nm].azim = pVecRov->azel[0];
				code_meas[code_nm].SNR = pObsRov->SNR[f];
				code_meas[code_nm].isd = isd;
				++code_nm;
				if (pObsRov->SNR[0] > 80.0 && f == 0)
				{
					mean_snr += pObsRov->SNR[f] * 0.25;
					++nm_snr;
				}
			}
        }
        *num_of_obs = code_nm;
		mean_snr /= nm_snr;
        scale->ave_snr[0] = mean_snr;
		// printf("\n");

        //while 
        if (measure_update_kalman_filter(cur_time, code_meas, code_nm, x, P, x_, P_, PHt, max_st, 'P') == 0)
        {
            n = comp_dop(code_meas, code_nm, dop);
            if (dop[2]!=0.0 && dop[2] <= 0.5 && mean_snr>35.0)
            {
                /* restore the full position and velocity */
                code_nm = 0;
                for (i = 0; i < 3; ++i)
                {
                    x[i] += obs_rov->pos[i];
                }
                return 0;
            }
        }

        if (code_nm > 0)
        {
            for (i = 0; i < code_nm; i++)
            {
                char syschar = satid(code_meas[i].sat, &prn);
                code_meas[i].z_ = 0.0;
                for (k = 0; k < code_meas[i].numl; ++k)
                {
                    code_meas[i].z_ += code_meas[i].H[k] * x[code_meas[i].L[k]];
                }
                double inov = code_meas[i].z - code_meas[i].z_;
                /*    printf("%10.1f,%c%02d,P%i,%6.2f,%10.3f,%10.3f\n", fmod(cur_time, 86400.0), syschar, prn, code2frq(code_meas[i].sys, code_meas[i].code) + 1, code_meas[i].elev*R2D, code_meas[i].z, inov);*/
            }
            double medres = median_measure(code_meas, code_nm);
            scale->medres[0] = medres;
            for (i = 0; i < code_nm - 1; ++i)
            {
                for (j = i + 1; j < code_nm; ++j)
                {
                    if (fabs(code_meas[i].z - code_meas[i].z_ - medres) > fabs(code_meas[j].z - code_meas[j].z_ - medres))
                    {
                        measure_t tmp_meas = code_meas[i];
                        code_meas[i] = code_meas[j];
                        code_meas[j] = tmp_meas;
                    }
                }
            }
			if (fabs(medres) > 10.0 && mean_snr > 35.0)
			{
				for (i = 0; i < 3; ++i)
				{
					P[SMI(i, i)] += medres * medres;
				}
			}
        }

        n = comp_dop(code_meas, code_nm, dop);
        double dNED[3] = { 0 };
        double C_en[3][3] = { 0 };
        double blh[3] = { 0 };
        ecef2pos(obs_rov->pos, blh);
        blh2C_en(blh, C_en);
        xyz2ned(C_en, x, NULL, dNED, NULL);
        double pos_det = s_norm(x, 3);
        double vel_det = sqrt(SQR(obs_rov->pos[3]) + SQR(obs_rov->pos[4]) + SQR(obs_rov->pos[5]));
        pos_std = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);
        if (is_codefilter_update(pos_det, pos_std, dop) && mean_snr>35.0)
        {
            //if (pos_det < 2.0*vel_det)
            {
                for (i = 0; i < 3; ++i)
                {
                    obs_rov->pos[i] += x[i];
                    x[i] = 0.0;
                }
            }
        }
        compute_vector_data(obs_rov, vec_rov);
    }

   // /* doppler measuremnt update */
    dop_nm = 0;
    for (j = 0; j < (int)obs_rov->n; ++j)
    {
        pObsRov = obs_rov->data + j;
        pVecRov = vec_rov + j;
        sat = pObsRov->sat;
        sys = satsys(sat, &prn);
        s = satidx(sat, &prn);
        elev = pVecRov->azel[1];
        if (elev < (maskElev * PI / 180.0)) continue;
        for (f = 0; f < NFREQ; ++f)
        {
            if (pObsRov->code[f] == 0)                                      continue;
            if (fabs(pObsRov->D[f]) < 0.001 || fabs(pVecRov->rate) < 0.001) continue;
            if (numD_sys[MI(s, f, NFREQ)] < 2)   continue;
            /* measurement */
            double w = satwavelen(sat, pObsRov->code[f]);
            if (coeff >= 0.2)
                sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, varfactor, &codeCOV, &phaseCOV);
            else
                sd_varerr_ele_snrdiff(satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, &codeCOV, &phaseCOV);
            /* doppler residual */
            z = -w * pObsRov->D[f] - (pVecRov->rate - CLIGHT * pVecRov->dts[1]);

            R = phaseCOV * 15.0 * 15.0;
            memset(dop_meas + dop_nm, 0, sizeof(measure_t));
            H = dop_meas[dop_nm].H + 0;
            L = dop_meas[dop_nm].L + 0;
            numl = 0;
            H[numl] = -pVecRov->e[0]; L[numl] = 3; ++numl;
            H[numl] = -pVecRov->e[1]; L[numl] = 4; ++numl;
            H[numl] = -pVecRov->e[2]; L[numl] = 5; ++numl;
            H[numl] = 1.0; L[numl] = 9; ++numl; /* clock drift */
            dop_meas[dop_nm].numl = numl;
            dop_meas[dop_nm].z = z;
            dop_meas[dop_nm].R = R;
            dop_meas[dop_nm].sat = sat;
            dop_meas[dop_nm].prn = prn;
            dop_meas[dop_nm].code = pObsRov->code[f];
            dop_meas[dop_nm].sys = sys;
            dop_meas[dop_nm].elev = elev;
			dop_meas[dop_nm].azim = pVecRov->azel[0];
            dop_meas[dop_nm].SNR = pObsRov->SNR[f];
            ++dop_nm;
        }
    }

    if (dop_nm > 0)
    {
        if (measure_update_kalman_filter(cur_time, dop_meas, dop_nm, x, P, x_, P_, PHt, max_st, 'D') == 0)
        {
            if (dop[2] != 0.0 && dop[2] <= 0.5 && mean_snr > 35.0)
            {
                /* current doppler measurements have problems */
                dop_nm = 0;
                for (i = 3; i < 6; ++i)
                {
                    x[i] += obs_rov->pos[i];
                }
                return 0;
            }
        }

        for (i = 0; i < dop_nm; i++)
        {
            dop_meas[i].z_ = 0.0;
            for (k = 0; k < dop_meas[i].numl; ++k)
            {
                dop_meas[i].z_ += dop_meas[i].H[k] * x[dop_meas[i].L[k]];
            }
        }
        double medres = median_measure(dop_meas, dop_nm);
        scale->medres[1] = medres;
        for (i = 0; i < dop_nm; ++i)
        {
            for (j = i + 1; j < dop_nm; ++j)
            {
                if (fabs(dop_meas[i].z - dop_meas[i].z_ - medres) > fabs(dop_meas[j].z - dop_meas[j].z_ - medres))
                {
                    measure_t tmp_meas = dop_meas[i];
                    dop_meas[i] = dop_meas[j];
                    dop_meas[j] = tmp_meas;
                }
            }
        }

        if (fabs(medres) > 0.3 && mean_snr > 35.0)
        {
            for (i = 3; i < 6; ++i)
            {
                P[SMI(i, i)] += medres * medres;
            }
        }

        n = comp_dop(dop_meas, dop_nm, dop);
        for (i = 0; i < 3; i++) dXYZ[i] = obs_rov->pos[i + 3] + x[i + 3];
        xyz2ned(C_en, dXYZ, NULL, dNED, NULL);
        double cur_heading = atan2(dNED[0], dNED[1]) * R2D;
        double hvel = sqrt(SQR(dNED[0]) + SQR(dNED[1]));
        double uvel = fabs(dNED[2]);
        for (i = 0; i < 3; i++) dXYZ[i] = x[i + 3];
        xyz2ned(C_en, dXYZ, NULL, dNED, NULL);

        /* restore the full position and velocity */
        if (is_doppfilter_update(dNED, cur_heading - pre_heading, dop))
        {
            for (i = 3; i < 6; ++i)
            {
                x[i] += obs_rov->pos[i];
                obs_rov->pos[i] = x[i];
                x[i] = 0.0;
            }
        }
        compute_vector_data(obs_rov, vec_rov);
    }

    int nP = compute_measure_scale_factor(cur_time, &code_nm, &dop_nm, code_meas, dop_meas, x, scale);
    *num_of_out = *num_of_obs - nP;
    
    return n;
}

RTK_RAM_CODE static int robust_filter_code(obs_t* obs_ref, obs_t* obs_rov, vec_t* vec_ref,
	vec_t* vec_rov, int* iref, int* irov, int nsd, double* x, double* P,
	int np, int ns, double maskElev)
{
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0, freqn = 0;
    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0, snr = 0.0;
    double codeCOV = 0.0, phaseCOV = 0.0, varfactor = 0.0;
    double PHt[NX_RTK] = { 0.0 };
    double x_[NX_RTK] = { 0.0 };
    double P_[SMD(NX_RTK)] = { 0.0 };
    double* H = NULL;
    int* L = NULL;
    int max_st = NX_RTK;
    int week = 0;
    int prn = 0;
    int nsat = 0;
    int satIDs[MAXOBS] = { 0 };
    double cur_time = time2gpst(obs_rov->time, &week);
    obsd_t* pObsRef = NULL;
    obsd_t* pObsRov = NULL;
    vec_t* pVecRef = NULL;
    vec_t* pVecRov = NULL;
    measure_t meas[MAXOBS * NFREQ] = { 0 };
    int nm = 0;
    double HH[4 * MAXOBS] = { 0.0 };
    int n = 0;

        nm = 0;
        for (isd = 0; isd < nsd; ++isd)
        {
            j = irov[isd];
            i = iref[isd];
            pObsRef = obs_ref->data + i;
            pObsRov = obs_rov->data + j;
            pVecRef = vec_ref + i;
            pVecRov = vec_rov + j;
            sat = pObsRov->sat;
            sys = satsys(sat, &prn);
            s = satidx(sat, &prn);
            elev = pVecRov->azel[1];
            if (elev < (maskElev * PI / 180.0)) continue;
            for (f = 0; f < NFREQ; ++f)
            {
                if (fabs(pObsRov->P[f]) < 0.001 || fabs(pObsRef->P[f]) < 0.001) continue;
                /* measurement */
                sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f], varfactor, &codeCOV, &phaseCOV);
                z = (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro));
                R = codeCOV;
                memset(meas + nm, 0, sizeof(measure_t));
                H = meas[nm].H + 0;
                L = meas[nm].L + 0;
                numl = 0;
                H[numl] = -pVecRov->e[0]; L[numl] = 0;  ++numl;
                H[numl] = -pVecRov->e[1]; L[numl] = 1;  ++numl;
                H[numl] = -pVecRov->e[2]; L[numl] = 2;  ++numl;
                H[numl] = 1.0;            L[numl] = 10; ++numl; /* clock bias */
                int curLOC = MI(s, f, NFREQ);
                if (curLOC > 0)
                {
                    H[numl] = 1.0;            L[numl] = 10 + curLOC; ++numl;
                }
                meas[nm].numl = numl;
                meas[nm].z = z;
                meas[nm].R = R;
                meas[nm].sat  = sat;
                meas[nm].prn  = prn;
                meas[nm].code = pObsRov->code[f];
                meas[nm].sys  =  sys;
                meas[nm].elev = elev;
                meas[nm].azim = pVecRov->azel[0];
                meas[nm].SNR  = pObsRov->SNR[f];
                ++nm;
            }
        }

        if (measure_update_kalman_filter(cur_time, meas, nm, x, P, x_, P_, PHt, max_st, 'P') == 0)
        {
            /* restore the full position and velocity */
            nm = 0;
            for (i = 0; i < 6; ++i)
            {
                x[i] += obs_rov->pos[i];
            }
            return 0;
        }
        double sigma0 = 0.0;
        int nobs = 0;
        for (s = 0; s < NSYS; s++)
        {
            for (f = 0; f < NFREQ; f++)
            {
                for (i = 0; i < nm; i++)
                {
                    int sat = meas[i].sat;
                    int sys = satidx(sat, &prn);
                    if (sys != s) continue;
                    if (code2frq(meas[i].sys, meas[i].code) != f) continue;
                    char syschar = satid(meas[i].sat, &prn);
                    z_ = 0.0;
                    for (j = 0; j < meas[i].numl; ++j)
                    {
                        z_ += meas[i].H[j] * x[meas[i].L[j]];
                    }
                    inov = meas[i].z - z_;
                    if (meas[i].flag == 1) continue;
                    sigma0 += SQR(inov);
                    nobs++;
                }
            }
        }
        sigma0 = sqrt(sigma0/ nobs);
        return 1;
}

RTK_RAM_CODE extern int rtk_pva_smoothing(double cur_time, rcv_pva_t *rtk_pva, rcv_rtk_t *rtk)
{
    int i;
    double xc[6] = { 0.0 };
    double Pc[SMD(6)] = { 0.0 };
    double pos_dsf, scaled_posvar[3] = { 0.0 }, sf=1.0;
    double C_en[3][3] = { 0 };
    double vxyz[3] = { 0 };
    memcpy(xc, rtk->x, sizeof(double)*6);
    memcpy(Pc, rtk->P, sizeof(double)*SMD(6));
    
    if (s_norm(xc, 3) < 0.1 || rtk->fixType <= 1)
        return 0;

    pos_dsf = fabs(rtk->scales.medres[0] / 0.3);
    pos_dsf = (pos_dsf < 1.0) ? 1.0 : pos_dsf;

    if (rtk->fixType == 5)
    {
        sf = 4.0;
    }
    else if (rtk->fixType < 4)
    {
        sf = 25.0;
    }

    scaled_posvar[0] = sf*(9.0 + SQR(pos_dsf))*Pc[SMI(0, 0)];
    scaled_posvar[1] = sf*(9.0 + SQR(pos_dsf))*Pc[SMI(1, 1)];
    scaled_posvar[2] = sf*(9.0 + SQR(pos_dsf))*Pc[SMI(2, 2)];
    
    if (rtk_pva->time == 0.0)
    {
        rtk_pva->time = cur_time;
        for (i = 0; i < 6; i++)
        {
            rtk_pva->x[i]        = xc[i];
            rtk_pva->P[SMI(i,i)] = scaled_posvar[i];
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            Pc[SMI(i, i)] = scaled_posvar[i];
        }
        rtk_pva_prediction(cur_time, rtk_pva);

        rtk_pva_filter(rtk_pva, xc, Pc);

        ecef2pos(rtk_pva->x, rtk_pva->blh);
        blh2C_en(rtk_pva->blh, C_en);
        vxyz[0] = rtk_pva->x[3];
        vxyz[1] = rtk_pva->x[4];
        vxyz[2] = rtk_pva->x[5];
        xyz2ned(C_en, vxyz, NULL, rtk_pva->vned, NULL);

    }
    return 1;
}

RTK_RAM_CODE extern int rtk_pva_prediction(double cur_time, rcv_pva_t *rtk_pva)
{
    int i,j,k;
    double F[9][9] = { 0 };
    double x_[9] = { 0 };
    double xp[9] = { 0 };
    double P_[9][9] = { 0 };
    double PP[9][9] = { 0 };
    double FP[9][9] = { 0 };
    double tt, tt2, tt3, tt4, tt5;
    double prnNoise[3] = { 0.5, 0.5, 0.5 };
    double qk;

    tt = rtk_pva->time > 0.0 ? (cur_time - rtk_pva->time) : (0.0);
    rtk_pva->time = cur_time;
        tt2 = tt * tt;
        tt3 = tt * tt2;
        tt4 = tt * tt3;
        tt5 = tt * tt4;
        for (i = 0; i < 9; ++i) F[i][i] = 1.0;
        for (i = 0; i < 6; i++)
        {
            F[i][i + 3] = tt;
        }

        for (i = 0; i < 3; i++)
        {
            F[i][i + 6] = tt * tt / 2.0;
        }
        for (i = 0; i < 9; i++)
        {
            x_[i] = rtk_pva->x[i];
            for (j = 0; j < 9; j++)
            {
                P_[i][j] = rtk_pva->P[SMI(i, j)];
            }
        }
        /* x=F*x, P=F*P*F+Q */
        for (i = 0; i < 9; ++i)
        {
            xp[i] = 0.0;
            for (j = 0; j < 9; ++j)
            {
                xp[i] += F[i][j] * x_[j];
            }
        }
        /* FP */

        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                FP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {
                    FP[i][j] += F[i][k] * P_[k][j];
                }
            }
        }
        /* F*P*F' */
        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                PP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {
                    PP[i][j] += FP[i][k] * F[j][k];
                }
            }
        }
        for (i = 0; i < 3; i++)
        {
            qk = prnNoise[i] * prnNoise[i];
            PP[i + 0][i + 0] += qk * tt5 / 20.0;
            PP[i + 0][i + 3] += qk * tt4 / 8.0;
            PP[i + 0][i + 6] += qk * tt3 / 6.0;
            PP[i + 3][i + 0] += qk * tt4 / 8.0;
            PP[i + 3][i + 3] += qk * tt3 / 3.0;
            PP[i + 3][i + 6] += qk * tt2 / 2.0;
            PP[i + 6][i + 0] += qk * tt3 / 6.0;
            PP[i + 6][i + 3] += qk * tt2 / 2.0;
            PP[i + 6][i + 6] += qk * tt;
        }
        for (i = 0; i < 9; ++i)
        {
            rtk_pva->x[i] = xp[i];
            for (j = i; j < 9; ++j)
            {
                rtk_pva->P[SMI(i, j)] = PP[i][j];
            }
        }

    return 1;
}


RTK_RAM_CODE extern int rtk_pva_filter(rcv_pva_t *rtk_pva, double *xc, double *Pc)
{
    int i;
    double omega = 0.0, z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0;
    double inov = 0.0, P_inov = 0.0, wsf = 0.0, refVar = 0.0;
    double H[6] = { 0 };
    int    L[6] = { 0 };
    double x[9] = { 0.0 }, P[SMD(9)] = { 0.0 };
    double PHt[NX_PVA] = { 0.0 };
    int max_st = NX_PVA;
    memcpy(x, rtk_pva->x, sizeof(double) * max_st);
    memcpy(P, rtk_pva->P, sizeof(double) * SMD(max_st));
    for (i = 0; i < 6; ++i)
    {
        H[0] = 1;
        L[0] = i;
        z = xc[i];
        R = Pc[SMI(i,i)];
        ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, 1, PHt);
        inov = z - z_;
        P_inov = R + R_;
        wsf = inov * inov / R;
        //if (wsf < 225.0)
        {
            ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, 1, PHt);
        }
    }

    memcpy(rtk_pva->x, x, sizeof(double) * max_st);
    memcpy(rtk_pva->P, P, sizeof(double) * SMD(max_st));

    return 1;
}


RTK_RAM_CODE extern int vel_processor(epoch_t *rov_epoch)
{
    obs_t* rov = &rov_epoch->obs;
    rcv_rtk_t rtk = { 0 };
    vec_t* vec_rov = rov_epoch->vec + 0;
    int nsd = 0, i = 0, j = 0;
    int num_of_sat = 0, fixID = 5, week = 0;
    double time = 0.0;
    if (rov->n < 4)
    {
        /* do not have enough satellites */
        return 0;
    }

    time = time2gpst(rov->time, &week);
    time += (double)week * SECONDS_IN_WEEK;
    /* xyz */
    for (j = 0; j < 3; ++j)
    {
        rtk.P[SMI(j, j)] = 10000.0 * 10000.0;
    }
    /* vel */
    for (j = 0; j < 3; ++j)
    {
        rtk.P[SMI(j + 3, j + 3)] = 100.0 * 100.0;
    }
    /* acc */
    for (j = 0; j < 3; ++j)
    {
        rtk.P[SMI(j + 6, j + 6)] = 10.0 * 10.0;
    }

    /* get the clock bias*/
    for (j = 9; j < 10; ++j)
    {
        rtk.P[SMI(j, j)] = 10000.0 * 10000.0;
    }

    robust_filter_doppler(rov, vec_rov, rtk.x, rtk.P, rtk.np, rtk.ns, 10.0);

    return 1;
}

RTK_RAM_CODE static int robust_filter_doppler(obs_t* obs_rov, vec_t* vec_rov, double* x, 
	double* P, int np, int ns, double maskElev)
{
	int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0;
	int sys = 0, numl = 0, freqn = 0;
	double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0;
	double P_inov = 0.0, wsf = 0.0, elev = 0.0, snr = 0.0;
    double codeCOV = 0.0, phaseCOV = 0.0, varfactor = 0.0;
    double PHt[NX_RTD] = { 0.0 };
    double x_[NX_RTD] = { 0.0 };
    double P_[SMD(NX_RTD)] = { 0.0 };
    double* H = NULL;
    int* L = NULL;
    int max_st = NX_RTD;
    int week = 0;
    int prn = 0;
    int nsat = 0;
    int satIDs[MAXOBS] = { 0 };
    double cur_time = time2gpst(obs_rov->time, &week);
    obsd_t* pObsRov = NULL;
    vec_t*  pVecRov = NULL;
    measure_t meas[MAXOBS * NFREQ] = { 0 };
    int nm = 0;
    double HH[4 * MAXOBS] = { 0.0 };
    int n = 0;
    /* note: the state vector is P(3), V(3), A(3),bias(1) */
    /* the bias vector can be used as the clock for all the system, need to reset before use it */

    /* doppler measuremnt update */
    nm = 0;
    for (j = 0; j < (int)obs_rov->n; ++j)
    {
        pObsRov = obs_rov->data + j;
        pVecRov = vec_rov + j;
        sat = pObsRov->sat;
        sys = satsys(sat, &prn);
        s = satidx(sat, &prn);
        elev = pVecRov->azel[1];
        if (elev < (maskElev * PI / 180.0)) continue;
        for (f = 0; f < NFREQ; ++f)
        {
            if (fabs(pObsRov->D[f]) < 0.001 || isnan(pObsRov->D[f]) || fabs(pVecRov->rate) < 0.001) continue;
            /* measurement */
            double w = satwavelen(sat, pObsRov->code[f]);
            sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f], varfactor, &codeCOV, &phaseCOV);
            /* doppler residual */
            z = -w * pObsRov->D[f] - (pVecRov->rate - CLIGHT * pVecRov->dts[1]);
            R = phaseCOV * 15.0 * 15.0;
            memset(meas + nm, 0, sizeof(measure_t));
            H = meas[nm].H + 0;
            L = meas[nm].L + 0;
            numl = 0;
            H[numl] = -pVecRov->e[0]; L[numl] = 3; ++numl;
            H[numl] = -pVecRov->e[1]; L[numl] = 4; ++numl;
            H[numl] = -pVecRov->e[2]; L[numl] = 5; ++numl;
            H[numl] = 1.0; L[numl] = 9; ++numl; /* clock drift */
            meas[nm].numl = numl;
            meas[nm].z = z;
            meas[nm].R = R;
            meas[nm].sat = sat;
            meas[nm].prn = prn;
            meas[nm].code = pObsRov->code[f];
            meas[nm].sys = sys;
            meas[nm].elev = elev;
            meas[nm].SNR = pObsRov->SNR[f];
            ++nm;
        }
    }

    if (nm > 0)
    {
        if (measure_update_kalman_filter(cur_time, meas, nm, x, P, x_, P_, PHt, max_st, 'D') == 0)
        {
            //printf("%10.1f,DP,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f,%7.3f\n", cur_time, x[0], x[1], x[2], x[3], x[4], x[5], x[9]);
            /* current doppler measurements have problems */
            //for (i = 0; i < 6; ++i)
            //{
            //    x[i] += obs_rov->pos[i];
            //}
            //return 0;
        }

        /* restore the full position and velocity */
        for (i = 0; i < 6; ++i)
        {
            x[i] += obs_rov->pos[i];
            obs_rov->pos[i] = x[i];
            x[i] = 0.0;
        }
        compute_vector_data(obs_rov, vec_rov);
    }
    return n;
}


RTK_RAM_CODE static int find_ref_sat_amb(ambset_t *ambset, int *refsat)
{
	int i, s1, s2, f, s, prn, k=0;
	for (i = 0; i < (int)ambset->n; ++i)
	{
		s1 = ambset->amb[i].s1;
		s2 = ambset->amb[i].s2;
		f = ambset->amb[i].f;
		s = satidx(s1, &prn);
		refsat[MI(s, f, NFREQ)] = s1;
		++k;
	}
	return k;
}


RTK_RAM_CODE static void find_ref_sat_from_state(state_tag_t *tag, int ns, int *refsat)
{
    int i = 0, f = 0, n = 0, s = 0, prn = 0;
    memset(refsat, 0, sizeof(int)*NSYS*NFREQ);
    for (i=0;i<ns;++i)
    {
        if (tag[i].s1 == 0) continue;
        f = tag[i].f;
        s = satidx(tag[i].s1, &prn);
        refsat[MI(s, f, NFREQ)] = tag[i].s1;
    }   
    return; 
}

RTK_RAM_CODE static void state_switch_ref_sat_convert(double *x, double *P, state_tag_t *tag, 
	int np, int ns, int oldrefsat, int newrefsat)
{
    /* convert the state vector while switch reference satellites */
    /* currently,just reset */
    if(!(x && P && tag))
    {
        return;
    }
    int i = 0, j = 0, loc = 0;
    for (i=0;i<ns;++i) 
    {
        if (tag[i].s1 == oldrefsat)
        {
            loc = np+i;
            x[loc] = 0.0;
            for (j=0;j<(np+ns);++j)
            {
                P[SMI(loc, j)] = 0.0;
            }
            memset(tag+i, 0, sizeof(state_tag_t));
        }
    }
    return;
}


RTK_RAM_CODE static int state_switch_ref_sat_convert_new(double *x, double *P, state_tag_t *tag,
	int np, int ns, int oldrefsat, int newrefsat)
{
    /* convert the state vector while switch reference satellites */
    if (!(x && P && tag) || ns == 0 || oldrefsat == 0)
    {
        return 0;
    }
    int i = 0, j = 0, loc = 0, ref_loc = 0;
    for (i = 0; i < ns; ++i)
    {
        if (tag[i].s2 == newrefsat && tag[i].s1 == oldrefsat)
        {
            loc = np + i;
            ref_loc = loc;
            break;
        }
    }

    /* don't pass ambiguity if no new ref sat or dd ambiguity (new - old ref sat) is not integer*/
    if (newrefsat == 0 || fabs(x[ref_loc] - floor(x[ref_loc])) > 0.01)
    {
        for (i = 0; i < ns; ++i)
        {
            if (tag[i].s1 == oldrefsat)
            {
                loc = np + i;
                x[loc] = 0.0;
                for (j = 0; j < (np + ns); ++j)
                {
                    P[SMI(loc, j)] = 0.0;
                }
                memset(tag + i, 0, sizeof(state_tag_t));
            }
        }
        return 0;
    }

    /* find new refsat in tag*/
    for (i = 0; i < ns; ++i)
    {
        if (tag[i].s2 == newrefsat && tag[i].s1 == oldrefsat)
        {
            loc = np + i;
            x[loc] = -x[loc];
            ref_loc = loc;
            tag[i].s2 = oldrefsat;
            tag[i].s1 = newrefsat;
            for (j = 0; j < (np + ns); ++j)
            {
                if (j != loc) P[SMI(loc, j)] = 0.0;
            }
            break;
        }
    }

    /* find other sat in tag*/
    for (i = 0; i < ns; ++i)
    {
        loc = np + i;
        if (loc != ref_loc && tag[i].s1 == oldrefsat)
        {
            loc = np + i;
            x[loc] = x[loc] + x[ref_loc];

            for (j = 0; j < (np + ns); ++j)
            {
                if (j != loc) P[SMI(loc, j)] = 0.0;
                else          P[SMI(loc, loc)] = P[SMI(loc, loc)] + P[SMI(ref_loc, ref_loc)];
            }
            tag[i].s1 = newrefsat;
        }
    }

    return 1;
}

RTK_RAM_CODE static void state_switch_ref_sat(double *x, double *P, state_tag_t *tag,
	int np, int ns, int *newrefsat)
{
    int oldrefsat[NSYS*NFREQ] = { 0 };
    int s = 0, f = 0;
    find_ref_sat_from_state(tag, ns, oldrefsat);
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            if (oldrefsat[MI(s, f, NFREQ)] != newrefsat[MI(s, f, NFREQ)])
            {
              /* need to work on it later */
             //state_switch_ref_sat_convert_new(x, P, tag, np, ns, oldrefsat[MI(s, f, NFREQ)], newrefsat[MI(s, f, NFREQ)]);
              state_switch_ref_sat_convert(x, P, tag, np, ns, oldrefsat[MI(s, f, NFREQ)], newrefsat[MI(s, f, NFREQ)]);
            }
        }
    }
    return;
}

RTK_RAM_CODE static int find_ref_sat(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, 
	vec_t *vec_rov, int *iref, int *irov, int nsd, state_tag_t *tag, 
	int ns, int *refsat, double maskElev, slipset_t *ssat)
{
	obsd_t *pObsRef = NULL;
	obsd_t *pObsRov = NULL;
	vec_t *pVecRef = NULL;
	vec_t *pVecRov = NULL;
	int s, f, isd, refloc, i, j, k, sat, prn;
	double w_ref;
	int num_valid_frq = 0;
	int oldrefsat[NSYS*NFREQ] = { 0 };
	int num_sat = 0;
	if (ns > 0)
	{
		find_ref_sat_from_state(tag, ns, oldrefsat);
		for (k = 0; k < NSYS*NFREQ; k++)
		{
			if (oldrefsat[k] <= 0) continue;
			for (isd = 0; isd < nsd; ++isd)
			{
				j = irov[isd];
				i = iref[isd];
				pObsRef = obs_ref->data + i;
				pObsRov = obs_rov->data + j;
				pVecRef = vec_ref + i;
				pVecRov = vec_rov + j;
				sat = pObsRov->sat;
				if (oldrefsat[k] == sat)
				{
					s = floor(k / NFREQ);
					f = fmod(k, NFREQ);
					int sat_id = find_sat_index(sat, ssat);
					if (sat_id == -1) continue;
                    if (pObsRov->code[f] == 0)                        continue;
					if (pVecRov->azel[1] < (15.0*PI / 180.0))         continue;
					if (pObsRov->SNR[f] <= 120)                       continue;
					if (pObsRov->L[f] == 0.0 || pObsRef->L[f] == 0.0) continue;
					if (ssat[sat_id].nlock[f] <=1)                    continue;
					refsat[MI(s, f, NFREQ)] = oldrefsat[k];
				}
			}
		}
	}

	for (s = 0; s < NSYS; ++s)
	{
		for (f = 0; f < NFREQ; ++f)
		{
			if (refsat[MI(s, f, NFREQ)] > 0)
			{
				++num_valid_frq;
				continue;
			}

			refloc = -1;
			num_sat = 0;
			for (isd = 0; isd < nsd; ++isd)
			{
				j = irov[isd];
				i = iref[isd];
				pObsRef = obs_ref->data + i;
				pObsRov = obs_rov->data + j;
				pVecRef = vec_ref + i;
				pVecRov = vec_rov + j;
				sat = pObsRov->sat;
                if (pObsRov->code[f] == 0)                        continue;
				if (satidx(sat, &prn)!=s)                         continue;
				if (pVecRov->azel[1] < (15.0*PI / 180.0))         continue;
				if (pObsRov->SNR[f]  <=  120)                     continue;   
				if (pObsRov->L[f] == 0.0 || pObsRef->L[f] == 0.0) continue;
				num_sat++;
				w_ref = satwavelen(sat, pObsRov->code[f]);
				if (w_ref<=0.0) continue;
				int sat_id = find_sat_index(sat, ssat);
				if (sat_id == -1) continue;
                if (s == 0 && sat > 32) continue;
				if (ssat[sat_id].slip[f] == 0) 
				{
					refloc = isd;
					refsat[MI(s, f, NFREQ)] = sat;
					break;
				}
			}

			if (refloc<0) continue; /* cannot find the reference satellite */
			++num_valid_frq;

		}
	}

	return num_valid_frq;
}


RTK_RAM_CODE int check_fixed_amb_valid(measure_t *meas, int nm, double *x)
{
    int fixed_check = 0;
    int i, j, numl;
    double z, z_, inov;
    double* H = NULL;
    int   * L = NULL;
    double omega = 0.0;

    for (i = 0; i < nm; ++i)
    {
        z_ = 0.0;
        numl = meas[i].numl;
        z    = meas[i].z;
        H    = meas[i].H;
        L    = meas[i].L;
        for (j = 0; j < numl; ++j)
        {
            z_ += H[j] * x[L[j]];
        }
        inov = z - z_;
        omega += inov * inov;
    }
    omega = sqrt(omega / nm);
    fixed_check = (omega < 1.0) ? 1 : 0;

    return fixed_check;
}

/*find refsat in cur_ambset*/
RTK_RAM_CODE static void get_cur_refsat(ambset_t *cur_ambset, int *cur_refsat)
{
    int i, s, f, prn;
    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            for (i = 0; i < (int)cur_ambset->n; i++)
            {
                if (s != satidx(cur_ambset->amb[i].s1, &prn)) continue;
                if (f != cur_ambset->amb[i].f) continue;
                cur_refsat[MI(s, f, NFREQ)] = cur_ambset->amb[i].s1;
                break;
            }
        }
    }
}

/*find refsat index in ambset*/
RTK_RAM_CODE static int find_cur_refsat_in_ambset(ambset_t *ambset, int *cur_refsat, int *refsat_idx)
{
    int i, s, f, prn;
    int refsat_changed = 0;
    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            for (i = 0; i < (int)ambset->n; i++)
            {
                if (ambset->amb[i].s1 == 0 || ambset->amb[i].s2 == 0) continue;
                if (s != satidx(ambset->amb[i].s2, &prn)) continue;
                if (f != ambset->amb[i].f) continue;
                if (ambset->amb[i].s2 != cur_refsat[MI(s, f, NFREQ)]) continue;
                refsat_idx[MI(s, f, NFREQ)] = i;
                refsat_changed = 1;
                break;
            }
        }
    }
    return refsat_changed;
}

/* switch refsat in ambset to keep same as cur_ambset*/
RTK_RAM_CODE static void switch_refsat_ambset(ambset_t *ambset, int *cur_refsat, int *refsat_idx)
{
    int i, s, f, prn, ref_idx;

    // switch ambset of refsat
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; f++)
        {
            if (refsat_idx[MI(s, f, NFREQ)] != -1)
            {
                ref_idx = refsat_idx[MI(s, f, NFREQ)];
                ambset->amb[ref_idx].s2   = ambset->amb[ref_idx].s1;
                ambset->amb[ref_idx].s1   = cur_refsat[MI(s, f, NFREQ)];
                ambset->amb[ref_idx].data = -ambset->amb[ref_idx].data;
            }
        }
    }

    // switch ambset of non refsat
    for (i = 0; i < (int)ambset->n; ++i)
    {
        if (ambset->amb[i].s1 == 0 || ambset->amb[i].s2 == 0) continue;
        s = satidx(ambset->amb[i].s2, &prn);
        f = ambset->amb[i].f;
        if (ambset->amb[i].s1 != cur_refsat[MI(s, f, NFREQ)] && cur_refsat[MI(s, f, NFREQ)] != 0)
        {
            if (i != refsat_idx[MI(s, f, NFREQ)])
            {
                ambset->amb[i].s1 = cur_refsat[MI(s, f, NFREQ)];
                ambset->amb[i].data = ambset->amb[i].data + ambset->amb[refsat_idx[MI(s, f, NFREQ)]].data;
            }
        }
    }
}

/* update ambset to new cur_ambset */
RTK_RAM_CODE void ambset_update(ambset_t *cur_ambset, ambset_t *ambset)
{
    int cur_refsat[NSYS * NFREQ] = {0};
    int refsat_idx[NSYS * NFREQ] = {-1, -1, -1, -1, -1, -1, -1, -1};
    int i, j;

    if (cur_ambset->n == 0)  return;  /* return if cur_ambset is empty */

    /* ambset is empty, update ambset if cur_amset is passed ambiguity validation */
    if (ambset->n == 0)
    {
        ambset->n = cur_ambset->n;
        ambset->ratio = cur_ambset->ratio;
        for (i = 0; i < (int)cur_ambset->n; ++i)
        {
            ambset->amb[i].data = cur_ambset->amb[i].data;
            ambset->amb[i].f = cur_ambset->amb[i].f;
            ambset->amb[i].s1 = cur_ambset->amb[i].s1;
            ambset->amb[i].s2 = cur_ambset->amb[i].s2;
            ambset->amb[i].time = cur_ambset->amb[i].time;
            ambset->amb[i].var = cur_ambset->amb[i].var;
            ambset->amb[i].nlock = 1;
        }
    }
    else
    {
        /* switch refsat from ambset to cur_ambset*/
        get_cur_refsat(cur_ambset, cur_refsat);
        if (find_cur_refsat_in_ambset(ambset, cur_refsat, refsat_idx))
        {
            switch_refsat_ambset(ambset, cur_refsat, refsat_idx);
        }

        /*update ambs existed in ambset*/
        for (i = 0; i < (int)cur_ambset->n; ++i)
        {
            int idx = -1;
            for (j = 0; j < (int)ambset->n; ++j)
            {
                if (ambset->amb[j].f  != cur_ambset->amb[i].f)                 continue;
                if (ambset->amb[j].s1 != cur_ambset->amb[i].s1)                continue;
                if (ambset->amb[j].s2 != cur_ambset->amb[i].s2)                continue;
                if (fabs(ambset->amb[j].data - cur_ambset->amb[i].data) > 0.1) continue;
                idx = j;
                break;
            }

            if (cur_ambset->ratio > ambset->ratio)  ambset->ratio = cur_ambset->ratio;

            /*amb not existed in ambset, update */
            if (idx == -1)
            {
                if (ambset->n >= MAXAMBINSET) continue;
                ambset->amb[ambset->n].f    = cur_ambset->amb[i].f;
                ambset->amb[ambset->n].s1   = cur_ambset->amb[i].s1;
                ambset->amb[ambset->n].s2   = cur_ambset->amb[i].s2;
                ambset->amb[ambset->n].data = cur_ambset->amb[i].data;
                ambset->amb[ambset->n].time = cur_ambset->amb[i].time;
                ambset->amb[ambset->n].var = cur_ambset->amb[i].var;
                ambset->amb[ambset->n].nlock = 1;
                ambset->n++;
            }
            else
            {
                /* check the ratio, if ratio in cur_ambset is higher than ratio in ambset, update */
                /*amb existed in ambset, update when ratio (cur_ambset) is higher than it in ambset or nlock==0 in ambset */
				if (ambset->amb[idx].data == cur_ambset->amb[i].data)
				{
					ambset->amb[idx].nlock++;
					if (ambset->amb[idx].nlock > 128)
					{
						ambset->amb[idx].nlock = 128;
					}
				}
				else
				{
					ambset->amb[idx].nlock = 1;
				}
                ambset->amb[idx].data = cur_ambset->amb[i].data;
                ambset->amb[idx].time = cur_ambset->amb[i].time;
                ambset->amb[idx].var  = cur_ambset->amb[i].var;       
            }
        }
    }
}

RTK_RAM_CODE void ambset_cycslip(slipset_t *ssat, ambset_t *ambset)
{
    int i, j, f, rsat, sat, sat_id1, sat_id2;
    for (i = 0; i < (int)ambset->n; ++i)
    {
        f = ambset->amb[i].f;
        rsat = ambset->amb[i].s1;
        sat = ambset->amb[i].s2;
        sat_id2 = find_sat_index(sat, ssat);
        sat_id1 = find_sat_index(rsat, ssat);
        if (sat_id1 == -1 || sat_id2 == -1) continue;

        if (ssat[sat_id1].slip[f] != 0 || ssat[sat_id2].slip[f] != 0)
        {
            for (j = i; j < (int)ambset->n - 1; j++)
            {
                ambset->amb[j].time  = ambset->amb[j+1].time;
                ambset->amb[j].data  = ambset->amb[j+1].data;
                ambset->amb[j].s1    = ambset->amb[j+1].s1;
                ambset->amb[j].s2    = ambset->amb[j+1].s2;
                ambset->amb[j].f     = ambset->amb[j+1].f;
                ambset->amb[j].nlock = ambset->amb[j+1].nlock;
                ambset->amb[j].var   = ambset->amb[j+1].var;
            }
            ambset->n--;
            memset(&ambset->amb[ambset->n], 0, sizeof(ambdata_t));
        }
    }
}

/* update ambset if new cur_ambset come*/
RTK_RAM_CODE void ambset_switch(int ns, state_tag_t *tag, ambset_t *ambset)
{
    int cur_refsat[NSYS*NFREQ] = { 0 };
    int refsat_idx[NSYS*NFREQ] = { -1,-1,-1,-1,-1,-1,-1,-1 };
    int i, s, f, prn;

    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            for (i = 0; i < ns; i++)
            {
                if (s != satidx(tag[i].s1, &prn)) continue;
                if (f != tag[i].f) continue;
                cur_refsat[MI(s, f, NFREQ)] = tag[i].s1;
                break;
            }
        }
    }

    if (find_cur_refsat_in_ambset(ambset, cur_refsat, refsat_idx))
    {
        switch_refsat_ambset(ambset, cur_refsat, refsat_idx);
    }
}

RTK_RAM_CODE int fixed_amb_constraint(double time, ambset_t *ambset, state_tag_t *tag, 
	slipset_t *ssat, double *xa, double *Pa, int np, int ns, int obs_nlock)
{
    int i, j, f, s1, s2, loc,  prn, sys, numl = 0, numfixed = 0;
    double z, z_, R, R_, inov, P_inov;
    double H[NX_RTK] = { 0.0 }, PHt[NX_RTK] = { 0.0 };
    int    L[NX_RTK] = { 0 };

    /* switch amset based on current tag*/
    ambset_switch(ns, tag, ambset);

    if (ambset->n < MIN_AMBNUM) return numfixed;

    /* fixed ambiguity constraint*/
    for (i = 0; i < ns; ++i)
    {
        f = tag[i].f;
        s1 = tag[i].s1;
        s2 = tag[i].s2;
        loc = i;
        int sat_id = find_sat_index(tag[i].s2, ssat);
        sys = satsys(tag[i].s2, &prn);
		if (tag[i].s1 == 0 || fabs(tag[i].time - time) > 0.1
			|| Pa[SMI(np + i, np + i)] > 25.0 || ssat[sat_id].nlock[f] < obs_nlock)
		{
			continue;
		}

        int idx = -1;
        for (j = 0; j < (int)ambset->n; j++)
        {
            if (ambset->amb[j].f != f)    continue;
            if (ambset->amb[j].s1 != s1)  continue;
            if (ambset->amb[j].s2 != s2)  continue;
            //if (ambset->amb[j].nlock <=10) continue;
            idx = j;
            break;
        }
        if (idx == -1) continue;

        z = ambset->amb[idx].data;
        R = 0.001 * 0.001;
        numl = 0;
        H[numl] = +1.0; L[numl] = loc + np; ++numl;
        ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, np + ns, numl, PHt);
        inov = z - z_;
        P_inov = R + R_;
        if (fabs(inov) > 3.0)
        {
            Pa[SMI(L[numl - 1], L[numl - 1])] += 1.0e4;
            continue;
        }
        ekf_measurement_update(xa, Pa, H, L, inov, P_inov, np + ns, numl, PHt);
        numfixed++;
    }

    return numfixed;
}

RTK_RAM_CODE static int ambres_state_vector(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref,
	vec_t *vec_rov, int *iref, int *irov, int nsd, double *x, double *P, 
	state_tag_t *tag, int *np, int *ns, int obs_nlock1, int obs_nlock2,
	double *xa, double *Pa, ambset_t *ambset, slipset_t *ssat)
{
    int f, sys, prn;
    int		 i, j, k;
    int numa = 0, nums = 0, numf = 0;
    double	 ai[MAXAMB * 2] = { 0 };
    double	 af[MAXAMB] = { 0 };
	double	 Qa[MAXAMB * MAXAMB] = { 0 }, s[2];
    int satIDs[MAXAMB] = { 0 };
    double z, R, z_, R_, inov, P_inov, wsf;
    double H[NX_RTK] = { 0.0 }, PHt[NX_RTK] = { 0.0 };
    int    L[NX_RTK] = { 0 }, numl = 0;
    ambloc_t ambloc[MAXAMB] = { 0 }, temp_ambloc = { 0 }, cur_ambloc = { 0 };
    ambset_t cur_ambset = { 0 };
    measure_t meas[MAXOBS * NFREQ] = { 0 };
    double x_[NX_RTK] = { 0.0 };
    double P_[SMD(NX_RTK)] = { 0.0 };
    int nm = 0;
    int fixID = 5;
    int week = 0;
    int numfixed = 0;
    int nfixed = 0;
    int max_st = NX_RTK;
    memcpy(xa, x, sizeof(double) *    (max_st));
    memcpy(Pa, P, sizeof(double) * SMD(max_st));
    double time = time2gpst(obs_rov->time, &week);
    double covP3 = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);
    if (sqrt(covP3) > 1.0) return fixID;

	for (i = 0; i < *ns; ++i)
	{
		if (tag[i].s1 == 0 || tag[i].s2 == 0) continue;
        sys = satsys(tag[i].s2, &prn);
#ifndef GLO_AR
        if (sys == _SYS_GLO_) continue;
#endif
        cur_ambloc.loc = i;
        f = tag[i].f;
        cur_ambloc.f  = tag[i].f;
        cur_ambloc.s1 = tag[i].s1;
        cur_ambloc.s2 = tag[i].s2;
        int sat_id  = find_sat_index(tag[i].s2, ssat);
        int rsat_id = find_sat_index(tag[i].s1, ssat);
        int sat_ix = -1, rsat_ix=-1;
        for (j = 0; j < (int)obs_rov->n; ++j)
        {
            if (obs_rov->data[j].sat == tag[i].s2)
            {
                sat_ix = j;
                break;
            }
        }
		if (sat_ix < 0 || sat_ix == obs_rov->n) continue; /* need to check valid or not */
        for (j = 0; j < (int)obs_rov->n; ++j)
        {
            if (obs_rov->data[j].sat == tag[i].s1)
            {
                rsat_ix = j;
                break;
            }
        }

		if (rsat_ix < 0 || rsat_ix == obs_rov->n) continue; /* need to check valid or not */
		if (tag[i].s1 == 0 || fabs(tag[i].time - time) > 0.1)                              continue;
        if (P[SMI(*np + i, *np + i)] > 25.0)                                               continue;
        if (ssat[sat_id].nlock[f] < obs_nlock2 || ssat[rsat_id].nlock[f] < obs_nlock1)     continue;
        if (obs_rov->data[sat_ix].SNR[f]  < 120)                                           continue;  

        double fAmbEst = xa[*np + i];
        double iAmbEst = floor(fAmbEst + 0.5);
        double rAmbEst = fAmbEst - iAmbEst;
        double fAmbCov = Pa[SMI(*np + i, *np + i)];
        cur_ambloc.value = rAmbEst * rAmbEst + fAmbCov;

        if (numa == MAXAMB)
        {
            for (k = 0; k < numa; ++k)
            {
                for (j = k + 1; j < numa; ++j)
                {
                    if (ambloc[k].value > ambloc[j].value)
                    {
                        temp_ambloc = ambloc[k];
                        ambloc[k] = ambloc[j];
                        ambloc[j] = temp_ambloc;
                    }
                }
            }
            if (ambloc[numa - 1].value < cur_ambloc.value)
            {
                continue;
            }
            ambloc[numa - 1] = cur_ambloc;
        }
        else
        {
            ambloc[numa] = cur_ambloc;
            ++numa;
        }
        for (j = 0; j < nums; ++j) { if (satIDs[j] == cur_ambloc.s2) break; }
        if (j == nums)
        {
            satIDs[nums] = cur_ambloc.s2;
            ++nums;
        }
    }

    if (numa < MIN_AMBNUM || nums < MIN_AMBNUM)
        return fixID;

    for (i = 0; i < numa; ++i)
    {
        for (j = i + 1; j < numa; ++j)
        {
            if (ambloc[i].value > ambloc[j].value)
            {
                temp_ambloc = ambloc[i];
                ambloc[i] = ambloc[j];
                ambloc[j] = temp_ambloc;
            }
        }
    }

    numf = numa;
    nfixed = 0;
    int idxIter = 0;
	int max_inter = (int)floor(numa * 0.5);
    int prn_ref = 0;
    while (numa > 0 && idxIter < max_inter)
    {
        double sr = 1.0;
        nums = 0;
        for (i = 0; i < numa; ++i)
        {
            af[i] = xa[*np + ambloc[i].loc];
            for (j = 0; j < numa; j++)
            {
                Qa[i + j * numa] = Pa[SMI(ambloc[i].loc + *np, ambloc[j].loc + *np)];
            }

            for (j = 0; j < nums; ++j) { if (satIDs[j] == ambloc[i].s2) break; }
            if (j == nums)
            {
                satIDs[nums] = ambloc[i].s2;
                ++nums;
            }
        }

        if (numa < MIN_AMBNUM || nums < MIN_AMBNUM) break;

        /* lambda/mlambda integer least-square estimation */
        if (!lambda(af, Qa, numa, NUM_INT_AMB_CANDIDATE, ai, s, &sr))
            break;

        cur_ambset.ratio = (s[1] / s[0]);
        if (cur_ambset.ratio > 999.9) cur_ambset.ratio = 999.9f;

        if (cur_ambset.ratio >= 2.5)
        {
            cur_ambset.n = 0;
            for (i = 0; i < numa; ++i)
            {
                z = ai[i];
                R = 0.001 * 0.001;
                numl = 0;
                H[numl] = 1.0; 
				L[numl] = ambloc[i].loc + *np; 
				++numl;
                ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, *np + *ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (fabs(inov) > 1.0)
                {
                    Pa[SMI(L[numl - 1], L[numl - 1])] += 1.0e4;
                    continue;
                }
                if (fabs(af[i] - ai[i]) > 1.0)
                {
                    P[SMI(L[numl - 1], L[numl - 1])] += 1.0e4;
                    Pa[SMI(L[numl - 1], L[numl - 1])] += 1.0e4;
                }
                ekf_measurement_update(xa, Pa, H, L, inov, P_inov, *np + *ns, numl, PHt);
                numfixed++;
                f = tag[ambloc[i].loc].f;
                cur_ambset.amb[cur_ambset.n].f = tag[ambloc[i].loc].f;
                cur_ambset.amb[cur_ambset.n].s1 = tag[ambloc[i].loc].s1;
                cur_ambset.amb[cur_ambset.n].s2 = tag[ambloc[i].loc].s2;
                cur_ambset.amb[cur_ambset.n].data = ROUND(ai[i]);
                cur_ambset.amb[cur_ambset.n].var = Pa[SMI(*np + ambloc[i].loc, *np + ambloc[i].loc)];
                cur_ambset.amb[cur_ambset.n].time = time;
                ++cur_ambset.n;
            }

            for (i = numa; i < numf; ++i)
            {
                double fAmbEst = fabs(xa[*np + i] - ai[i]);
                double fAmbCov = Pa[SMI(*np + i, *np + i)];
                if (fAmbEst < 0.1 && fAmbCov < 0.04)
                {
                    sys = satsys(tag[ambloc[i].loc].s2, &prn);
                    if (sys == _SYS_GLO_) continue;
                    f = tag[ambloc[i].loc].f;
                    int newfix = 1;
                    for (j = 0; j < (int)ambset->n; j++)
                    {
                        if (tag[ambloc[i].loc].s1 == ambset->amb[j].s1 &&
                            tag[ambloc[i].loc].s2 == ambset->amb[j].s2 &&
                            ambset->amb[j].f == f)
                        {
                            newfix = 0;
                            break;
                        }
                    }
                    if (newfix)
                    {
                        z = ROUND(xa[*np + i]);
                        R = 0.001 * 0.001;
                        numl = 0;
                        H[numl] = +1.0; L[numl] = ambloc[i].loc + *np; ++numl;
                        ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, *np + *ns, numl, PHt);
                        inov = z - z_;
                        P_inov = R + R_;
                        if (fabs(inov) > 1.0)
                        {
							Pa[SMI(L[numl - 1], L[numl - 1])] += 1.0e4; 
							continue;
                        }
                        ekf_measurement_update(xa, Pa, H, L, inov, P_inov, *np + *ns, numl, PHt);
                        numfixed++;
                        cur_ambset.amb[cur_ambset.n].f = f;
                        cur_ambset.amb[cur_ambset.n].s1 = tag[ambloc[i].loc].s1;
                        cur_ambset.amb[cur_ambset.n].s2 = tag[ambloc[i].loc].s2;
                        cur_ambset.amb[cur_ambset.n].data = ai[i];
                        cur_ambset.amb[cur_ambset.n].var = Qa[i + i * numf];
                        ++cur_ambset.n;
                    }
                }
            }
            break;
        }
        /* remove the worst one */
        ++idxIter;
        --numa;
    }

    if (cur_ambset.ratio < 2.5)
    {
        for (i = 0; i < numf; ++i)
        {
            double fAmbEst = fabs(xa[*np + i] - ai[i]);
            double fAmbCov = Pa[SMI(*np + i, *np + i)];
            if (fAmbEst < 0.1 && fAmbCov < 0.04)
            {
                sys = satsys(tag[ambloc[i].loc].s2, &prn);
                if (sys == _SYS_GLO_) continue;
                numfixed++;
                f = tag[ambloc[i].loc].f;
                int newfix = 1;
                for (j = 0; j < (int)ambset->n; j++)
                {
                    if (tag[ambloc[i].loc].s1 == ambset->amb[j].s1 &&
                        tag[ambloc[i].loc].s2 == ambset->amb[j].s2 &&
                        ambset->amb[j].f == f)
                    {
                        newfix = 0;
                        break;
                    }
                }
                if (newfix)
                {                    
                    z = ROUND(xa[*np + i]);
                    R = 0.001 * 0.001;
                    numl = 0;
                    H[numl] = +1.0; L[numl] = ambloc[i].loc + *np; ++numl;
                    ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, *np + *ns, numl, PHt);
                    inov = z - z_;
                    P_inov = R + R_;
                    if (fabs(inov) > 1.0)
                    {
                        Pa[SMI(L[numl - 1], L[numl - 1])] += 1.0e4;
                        continue;
                    }
                    ekf_measurement_update(xa, Pa, H, L, inov, P_inov, *np + *ns, numl, PHt);
                    numfixed++;
                    cur_ambset.amb[cur_ambset.n].f = f;
                    cur_ambset.amb[cur_ambset.n].s1 = tag[ambloc[i].loc].s1;
                    cur_ambset.amb[cur_ambset.n].s2 = tag[ambloc[i].loc].s2;
                    cur_ambset.amb[cur_ambset.n].data = ai[i];
                    cur_ambset.amb[cur_ambset.n].var = Qa[i + i * numf];
                    ++cur_ambset.n;
                }
            }
        }
    }

    //check_fixed_amb_valid
    if (numfixed >= MIN_AMBNUM)
    {
        fixID = 4;
        ambset->ratio = cur_ambset.ratio;
        /* fix the filter */
        memcpy(x, xa, sizeof(double) * (max_st));
    }
    /* update cur_ambset to amset*/
    ambset_update(&cur_ambset, ambset);
    return fixID;
}

RTK_RAM_CODE double adjust_sd_glo_obs(int sat, int f, double sdL, double sdP)
{
    double obs = 0.0, w = 0;

    w = satwavelenbyfreq(sat, f);
    obs = sdL - sdP / w;

    return obs;
}

/* float filter */
RTK_RAM_CODE static int robust_filter_RTK(double cur_time, obs_t* obs_rov, vec_t* vec_rov, 
	obs_t* obs_ref, vec_t* vec_ref, int* irov, int* iref, int nsd, double* x, 
	double* P, state_tag_t* tag, slipset_t *ssat, measure_t *meas, const int np, 
	int* ns, int *nobs, glo_ifb_t* glo_ifb, double maskElev, double coeff)
{
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, numl = 0, locAmb = 0;
    double z = 0.0, R = 0.0, elev = 0.0, snr = 0.0;
    double PHt[NX_RTK] = { 0.0 };
    double x_[NX_RTK] = { 0.0 };
    double P_[SMD(NX_RTK)] = { 0.0 };
    int refsat[NSYS * NFREQ] = { 0 };        /* reference satellite */
    double z_ref = 0.0; /* residuals */
    double R_ref = 0.0; /* covariance */
    double w_ref = 0.0; /* wave */
    int week = 0;
    int prn = 0, sys = 0;
    int refloc = 0;
    double w = 0.0; /* wave */
    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t  *pVecRef = NULL;
    vec_t  *pVecRov = NULL;
    obsd_t *pObsRef_0 = NULL;
    obsd_t *pObsRov_0 = NULL;
    vec_t  *pVecRef_0 = NULL;
    vec_t  *pVecRov_0 = NULL;
    double *H = NULL;
    int *L = NULL;
    int nm = 0;
    double codeCOV = 0.0;
    double phaseCOV = 0.0;
    double cdt_bias = 0.0;
    int fixID = 5;
    int max_st = NX_RTK;

    /* reset the ambiguity state if there is not reference satellites */
    if (find_ref_sat(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, tag, *ns, refsat, maskElev, ssat) == 0)
    {
        *ns = 0;
        return fixID;
    }

    /* ambiguities */
    /* check the reference satellite or switch if the reference satellite changes */
    state_switch_ref_sat(x, P, tag, np, *ns, refsat);
#if 0
	for (i = 0; i < *ns; i++)
	{
		printf("%14.3lf,%2d,%3d,%3d\n", tag[i].time, tag[i].f, tag[i].s1, tag[i].s2);
	}
#endif
    int numL_sys[NFREQ*NSYS] = { 0 };
    int numa_sys[NFREQ*NSYS] = { 0 };
    for (isd = 0; isd < nsd; ++isd)
    {
        j = irov[isd];
        i = iref[isd];
        pObsRef = obs_ref->data + i;
        pObsRov = obs_rov->data + j;
        pVecRef = vec_ref + i;
        pVecRov = vec_rov + j;
        sat = pObsRov->sat;
        sys = satsys(sat, &prn);
        s = satidx(sat, &prn);
        elev = pVecRov->azel[1];
        if (elev < (maskElev * PI / 180.0)) continue;
        for (f = 0; f < NFREQ; ++f)
        {
            if (fabs(pObsRov->L[f]) > 0.001 && fabs(pObsRef->L[f]) > 0.001)
                numL_sys[MI(s, f, NFREQ)]++;
        }
    }
#if 0
	printf("numL_sys:\n");
	for (i = 0; i < NFREQ*NSYS; i++)
	{
		printf("%2d\n", numL_sys[i]);
	}
#endif

#ifndef GLO_AR
    glo_ifb->glo_ifbflag = 0;
#endif

    /* phase measurement update */
    for (s = 0; s < NSYS; ++s)
    {
        int rsat = 0;
        if (s == 3 && glo_ifb->glo_ifbflag==0) continue; /* skip GLONASS AR */
        for (f = 0; f < NFREQ; ++f)
        {
            if (numL_sys[MI(s, f, NFREQ)] < 2)   continue;
            int curLOC = MI(s, f, NFREQ); 
            cdt_bias =(curLOC>0)?x[10]+x[10 + curLOC]:x[10];
            /* reference satellite measurement update */
            refloc = -1;
            double sdN = 0.0;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                if (pObsRov->sat != refsat[MI(s, f, NFREQ)]) continue;
                elev = pVecRov->azel[1];
                sat = pObsRov->sat;
                sys = satsys(sat, &prn);
                if (pObsRov->code[f] == 0) continue;
                if (elev < (maskElev*D2R)) continue;
                w_ref = satwavelen(sat, pObsRov->code[f]);
                if (w_ref <= 0.0) continue;
                //if (coeff >= 0.3)
                    sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, 1.0, &codeCOV, &phaseCOV);
                //else
                //    sd_varerr_ele_snrdiff(satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, &codeCOV, &phaseCOV);
                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    if (pObsRov->P[f] != 0.0 && pObsRef->P[f] != 0.0)
                    {
                        double sdL = pObsRov->L[f] - pObsRef->L[f];
                        double sdP = pObsRov->P[f] - pObsRef->P[f];
                        sdN = adjust_sd_glo_obs(sat, f, sdL, sdP);
                    }
                    else
                    {
                        sdN = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro) + cdt_bias) / w_ref;
                    }

                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)+ cdt_bias) / w_ref;
                    R = phaseCOV / SQR(w_ref);
                    x[IX_PHB + curLOC] = floor(z + 0.5);

                    for (k = 0; k < (np + *ns); ++k)
                    {
                        P[SMI(k, IX_PHB + curLOC)] = 0.0;
                    }
					/* SD ambiguities for the reference satellites */
                    P[SMI(IX_PHB + curLOC, IX_PHB + curLOC)] = 1000.0 * 1000.0; 
                }
                else
                    continue;

                int rsat_id = find_sat_index(pObsRov->sat, ssat);
                if (s == 3)
                {
                    rsat = sat;
                }
                memset(meas + nm, 0, sizeof(measure_t));
                H = meas[nm].H + 0;
                L = meas[nm].L + 0;
                numl = 0;
                H[numl] = -pVecRov->e[0] / w_ref; L[numl] = 0;               ++numl;
                H[numl] = -pVecRov->e[1] / w_ref; L[numl] = 1;               ++numl;
                H[numl] = -pVecRov->e[2] / w_ref; L[numl] = 2;               ++numl;
                H[numl] = 1.0;                    L[numl] = IX_PHB + curLOC; ++numl;
                meas[nm].numl   = numl;
                meas[nm].z      = z;
                meas[nm].R      = R;
                meas[nm].sat    = sat;
                meas[nm].prn    = prn;
                meas[nm].code   = pObsRov->code[f];
                meas[nm].sys    = sys;
                meas[nm].elev   = elev;
                meas[nm].SNR    = pObsRov->SNR[f];
                meas[nm].refsat = 1;
                meas[nm].cslip  = ssat[rsat_id].slip[f];
                meas[nm].nlock  = ssat[rsat_id].nlock[f];
#if 0
				printf("%3d,%10.3lf,%10.6lf,%2d,%2d\n", sat, z, R, meas[nm].refsat, f);
#endif
                ++nm;
                refloc = isd;
				
                break;
            }
            if (refloc < 0) continue;
            j = irov[refloc];
            i = iref[refloc];
            pObsRef_0 = obs_ref->data + i;
            pObsRov_0 = obs_rov->data + j;
            pVecRef_0 = vec_ref + i;
            pVecRov_0 = vec_rov + j;
            int prn_ref = 0;
            sys = satsys(pObsRov_0->sat, &prn_ref);
            /* rover satellites */
            for (isd = 0; isd < nsd; ++isd)
            {
                if (isd == refloc) continue;

                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;

                if (satidx(sat, &prn) != s) continue;
                elev = pVecRov->azel[1];
				if (elev < (maskElev*PI / 180.0))
				{
					continue;
				}
                w = satwavelen(sat, pObsRov->code[f]);
                sys = satsys(sat, &prn);
                int sat_id = find_sat_index(sat, ssat); 
                if (w <= 0.0) continue;
				
                //if (coeff >= 0.3)
                    sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, 1.0, &codeCOV, &phaseCOV);
                //else
                //    sd_varerr_ele_snrdiff(satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f] / 4, &codeCOV, &phaseCOV);

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)+ cdt_bias) / w;
                    if (s == 3)
                    {
                        int ref_prn = 0;
                        int sys = satidx(rsat, &ref_prn);
                        int dfrq = get_glo_frq(prn) - get_glo_frq(ref_prn);
                        z += ((w - w_ref)*sdN / w - dfrq * glo_ifb->glo_ifb);
                    }

                    R = phaseCOV / SQR(w);
					//printf("rovsat:%2d,%2d,%2d\n", *ns, sat, f);
                    locAmb = find_state_index(tag, *ns, pObsRov_0->sat, pObsRov->sat, f);
                    if (locAmb < 0) 
                    {
                        /* new state vector, need to add */
                        locAmb = find_next_state_index(tag, np, *ns, max_st);
                        if (locAmb < 0)
                        {
                            /* cannot add ambiguity state vector */
                            //if (numa_sys[MI(s, f, NFREQ)]>0)
                            {
                                continue;
                            }                    
                            int bestL = -1;
                            double bestT = 0.0;
                            /* find the oldest one and replace it */
                            for (i = 0; i < *ns; ++i)
                            {
                                int ts = satidx(tag[i].s1, NULL);
                                int tf = tag[i].f;
                                double diffT = fabs(cur_time - tag[i].time);
                                if (numa_sys[MI(ts, tf, NFREQ)]>6 && tf==0)
                                {
                                    numa_sys[MI(ts, tf, NFREQ)]--;
                                    bestL = i;
                                    bestT = diffT;
                                }
                            }
                            if (bestL < 0)
                            {
                                continue;
                            }
                            locAmb = bestL;
                        }
                        tag[locAmb].s1 = pObsRov_0->sat;
                        tag[locAmb].s2 = pObsRov->sat;
                        tag[locAmb].f = f;
                        tag[locAmb].time = cur_time;
                        if (locAmb == *ns)
                        {
                            ++(*ns);
                        }
                        else
                        {
                            k = 0;
                        }
                        locAmb += np;
                        x[locAmb] = floor(z - x[IX_PHB + curLOC] + 0.5);
                        for (k = 0; k < (np + *ns); ++k)
                        {
                            P[SMI(k, locAmb)] = 0.0;
                        }
                        P[SMI(locAmb, locAmb)] = 1.0e4;
                        numa_sys[MI(s, f, NFREQ)]++;
                    }
                    else
                    {
                        numa_sys[MI(s, f, NFREQ)]++;
                        /* add a small process noise */
                        P[SMI(locAmb, locAmb)] += 1.0e-8;
                        /* old state vector, already exist */
                        /* check cycle slip */
                        double time_gap = cur_time - tag[locAmb].time;
                        tag[locAmb].time = cur_time;
                        locAmb += np;
                        /* need to do both reference and rover satellite */
                        int rsat_id = find_sat_index(pObsRov_0->sat, ssat);
                        if (ssat[sat_id].slip[f] != 0 || ssat[rsat_id].slip[f] != 0)
                        {
                            for (k = 0; k < (np + *ns); ++k)
                            {
                                P[SMI(k, locAmb)] = 0.0;
                            }
                            P[SMI(locAmb, locAmb)] = 1.0e4;
                            x[locAmb] = floor(z - x[IX_PHB + curLOC] + 0.5);
                        }
                    }
                }
                else
                    continue;

                memset(meas + nm, 0, sizeof(measure_t));
                H = meas[nm].H + 0;
                L = meas[nm].L + 0;
                numl = 0;
                H[numl] = -pVecRov->e[0] / w; L[numl] = 0;               ++numl;
                H[numl] = -pVecRov->e[1] / w; L[numl] = 1;               ++numl;
                H[numl] = -pVecRov->e[2] / w; L[numl] = 2;               ++numl;
                H[numl] = 1.0;                L[numl] = locAmb;          ++numl;
                H[numl] = 1.0;                L[numl] = IX_PHB + curLOC; ++numl;
                meas[nm].numl = numl;
                meas[nm].z = z;
                meas[nm].R = R;
                meas[nm].sat    = sat;
                meas[nm].prn    = prn;
                meas[nm].code   = pObsRov->code[f];
                meas[nm].sys    = sys;
                meas[nm].elev   = elev;
                meas[nm].SNR    = pObsRov->SNR[f];
                meas[nm].refsat = 0;
                meas[nm].cslip  = ssat[sat_id].dph[f];
                meas[nm].nlock  = ssat[sat_id].nlock[f];
#if 0
				printf("%3d,%10.3lf,%10.6lf,%2d,%2d\n", sat, z, R, meas[nm].refsat,f);
#endif
                ++nm;
            }
        }
    }

    if (measure_update_kalman_filter(cur_time, meas, nm, x, P, x_, P_, PHt, max_st, 'L') == 0)
    {
        return 0; 
    }
    *nobs = nm;
    return fixID;
}

extern int reset_rcv(rcv_rtk_t *rcv)
{
    if (rcv == NULL) return 0;
    rcv->tt = 1.0;
    rcv->time = 0.0;
    rcv->ns = 0;
    rcv->num_fixepoch = 0;
    memset(rcv->x, 0, sizeof(double)*NX_RTK);
    memset(rcv->P, 0, sizeof(double)*SMD(NX_RTK));
    memset(rcv->slip, 0, sizeof(slipset_t)*MAXOBS);
    memset(&rcv->ambset, 0, sizeof(ambset_t));
    memset(rcv->tag, 0, sizeof(state_tag_t)*MAXAMB);
    rcv->num_goodepoch = 0;
    return 1;
}

int reset_pva_cov(rcv_rtk_t *rcv)
{
    if (rcv == NULL) return 0;
    memset(rcv->P, 0, sizeof(double)*SMD(9));
    int i;
    /* xyz */
    for (i = 0; i < 3; ++i)
    {
        rcv->P[SMI(i, i)] = 100.0 * 100.0;
    }
    /* vel */
    for (i = 0; i < 3; ++i)
    {
        rcv->P[SMI(i + 3, i + 3)] = 100.0 * 100.0;
    }
    /* acc */
    for (i = 0; i < 3; ++i)
    {
        rcv->P[SMI(i + 6, i + 6)] = 10.0 * 10.0;
    }
    return 1;
}

/* the filter for RTK */
RTK_RAM_CODE int rtk_filter(obs_t *rov, obs_t *ref, vec_t *vec_rov, vec_t *vec_ref, 
	int *iref, int *irov, int nsd, rcv_rtk_t *rcv, int *num_of_sat, 
	double *dop, double maskElev, int isSearchAMB)
{
	/*start to do RTK filter */
	int fixID = 5, i = 0, j = 0;
	*num_of_sat = 0;
    int week = 0;
    int num_of_obs = 0;
    int num_of_out = 0;
    int obs_nlock1  = MIN_NLOCK1;
    int obs_nlock2  = MIN_NLOCK2;
    if (rcv->tt > 1.0) obs_nlock2 = floor(obs_nlock2 / MIN_NLOCK2 + 0.5);
    if (rcv->np < NX_RTD) rcv->np = NX_RTD; /* make sure to hold at least RTD */
    double curTime = time2gpst(rov->time, &week);

    if (fabs(rcv->scales.medres[0]) > 300.0)
    {
        rcv->time = 0.0;
        rcv->nep = 0;
    }
    filter_RTD_update(rov, vec_rov, ref, vec_ref, rcv->x, rcv->P, &rcv->time, NX_RTK);
#if 0
	printf("rtk3:%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf\n",
		rcv->x[0], rcv->x[1], rcv->x[2], rcv->x[3], rcv->x[4], rcv->x[5]);
	printf("P:\r\n");
	for (i = 0; i < NX_RTK; i++)
	{
		printf("%14.4lf\r\n", rcv->P[SMI(i, i)]);
	}
	printf("\r\n");
#endif
    if ((*num_of_sat = robust_filter_RTD(ref, rov, vec_ref, vec_rov, iref, irov,
        nsd, rcv->x, rcv->P, rcv->np, rcv->ns, &num_of_obs, &num_of_out,
        dop, &rcv->scales, maskElev, rcv->coreff)) == 0)
    {
           rcv->nep = 0;
           reset_rcv(rcv);
           return 1;
    }
#if 0
	printf("rtk4:%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf\n",
		rcv->x[0], rcv->x[1], rcv->x[2], rcv->x[3], rcv->x[4], rcv->x[5]);
	printf("P:\r\n");
	for (i = 0; i < NX_RTK; i++)
	{
		printf("%14.4lf\r\n", rcv->P[SMI(i, i)]);
	}
	printf("\r\n");
#endif
    rcv->code_blunder_rate = (double)num_of_out / num_of_obs;

    measure_t meas[MAXOBS * NFREQ] = { 0 };
    int nm = 0;
    if (!robust_filter_RTK(curTime, rov, vec_rov, ref, vec_ref, irov, iref, 
		nsd, rcv->x, rcv->P, rcv->tag, rcv->slip, meas, rcv->np, &rcv->ns,
		&nm, &rcv->glo_ifb, maskElev, rcv->coreff))
    {
        memset(&rcv->ambset, 0, sizeof(ambset_t));
        memset( rcv->tag,    0, sizeof(state_tag_t)*MAXAMB);
        return 2;
    } 
#if 0
	printf("rtk5:%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf,%14.3lf\n",
		rcv->x[0], rcv->x[1], rcv->x[2], rcv->x[3], rcv->x[4], rcv->x[5]);
	printf("P:\r\n");
	for (i = 0; i < NX_RTK; i++)
	{
		printf("%14.4lf\r\n", rcv->P[SMI(i, i)]);
	}
	printf("\r\n");
#endif
    if (isSearchAMB == 1)
    {
        double xa[NX_RTK] = { 0.0 };
        double Pa[SMD(NX_RTK)] = { 0.0 };
        memcpy(xa, rcv->x, sizeof(double)*NX_RTK);
        memcpy(Pa, rcv->P, sizeof(double)*SMD(NX_RTK));
        int fixed_num = 0;

        /* check if previous epoch ambiguity is fixed or not */
        if (rcv->ambset.n >= MIN_AMBNUM)
        {
            fixed_num = fixed_amb_constraint(curTime, &rcv->ambset, rcv->tag, 
				rcv->slip, xa, Pa, rcv->np, rcv->ns, obs_nlock2);
            if (fixed_num >= MIN_AMBNUM && check_fixed_amb_valid(meas, nm, xa))
            {
                fixID = 4;
                fixID = ambres_state_vector(ref, rov, vec_ref, vec_rov, iref, 
					irov, nsd, rcv->x, rcv->P, rcv->tag, &rcv->np, &rcv->ns,
					obs_nlock1, obs_nlock2, xa, Pa, &rcv->ambset, rcv->slip);
                for (int i = 0; i < 6; ++i)
                {
                    xa[i] += rov->pos[i];
                }
                memcpy(rcv->x_fixed, xa, sizeof(xa));
                //memcpy(rcv->P, Pa, sizeof(Pa));
                return fixID;
            }
            else
            {
                fixID = 5;
            }
        }
        memcpy(xa, rcv->x, sizeof(double)*NX_RTK);
        fixID = ambres_state_vector(ref, rov, vec_ref, vec_rov, iref, irov, 
			nsd, rcv->x, rcv->P, rcv->tag, &rcv->np, &rcv->ns, obs_nlock1,
			obs_nlock2, xa, Pa, &rcv->ambset, rcv->slip);

        if (fixID == 4)
        {
            if (check_fixed_amb_valid(meas, nm, xa))
            {
                /* store the fixed solution */
                for (int i = 0; i < 6; ++i)
                {
                    xa[i] += rov->pos[i];
                }
                memcpy(rcv->x_fixed, xa, sizeof(xa));
            }
            else
            {
                //memset(rcv->tag, 0, sizeof(state_tag_t)*MAXAMB);
                //memset(&rcv->ambset, 0, sizeof(ambset_t));
                fixID = 5;
            }
        }
	}
	return fixID;
}

RTK_RAM_CODE int check_rtk_solu_valid(obs_t *rov, rcv_rtk_t *rcv, double *prev_pos,
	double *dop, int fixID, int num_sat)
{
    double detpos[3] = { 0.0 }, detpos_spp[3] = { 0.0 };
    double dpos = 0.0, dpos_spp = 0.0, dhpos = 0.0, dvpos = 0.0;
    int isReset = 0;
    int isValid = 1;

    if (rcv->P[SMI(0, 0)] < 0.0 || rcv->P[SMI(1, 1)] < 0.0 || rcv->P[SMI(2, 2)] < 0.0)
    {
        double pos_var = (SQR(rcv->P[SMI(0, 0)]) + SQR(rcv->P[SMI(1, 1)]) + SQR(rcv->P[SMI(2, 2)])) / 3.0;
        for (int i = 0; i < 3; i++)
        {
            rcv->P[SMI(i, i)] = pos_var;
        }
    }

    if (rcv->P[SMI(3, 3)] < 0.0 || rcv->P[SMI(4, 4)] < 0.0 || rcv->P[SMI(5, 5)] < 0.0)
    {
        double vel_var = (SQR(rcv->P[SMI(3, 3)]) + SQR(rcv->P[SMI(4, 4)]) + SQR(rcv->P[SMI(5, 5)])) / 3.0;
        for (int i = 0; i < 3; i++)
        {
            rcv->P[SMI(i+3, i+3)] = vel_var;
        }
    }

    if (fixID == 4)
    {
        rcv->num_goodepoch  = 0;
        rcv->num_unfixepoch = 0;
    }

    if (s_norm(prev_pos, 3) > 1.0 && rcv->tt != 0.0)
    {
        detpos[0] = rov->pos[0] - prev_pos[0];
        detpos[1] = rov->pos[1] - prev_pos[1];
        detpos[2] = rov->pos[2] - prev_pos[2];
        dpos = s_norm(detpos, 3);
    }

    detpos_spp[0] = rov->pos[0] - rcv->spp.x[0];
    detpos_spp[1] = rov->pos[1] - rcv->spp.x[1];
    detpos_spp[2] = rov->pos[2] - rcv->spp.x[2];
    dpos_spp = s_norm(detpos_spp,3);
    /* compute velocity */
    double vxyz[3] = { rcv->x[3], rcv->x[4], rcv->x[5] }, vned[3] = { 0.0 }, dposned[3] = { 0.0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    ecef2pos(rcv->x, blh);
    blh2C_en(blh, C_en);
    xyz2ned(C_en, detpos, NULL, dposned, NULL);
    dhpos = sqrt(dposned[0]* dposned[0] + dposned[1]* dposned[1]);
    dvpos = fabs(dposned[2]);
    double vel3D = sqrt(rcv->x[3] * rcv->x[3] + rcv->x[4] * rcv->x[4] + rcv->x[5] * rcv->x[5]);
    if (vel3D < 2*MIN_HOR_VEL)  vel3D = 2*MIN_HOR_VEL;
    double det_pos_thres = 3.0 * vel3D*rcv->tt;
    if (dpos_spp > 50.0 && fixID !=4 && rcv->nep<=5)                      isValid = 0;
    if (dhpos > MAX_HOR_VEL*rcv->tt)                                      isValid = 0;
    if (dvpos > MAX_VER_VEL*rcv->tt && fixID>=4)                          isValid = 0;
    if (dpos > det_pos_thres)                                             isValid = 0;
    if (dop[2] > MAX_HDOP || dop[3] > MAX_VDOP)                           isValid = 0;
    if (num_sat<=6)                                                       isValid = 0;
    if (fixID == 0)                                                       isValid = 0;
    return isValid;
}

RTK_RAM_CODE extern int timediff_code_filter(int mod, obs_t *obs_rov, obs_t *obs_ref, vec_t *vec_rov,
    vec_t *vec_ref, int *iref, int *irov, int nsd, rcv_tdp_t *rcv,double maskElev)
{
    int i, j, s, f, code, sys, isd, prn, sat, numl, week;
    double elev, z, z_, inov, R;
    double PHt[4]     = { 0.0 };
    double x_[4]      = { 0.0 };
    double P_[SMD(4)] = { 0.0 };
    double resL[MAXOBS]    = { 0 };
    double* H       = NULL;
    int*    L       = NULL;
    int max_st      = 4;
    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t  *pVecRef = NULL;
    vec_t  *pVecRov = NULL;
    measure_t meas[MAXOBS] = { 0 };
    int nm = 0;
    double codeCOV = 0.0, phaseCOV = 0.0;
    double omega0 = 0.0, omega1 = 0.0, rcv_bias = 0.0;
    rcv->np = 4;
    double cur_time1 = time2gpst(obs_rov->time, &week);
    double cur_time0 = time2gpst(obs_ref->time, &week);

    /* code measurement update */
    for (s = 0; s < NSYS; ++s)
    {
        if (s == 3)  continue;
        int satnum_sys = 0;
        for (isd = 0; isd < nsd; ++isd)
        {
            j = irov[isd];
            i = iref[isd];
            pObsRef = obs_ref->data + i;
            pObsRov = obs_rov->data + j;
            pVecRef = vec_ref + i;
            pVecRov = vec_rov + j;
            elev = pVecRov->azel[1];
            sat = pObsRov->sat;
            if (satidx(sat, &prn) != s) continue;
            sys = satsys(sat, &prn);
            if (elev < (maskElev*D2R)) continue;
            if (fabs(pObsRov->P[0]) < 0.001 || fabs(pObsRef->P[0]) < 0.001)
            {
                continue;
            }
            satnum_sys++;
        }
        if (satnum_sys < 3)  continue;
        for (f = 0; f < NFREQ; ++f)
        {
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                elev = pVecRov->azel[1];
                sat = pObsRov->sat;
                if (satidx(sat, &prn) != s) continue;
                sys = satsys(sat, &prn);
                if (elev < (maskElev*D2R)) continue;

                sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f], 1.0, &codeCOV, &phaseCOV);

                if (f == 1)       continue;
                if (fabs(pObsRov->P[f]) < 0.001 || fabs(pObsRef->P[f]) < 0.001)
                {
                    continue;
                }
                z = (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro));
                R = codeCOV;

                memset(meas + nm, 0, sizeof(measure_t));
                H = meas[nm].H + 0;
                L = meas[nm].L + 0;
                numl = 0;
                H[numl] = -pVecRov->e[0]; L[numl] = 0; ++numl;
                H[numl] = -pVecRov->e[1]; L[numl] = 1; ++numl;
                H[numl] = -pVecRov->e[2]; L[numl] = 2; ++numl;
                H[numl] = 1.0;            L[numl] = 3; ++numl; /* clock bias */
                meas[nm].numl = numl;
                meas[nm].z = z;
                meas[nm].R = R;
                meas[nm].sat = sat;
                meas[nm].prn = prn;
                meas[nm].code = pObsRov->code[f];
                meas[nm].sys = sys;
                meas[nm].elev = elev;
                meas[nm].azim = pVecRov->azel[0];
                meas[nm].SNR = pObsRov->SNR[f];
                resL[nm] = z;
                ++nm;
            }
        }
    }
    rcv_bias = median_dat(resL, nm);
    if (nm <= 4 || measure_update_kalman_filter(cur_time1, meas, nm, rcv->x, rcv->P, x_, P_, PHt, max_st, 'P') == 0)
    {
        /* restore the full position */
        nm = 0;
        for (i = 0; i < 3; ++i)
        {
            rcv->x[i] = obs_rov->pos[i];
        }
        return 5;
    }
    omega0 = 0.0;
    omega1 = 0.0;
    int obs_num = 0;
    for (i = 0; i < nm; ++i)
    {
        sat = meas[i].sat;
        prn = meas[i].prn;
        sys = meas[i].sys;
        elev = meas[i].elev;
        code = meas[i].code;
        if (meas[i].flag == 1) continue;
        numl = meas[i].numl;
        H = meas[i].H;
        L = meas[i].L;
        z = meas[i].z;
        z_ = 0.0;
        for (j = 0; j < numl; ++j)
        {
            z_ += H[j] * rcv->x[L[j]];
        }
        inov = z - z_;
        omega0 += SQR(z - rcv_bias);
        omega1 += SQR(inov);
        obs_num++;
    }

    omega0 = sqrt(omega0 / obs_num);
    omega1 = sqrt(omega1 / obs_num);
    comp_dop(meas, nm, rcv->dop);
    double bs[3] = { 0.0 };
    for (i = 0; i < 3; i++)
    {
        bs[i] = obs_rov->pos[i] + rcv->x[i] - obs_ref->pos[i];
    }

    rcv->code_num = obs_num;
    rcv->code_omg = omega1;

    return 4;
}

RTK_RAM_CODE extern int timediff_phase_filter(int mod, obs_t *obs_rov, obs_t *obs_ref, vec_t *vec_rov,
	vec_t *vec_ref, int *iref, int *irov, int nsd, rcv_tdp_t *rcv, double maskElev)
{
	int i, j, s, f, code, sys, isd, prn, sat, numl, week;
	double elev, z, z_, inov, R;
	double w = 0.0; /* wave */
	double PHt[4] = { 0.0 };
	double x_[4] = { 0.0 };
	double P_[SMD(4)] = { 0.0 };
	double* H = NULL;
	int*    L = NULL;
	int max_st = 4;
	obsd_t *pObsRef = NULL;
	obsd_t *pObsRov = NULL;
	vec_t  *pVecRef = NULL;
	vec_t  *pVecRov = NULL;
	measure_t meas[MAXOBS] = { 0 };
	double    resL[MAXOBS] = { 0 };
	int nm = 0;
	double codeCOV = 0.0, phaseCOV = 0.0;
	double omega0 = 0.0, omega1 = 0.0, rcv_bias = 0.0;
	double cur_time1 = time2gpst(obs_rov->time, &week);
	double cur_time0 = time2gpst(obs_ref->time, &week);

	/* phase measurement update */
	for (s = 0; s < NSYS; ++s)
	{
		if (s == 3)  continue;
		int satnum_sys = 0;
		for (isd = 0; isd < nsd; ++isd)
		{
			j = irov[isd];
			i = iref[isd];
			pObsRef = obs_ref->data + i;
			pObsRov = obs_rov->data + j;
			pVecRef = vec_ref + i;
			pVecRov = vec_rov + j;
			elev = pVecRov->azel[1];
			sat = pObsRov->sat;
			if (satidx(sat, &prn) != s) continue;
			sys = satsys(sat, &prn);
			if (elev < (maskElev*D2R)) continue;
			//int sat_id = find_sat_index(sat, slip);
			//if (sat_id == -1) continue;
			if (fabs(pObsRov->L[0]) < 0.001 || fabs(pObsRef->L[0]) < 0.001 /*|| slip[sat_id].slip[0] != 0*/)
			{
				continue;
			}
			satnum_sys++;
		}
		if (satnum_sys < 3)  continue;
		for (f = 0; f < NFREQ; ++f)
		{
			for (isd = 0; isd < nsd; ++isd)
			{
				j = irov[isd];
				i = iref[isd];
				pObsRef = obs_ref->data + i;
				pObsRov = obs_rov->data + j;
				pVecRef = vec_ref + i;
				pVecRov = vec_rov + j;
				elev = pVecRov->azel[1];
				sat = pObsRov->sat;
				if (satidx(sat, &prn) != s) continue;
				sys = satsys(sat, &prn);
				if (elev < (maskElev*D2R)) continue;
				w = satwavelen(sat, pObsRov->code[f]);
				if (w <= 0.0) continue;
				sdvarerr(prn, satsys(pObsRov->sat, &prn), elev, pObsRov->SNR[f], 1.0, &codeCOV, &phaseCOV);
				//int sat_id = find_sat_index(sat, slip);
				//if (sat_id == -1) continue;
				if (f == 1)       continue;
				if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001 /*|| slip[sat_id].slip[f] != 0*/)
				{
					continue;
				}
				z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
				R = phaseCOV / SQR(w);

				memset(meas + nm, 0, sizeof(measure_t));
				H = meas[nm].H + 0;
				L = meas[nm].L + 0;
				numl = 0;
				H[numl] = -pVecRov->e[0] / w; L[numl] = 0; ++numl;
				H[numl] = -pVecRov->e[1] / w; L[numl] = 1; ++numl;
				H[numl] = -pVecRov->e[2] / w; L[numl] = 2; ++numl;
				H[numl] = 1.0;                L[numl] = 3; ++numl;   /* clock bias */
				meas[nm].numl = numl;
				meas[nm].z = z;
				meas[nm].R = R;
				meas[nm].sat = sat;
				meas[nm].prn = prn;
				meas[nm].code = pObsRov->code[f];
				meas[nm].sys = sys;
				meas[nm].elev = elev;
				meas[nm].azim = pVecRov->azel[0];
				meas[nm].SNR = pObsRov->SNR[f];
				resL[nm] = z * w;
				++nm;
			}
		}
	}
	rcv_bias = median_dat(resL, nm);

	int obs_num = 0;
	if (mod == 1)
	{

		if (nm <= 4 || measure_update_kalman_filter(cur_time1, meas, nm, rcv->x, rcv->P, x_, P_, PHt, max_st, 'P') == 0)
		{
			/* restore the full position */
			nm = 0;
			for (i = 0; i < 3; ++i)
			{
				rcv->x[i] = obs_rov->pos[i];
			}
			return 5;
		}
		omega0 = 0.0;
		omega1 = 0.0;
		for (i = 0; i < nm; ++i)
		{
			sat = meas[i].sat;
			prn = meas[i].prn;
			sys = meas[i].sys;
			elev = meas[i].elev;
			code = meas[i].code;
			w = satwavelen(sat, code);
			if (meas[i].flag == 1) continue;
			numl = meas[i].numl;
			H = meas[i].H;
			L = meas[i].L;
			z = meas[i].z;
			z_ = 0.0;
			for (j = 0; j < numl; ++j)
			{
				z_ += H[j] * rcv->x[L[j]];
			}
			inov = z - z_;
			omega0 += SQR(z*w - rcv_bias);
			omega1 += SQR(inov*w);
			obs_num++;
		}

		omega0 = sqrt(omega0 / obs_num);
		omega1 = sqrt(omega1 / obs_num);
		comp_dop(meas, nm, rcv->dop);
		double bs[3] = { 0.0 };
		for (i = 0; i < 3; i++)
		{
			bs[i] = obs_rov->pos[i] + rcv->x[i] - obs_ref->pos[i];
		}
	}

	rcv->phas_num = obs_num;
	rcv->phas_omg = omega1;

	if (omega1 < 0.03 && nm > 6)
	{
		/* save the coordinate and velocity in the state vector */
		for (i = 0; i < 3; ++i)
		{
			rcv->x[i] += obs_rov->pos[i];
		}
		return 4;
	}
	else
		return 5;
}

RTK_RAM_CODE extern int timediff_processor(epoch_t *rov_epoch, epoch_t *rov_lastepoch,
	rcv_tdp_t *rcv)
{
	/* ref is last epoch rover data*/
	memset(rcv, 0, sizeof(rcv_tdp_t));
	obs_t* rov = &rov_epoch->obs;
	obs_t* ref = &rov_lastepoch->obs;
	vec_t* vec_rov = rov_epoch->vec + 0;
	vec_t* vec_ref = rov_lastepoch->vec + 0;
	int week = 0, num_of_sat = 0, n_bias = 0, cf_id, pf_id = 5;
	double time = 0.0, dop[5] = { 0.0 };
	double cur_res = 0.0, cdt_bias[MAXOBS] = { 0 };
	int iref[MAXOBS] = { 0 }, irov[MAXOBS] = { 0 }, nsd = 0, i = 0, j = 0, f = 0;

	time = time2gpst(rov->time, &week);
	double  cur_time = time;
	time += week * 7 * 24 * 3600.0;


	if (!check_coordinate(ref->pos))
	{
		return 1;
	}

	/* do not have enough satellites */
	if (rov->n < 4)
	{
		return 0;
	}

	if (fabs(time - rcv->time) < 0.001)
	{ /* already processed */
		return -1;
	}

	nsd = get_match_epoch(ref, rov, vec_ref, vec_rov, iref, irov);

	if (nsd <= 6)
	{
		return 1;
	}

	rcv->age = (float)timediff(rov->time, ref->time);

	for (j = 0; j < 3; ++j)
	{
		rcv->P[SMI(j, j)] = 100.0 * 100.0;
	}

	for (f = 0; f < NFREQ; ++f)
	{
		if (!(fabs(rov->data[i].P[f]) < 0.01 || fabs(ref->data[j].P[f]) < 0.01 ||
			fabs(vec_rov[i].r) < 0.01 || fabs(vec_ref[j].r) < 0.01))
		{
			cur_res = 0.0;
			cur_res += rov->data[i].P[f] - (vec_rov[i].r + vec_rov[i].tro);
			cur_res -= ref->data[j].P[f] - (vec_ref[j].r + vec_ref[j].tro);
			cdt_bias[n_bias] = cur_res;
			++(n_bias);
		}
	}

	rcv->x[3] = median_dat(cdt_bias, n_bias);
	rcv->P[SMI(3, 3)] = 1000.0 * 1000.0;
	rcv->time = time;

	cf_id = timediff_code_filter(1, rov, ref, vec_rov, vec_ref, iref, irov, nsd, rcv, 10.0);

	pf_id = timediff_phase_filter(1, rov, ref, vec_rov, vec_ref, iref, irov, nsd, rcv, 10.0);

	if (pf_id != 4)
	{
		num_of_sat = 0;
	}

	return pf_id;
}


RTK_RAM_CODE int adjust_glo_floatamb(rcv_rtk_t *rcv)
{
	unsigned int i;
	int sat[2] = { 0 }, prn[2] = { 0 }, s, frqnum[2] = { 0 };
	int np = rcv->np;
	for (i = 0; i < rcv->ns; i++)
	{
		sat[1] = rcv->tag[i].s2;
		sat[0] = rcv->tag[i].s1;
		if (sat[0] == 0 || sat[1] == 0) continue;
		s = satidx(sat[0], &prn[0]);
		if (s != 3)  continue;
		s = satidx(sat[1], &prn[1]);
		frqnum[0] = get_glo_frq(prn[0]);
		frqnum[1] = get_glo_frq(prn[1]);
		rcv->x[np + i] -= (frqnum[1] - frqnum[0])*rcv->glo_ifb.glo_ifb;
	}
	return 1;
}

RTK_RAM_CODE int glonass_rcv_bias_computation(rcv_rtk_t *rcv, obs_t *rov, obs_t *ref, vec_t *vec_rov, vec_t *vec_ref,
	int *iref, int *irov, int nsd)
{
	int i, j, f, isd, sat, prn, sys, week;
	double elev, w, sdN, cdt_bias, ddL;
	obsd_t *pObsRef = NULL;
	obsd_t *pObsRov = NULL;
	vec_t  *pVecRef = NULL;
	vec_t  *pVecRov = NULL;
	obsd_t *pObsRef_0 = NULL;
	obsd_t *pObsRov_0 = NULL;
	vec_t  *pVecRef_0 = NULL;
	vec_t  *pVecRov_0 = NULL;

	double sdL[10] = { 0.0 }, sdP[10] = { 0.0 }, wavelen[10] = { 0.0 }, glo_rcv_bias[10] = { 0.0 }, glo_ifb = 0.0;
	int    frqnum[10] = { 0 }, nobs = 0, nbias = 0;
	unsigned char glosat[10] = { 0 };

	double cur_time = time2gpst(rov->time, &week);
	for (f = 0; f < NFREQ; ++f)
	{
		int curLOC = MI(3, f, NFREQ);
		cdt_bias = (curLOC > 0) ? rcv->x[10] + rcv->x[10 + curLOC] : rcv->x[10];
		for (isd = 0; isd < nsd; ++isd)
		{
			j = irov[isd];
			i = iref[isd];
			pObsRef = ref->data + i;
			pObsRov = rov->data + j;
			pVecRef = vec_ref + i;
			pVecRov = vec_rov + j;
			elev = pVecRov->azel[1];
			sat = pObsRov->sat;
			sys = satsys(sat, &prn);
			if (sys != _SYS_GLO_) continue;
			w = satwavelen(sat, pObsRov->code[f]);
			if (w <= 0.0) continue;
			if (pObsRov->P[f] != 0.0 && pObsRef->P[f] != 0.0)
			{
				sdP[nobs] = (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r));
			}

			if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
			{
				glosat[nobs] = sat;
				sdL[nobs] = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r)) / w;
				frqnum[nobs] = get_glo_frq(prn);
				wavelen[nobs] = w;
				nobs++;
			}
		}
	}

	for (i = 0; i < nobs; i++)
	{
		for (j = i + 1; j < nobs; j++)
		{
			if (fabs(frqnum[j] - frqnum[i]) == 1)
			{
				double dwavelen = wavelen[j] - wavelen[i];
				if (sdP[j] != 0.0 && sdL[j] != 0.0)
				{
					sdN = adjust_sd_glo_obs(glosat[i], 0, sdL[i], sdP[i]);
				}
				else
				{
					sdN = (sdL[i] - cdt_bias) / wavelen[i];
				}
				ddL = (wavelen[j] * sdL[j] - wavelen[i] * sdL[i] + dwavelen * sdN) / wavelen[j];
				glo_rcv_bias[nbias] = (ddL - ROUND(ddL)) / (frqnum[j] - frqnum[i]);

				if (rcv->glo_ifb.ifb_num == 0)
				{
					rcv->glo_ifb.glo_ifb = glo_rcv_bias[nbias];
					rcv->glo_ifb.glo_ifb_var = 0.01;
					rcv->glo_ifb.ifb_num++;
				}
				else
				{
					if (rcv->glo_ifb.glo_ifb_var < 1.0e-4)  rcv->glo_ifb.glo_ifb_var = 1.0e-4;
					double inov = glo_rcv_bias[nbias] - rcv->glo_ifb.glo_ifb;
					if (fabs(inov) > 0.15 && rcv->glo_ifb.ifb_num > 5) continue;
					double P_inov = rcv->glo_ifb.glo_ifb_var + 0.01;
					rcv->glo_ifb.glo_ifb += rcv->glo_ifb.glo_ifb_var * inov / P_inov;
					rcv->glo_ifb.glo_ifb_var -= rcv->glo_ifb.glo_ifb_var * rcv->glo_ifb.glo_ifb_var / P_inov;
					rcv->glo_ifb.ifb_num++;
				}

				nbias++;
			}
		}
	}

	if (rcv->glo_ifb.ifb_num > 30)
	{
		//glo_ifb = median_dat(glo_rcv_bias, nbias);
		//adjust_glo_floatamb(rcv);
		rcv->glo_ifb.glo_ifbflag = 1;
	}
	return (nbias > 0) ? 1 : 0;
}

RTK_RAM_CODE extern int doppler_smoothed_code(epoch_t *rov_epoch, rcv_rtk_t *rcv)
{
	int ret = 0;
	int i, j, k, f, s, week, prn;
	double aveD = 0.0, lamd = 0.0, dt = 0.0;
	double last_time = fmod(rcv->time, 86400.0);
	double cur_time = fmod(time2gpst(rov_epoch->obs.time, &week), 86400.0);
	double vel = sqrt(SQR(rov_epoch->obs.pos[3]) + SQR(rov_epoch->obs.pos[4]) + SQR(rov_epoch->obs.pos[5]));
	double smcode_thres = 2.0;
	for (i = 0; i < MAXOBS; i++)
	{
		for (f = 0; f < NFREQ; f++)
		{
			if (rcv->sm_codeobs[i].time[f] == 0.0)
			{
				int idx2 = -1;
				if (f == 0)
				{
					for (j = 0; j < rov_epoch->obs.n; j++)
					{
						int idx = -1;
						for (k = 0; k < MAXOBS; k++)
						{
							if (rov_epoch->obs.data[j].sat == rcv->sm_codeobs[k].sat)
							{
								idx = 1;
								break;
							}
						}
						if (idx == -1)
						{
							idx2 = j;
							break;
						}
					}
				}
				else
				{
					for (j = 0; j < rov_epoch->obs.n; j++)
					{
						if (rov_epoch->obs.data[j].sat == rcv->sm_codeobs[i].sat)
						{
							idx2 = j;
							break;
						}
					}
				}

				s = satidx(rov_epoch->obs.data[idx2].sat, &prn);
				if (rov_epoch->obs.data[idx2].P[f] < 1.5e7) continue;
				if (s == 0 && prn > 32)                     continue;

				rcv->sm_codeobs[i].sat = rov_epoch->obs.data[idx2].sat;
				rcv->sm_codeobs[i].prn = prn;
				rcv->sm_codeobs[i].sys = s;
				rcv->sm_codeobs[i].sys = f;
				rcv->sm_codeobs[i].P[f] = rov_epoch->obs.data[idx2].P[f];
				rcv->sm_codeobs[i].D[f] = rov_epoch->obs.data[idx2].D[f];
				rcv->sm_codeobs[i].time[f] = cur_time;
			}
			else
			{
				int idx = -1;
				for (j = 0; j < rov_epoch->obs.n; j++)
				{
					if (rcv->sm_codeobs[i].sat == rov_epoch->obs.data[j].sat)
					{
						idx = j;
						break;
					}
				}
				s = satidx(rov_epoch->obs.data[idx].sat, &prn);
				if (rov_epoch->obs.data[idx].P[f] == 0) continue;
				if (s == 0 && prn > 32)                 continue;

				dt = cur_time - rcv->sm_codeobs[i].time[f];
				rcv->sm_codeobs[i].sat = rov_epoch->obs.data[idx].sat;
				rcv->sm_codeobs[i].prn = prn;
				rcv->sm_codeobs[i].sys = s;
				if (idx >= 0 && dt <= 1.0)
				{
					lamd = satwavelen(rcv->sm_codeobs[i].sat, rov_epoch->obs.data[idx].code[f]);
					if (rcv->sm_codeobs[i].D[f] != 0.0 && rov_epoch->obs.data[idx].D[f] != 0.0 && rov_epoch->obs.data[idx].P[f] > 1.5e7 && rov_epoch->obs.data[idx].P[f] < 3.0e7)
					{
						aveD = 0.5*(rcv->sm_codeobs[i].D[f] + rov_epoch->obs.data[idx].D[f])*dt*lamd;
						if (fabs(rcv->sm_codeobs[i].P[f] - aveD - rov_epoch->obs.data[idx].P[f]) < smcode_thres)
						{
							rcv->sm_codeobs[i].P[f] = 1 / 100.0*(rov_epoch->obs.data[idx].P[f])
								+ (1 - 1 / 100.0)*(rcv->sm_codeobs[i].P[f] - aveD);
							rcv->sm_codeobs[i].D[f] = rov_epoch->obs.data[idx].D[f];
							rcv->sm_codeobs[i].time[f] = cur_time;
						}
						else
						{
							rcv->sm_codeobs[i].P[f] = rov_epoch->obs.data[idx].P[f];
							rcv->sm_codeobs[i].D[f] = rov_epoch->obs.data[idx].D[f];
							rcv->sm_codeobs[i].time[f] = cur_time;
						}
					}
					else if (rov_epoch->obs.data[idx].P[f] > 1.5e7 && rov_epoch->obs.data[idx].P[f] < 3.0e7)
					{
						rcv->sm_codeobs[i].P[f] = rov_epoch->obs.data[idx].P[f];
						rcv->sm_codeobs[i].D[f] = rov_epoch->obs.data[idx].D[f];
						rcv->sm_codeobs[i].time[f] = cur_time;
					}
					else
					{
						rcv->sm_codeobs[i].sat = 0;
						rcv->sm_codeobs[i].prn = 0;
						rcv->sm_codeobs[i].sys = 0;
						rcv->sm_codeobs[i].P[0] = 0.0;
						rcv->sm_codeobs[i].D[0] = 0.0;
						rcv->sm_codeobs[i].time[0] = 0.0;
						rcv->sm_codeobs[i].var[0] = 0.0;
						rcv->sm_codeobs[i].P[1] = 0.0;
						rcv->sm_codeobs[i].D[1] = 0.0;
						rcv->sm_codeobs[i].time[1] = 0.0;
						rcv->sm_codeobs[i].var[1] = 0.0;
					}
				}
				else
				{
					rcv->sm_codeobs[i].sat = 0;
					rcv->sm_codeobs[i].prn = 0;
					rcv->sm_codeobs[i].sys = 0;
					rcv->sm_codeobs[i].P[0] = 0.0;
					rcv->sm_codeobs[i].D[0] = 0.0;
					rcv->sm_codeobs[i].time[0] = 0.0;
					rcv->sm_codeobs[i].var[0] = 0.0;
					rcv->sm_codeobs[i].P[1] = 0.0;
					rcv->sm_codeobs[i].D[1] = 0.0;
					rcv->sm_codeobs[i].time[1] = 0.0;
					rcv->sm_codeobs[i].var[1] = 0.0;
				}
			}
		}

		if (fabs(cur_time - ROUND(cur_time)) < 0.01 && rcv->sm_codeobs[i].sys < 4 && rcv->sm_codeobs[i].sat >0)
		{
			int idx2 = -1;
			for (j = 0; j < rov_epoch->obs.n; j++)
			{
				if (rov_epoch->obs.data[j].sat == rcv->sm_codeobs[i].sat)
				{
					idx2 = j;
					break;
				}
			}
			s = satidx(rov_epoch->obs.data[idx2].sat, &prn);
			if (rov_epoch->obs.data[idx2].P[f] == 0) continue;
			if (s == 0 && prn > 32)                  continue;
			if (rov_epoch->obs.data[idx2].P[1] == 0.0)
				rcv->sm_codeobs[i].P[1] = 0.0;

			//double obsdif[2] = { 0.0 };
			//if (rcv->sm_codeobs[i].P[0] > 1.5e7 && rov_epoch->obs.data[idx2].P[0] > 1.5e7)
			//{
			//    obsdif[0] = rcv->sm_codeobs[i].P[0] - rov_epoch->obs.data[idx2].P[0];
			//}
			//if (rcv->sm_codeobs[i].P[1] > 1.5e7 && rov_epoch->obs.data[idx2].P[1] > 1.5e7)
			//{
			//    obsdif[1] = rcv->sm_codeobs[i].P[1] - rov_epoch->obs.data[idx2].P[1];
			//}
			//double lamd1 = satwavelen(rcv->sm_codeobs[i].sat, rov_epoch->obs.data[idx2].code[0]);
			//double lamd2 = satwavelen(rcv->sm_codeobs[i].sat, rov_epoch->obs.data[idx2].code[1]);
			//printf("smobs_code,%10.2f,%3i,%3i,%14.3f,%14.3f,%14.3f,%14.3f,%10.3f,%10.3f,%10.3f,%10.3f\n"
			//    , cur_time, rcv->sm_codeobs[i].sys, rcv->sm_codeobs[i].prn
			//    , rcv->sm_codeobs[i].P[0], rov_epoch->obs.data[idx2].P[0]
			//    , rcv->sm_codeobs[i].P[1], rov_epoch->obs.data[idx2].P[1]
			//    , obsdif[0], obsdif[1]
			//    , rov_epoch->obs.data[idx2].D[0] * lamd1, rov_epoch->obs.data[idx2].D[1] * lamd2);
		}
	}
	return ret;
}

RTK_RAM_CODE extern void use_sm_codeobs(epoch_t *rov_epoch, rcv_rtk_t *rcv)
{
	int i, j;
	for (i = 0; i < rov_epoch->obs.n; i++)
	{
		for (j = 0; j < MAXOBS; j++)
		{
			if (rov_epoch->obs.data[i].sat == rcv->sm_codeobs[j].sat)
			{
				rov_epoch->obs.data[i].P[0] = rcv->sm_codeobs[j].P[0];
				rov_epoch->obs.data[i].P[1] = rcv->sm_codeobs[j].P[1];
			}
		}
	}
}

RTK_RAM_CODE extern int rtk_processor(epoch_t *rov_epoch, epoch_t *ref_epoch, rcv_rtk_t *rcv)
{
	obs_t* rov = &rov_epoch->obs;
	obs_t* ref = &ref_epoch->obs;
	vec_t* vec_rov = rov_epoch->vec + 0;
	vec_t* vec_ref = ref_epoch->vec + 0;
	int iref[MAXOBS] = { 0 }, irov[MAXOBS] = { 0 }, nsd = 0, i = 0, j = 0;
	int num_of_sat = 0, fixID = 5, week = 0, isSearchAMB = 1;
	double time = 0.0, dop[5] = { 0.0 }, prev_pos[3] = { 0.0 };

	if (rov->n < MIN_OBS || !check_coordinate(rov->pos))
	{   /* do not have enough satellites */
		rcv->nep = 0;
		return 0;
	}

	if (!check_coordinate(ref->pos))
	{
		return 1;
	}

	time = time2gpst(rov->time, &week);
	time += (double)week * SECONDS_IN_WEEK;

	if (fabs(time - rcv->time) < 0.001)
	{ /* already processed */
		return -1;
	}

	nsd = get_match_epoch(ref, rov, vec_ref, vec_rov, iref, irov);


	if (nsd <= MIN_SD_OBS)
	{
		rcv->nep = 0;
		return 1;
	}

	rcv->age = (float)timediff(rov->time, ref->time);
	rcv->tt = rcv->time > 0.0 ? time - rcv->time : 0.0;

	if (rcv->age > MAX_REFAGE) 
    {
		return 1;
	}

	if (rcv->time != 0.0)
	{
		detslp_rov(rov, rcv->slip, time - rcv->time);
		detslp_ref(ref, rcv->slip);
		/* set nlock =0 in ambset if cycleslip is detected*/
		ambset_cycslip(rcv->slip, &rcv->ambset);
	}

	if (s_norm(rcv->x, 3) > 0.1)
	{
		prev_pos[0] = rcv->x[0];
		prev_pos[1] = rcv->x[1];
		prev_pos[2] = rcv->x[2];
	}

	fixID = rtk_filter(rov, ref, vec_rov, vec_ref, iref, irov, nsd, rcv,
		&num_of_sat, dop, 10.0, isSearchAMB);

	rcv->fixType = fixID;
	if (fixID > 1)
	{
		/* save the coordinate and velocity in the state vector */
		for (i = 0; i < 6; ++i)
		{
			rov->pos[i] += rcv->x[i];
			rcv->x[i] = rov->pos[i];
		}
		rcv->num_of_sat = num_of_sat;
		memcpy(rcv->dop, dop, sizeof(dop));
	}
	else
	{
		for (i = 0; i < 3; ++i)
		{
			rov->pos[i] += rcv->spp.x[i];
			rcv->x[i] = rov->pos[i];
		}
		rcv->num_of_sat = rcv->spp.n_used;
	}

	//printf("amb_Res\n");
	//print_rtk_info(rov, rcv);

	compute_vector_data(rov, vec_rov);

#ifdef GLO_AR
	if (rcv->fixType == 4)
	{
		glonass_rcv_bias_computation(rcv, rov, ref, vec_rov, vec_ref, iref, irov, nsd);
	}
#endif

	if (check_rtk_solu_valid(rov, rcv, prev_pos, dop, fixID, num_of_sat) != 1)
	{
		/* need to define what will be output if this is failed */
		/* I will set to 0, then output */
		rcv->nep = 0;
		num_of_sat = 0;
		fixID = 1;
	}

	//print_rtk_info(rov, rcv);

	return fixID;
}
