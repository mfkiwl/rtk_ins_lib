#include "gnss_filter.h"
#include <math.h>
#include <string.h>
#include "rtcm.h"
#include "rtk_math.h"
#include "rtklib_core.h"

RTK_ITCM_CODE extern double median_measure(measure_t* dat, int n)
{
    int		 i, j, loc;
    measure_t temp = { 0 };

    if (n == 0 || dat == NULL) return 0.0;
    /* sort */
    for (i = 0; i < n; ++i) {
        for (j = i + 1; j < n; ++j) {
            if (fabs(dat[i].z - dat[i].z_) > fabs(dat[j].z - dat[j].z_)) {
                temp = dat[i];
                dat[i] = dat[j];
                dat[j] = temp;
            }
        }
    }
    loc = n / 2;
    if (n % 2 == 0)
    {
        double res[MAXOBS * 2] = { 0 };
        for (i = 0; i < n; ++i)
            res[i] = dat[i].z - dat[i].z_;
        double m1 = dat[loc - 1].z - dat[loc - 1].z_;
        double m2 = dat[loc - 0].z - dat[loc - 0].z_;
        double std1 = std_dat1(res, m1, n);
        double std2 = std_dat1(res, m2, n);
        if (std1 < std2)
            return m1;
        else
            return m2;
    }
    else
    {
        return dat[loc].z - dat[loc].z_;
    }
}

RTK_ITCM_CODE extern void ekf_measurement_predict(double *x, double *P, const double *H, const int *L, double *z_, double *R_, int n, int m, double *PHt)
{
    /* kalman filter measurement update for single measurement */
    /* n: number of state vector */
    /* m: number of measurement index */
    int i, j;
    /* PHt = P*H' */
    if (PHt != NULL)
    {
        for (i = 0; i < n; ++i)
        {
            PHt[i] = 0.0;
            for (j = 0; j < m; ++j)
            {
                PHt[i] += P[SMI(i, L[j])] * H[j];
            }
        }
        /* R_ = H*P*H' */
        if (R_ != NULL)
        {
            *R_ = 0.0;
            for (i = 0; i < m; ++i)
            {
                *R_ += H[i] * PHt[L[i]];
            }
        }
    }
    /* z_ = H*x */
    *z_ = 0.0;
    for (i = 0; i < m; ++i)
    {
        *z_ += H[i] * x[L[i]];
    }
    return;
}

RTK_ITCM_CODE extern void compute_Qvv(double *P, const double *H, const int *L, double *R_, int n, int m, double *PHt)
{
    /* kalman filter measurement update for single measurement */
    /* n: number of state vector */
    /* m: number of measurement index */
    int i, j;
    /* PHt = P*H' */
    if (PHt != NULL)
    {
        for (i = 0; i < n; ++i)
        {
            PHt[i] = 0.0;
            for (j = 0; j < m; ++j)
            {
                PHt[i] += P[SMI(i, L[j])] * H[j];
            }
        }
        /* R_ = H*P*H' */
        if (R_ != NULL)
        {
            *R_ = 0.0;
            for (i = 0; i < m; ++i)
            {
                *R_ += H[i] * PHt[L[i]];
            }
        }
    }
    return;
}

RTK_ITCM_CODE extern void ekf_measurement_update(double *x, double *P, const double *H, const int *L, double inov, double P_inov, int n, int m, double *PHt)
{
    /* kalman filter measurement update for single measurement */
    /* n: number of state vector */
    /* m: number of measurement index */
    int i, j;
    /* P_inov = R + H*P*H' */
    /* inov = z-H*x */
    /* x = x + K * inov = x + P*H'*inv(P_inov)*inov */
    for (i = 0; i < n; ++i)
    {
        x[i] += PHt[i] * inov / P_inov;
    }
    /* P = P - (P*H')*inv(P_inov)*(H*P) */
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j <= i; ++j)
        {
            P[SMI(i, j)] -= PHt[i] * PHt[j] / P_inov;
        }  
    }
    return;
}

/* Robust kalman filter measurement update for all vector */
RTK_ITCM_CODE extern int measure_update_kalman_filter(double time, measure_t* meas, int nm, 
	double* x_, double* P_, double *x, double *P, double *PHt, int max_st,
	char type)
{
	int i, j, sys, sat, prn, code, numl, SNR = 0;
	double omega = 0.0, elev = 0.0, z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0;
	double inov = 0.0, P_inov = 0.0, wsf = 0.0, bias = 0.0, refVar = 0.0;
	double* H = NULL;
	int*    L = NULL;
	int num_of_out = 0;
	int num_of_obs = 0;
	int num_of_check = 0;
	int idxIter = 0, maxLOC = -1;
	double maxRES = 0.0;
    int isPassed = 0, cov_flag = 0;
    int maxIter = (int)floor(0.9*nm + 0.5);
    double code_thres = 3.0, dopp_thres = 0.3, var_thres=3.0;

	while (idxIter <= maxIter)
	{
        memcpy(x, x_, sizeof(double) * max_st);
		memcpy(P, P_, sizeof(double) * SMD(max_st));
		for (i = 0; i < nm; ++i)
		{
			sat = meas[i].sat;
			prn = meas[i].prn;
			sys = meas[i].sys;
			elev = meas[i].elev;
			code = meas[i].code;
			numl = meas[i].numl;
			H = meas[i].H;
			L = meas[i].L;
			z = meas[i].z;
			R = meas[i].R;
			SNR = meas[i].SNR;
			ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
			inov = z - z_;
			P_inov = R + R_;
			if (meas[i].flag == 0)     /* not marked as outlier */
			{
				ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
			}
		}
		omega = 0.0;
		maxLOC = -1;
		maxRES = 0.0;
		num_of_check = 0;
		num_of_out = 0;
		for (i = 0; i < nm; ++i)
		{
			sat = meas[i].sat;
			prn = meas[i].prn;
			sys = meas[i].sys;
			elev = meas[i].elev;
			code = meas[i].code;
			numl = meas[i].numl;
			H = meas[i].H;
			L = meas[i].L;
			z = meas[i].z;
			R = meas[i].R;
			SNR = meas[i].SNR;
            z_ = 0.0;
            for (j = 0; j < numl; ++j)
            {
                z_ += H[j] * x[L[j]];
            }
            inov = z - z_;
            wsf = inov * inov / R;
			if (wsf > 9.0)
			{
				++num_of_check;
			}
			if (meas[i].flag == 1)
			{
				if (wsf < 9.0)
				{
					omega += wsf;
                    meas[i].flag = 0;
				}
				else
				{
					++num_of_out;
				}
			}
			else
			{
				omega += wsf;
				if (maxLOC<0 || wsf>maxRES)
				{
					maxLOC = i;
					maxRES = wsf;
				}
			}
		}

		if (num_of_check > (0.35 * nm) && idxIter> floor(maxIter*0.3+0.5) && type!='L')
		{
			/* need to scale prior */
            for (i = 0; i < nm; ++i)
            {
                meas[i].R *= 1.1;
            }
		}

		if ((nm - num_of_out) <= 4)
		{
			/* failed */
			num_of_obs = 0;
			break;
		}

		refVar = sqrt(omega / ((nm - num_of_out) * 1.0));
		if ((cov_flag >=2 && type == 'P') || (refVar < 3.0 && type != 'P'))
		{
			num_of_obs = nm - num_of_out;
            isPassed = 1;
			break;
		}
		if (refVar < var_thres)	++cov_flag;
		/* remove outlier */
		if (maxLOC < 0)
		{
			/* failed */
			num_of_obs = 0;
			break;
		}
		/* mark outlier */
		if(cov_flag == 0)	meas[maxLOC].flag = 1;
		++idxIter;
	}

    if (num_of_obs<=0) return num_of_obs;

    int isBlunder = 0;
    for (i = 0; i < nm; ++i)
    {
        sat = meas[i].sat;
        prn = meas[i].prn;
        sys = meas[i].sys;
        elev = meas[i].elev;
		code = meas[i].code;
        numl = meas[i].numl;
        H = meas[i].H;
        L = meas[i].L;
        z = meas[i].z;
        R = meas[i].R;
        z_ = 0.0;
        for (j = 0; j < numl; ++j)
        {
            z_ += H[j] * x[L[j]];
        }
        inov = z - z_;
        if (type == 'L' && fabs(inov) > 1.0 && num_of_obs > 0 && meas[i].refsat==0)
        {
            isBlunder = 1;
            meas[i].flag = 1;
            num_of_obs--;
        }
        else if (type == 'P' && fabs(inov) > code_thres &&  meas[i].flag ==0)
        {
            isBlunder = 1;
            meas[i].flag = 1;
            num_of_obs--;
        }
        else if (type == 'D' && fabs(inov) > dopp_thres &&  meas[i].flag == 0)
        {
            isBlunder = 1;
            meas[i].flag = 1;
            num_of_obs--;
        }
    }

    if (num_of_obs <= 10 && type != 'L')
    {
        isBlunder = 0;
        for (i = 0; i < nm; ++i)
        {
            if (meas[i].flag == 1)
            {
                num_of_obs++;
            }
        }
    }

    if (isBlunder)
    {
        memcpy(x, x_, sizeof(double) * max_st);
        memcpy(P, P_, sizeof(double) * SMD(max_st));
        for (i = 0; i < nm; ++i)
        {
            sat = meas[i].sat;
            prn = meas[i].prn;
            sys = meas[i].sys;
            elev = meas[i].elev;
            code = meas[i].code;
            numl = meas[i].numl;
            H = meas[i].H;
            L = meas[i].L;
            z = meas[i].z;
            R = meas[i].R;
            if (meas[i].flag > 0)
            {
                if (type == 'L')
                {
                    for (j = 0; j < max_st; j++)
                    {
                        P[SMI(L[numl - 2], j)] = 0.0;
                    }
                    P[SMI(L[numl - 2], L[numl - 2])] += 1.0e4;
                    meas[i].flag = 0;
                }
                else if (type == 'P' || type == 'D')
                {
                    continue;
                }
            }
            ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
            inov = z - z_;
            P_inov = R + R_;
            ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
        }
    }

    double pos_det = s_norm(x, 3);
    if (isPassed == 1 && num_of_obs > 4)
    {
        if (type == 'L' && pos_det < 3.0)
        {
            memcpy(x_, x, sizeof(double) * max_st);
            memcpy(P_, P, sizeof(double) * SMD(max_st));
        }
        else if (type != 'L')
        {
            memcpy(x_, x, sizeof(double) * max_st);
            memcpy(P_, P, sizeof(double) * SMD(max_st));
        }
    }
    else
        num_of_obs = 0;

    return num_of_obs;
}
RTK_ITCM_CODE extern int week_number(double sec)
{
	return (int)floor(sec / (7 * 24 * 3600.0));
}
RTK_ITCM_CODE extern double week_second(double sec)
{
	return sec - week_number(sec) * (7 * 24 * 3600.0);
}


/* functions related to float ambiguity filter */
RTK_ITCM_CODE extern int find_next_state_index(state_tag_t* tag, int np, int ns, int max_st)
{
	int i = 0;
	for (i = 0; i < ns; ++i)
	{
		if (tag[i].time == 0.0 && tag[i].s1 == 0 && tag[i].s2 == 0 && tag[i].f == 0)
		{
			break;
		}
	}
	if (i == ns)
	{
		if ((np + ns) >= max_st)
			i = -1;
	}
	return i;
}

RTK_ITCM_CODE extern int find_state_index(state_tag_t* tag, int ns, int s1, int s2, int f)
{
	int i = 0;
	for (i = 0; i < ns; ++i)
	{
		if (tag[i].s1 == s1 && tag[i].s2 == s2 && tag[i].f == f)
		{
			break;
		}
	}
	if (i == ns)
		i = -1;
	return i;
}

