/*------------------------------------------------------------------------------
* ephemeris.c : satellite ephemeris and clock functions
*-----------------------------------------------------------------------------*/
#include "rtcm.h"
#include "gnss_datatype.h"
#include "ephemeris.h"
#include "rtk_math.h"
#include "model.h"
#include <math.h>
#include "rtklib_core.h"

/* constants and macros ------------------------------------------------------*/
#define MAXDTOE     7200.0        /* max time difference to GPS Toe (s) */
#define MAXDTOE_QZS 7200.0        /* max time difference to QZSS Toe (s) */
#define MAXDTOE_GAL 14400.0       /* max time difference to Galileo Toe (s) */
#define MAXDTOE_CMP 21600.0       /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_GLO 1800.0        /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_SBS 360.0         /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0       /* max time difference to ephem toe (s) for other */
#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */
#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */
#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */
#define ERREPH_GLO 5.0            /* error of glonass ephemeris (m) */
#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-13         /* relative tolerance for Kepler equation */
#define DEFURASSR 0.15            /* default accurary of ssr corr (m) */
#define MAXECORSSR 90.0           /* max orbit correction of ssr (m) */
#define MAXCCORSSR (1E-6*CLIGHT)  /* max clock correction of ssr (m) */
#define MAXAGESSR    90.0         /* max age of ssr orbit and clock (s) */
#define MAXAGESSR_HRCLK 10.0      /* max age of ssr high-rate clock (s) */
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
#define STD_GAL_NAPA 500.0        /* error of galileo ephemeris for NAPA (m) */
#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler */

/* ephemeris selections ------------------------------------------------------*/
RTK_DTCM_DATA static int eph_sel[]={ /* GPS,GLO,GAL,QZS,BDS,SBS */
//    0,0,1,0,0,0
	0,0,0,0,0,0
};

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE static double dot_(const double *a, const double *b, int n)
{
    double c = 0.0;

	while (--n >= 0)
	{
		c += a[n] * b[n];
	}

    return c;
}

/* variance by ura ephemeris -------------------------------------------------*/
RTK_RAM_CODE static double var_uraeph(int sys, int ura)
{
    const double ura_value[] = 
	{   
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0
    };
    if (sys==_SYS_GAL_) 
	{ /* galileo sisa (ref [7] 5.1.11) */
        if (ura<= 49) return SQR(ura*0.01);
        if (ura<= 74) return SQR(0.5+(ura- 50)*0.02);
        if (ura<= 99) return SQR(1.0+(ura- 75)*0.04);
        if (ura<=125) return SQR(2.0+(ura-100)*0.16);
        return SQR(STD_GAL_NAPA);
    }
    else 
	{ /* gps ura (ref [1] 20.3.3.3.1.1) */
        return ura<0||15<ura?SQR(6144.0):SQR(ura_value[ura]);
    }
}

/* variance by ura ssr (ref [4]) ---------------------------------------------*/
RTK_RAM_CODE static double var_urassr(int ura)
{
	double std;

	if (ura <= 0)
	{
		return SQR(DEFURASSR);
	}
	if (ura >= 63)
	{
		return SQR(5.4665);
	}
	std = (pow(3.0, (ura >> 3) & 7)*(1.0 + (ura & 7) / 4.0) - 1.0)*1E-3;

	return SQR(std);
}

/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativeity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern double eph2clk(gtime_t time, const eph_t *eph)
{
	double t;
	int i, prn, sys = satsys(eph->sat, &prn);

	t = timediff(time, eph->toc);

	for (i = 0; i < 2; i++) 
	{
		t -= eph->f0 + eph->f1*t + eph->f2*t*t;
	}
	
	return eph->f0 + eph->f1*t + eph->f2*t*t;
}

/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
	double *var)
{
	double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
	double xg, yg, zg, sino, coso;
	int n, prn, sys = satsys(eph->sat, &prn);

	if (eph->A <= 0.0)
	{
		rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
		return;
	}
	tk = timediff(time, eph->toe);

	switch ((sys = satsys(eph->sat, &prn))) {
	case _SYS_GAL_: mu = MU_GAL; omge = OMGE_GAL; break;
	case _SYS_BDS_: mu = MU_CMP; omge = OMGE_CMP; break;
	default:        mu = MU_GPS; omge = OMGE;     break;
	}
	M = eph->M0 + (sqrt(mu / (eph->A*eph->A*eph->A)) + eph->deln)*tk;

	for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER&&n < MAX_ITER_KEPLER; n++)
	{
		Ek = E;
		E -= (E - eph->e*sin(E) - M) / (1.0 - eph->e*cos(E));
	}
	if (n >= MAX_ITER_KEPLER)
	{
		return;
	}
	sinE = sin(E);
	cosE = cos(E);

	u = atan2(sqrt(1.0 - eph->e*eph->e)*sinE, cosE - eph->e) + eph->omg;
	r = eph->A*(1.0 - eph->e*cosE);
	i = eph->i0 + eph->idot*tk;
	sin2u = sin(2.0*u);
	cos2u = cos(2.0*u);
	u += eph->cus*sin2u + eph->cuc*cos2u;
	r += eph->crs*sin2u + eph->crc*cos2u;
	i += eph->cis*sin2u + eph->cic*cos2u;
	x = r * cos(u);
	y = r * sin(u);
	cosi = cos(i);

	/* beidou geo satellite */
	if (sys == _SYS_BDS_ && (eph->flag == 2 || (eph->flag == 0 && prn <= 5)))
	{
		O = eph->OMG0 + eph->OMGd*tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		xg = x * cosO - y * cosi*sinO;
		yg = x * sinO + y * cosi*cosO;
		zg = y * sin(i);
		sino = sin(omge*tk); coso = cos(omge*tk);
		rs[0] = xg * coso + yg * sino*COS_5 + zg * sino*SIN_5;
		rs[1] = -xg * sino + yg * coso*COS_5 + zg * coso*SIN_5;
		rs[2] = -yg * SIN_5 + zg * COS_5;
	}
	else
	{
		O = eph->OMG0 + (eph->OMGd - omge)*tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		rs[0] = x * cosO - y * cosi*sinO;
		rs[1] = x * sinO + y * cosi*cosO;
		rs[2] = y * sin(i);
	}
	tk = timediff(time, eph->toc);
	*dts = eph->f0 + eph->f1*tk + eph->f2*tk*tk;

	/* relativity correction */
	*dts -= 2.0*sqrt(mu*eph->A)*eph->e*sinE / SQR(CLIGHT);

	/* position and clock error variance */
	*var = var_uraeph(sys, eph->sva);
}

/* glonass orbit differential equations --------------------------------------*/
RTK_RAM_CODE static void deq(const double *x, double *xdot, const double *acc)
{
    double a,b,c,r2=dot_(x,x,3),r3=r2*sqrt(r2),omg2=SQR(OMGE_GLO);
    
    if (r2<=0.0) 
	{
        xdot[0]=xdot[1]=xdot[2]=xdot[3]=xdot[4]=xdot[5]=0.0;
        return;
    }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a=1.5*J2_GLO*MU_GLO*SQR(RE_GLO)/r2/r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b=5.0*x[2]*x[2]/r2;                    /* 5*z^2/r^2 */
    c=-MU_GLO/r3-a*(1.0-b);                /* -mu/r^3-a(1-b) */
    xdot[0]=x[3]; 
	xdot[1]=x[4]; 
	xdot[2]=x[5];
    xdot[3]=(c+omg2)*x[0]+2.0*OMGE_GLO*x[4]+acc[0];
    xdot[4]=(c+omg2)*x[1]-2.0*OMGE_GLO*x[3]+acc[1];
    xdot[5]=(c-2.0*a)*x[2]+acc[2];
}

/* glonass position and velocity by numerical integration --------------------*/
RTK_RAM_CODE static void glorbit(double t, double *x, const double *acc)
{
	double k1[6], k2[6], k3[6], k4[6], w[6];
	int i;

	deq(x, k1, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0;
	deq(w, k2, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0;
	deq(w, k3, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;
	deq(w, k4, acc);
	for (i = 0; i < 6; i++)
	{
		x[i] += (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])*t / 6.0;
	}
}

/* glonass ephemeris to satellite clock bias -----------------------------------
* compute satellite clock bias with glonass ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          geph_t *geph     I   glonass ephemeris
* return : satellite clock bias (s)
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern double geph2clk(gtime_t time, const geph_t *geph)
{
	double t;
	int i, prn, sys = satsys(geph->sat, &prn);

	t = timediff(time, geph->toe);

	for (i = 0; i < 2; i++)
	{
		t -= -geph->taun + geph->gamn*t;
	}

	return -geph->taun + geph->gamn*t;
}
/* glonass ephemeris to satellite position and clock bias ----------------------
* compute satellite position and clock bias with glonass ephemeris
* args   : gtime_t time     I   time (gpst)
*          geph_t *geph     I   glonass ephemeris
*          double *rs       O   satellite position {x,y,z} (ecef) (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
	double *var)
{
	double t, tt, x[6];
	int i, prn, sys = satsys(geph->sat, &prn);

	t = timediff(time, geph->toe);

	*dts = -geph->taun + geph->gamn*t;

	for (i = 0; i < 3; i++)
	{
		x[i] = geph->pos[i];
		x[i + 3] = geph->vel[i];
	}
	for (tt = t < 0.0 ? -TSTEP : TSTEP; fabs(t) > 1E-9; t -= tt)
	{
		if (fabs(t) < TSTEP)
		{
			tt = t;
		}
		glorbit(tt, x, geph->acc);
	}
	for (i = 0; i < 3; i++)
	{
		rs[i] = x[i];
	}

	*var = SQR(ERREPH_GLO);
}

/* select ephememeris --------------------------------------------------------*/
RTK_RAM_CODE static const eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav)
{
	double t, tmax, tmin;
	int i, j = -1, prn, sys = satsys(sat, &prn), sel = 0;

	sys = satsys(sat, NULL);
	switch (sys) {
	case _SYS_GPS_: tmax = MAXDTOE + 1.0; sel = eph_sel[0]; break;
	case _SYS_GAL_: tmax = MAXDTOE_GAL; sel = eph_sel[2]; break;
	case _SYS_QZS_: tmax = MAXDTOE_QZS + 1.0; sel = eph_sel[3]; break;
	case _SYS_BDS_: tmax = MAXDTOE_CMP + 1.0; sel = eph_sel[4]; break;
	default: tmax = MAXDTOE + 1.0; break;
	}
	tmin = tmax + 1.0;

	for (i = 0; i < (int)nav->n; i++)
	{
		if (nav->eph[i].sat != sat)
		{
			continue;
		}
		if (iode >= 0 && nav->eph[i].iode != iode)
		{
			continue;
		}
		if (sys == _SYS_GAL_ && sel)
		{
			if (sel == 1 && !(nav->eph[i].code&(1 << 9)))
			{
				continue; /* I/NAV */
			}
			if (sel == 2 && !(nav->eph[i].code&(1 << 8)))
			{
				continue; /* F/NAV */
			}
		}
		if ((t = fabs(timediff(nav->eph[i].toe, time))) > tmax)
		{
			continue;
		}
		if (iode >= 0)
		{
			return nav->eph + i;
		}
		if (t <= tmin)
		{
			j = i;
			tmin = t;
		} /* toe closest to time */
	}
	if (iode >= 0 || j < 0)
	{
		return NULL;
	}

	return nav->eph + j;
}

/* select glonass ephememeris ------------------------------------------------*/
RTK_RAM_CODE static const geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav)
{
	double t, tmax = MAXDTOE_GLO, tmin = tmax + 1.0;
	int i, j = -1, prn, sys = satsys(sat, &prn);

	for (i = 0; i < (int)nav->ng; i++)
	{
		if (nav->geph[i].sat != sat)
		{
			continue;
		}
		if (iode >= 0 && nav->geph[i].iode != iode)
		{
			continue;
		}
		if ((t = fabs(timediff(nav->geph[i].toe, time))) > tmax)
		{
			continue;
		}
		if (iode >= 0)
		{
			return nav->geph + i;
		}
		if (t <= tmin)
		{
			j = i;
			tmin = t;
		} /* toe closest to time */
	}

	if (iode >= 0 || j < 0)
	{
		return NULL;
	}

	return nav->geph + j;
}

/* satellite clock with broadcast ephemeris ----------------------------------*/
RTK_RAM_CODE static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
	double *dts)
{
	const eph_t  *eph;
	const geph_t *geph;
	int prn, sys = satsys(sat, &prn);

	sys = satsys(sat, NULL);

	if (sys == _SYS_GPS_ || sys == _SYS_GAL_ || sys == _SYS_QZS_ || sys == _SYS_BDS_)
	{
		if (!(eph = seleph(teph, sat, -1, nav)))
		{
			return 0;
		}
		*dts = eph2clk(time, eph);
	}
	else if (sys == _SYS_GLO_)
	{
		if (!(geph = selgeph(teph, sat, -1, nav)))
		{
			return 0;
		}
		*dts = geph2clk(time, geph);
	}
	else
	{
		return 0;
	}

	return 1;
}

/* satellite position and clock by broadcast ephemeris -----------------------*/
RTK_RAM_CODE static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
	int iode, double *rs, double *dts, double *var, int *svh)
{
	const eph_t  *eph;
	const geph_t *geph;
	double rst[3], dtst[1], tt = 1E-3;
	int i, prn, sys = satsys(sat, &prn);

	sys = satsys(sat, NULL);

	*svh = -1;

	if (sys == _SYS_GPS_ || sys == _SYS_GAL_ || sys == _SYS_QZS_ || sys == _SYS_BDS_)
	{
		if (!(eph = seleph(teph, sat, iode, nav)))
		{
			return 0;
		}
		eph2pos(time, eph, rs, dts, var);
		time = timeadd(time, tt);
		eph2pos(time, eph, rst, dtst, var);
		*svh = eph->svh;
	}
	else if (sys == _SYS_GLO_)
	{
		if (!(geph = selgeph(teph, sat, iode, nav)))
		{
			return 0;
		}
		geph2pos(time, geph, rs, dts, var);
		time = timeadd(time, tt);
		geph2pos(time, geph, rst, dtst, var);
		*svh = geph->svh;
	}
	else
	{
		return 0;
	}

	/* satellite velocity and clock drift by differential approx */
	for (i = 0; i < 3; i++)
	{
		rs[i + 3] = (rst[i] - rs[i]) / tt;
	}
	dts[1] = (dtst[0] - dts[0]) / tt;

	return 1;
}


/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern int satpos(gtime_t time, gtime_t teph, int sat, int ephopt, const nav_t *nav,
	double *rs, double *dts, double *var, int *svh)
{
	int prn, sys = satsys(sat, &prn);

	*svh = 0;
	switch (ephopt)
	{
	case EPHOPT_BRDC: return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
	}
	*svh = -1;

	return 0;
}

/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern void satposs(obs_t *obs, vec_t *vec, nav_t *nav, int ephopt)
{
	gtime_t teph = obs->time, time[MAXOBS] = { 0 };
	double dt, pr;
	int i, j, sys = 0, prn = 0;

	for (i = 0; i < (int)obs->n; i++)
	{
		vec[i].sat = obs->data[i].sat;
		sys = satsys(obs->data[i].sat, &prn);
		/* search any pseudorange */
		for (j = 0, pr = 0.0; j < NFREQ; j++)
		{
			if ((pr = obs->data[i].P[j]) != 0.0)
			{
				break;
			}
		}
		if (j >= NFREQ)
		{
			continue;
		}
		/* transmission time by satellite clock */
		time[i] = timeadd(obs->time, -pr / CLIGHT);

		/* satellite clock bias by broadcast ephemeris */
		if (!ephclk(time[i], teph, obs->data[i].sat, nav, &dt))
		{
			continue;
		}
		time[i] = timeadd(time[i], -dt);

		/* satellite position and clock at transmission time */
		if (!satpos(time[i], teph, obs->data[i].sat, ephopt, nav,
			vec[i].rs + 0, vec[i].dts + 0, &vec[i].var, &vec[i].svh))
		{
			continue;
		}
		/* if no precise clock available, use broadcast clock instead */
		if (vec[i].dts[0] == 0.0)
		{
			if (!ephclk(time[i], teph, obs->data[i].sat, nav, vec[i].rs + 6))
			{
				continue;
			}
			vec[i].var = SQR(STD_BRDCCLK);
		}
	}
	for (i = 0; i < (int)obs->n; i++)
	{
		if (vec[i].rs[0] == 0.0)
		{
			continue;
		}
		sys = satsys(obs->data[i].sat, &prn);
	}
}

/* select satellite ephemeris --------------------------------------------------
* select satellite ephemeris. call it before calling satpos(),satposs().
* args   : int    sys       I   satellite system (SYS_???)
*          int    sel       I   selection of ephemeris
*                                 _SYS_GAL_: 0:any,1:I/NAV,2:F/NAV
*                                 others : undefined
* return : none
* notes  : default ephemeris selection for galileo is any.
*-----------------------------------------------------------------------------*/
RTK_RAM_CODE extern void satseleph(int sys, int sel)
{
    switch (sys) 
	{
        case _SYS_GPS_: eph_sel[0]=sel; break;
        case _SYS_GLO_: eph_sel[1]=sel; break;
        case _SYS_GAL_: eph_sel[2]=sel; break;
        case _SYS_QZS_: eph_sel[3]=sel; break;
        case _SYS_BDS_: eph_sel[4]=sel; break;
        case _SYS_SBS_: eph_sel[5]=sel; break;
    }
}

RTK_RAM_CODE extern int compute_vector_data(obs_t *obs, vec_t *vec)
{
	int i, sys, prn;
	int n = 0;
	vec_t  *vecd = NULL;
	double blh[3] = { 0.0 };

	ecef2pos(obs->pos, blh);

	for (i = 0; i < (int)obs->n; ++i)
	{
		sys = satsys(obs->data[i].sat, &prn);
		vecd = vec + i;
		if (s_norm(vecd->rs, 3) < 0.01)
		{
			continue;
		}
		/* compute geometric-range and azimuth/elevation angle */
		vecd->r = geodist(vecd->rs, obs->pos, vecd->e);
		vecd->r -= CLIGHT * vecd->dts[0];
		vecd->rate = geovel(vecd->rs, obs->pos, vecd->e);

		satazel(blh, vecd->e, vecd->azel);
		/* satellite clock-bias */
		vecd->sat = obs->data[i].sat;
		vecd->tro = tropmodel(blh, vecd->azel, 0.7);
		++n;
	}

	return n;
}

