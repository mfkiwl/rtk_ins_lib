#ifndef _GNSS_DATATYPE_H_
#define _GNSS_DATATYPE_H_
#include "rtklib_core.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAXOBS
#define MAXOBS      48
#endif

#ifndef NFREQ
#define NFREQ       2
#endif

#ifndef NSYS
#define NSYS        4  /* only use GPS, GLO, GAL, BDS */
#endif

#ifdef _POST_RTK_
#define MAXAMB       (20)
#else
#define MAXAMB       (20)
#endif // _POST_RTK_

#define MAXAMBINSET (MAXAMB)

#define MI(i, j, n) ((i) * (n) + (j))
#define SMD(i) ((i)*((i)+1)/2)
#define SMI(i, j)	((i) > (j) ? (SMD(i) + (j)) : (SMD(j) + (i)))
#define NX_CLK (NSYS*NFREQ)
#define NX_PHB (NSYS*NFREQ)              /*refsat ambiguity + phase bias*/
#define NX_RTD (9+1+NX_CLK+NX_PHB)       /* p(3),v(3),a(3),cdt_rate(1), cdt(NSYS*NFREQ),
                                            glonass_bias(NFREQ), sd_refamb(NSYS*NFREQ) */
#define NP_RTD (SMD(NX_RTD))
#ifndef NX_RTK
#define NX_RTK (NX_RTD+MAXAMB)           /* p(3),v(3),a(3),cdt_rate(1), cdt(NSYS*NFREQ),
                                            glonass_bias(NFREQ), sd_refamb(NSYS*NFREQ),
                                            dd_amb(MAXAMB) */
#endif
#define NP_RTK (SMD(NX_RTK))
#define NX_PVA (9)
#define NP_PVA (SMD(NX_PVA))
#define NX_SPP (6+NX_CLK)
#define NX     (3+NX_CLK)         /* # of estimated parameters */
#define NP_SPP (SMD(NX_SPP))

enum integrity_event
{
    Available,
    Nominal,
    Misleading,
    Hazardous,
    Unavailable
};

enum integrity_type
{
    horizontal  = 1 << 0,
    vertical    = 1 << 1,
    lateral     = 1 << 2,
    longitu     = 1 << 3,
    hor_ver     = 1 << 4,
    lat_lon_ver = 1 << 5
};

typedef struct {
	unsigned char sat;   /*prn*/
    double rs[6];
    double dts[2];
    double var;
    int svh;    
	double azel[2];    /*azimuth,elevation*/
	double e[3];       /*partial deviation*/
	double r;          /* vector */
	double rate;
	double tro;        /* tropospheric */
}vec_t;

typedef struct
{
    obs_t obs;
    vec_t vec[MAXOBS];
}epoch_t;

typedef struct {        /* observation data */
    double x[NX_SPP];
    double P[NP_SPP];
    double time;
    unsigned char n_used;
    unsigned char solType;
} rcv_spp_t;

typedef struct {        /* observation data */
    double x[NX_PVA];
    double P[NP_PVA];
    double blh[3];
    double vned[3];
    double time;
} rcv_pva_t;

typedef struct
{
    unsigned char s1, s2, f, nlock; /*nlock=0-> cycleslip*/
    double data;
    double var;
    double time;
}ambdata_t;

typedef struct
{
    unsigned char loc;
    unsigned char s1;
    unsigned char s2;
    unsigned char f;
    double value;
}ambloc_t;

typedef struct
{
    unsigned char n;
    ambdata_t amb[MAXAMBINSET];
    double ratio;
    unsigned char nsat;
}ambset_t;

typedef struct
{
    unsigned char  sat;               /* satellite status type */
    unsigned char slip[NFREQ];        /* cycle-slip flag */
    unsigned char nlock[NFREQ];       /* data lock epoch */
    double  gf;                       /* geometry-free phase L1-L2 (m) */
    double  gf2;                      /* geometry-free phase L1-L2 (m) for rover*/
    double  L[NFREQ];                 /* phase (cycle) */
    double  D[NFREQ];                 /* doppler (cycle) */
    double dph[NFREQ];                /* delta phase (cycle)*/
} slipset_t;

typedef struct
{
    unsigned char glo_ifbflag;
    double glo_ifb;
    double glo_ifb_var;
    int ifb_num;
} glo_ifb_t;

#define NX_MAX (60)
#define MAXPAR 10 /* maximum parameters in one design matrix row */

typedef struct
{
    unsigned int s1, s2, f;
    double time;
} state_tag_t;

typedef struct
{
    double H[MAXPAR];
    int L[MAXPAR];
    int numl;
    double z;
    double R;
    double z_;
    double R_;
    double elev;
    double azim;
    int SNR;
    int refsat;
    int sat, sys, prn, code, flag, cslip, nlock;
    unsigned char isd;
} measure_t;

typedef struct
{
    double sigma0[2];
    double medres[2];
    double ave_snr[2];
    int    nep[2];
    double scale[2];
    double ave_elev;
    double max_elev;
} meas_scale_t;

typedef struct 
{                                        /* observation data record */
    unsigned char sat;                   /* satellite/receiver number */
    unsigned char sys, prn, f;
    double P[NFREQ + NEXOBS];            /* observation data pseudorange (m) */
    double D[NFREQ + NEXOBS];
    double var[NFREQ + NEXOBS];
    double time[NFREQ + NEXOBS];
} smobsd_t;

typedef struct
{
  double prev_heading;
  double heading_vel;
  double heading_detpos;
  double vel_hor;
  double vel_dwn;
  double dpos_hor;
  double dpos_dwn;
  double prev_pos[6];
  double cur_pos[6];
  int    pos_rel;
  int    vel_rel;
  int    heading_rel;
} dynamic_t;


typedef struct
{
    double hor_pl;
    double ver_pl;
    double lateral_pl;
    double longitu_pl;
    enum integrity_event integrity_status;
    enum integrity_type  integrity_type;
} integrity_t;

typedef struct
{
    double x[NX_RTK];
    double P[NP_RTK];
    smobsd_t sm_codeobs[MAXOBS];
    state_tag_t tag[MAXAMB];
    meas_scale_t scales;
    double x_fixed[NX_RTK];
    slipset_t slip[MAXOBS];
    ambset_t ambset;
    rcv_spp_t spp;
    glo_ifb_t glo_ifb;
    unsigned int np;
    unsigned int ns;
    double time;
    double tt;
    double code_blunder_rate;
    double age;
    double lev_pos[3];                    /* Protection Level of position */
	float  geo_sep;                       /* Geoidal separation, meters */
	unsigned char leap_sec;               /* Leap Seconds, GPS-UTC */
    unsigned int num_fixepoch;
    unsigned int num_floatpoch;
    unsigned int num_unfixepoch;
    unsigned int num_goodepoch;
    unsigned int num_of_sat;
    double dop[5];
    dynamic_t dynamic_stat;
    int fixType;
    int epoch_quality_flag;
    int nep;
    double coreff;
    double lastTimeRef;
    integrity_t pos_integrity;
    integrity_t vel_integrity;
}rcv_rtk_t;

#ifdef __cplusplus
}
#endif
#endif
