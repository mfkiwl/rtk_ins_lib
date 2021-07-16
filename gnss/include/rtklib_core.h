#ifndef _RTKLIB_CORE_H_
#define _RTKLIB_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <time.h>

#define _USE_RTK_
// #define _POST_RTK_
#define RTK_ENABLE
//#define _USE_PPP_
//#define _USE_INS_

#define RTK_RAM_CODE              	__attribute__((section("RTK_ALO_REGION")))
#define RTK_ITCM_CODE				__attribute__((section("RTK_ITCM_REGION")))
#define RTK_DTCM_DATA				__attribute__((section("RTK_DTCM_REGION")))

// #define RTK_RAM_CODE              
// #define RTK_ITCM_CODE			
// #define RTK_DTCM_DATA			

#define ENAGLO
#define ENACMP
#define ENAGAL
#define ENAQZS

#define NFREQ       2
#ifndef NFREQ
#define NFREQ       2                    /* number of carrier frequencies */
#endif
#define NFREQGLO    2                    /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                    /* number of extended obs codes */
#endif

#define RANGE_MS    (299792.458)         /* range in 1 ms */
#define CLIGHT      299792458.0          /* speed of light (m/s) */
#define PI          3.1415926535897932   /* pi */
#define R2D         (57.295779513082320)
#define D2R         (0.017453292519943)
#define OMGE        7.2921151467E-5      /* earth angular velocity (IS-GPS) (rad/s) */
#define RE_WGS84    6378137.0            /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563)  /* earth flattening (WGS84) */
#define SECONDS_IN_WEEK (604800)

#define SC2RAD      PI                   /* semi-circle to radian (IS-GPS) */
#define SQR(x)      ((x)*(x))
#define ROUND(x)    ((int)floor((x)+0.5))
#define ABS(a)	    (((a) < 0) ? -(a) : (a))
#define MIN(x, y)   ((x) < (y) ? (x) : (y))
#define MI(i, j, n) ((i) * (n) + (j))

#define SOLQ_NONE   0                     /* solution status none */
#define SOLQ_SPP    1                     /* solution status SPP */
#define SOLQ_RTD    2                     /* solution status RTD */
#define SOLQ_PPP    3                     /* solution status PPP */
#define SOLQ_FIXED  4                     /* solution status FIXED */
#define SOLQ_FLOAT  5                     /* solution status FLOAT */

#define P2_5 0.03125                      /* 2^-5 */
#define P2_6 0.015625                     /* 2^-6 */
#define P2_10 0.0009765625                /* 2^-10 */
#define P2_11 4.882812500000000E-04       /* 2^-11 */
#define P2_15 3.051757812500000E-05       /* 2^-15 */
#define P2_17 7.629394531250000E-06       /* 2^-17 */
#define P2_19 1.907348632812500E-06       /* 2^-19 */
#define P2_20 9.536743164062500E-07       /* 2^-20 */
#define P2_21 4.768371582031250E-07       /* 2^-21 */
#define P2_23 1.192092895507810E-07       /* 2^-23 */
#define P2_24 5.960464477539063E-08       /* 2^-24 */
#define P2_27 7.450580596923828E-09       /* 2^-27 */
#define P2_29 1.862645149230957E-09       /* 2^-29 */
#define P2_30 9.313225746154785E-10       /* 2^-30 */
#define P2_31 4.656612873077393E-10       /* 2^-31 */
#define P2_32 2.328306436538696E-10       /* 2^-32 */
#define P2_33 1.164153218269348E-10       /* 2^-33 */
#define P2_34 5.820766091346740E-11       /* 2^-34 */
#define P2_35 2.910383045673370E-11       /* 2^-35 */
#define P2_38 3.637978807091710E-12       /* 2^-38 */
#define P2_39 1.818989403545856E-12       /* 2^-39 */
#define P2_40 9.094947017729280E-13       /* 2^-40 */
#define P2_43 1.136868377216160E-13       /* 2^-43 */
#define P2_46 1.421085471520200E-14       /* 2^-46 */
#define P2_48 3.552713678800501E-15       /* 2^-48 */
#define P2_50 8.881784197001252E-16       /* 2^-50 */
#define P2_55 2.775557561562891E-17       /* 2^-55 */
#define P2_59 1.734723475976810E-18       /* 2^-59 */
#define P2_66 1.355252715606880E-20       /* 2^-66 */

#define MAXRAWLEN     4096                /* max length of receiver raw message */
#define MAXANT_UBX    64                  /* max length of station name/antenna type */
#define MAXLEAPS      64                  /* max number of leap seconds table */

#define SYS_NONE      0x00                /* navigation system: none */
#define SYS_GPS       0x01                /* navigation system: GPS */
#define SYS_SBS       0x02                /* navigation system: SBAS */
#define SYS_GLO       0x04                /* navigation system: GLONASS */
#define SYS_GAL       0x08                /* navigation system: Galileo */
#define SYS_QZS       0x10                /* navigation system: QZSS */
#define SYS_CMP       0x20                /* navigation system: BeiDou */
#define SYS_IRN       0x40                /* navigation system: IRNS */
#define SYS_LEO       0x80                /* navigation system: LEO */
#define SYS_ALL       0xFF                /* navigation system: all */

#define _SYS_NONE_    0x00                /* navigation system: none */
#define _SYS_GPS_     0x01                /* navigation system: GPS */
#define _SYS_SBS_     0x02                /* navigation system: SBAS */
#define _SYS_GLO_     0x04                /* navigation system: GLONASS */
#define _SYS_GAL_     0x08                /* navigation system: Galileo */
#define _SYS_QZS_     0x10                /* navigation system: QZSS */
#define _SYS_BDS_     0x20                /* navigation system: BeiDou */
#define _SYS_IRN_     0x40                /* navigation system: IRNSS */
#define _SYS_LEO_     0x80                /* navigation system: LEO */
#define _SYS_ALL_     0xFF                /* navigation system: all */

#define MINPRNGPS     1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS     40                  /* max satellite PRN number of GPS */
#define NSATGP0       32                  /* number of GPS satellites */
#define NSATGPS       (MAXPRNGPS-MINPRNGPS+1) /* number of GPS + QZSS satellites */
#define NSYSGPS       1                   /* 1-32 for GPS, 33-40 for QZSS */

#ifdef ENAGLO
#define MINPRNGLO     1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO     30                  /* max satellite slot number of GLONASS */
#define NSATGLO       (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO       1
#else
#define MINPRNGLO     0
#define MAXPRNGLO     0
#define NSATGLO	      0
#define NSYSGLO       0
#endif

#define MINPRNGAL     1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL     40                  /* max satellite PRN number of Galileo */
#define NSATGAL      (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL       1

#ifdef ENAQZS
#define MINPRNQZS     1                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS     8                 /* max satellite PRN number of QZSS */
//#define MINPRNQZS_S   183                 /* min satellite PRN number of QZSS SAIF */
//#define MAXPRNQZS_S   189                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS       (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS       1
#else
#define MINPRNQZS     0
#define MAXPRNQZS     0
#define MINPRNQZS_S   0
#define MAXPRNQZS_S   0
#define NSATQZS       0
#define NSYSQZS       0
#endif

#define MINPRNCMP     1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP     50                  /* max satellite sat number of BeiDou */
#define NSATCMP       (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP       1

#ifdef ENAIRN
#define MINPRNIRN     1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN     7                   /* max satellite sat number of IRNSS */
#define NSATIRN       (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN       1
#else
#define MINPRNIRN     0
#define MAXPRNIRN     0
#define NSATIRN       0
#define NSYSIRN       0
#endif

#ifdef ENALEO
#define MINPRNLEO     1                   /* min satellite sat number of LEO */
#define MAXPRNLEO     10                  /* max satellite sat number of LEO */
#define NSATLEO       (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO       1
#else
#define MINPRNIRN     0	
#define MINPRNLEO     0
#define MAXPRNLEO     0
#define NSATLEO       0
#define NSYSLEO       0
#endif

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT_UBX      (NSATGPS+NSATGLO+NSATGAL+NSATCMP)

#define NSYS (NSYSGPS + NSYSGLO + NSYSGAL + NSYSCMP) /* only use GPS, GLO, GAL, BDS */

#define MAXSAT		(NSATGPS + NSATGLO + NSATGAL + NSATCMP)
                                        /* max satellite number (1 to MAXSAT) */
#define MAXSTA      255

#ifndef MAXOBS_UBX
#define MAXOBS_UBX  64                  /* max number of obs in an epoch */
#endif
#define MAXRCV      64                  /* max receiver number (1 to MAXRCV) */
#ifdef OBS_100HZ
#define DTTOL       0.005               /* tolerance of time difference (s) */
#else
#define DTTOL       0.025               /* tolerance of time difference (s) */
#endif

//rtcm
#define FREQ1 1.57542E9                 /* L1/E1  frequency (Hz) */
#define FREQ2 1.22760E9                 /* L2     frequency (Hz) */
#define FREQ5 1.17645E9                 /* L5/E5a frequency (Hz) */
#define FREQ6 1.27875E9                 /* E6/LEX frequency (Hz) */
#define FREQ7 1.20714E9                 /* E5b    frequency (Hz) */
#define FREQ8 1.191795E9                /* E5a+b  frequency (Hz) */
#define FREQ9 2.492028E9                /* S      frequency (Hz) */

//ubx
#define FREQL1      1.57542E9           /* L1/E1/B1  frequency (Hz) */
#define FREQL2      1.22760E9           /* L2     frequency (Hz) */
#define FREQE5b     1.20714E9           /* E5b    frequency (Hz) */
#define FREQL5      1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQE6      1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQE5ab    1.191795E9          /* E5a+b/B2  frequency (Hz) */
#define FREQs       2.492028E9           /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 frequency (Hz) */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A        (GAL) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,BDS,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,BDS,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,BDS,QZS,IRN,SBS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A    30                  /* obs code: E6A        (GAL) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
#define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
#define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I    40                  /* obs code: B1I        (BDS) */
#define CODE_L2Q    41                  /* obs code: B1Q        (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define MAXCODE     55                  /* max number of obs code */

#define MAXFREQ      7 /* max NFREQ */
#define MAXOBS      48
#define MAXEPH      55
#define MAXEPH_R    15
#define MAXSSR      24
#define MAXANT       2

#define MAXEXFILE   1024                /* max number of expanded files */
#define NUMSYS      6                   /* number of systems */
#define TSYS_GPS    0                   /* time system: GPS time */
#define TSYS_UTC    1                   /* time system: UTC */
#define TSYS_GLO    2                   /* time system: GLONASS time */
#define TSYS_GAL    3                   /* time system: Galileo time */
#define TSYS_QZS    4                   /* time system: QZSS time */
#define TSYS_CMP    5                   /* time system: BeiDou time */
#define TSYS_IRN    6                   /* time system: IRNSS time */

#define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
#define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
#define STRFMT_OEM4  2                  /* stream format: NovAtel OEMV/4 */
#define STRFMT_CNAV  3                  /* stream format: ComNav */
#define STRFMT_UBX   4                  /* stream format: u-blox LEA-*T */
#define STRFMT_SBP   5                  /* stream format: Swift Navigation SBP */
#define STRFMT_CRES  6                  /* stream format: Hemisphere */
#define STRFMT_STQ   7                  /* stream format: SkyTraq S1315F */
#define STRFMT_GW10  8                  /* stream format: Furuno GW10 */
#define STRFMT_JAVAD 9                  /* stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS   10                 /* stream format: NVS NVC08C */
#define STRFMT_BINEX 11                 /* stream format: BINEX */
#define STRFMT_RT17  12                 /* stream format: Trimble RT17 */
#define STRFMT_SEPT  13                 /* stream format: Septentrio */
#define STRFMT_CMR   14                 /* stream format: CMR/CMR+ */
#define STRFMT_TERSUS 15                /* stream format: TERSUS */
#define STRFMT_LEXR  16                 /* stream format: Furuno LPY-10000 */
#define STRFMT_RINEX 17                 /* stream format: RINEX */
#define STRFMT_SP3   18                 /* stream format: SP3 */
#define STRFMT_RNXCLK 19                /* stream format: RINEX CLK */
#define STRFMT_SBAS  20                 /* stream format: SBAS messages */
#define STRFMT_NMEA  21                 /* stream format: NMEA 0183 */

#ifdef WIN32
typedef struct {                        /* time struct */
	time_t time;                        /* time (s) expressed by standard time_t */
	double sec;                         /* fraction of second under 1 s */
} gtime_t;
#else
typedef struct {                        /* time struct */
	time_t time;                        /* time (s) expressed by standard time_t */
	double sec;                         /* fraction of second under 1 s */
} gtime_t;
#endif /*WIN32*/

typedef struct {                        /* observation data record */
	gtime_t time;                       /* receiver sampling time (GPST) */
	unsigned char sat; /* satellite/receiver number */
	unsigned char sys, prn;
	unsigned char SNR[NFREQ + NEXOBS];  /* signal strength (0.25 dBHz) */
	unsigned char LLI[NFREQ + NEXOBS];  /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	/* set bit0 : ok; set bit1 : big error;	set bit2 : heavy multipath error;
	 * set bit3 : invaild. */
	unsigned char qualL[NFREQ + NEXOBS];/* quality of carrier phase measurement */
	unsigned char qualP[NFREQ + NEXOBS];/* quality of pseudorange measurement */
	//unsigned char freq;               /* GLONASS frequency channel (0-13) */
	unsigned char flag_used;            /* used in solution flag */
	double L[NFREQ + NEXOBS];           /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS];           /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS];           /* observation data doppler frequency (Hz) */
} obsd_t;

typedef struct {                        /* GPS/QZS/GAL broadcast ephemeris type */
	unsigned char sat;                  /* satellite number */	
	unsigned char sva;                  /* SV accuracy (URA index) */
	int svh;                            /* SV health (0:ok) */
	unsigned char flag;                 /* GPS/QZS: L2 P data flag */
						                /* BDS: nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	short week;                         /* GPS/QZS: gps week, GAL: galileo week */
	int iode, iodc;                     /* IODE,IODC */
	int code;                           /* GPS/QZS: code on L2 */
						                /* GAL: data source defined as rinex 3.03 */
						                /* BDS: data source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q) */
	gtime_t toe, toc;                   /* Toe,Toc,T_trans */
#ifdef _POST_RTK_
	gtime_t ttr;                       /* T_trans */
#endif // _POST_RTK_
	/* SV orbit parameters */
	double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;                       /* Toe (s) in week */
	double fit;                        /* fit interval (h) */
	double f0, f1, f2;                 /* SV clock parameters (af0,af1,af2) */
	double tgd[2];                     /* group delay parameters */
						               /* GPS/QZS:tgd[0]=TGD */
						               /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
						               /* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
	//double Adot, ndot;               /* Adot,ndot for CNAV */
} eph_t;

typedef struct {                       /* GLONASS broadcast ephemeris type */
	unsigned char sat;                 /* satellite number */
	int iode;                          /* IODE (0-6 bit of tb field) */
	int frq;                           /* satellite frequency number */
	int svh;                           /* satellite health */
	//int sva;                         /* satellite accuracy */
	//int age;                         /* satellite age of operation */
	gtime_t toe;                       /* epoch of epherides (gpst) */
#ifdef _POST_RTK_
	gtime_t tof;                      /* message frame time (gpst) */
#endif // _POST_RTK_

	//
	double pos[3];                     /* satellite position (ecef) (m) */
	double vel[3];                     /* satellite velocity (ecef) (m/s) */
	double acc[3];                     /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;                 /* SV clock bias (s)/relative freq bias */
	double dtaun;                      /* delay between L1 and L2 (s) */
} geph_t;

//typedef struct {                       /* station parameter type */
//	char name[MAXANT];                 /* marker name */
//	char marker[MAXANT];               /* marker number */
//	char antdes[MAXANT];               /* antenna descriptor */
//	char antsno[MAXANT];               /* antenna serial number */
//	char rectype[MAXANT];              /* receiver type descriptor */
//	char recver[MAXANT];               /* receiver firmware version */
//	char recsno[MAXANT];               /* receiver serial number */
//	int antsetup;                      /* antenna setup id */
//	int itrf;                          /* ITRF realization year */
//	int deltype;                       /* antenna delta type (0:enu,1:xyz) */
//	double pos[3];                     /* station position (ecef) (m) */
//	double del[3];                     /* antenna position delta (e/n/u or x/y/z) (m) */
//	double hgt;                        /* antenna height (m) */
//} sta_t;

#ifdef _USE_PPP_
typedef struct
{
	double       Height;  /* m */
	unsigned int Degree;  /* 1-16 */
	unsigned int Order;   /* 1-16 */
	double       Sinus[MAXIONODEGREE][MAXIONOORDER];
	double       Cosinus[MAXIONODEGREE][MAXIONOORDER];
} IonoLayers;

typedef struct
{
	unsigned int EpochTime; /* GPS */
	unsigned int UpdateInterval;
	unsigned int SSRIOD;
	unsigned int NumLayers; /* 1-4 */
	double Quality;
	IonoLayers Layers[NUMIONOLAYERS];
} vtec_t;

typedef struct {                      /* SSR correction type */
	unsigned char sat;
	gtime_t t0[6];                    /* epoch time (GPST) {eph,clk,hrclk,ura,bias,pbias} */
	double udi[6];                    /* SSR update interval (s) */
	int iod[6];                       /* iod ssr {eph,clk,hrclk,ura,bias,pbias} */
	int iode;                         /* issue of data */
	int iodcrc;                       /* issue of data crc for beidou/sbas */
	int ura;                          /* URA indicator */
	int refd;                         /* sat ref datum (0:ITRF,1:regional) */
	double deph[3];                   /* delta orbit {radial,along,cross} (m) */
	double ddeph[3];                  /* dot delta orbit {radial,along,cross} (m/s) */
	double dclk[3];                   /* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
	double hrclk;                     /* high-rate clock corection (m) */
	double  cbias[NFREQ];             /* code biases (m) */
	double pbias[NFREQ];              /* phase biases (m) */
	double yaw_ang, yaw_rate;         /* yaw angle and yaw rate (deg,deg/s) */
	unsigned char update;             /* update flag (0:no update,1:update) */
} ssr_t;
#endif 

typedef struct {                      /* navigation data type */
	unsigned int n;                   /* number of broadcast ephemeris */
	unsigned int ng;                  /* number of glonass ephemeris */
	unsigned int n_gps;
	unsigned int n_gal;
	unsigned int n_bds;
	unsigned int n_qzs;
	unsigned int ns;
	eph_t eph[MAXEPH];                /* GPS/QZS/GAL ephemeris */
	geph_t geph[MAXEPH_R];            /* GLONASS ephemeris */
	unsigned char ephsat;
} nav_t;

typedef struct {                      /* observation data */
	unsigned int n;                   /* number of obervation data/allocated */
	obsd_t data[MAXOBS];              /* observation data records */
	gtime_t time;
	double pos[6];                    /* station position (ecef) (m) */
	double refpos[6];                 /* reference pos & vel for comparison purpose */
	unsigned char obsflag;            /* obs data complete flag (1:ok,0:not complete) */
	unsigned int staid;               /* station id */
    unsigned int staid_obs;           /* station id */
	unsigned char rtcmtype;           /* flag of rtcm 999 */ 
	unsigned short fwver;             /* version ofSDK firmware */
	float geo_sep;                    /* Geoidal separation, meters */
    double heading;
} obs_t;

typedef struct
{
	/* available flag of code & carrier.
	 * set bit0: ok; set bit1: big error; set bit2: heavy multipath error;
	 * set bit3: invaild. RTCM_999_sigqm2_STRUCT */
	unsigned char sigqm2[MAXSAT*NFREQ][2];
	unsigned char fix_status;         /* GPS Quality Indicator (fix status) */
	unsigned char nsat_use;           /* Number of satellites in use */
	unsigned char nsat_view;          /* Number of satellites in view */
	unsigned char hdop;               /* HDOP, 0.1 */
	unsigned char vdop;               /* VDOP, 0.1 */
	unsigned char pdop;               /* PDOP, 0.1 */
	float geo_sep;                    /* Geoidal separation, meters */
	float age;                        /* Age of Differentials, s */
	unsigned int  ref_id;             /* Differential Reference Station ID */
	int sys;                          /* Time ID */
	double  time;                     /* GNSS time, second */
	// unsigned int  week;            /* Extended Week Number */
	unsigned char leap_sec;           /* Leap Seconds, GPS-UTC */
	double pos[3];                    /* position XYZ */
	double vel[3];                    /* velocity XYZ */
	double lev_pos[3];                /* Protection Level of position */
	double std_pos[3];                /* position variance*/
	double std_vel[3];                /* vel variance*/
	double rms;                       /* RMS value of the standard deviation of the range inputs to the navigation process*/
	double std_ellipse[3];            /* semimajor, semiminor axis of error ellipse, Orientation of semi-major axis of error ellipse*/
	double cbias;                     /* Receiver clock bias, mm */
	double cdt;                       /* Receiver Clock Drift, hz/100 */
    double heading;
	unsigned char PPS_Status;         /* Timing/PPS Status */
} type_999_t;

typedef struct {                      /* RTCM control struct type */
	gtime_t time;                     /* message time */
	char msmtype[6][128];             /* msm signal types */
	unsigned short lock[MAXSAT][NFREQ + NEXOBS]; /* lock time */
	int nbyte;                        /* number of bytes in message buffer */
	int nbit;                         /* number of bits in word buffer */
	int len;                          /* message length (bytes) */
	int type;                         /* last rtcm type */
	unsigned char buff[1200];         /* message buffer */
	unsigned char key;
	unsigned char icode[NSYS*NFREQ];  /* code indicator (CODE_???) */	
	type_999_t teseo;                 /* Teseo V output rtcm type 999 */
	int seqno;                        /* sequence number for rtcm 2 or iods msm */
} rtcm_t;

#define MAXSTN (2)

#define _OPENRTK_

#ifdef __cplusplus
}
#endif

#endif
