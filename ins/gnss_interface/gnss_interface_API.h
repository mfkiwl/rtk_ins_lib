#ifndef _GNSS_INTERFACE_API_H_
#define _GNSS_INTERFACE_API_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <time.h>
#include "typedefs.h"
#include <stdint.h>


#define PI          3.1415926535897932   /* pi */
#define R2D         (57.295779513082320)
#define D2R         (0.017453292519943)
#define RE_WGS84    6378137.0            /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563)  /* earth flattening (WGS84) */
#define SECONDS_IN_WEEK (604800)

#define MAXLEAPS      64                  /* max number of leap seconds table */
#define MAXOBS      48

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

#pragma pack(1)
typedef struct mcu_time_base_t_
{
    long long time;
    long long  msec;
} mcu_time_base_t;
typedef struct {
    uint8_t  satelliteId;
    uint8_t  systemId;
    uint8_t  antennaId;
    uint8_t  l1cn0;
    uint8_t  l2cn0;
    float    azimuth;
    float    elevation;
    float    snr;
    
} satellite_struct;
typedef struct  {
    char frame_head[3];                  //sta8100
	uint16_t alo_time;
    uint16_t len;    

    uint16_t gps_week;
    uint32_t gps_tow; // gps Time Of Week, miliseconds

    uint8_t gnss_fix_type; // 1 if data is valid
    uint8_t vel_mode;
    uint8_t gnss_update; // 1 if contains new data
    uint8_t num_sats; // num of satellites in the solution 

    double latitude; // latitude ,  degrees 
    double longitude; // longitude,  degrees 
    double height; // above mean sea level [m]
    double geo_sep;
    double pos_ecef[3];
    float vel_ned[3]; // velocities,  m/s  NED (North East Down) x, y, z
    float heading; // [deg]

    float dops[5];
    float sol_age;

	float std_lat;	//!< latitude standard deviation (m)
	float std_lon;	//!< longitude standard deviation (m)
	float std_hgt;	//!< height standard deviation (m)
    float std_vn;
    float std_ve;
    float std_vd;

    uint8_t rov_n;
    satellite_struct rov_satellite[MAXOBS];
    uint8_t crc_value[2];
    uint8_t gnss_to_ins_flag;
} gnss_solution_t;


#pragma pack()

extern gnss_solution_t *g_ptr_gnss_sol;
extern gnss_solution_t g_gnss_sol;


#ifdef __cplusplus
}
#endif

#endif
