/** ***************************************************************************
 * @file   taskRTK.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************
*/

#ifndef _TASK_RTK_H_
#define _TASK_RTK_H_

#include "typedefs.h"

#ifndef MAXOBS
#define MAXOBS      48
#endif
#define D2R         (0.017453292519943)

#define NMEA_M_LEN 1024
#define NMEA_S_LEN 128
#define R2D         (57.295779513082320)
#define SECONDS_IN_WEEK (604800)


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

typedef struct  {
    char frame_head[3];                  //sta8100
    uint16_t len;    
    uint8_t rov_n;
    satellite_struct rov_satellite[MAXOBS];
    uint8_t crc_value[2];

} sky_view_t;

#pragma pack()


typedef enum{
    AUTODETECT              = -1,
  UBLOX_BINARY            =  0,
  NOVATEL_BINARY          =  1,
  NOVATEL_ASCII           =  2,
  NMEA_TEXT               =  3,
    DEFAULT_SEARCH_PROTOCOL =  NMEA_TEXT, // 3
  SIRF_BINARY             =  4,
  INIT_SEARCH_PROTOCOL    =  SIRF_BINARY, ///< 4 max value, goes through each until we hit AUTODETECT
  RTCM3           =  5,
  UNKNOWN                 = 0xFF
} enumGPSProtocol;


typedef struct {
    unsigned long  GPSheader; ///< could be 1, 2, 3, or 4 bytes - see message headers ^
    unsigned char  GPSheaderLength;
    unsigned short lengthOfHeaderIDLength;
    unsigned char  crcLength;
    unsigned char  binaryOrAscii; // 1 = ascii, 0 binary
    unsigned char  startByte; // pulled out of header
} universalMSGSpec;


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

typedef struct  {
    tBool                 gpsFixType;
    int                  latSign;
    int                  lonSign;
    long double          lat; // concatinated from int components [deg.dec]
    long double          lon;
    double               vNed[3];    // NED North East Down [m/s] x, y, z
    tU32             itow;           ///< gps milisecond Interval Time Of Week

    int                  updateFlagForEachCall; /// changed to 16 bits
    int                  totalGGA;
    int                  totalVTG;

    double               trueCourse; // [deg]
    double               rawGroundSpeed; // NMEA kph, SiRf m/s

    double               alt;          // above mean sea level [m]
    double               filteredAlt; // FIXME should this be local?
    float                altEllipsoid; // [km] altitude above ellipsoid for WMM
    // uint8_t              GPSmonth;     // mm
    // uint8_t              GPSday;       // dd
    // uint8_t              GPSyear;      // yy last two digits of year
    // char                 GPSHour;      // hh
    // char                 GPSMinute;    // mm
    // char                 GPSSecond;    // ss
    // double               GPSSecondFraction; // FIXME used?

    /// compatible with Ublox driver FIXME should these be seperate data structure?
#ifdef USE_UBLOX
    unsigned char        ubloxClassID;
    unsigned char        ubloxMsgID;
    unsigned char        ubloxOldVersion;
    float                UbloxSoftwareVer;
#endif
    signed long          LonLatH[3]; // SiRF Lat Lon[deg] * 10^7 Alt ellipse [m]*100 <-- UNUSED
    char                 GPSFix;
    float                HDOP;       // Horizontal Dilution Of Precision x.x
    double               GPSVelAcc;
    unsigned short       GPSStatusWord;  /// will replace GPSfix
    unsigned char        isGPSFWVerKnown;
    unsigned char        isGPSBaudrateKnown;
    unsigned long        Timer100Hz10ms;
 
    unsigned int         navCFGword;
    unsigned int         nav2CFGword;
    char                 GPSConfigureOK; /// always needs to be initialized as -1

    unsigned char        reClassID;
    unsigned char        reMsgID;

    //unsigned long        LLHCounter;
    //unsigned long        VELCounter;
    //unsigned long        STATUSCounter; // UBLOX - or first flag SiRF
    //unsigned long        SBASCounter;
    //unsigned long        firewallCounter;
    //unsigned long        firewallRunCounter;
    //unsigned long        reconfigGPSCounter;

    /// GPS Baudrate and protocal: -1, 0,1, 2, 3 corresponding to
    int                  GPSbaudRate;    /// 4800, 9600, 19200, 38400, 57600, 115200, etc
    /// AutoDect, Ublox Binary, NovAtel binary, NovAtel ASCII, NMEA
    enumGPSProtocol      GPSProtocol;

    universalMSGSpec     GPSMsgSignature;
    unsigned char        GPSAUTOSetting;
    unsigned char        GPSTopLevelConfig; // UBLOX
    //unsigned char        resetAutoBaud;
    //unsigned char        autoBaudCounter;

    //uint8_t              sirfInitialized;
    //float                latQ;
    //float                lonQ;
    //float                hgtQ;
    //uint8_t              useSigmas;

    float                GPSHorizAcc;
    float                GPSVertAcc;

    int                  numSatelites;

    unsigned char gnss_data_flag;
    unsigned char new_eph_flag;
    gnss_rtcm_t rtcm; /* store RTCM data struct for RTK and PPP */

    /* for algorithm */
    obs_t rov;
    obs_t ref;
    nav_t nav;

} GpsData_t;

extern char gga_buff[];
extern char gsa_buff[];
extern char rmc_buff[];
extern char gsv_buff[];
extern char zda_buff[];
extern char nema_update_flag;

void rtk_run(void);
int32_t copy_gnss_result_from_rtk(gnss_solution_t *gnss_sol);

#endif
