#ifndef DATETYPE_H_
#define DATETYPE_H_
#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifndef WIN32
#define ARM_MCU
#endif


#ifndef SECONDSOFWEEK
#define SECONDSOFWEEK (604800)
#endif // !SECONDSOFWEEK

#pragma pack(1)
typedef struct  ImuData//ǰ����
{
	int32_t week;
	double timestamp;  //GPS time

	double timestamped; //received time 

	double Gyrox;	// angle velocity along x axis  rad/s
	double Gyroy;	// angle velocity along y axis  rad/s
	double Gyroz;	// angle velocity along z axis  rad/s

	double Accx;	// line acceleration along x axis  m/s^2
	double Accy;	// line acceleration along y axis  m/s^2
	double Accz;	// line acceleration along z axis  m/s^2

	uint32_t IMUstatus;
	double flag;    // The first data in a package
}ImuData;

typedef struct OdoData
{
	int32_t week;
	double timestamp;   //GNSS time
	double timestampd;  //systemtime
	double vehicle_speed;  //m/s
	int8_t mode;
	uint64_t wheel_tick;
	int8_t fwd;
}OdoData;

typedef struct  GnssData
{

	double flag;
	int32_t week;
	uint32_t itow;
	double timestamp;  //GPS time 
	double timestampd; //received time

	double longitude;	// degrees
	double latitude;	// degrees
	double altitude;	// ellipsoidal height - WGS84 (m)

	float north_velocity;	// m/s
	float east_velocity;	// m/s
	float up_velocity;		// m/s

	float longitude_std;	// longitude standard deviation
	float latitude_std;	// latitude standard deviation
	float altitude_std;	// altitude standard deviation

	float north_velocity_std;	// velocity standard deviation
	float east_velocity_std;
	float up_velocity_std;

	float length; //Dual antenna baseline length
	float pitch;
	float heading; //Dual antenna

	float pitch_std;
	float heading_std; //Dual antenna

	uint16_t Num_track; //number of stars visible
	uint16_t Num_sol; //number of stars visible

	uint16_t average_snr;
	double HDOP;
	double PDOP;
	uint16_t Mode;//0:NGNSS  1:spp, 2:PSR,  4:fixed,5:float
	float sol_age;
	int32_t numSatellites;
	uint16_t gpsFixType;

	int useNativeGNSSPOS;
	int useNativeGNSSVEL;

	float hor_pos_pl;
	float ver_pos_pl;
	float hor_vel_pl;
	float ver_vel_pl; 
	uint8_t pos_integrity_status;
    uint8_t vel_integrity_status;

} GnssData;


#pragma pack()

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif