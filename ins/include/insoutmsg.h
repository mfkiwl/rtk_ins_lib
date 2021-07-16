#ifndef _INS_OUTMESGS_
#define _INS_OUTMESGS_
#include <stdint.h>
#include <stdio.h>
#include<string.h>
#include "datatype.h"
#include "lcstruct.h"

typedef enum MessageFormat //!< Bits 5-6 of MessageType struct
{
    BINARY = 0,
    ASCII = 1,
    ABREVIATED_ASCII = 2,
    NMEA = 3,
} MessageFormat;
typedef enum ResponseBit //!< Last bit (7) of MessageType struct
{
    ORIGINAL_MESSAGE = 0,
    RESPONSE_MESSAGE = 1,
} ResponseBit;
#pragma pack(1)

#if 0
typedef union MessageType{
  struct {
    unsigned char reserved : 5;
    MessageFormat format : 2;
    ResponseBit response : 1;
  } mess_bit;
  uint8_t    mess;
} MessageType;
#endif

typedef struct MessageType{
    unsigned char reserved : 5;
    MessageFormat format : 2;
    ResponseBit response : 1;
}MessageType;
// __attribute__ ((__aligned__(1)))
typedef struct Oem4BinaryHeader
{
    uint8_t sync1;            //!< start of packet first byte (0xAA)
    uint8_t sync2;            //!< start of packet second byte (0x44)
    uint8_t sync3;            //!< start of packet third  byte (0x12)
    uint8_t header_length;    //!< Length of the header in bytes ( From start of packet )
    uint16_t message_id;      //!< Message ID number
    MessageType message_type; //!< Message type - binary, ascii, nmea, etc...
    uint8_t port_address;     //!< Address of the data port the log was received on
    uint16_t message_length;  //!< Message length (Not including header or CRC)
    uint16_t sequence;        //!< Counts down from N-1 to 0 for multiple related logs
    uint8_t idle;             //!< Time the processor was idle in last sec between logs with same ID
    uint8_t time_status;      //!< Indicates the quality of the GPS time
    uint16_t gps_week;        //!< GPS Week number
    uint32_t gps_millisecs;   //!< Milliseconds into week
    uint32_t status;          //!< Receiver status word
    uint16_t Reserved;        //!< Reserved for internal use
    uint16_t version;         //!< Receiver software build number (0-65535)
} Oem4BinaryHeader;
typedef struct Position
{
    Oem4BinaryHeader header;                  //!< Message header
    uint32_t solution_status;                 //!< Solution status
    uint32_t position_type;                   //!< Position type
    double latitude;                          //!< latitude (deg)
    double longitude;                         //!< longitude (deg)
    double height;                            //!< height above mean sea level (m)
    float undulation;                         //!< Undulation - the relationship between the geoid and the ellipsoid (m)
    uint32_t datum_id;                        //!< datum id number
    float latitude_standard_deviation;        //!< latitude standard deviation (m)
    float longitude_standard_deviation;       //!< longitude standard deviation (m)
    float height_standard_deviation;          //!< height standard deviation (m)
    int8_t base_station_id[4];                //!< base station id
    float differential_age;                   //!< differential position age (sec)
    float solution_age;                       //!< solution age (sec)
    uint8_t number_of_satellites;             //!< number of satellites tracked
    uint8_t number_of_satellites_in_solution; //!< number of satellites used in solution
    uint8_t num_gps_plus_glonass_l1;          //!< number of GPS plus GLONASS L1 satellites used in solution
    uint8_t num_gps_plus_glonass_l2;          //!< number of GPS plus GLONASS L2 satellites used in solution
    uint8_t reserved;                         //!< reserved
    uint8_t extended_solution_status;         //!< extended solution status - OEMV and greater only
    uint8_t reserved2;                        //!< reserved
    uint8_t signals_used_mask;                //!< signals used mask - OEMV and greater only
    uint8_t crc[4];                           //!< 32-bit cyclic redundancy check (CRC)
} Position;
/*!
 * Velocity Message Structure
 * This log contains the best available velocity
 * information computed by the receiver. In addition,
 * it reports a velocity status indicator, which is
 * useful in indicating whether or not the corresponding
 * data is valid. The velocity measurements sometimes
 * have a latency associated with them. The time of validity
 * is the time tag in the log minus the latency value.
 *
 * This structure represents the format of the following messages:
 *  - BESTVEL
 *  - RTKVEL
 *  - PSRVEL
 */
typedef struct Velocity
{
	Oem4BinaryHeader header;			//!< Message header
	uint32_t solution_status;		//!< Solution status
	uint32_t position_type;			//!< Position type
	float latency;						//!< measure of the latency of the velocity time tag in seconds
	float age;							//!< differential age in seconds
	double horizontal_speed;			//!< horizontal speed in m/s
	double track_over_ground;			//!< direction of travel in degrees
	double vertical_speed; 				//!< vertical speed in m/s
	float reserved;
	int8_t crc[4];
}Velocity;
/*!
 * INSPVA Message Structure
 * This log allows INS position, velocity and
 * attitude to be collected in one log, instead
 * of using three separate logs.
 */
 typedef struct InsPositionVelocityAttitude
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Ellipsoidal height - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	int32_t status;			//!< status of the INS system
	int8_t crc[4];
}InsPositionVelocityAttitude;
typedef struct INSPVAX
{
	Oem4BinaryHeader header;	//!< Message header
	int32_t ins_status;         //!< Solution status
	int32_t pos_type;           //!< Position type
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Height above mean sea level  - WGS84 (m)
	float undulation;          //!< Undulation (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	float latitude_std;
	float longitude_std;
	float altitude_std;
	float north_velocity_std;
	float east_velocity_std;
	float up_velocity_std;
	float roll_std;
	float pitch_std;
	float azimuth_std;
	int32_t Ext_sol_stat;			//!< Extended solution status
	int16_t time_since_update;      //!< Elapsed time since the last ZUPT or positionupdate (seconds)
	int8_t crc[4];
}INSPVAX;
typedef struct ImuStatus
{
    unsigned counter : 4;                    //!< 4 byte counter
    unsigned imu_test : 1;                   //!< IMU test: Passed=0, Failed=1
    unsigned z_axis_path_length_control : 1; //!< Z-axis path length control: Good=0, Reset=1
    unsigned y_axis_path_length_control : 1; //!< Y-axis path length control: Good=0, Reset=1
    unsigned x_axis_path_length_control : 1; //!< X-axis path length control: Good=0, Reset=1
    unsigned accelerometer_temperature : 8;  //!< Accelerometer temperature
    unsigned software_version : 8;           //!< IMU software version number
    unsigned reserved : 3;                   //!< Reserved
    unsigned gyro_test : 1;                  //!< Gyro tests: Passed=0, Failed=1
    unsigned accel_test : 1;                 //!< Accelerometer tests: Passed=0, Failed=1
    unsigned other_tests : 1;                //!< Other tests: Passed=0, Failed=1
    unsigned memory_tests : 1;               //!< Memory tests: Passed=0, Failed=1
    unsigned processor_tests : 1;            //!< Processor tests: Passed=0, Failed=1
} ImuStatus;
typedef struct RawImu
{
    Oem4BinaryHeader header;  //!< Message header
    uint32_t gps_week;        //!< GPS week number
    double gps_millisecs;     //!< Milliseconds into GPS week
    ImuStatus imuStatus;      //!< Status of the IMU
    float z_acceleration;     //!< change in velocity along z axis in scaled m/s
    float y_acceleration_neg; //!< -change in velocity along y axis in scaled m/s
    float x_acceleration;     //!< change in velocity along x axis in scaled m/s
    float z_gyro_rate;        //!< change in angle around z axis in radians
    float y_gyro_rate_neg;    //!< -(change in angle around y axis) in radians
    float x_gyro_rate;        //!< change in angle around x axis in radians
    int8_t crc[4];
} RawImu;
typedef struct OdoSpeed
{
    Oem4BinaryHeader header;  //!< Message header
    int week;
    double gps_millisecs;     //!< Milliseconds into GPS week
    char mode;
    double speed;
    char fwd;
    uint64_t wheel_tick;
    int8_t crc[4];
} OdoSpeed;
#pragma pack()



unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer); /* Number of bytes in the data block  Data block */

int writeRawImuMsg(const int gps_update,const int16_t week1,const uint32_t sec1, const int32_t week2, const uint32_t sec2,const ImuData* p_ImuData);

int writePositionMsg(const int week, const uint32_t gps_millisecs,const GnssData* p_GnssData);

int writeVelocityMsg(const int week, const uint32_t gps_millisecs,const GnssData* p_GnssData);

int writeGnssRawMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData);

int writeOdoDataMsg(const OdoData* p_OdoData);

int writeGGAMsg(int week, double time, const GnssInsSystem* p_gnssInsSystem);

int writeINSPVAXMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem);

int writeINSPVAMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem);




extern RawImu imuStr;
extern Position positionStr ;
extern Velocity velocityStr;
extern InsPositionVelocityAttitude inspvastr;
extern INSPVAX inspvaxstr;
extern char ggaBuff_bt[120];
extern char pashrBuff_bt[120];
extern char rmcBuff[200];

#endif // !_INS_OUTMESGS_
