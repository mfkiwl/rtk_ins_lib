#include <math.h>
#include <stdio.h> 
#include<string.h>
#include <stdlib.h>
#include "insoutmsg.h"
#include "datatype.h"
#include "lcstruct.h"
#include "earth.h"
#include "ins_interface_API.h"
// #include "rtklib_core.h"
#include "gnss_interface_API.h"
#include "rtkcmn.h"

#ifndef CRC32_POLYNOMIAL
#define CRC32_POLYNOMIAL 0xEDB88320L
#endif // !CRC32_POLYNOMIAL


#ifndef RAD2DEG
#define  RAD2DEG (57.295779513082320)
#endif // !RAD2DEG
#ifndef PI
#define PI (3.141592653589793)
#endif // !PI
#ifndef grav_WGS84
#define	grav_WGS84 9.7803267714e0
#endif

 OdoSpeed odoStr = {0};
 RawImu imuStr = {0};
 Position positionStr = { 0 };
 Velocity velocityStr = { 0 };
 InsPositionVelocityAttitude inspvastr = { 0 };
 INSPVAX inspvaxstr = { 0 };
 char ggaBuff_bt[120] = { 0 };
 char pashrBuff_bt[120] = { 0 };
 char rmcBuff[200]  = { 0 };

// typedef struct {        /* time struct */
// 	time_t time;        /* time (s) expressed by standard time_t */
// 	double sec;         /* fraction of second under 1 s */
// } gtime_t;
#define MAXLEAPS    64                  /* max number of leap seconds table */
const static double gpst0[] = { 1980,1,6,0,0,0 }; /* gps time reference */
static double leaps[MAXLEAPS + 1][7] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};


static void RealToArray(double value, char *a, unsigned char id, unsigned char dd) //id:integer digits¡ê?dd:decimal digits
{
	uint64_t temp1;
	uint64_t temp2;
	char i;
	if (id == 0)
	{
		if (abs(value) < 10)
		{
			id = 1;
		}
		else if (abs(value) < 100)
		{
			id = 2;
		}
		else if (abs(value) < 1000)
		{
			id = 3;
		}
		else if (abs(value) < 10000)
		{
			id = 4;
		}
		else
		{
			id = 8;
		}
	}

	if (value >= 0)
	{
		temp1 = value * pow(10, dd);
		temp2 = ((int)(value)) * pow(10, dd);
		temp2 = temp1 - temp2; 
		temp1 = (int)value;	 
		for (i = id; i > 0; i--)
		{
			*(a + i - 1) = (int)temp1 % 10 + 0x30;
			temp1 = (int)(temp1 / 10);
		}
		*(a + id) = '.'; 
		for (i = id + dd; i > id; i--)
		{
			*(a + i) = temp2 % 10 + 0x30;
			temp2 = temp2 / 10;
		}
	}
	else
	{
		value = 0 - value;
		temp1 = value * pow(10, dd);
		temp2 = ((int)(value)) * pow(10, dd);
		temp2 = temp1 - temp2; 
		temp1 = (int)value;	 
		*a = 0x2D;			   
		for (i = id; i > 0; i--)
		{
			*(a + i) = (int)temp1 % 10 + 0x30;
			temp1 = (int)(temp1 / 10);
		}
		*(a + id + 1) = '.'; 
		for (i = id + dd + 1; i > id + 1; i--)
		{
			*(a + i) = temp2 % 10 + 0x30;
			temp2 = temp2 / 10;
		}
	}
}

static char *itoa_user(int num, char *str, int radix)
{ 
	char index[] = "0123456789ABCDEF";
	unsigned unum; 
	int i = 0, j, k;
	
	if (radix == 10 && num < 0) 
	{
		unum = (unsigned)-num;
		str[i++] = '-';
	}
	else
		unum = (unsigned)num; 
	do
	{
		str[i++] = index[unum % (unsigned)radix];
		unum /= radix;
	} while (unum);
	str[i] = '\0';

	if (str[0] == '-')
		k = 1; 
	else
		k = 0;

	for (j = k; j <= (i - 1) / 2; j++)
	{
		char temp;
		temp = str[j];
		str[j] = str[i - 1 + k - j];
		str[i - 1 + k - j] = temp;
	}
	return str;
}
static void deg2dms(double deg, double *dms, int ndec)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a);
	a = (a - dms[0]) * 60.0;
	dms[1] = floor(a);
	a = (a - dms[1]) * 60.0;
	dms[2] = floor(a / unit + 0.5) * unit;
	if (dms[2] >= 60.0)
	{
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0)
		{
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}
static  int print_nmea_gga(double *ep, double *pos, int nsat, int type, double dop, double age, char *buff)
{
	double h, dms1[3], dms2[3];
	char *p = (char *)buff, *q, sum;
	char buf[20] = {0};

	if ((pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]) < 1.0)
	{
		strcpy(buff,"$GPGGA,,,,,,,,,,,,,,");
		for (q = (char *)buff + 1, sum = 0; *q; q++)
			sum ^= *q;
		strcat(buff, "*");
		memset(buf, 0, 20);
		itoa_user(sum, buf, 16);
		strcat(buff, buf);
		strcat(buff, "\r\n");

	}
	else
	{
		h = 0.0; 
		deg2dms(fabs(pos[0]) * RAD2DEG, dms1, 7);
		deg2dms(fabs(pos[1]) * RAD2DEG, dms2, 7);

		strcpy(buff,"$GPGGA,");

		RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(dms1[0] * 100 + dms1[1] + dms1[2] / 60.0, buf, 4, 7);
		strcat(buff, buf);
		strcat(buff, pos[0] >= 0 ? ",N," : ",S,");

		memset(buf, 0, 20);
		RealToArray(dms2[0] * 100 + dms2[1] + dms2[2] / 60.0, buf, 5, 7);
		strcat(buff, buf);
		strcat(buff, pos[1] >= 0 ? ",E," : ",W,");

		memset(buf, 0, 20);
		// RealToArray(type,buf,1,0);
		itoa_user(type, buf, 10);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(nsat, buf, 2, 0);
		buf[2] = 0;
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(dop, buf, 0, 1);
		strcat(buff, buf);
		strcat(buff, ",");

		memset(buf, 0, 20);
		RealToArray(pos[2] - h, buf, 0, 3);
		strcat(buff, buf);
		strcat(buff, ",M,");

		memset(buf, 0, 20);
		RealToArray(h, buf, 0, 3);
		strcat(buff, buf);
		strcat(buff, ",M,");

		memset(buf, 0, 20);
		RealToArray(age, buf, 0, 1);
		strcat(buff, buf);
		strcat(buff, ",");

		for (q = (char *)buff + 1, sum = 0; *q; q++)
			sum ^= *q; /* check-sum */

		strcat(buff, "*");
		memset(buf, 0, 20);
		itoa_user(sum, buf, 16);
		strcat(buff, buf);

		strcat(buff, "\r\n");
	}
	return strlen(buff);
}

static int print_nmea_pashr_bt(double *ep, char *buff)
{
    char *q, sum;
    char buf[20] = {0};

    // $PASHR,hhmmss.ss,HHH.HH,T,RRR.RR,PPP.PP,heave,rr.rrr,pp.ppp,hh.hhh,QF*CC<CR><LF>
    strcpy(buff, "$PASHR,");

    RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->azimuth, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",T,");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->pitch, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->roll, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",+0.00,");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->azimuth_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->pitch_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->roll_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    strcat(buff, "G");
    
    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */

    strcat(buff, "*");
    memset(buf, 0, 20);
    itoa_user(sum, buf, 16);
    strcat(buff, buf);

    strcat(buff, "\r\n");

    return strlen(buff);
}

int print_nmea_pashr(double *ep, char *buff)
{
    char *q, sum;
    char buf[20] = {0};

    // $PASHR,hhmmss.ss,HHH.HH,T,RRR.RR,PPP.PP,heave,rr.rrr,pp.ppp,hh.hhh,QF*CC<CR><LF>
    strcpy(buff, "$PASHR,");

    RealToArray(ep[3] * 10000 + ep[4] * 100 + ep[5] + 0.001, buf, 6, 2);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->azimuth, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",T,");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->pitch, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->roll, buf, 3, 2);
    strcat(buff, buf);
    strcat(buff, ",+0.00,");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->azimuth_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->pitch_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    memset(buf, 0, 20);
    RealToArray(g_ptr_ins_sol->roll_std, buf, 2, 3);
    strcat(buff, buf);
    strcat(buff, ",");

    if (g_ptr_ins_sol->pos_type == 4) {
        strcat(buff, "2,");
    } else if (g_ptr_ins_sol->pos_type != 0) {
        strcat(buff, "1,");
    } else {
        strcat(buff, "0,");
    }

    if (g_ptr_ins_sol->ins_status > INS_ALIGNING) {
        strcat(buff, "1");
    } else {
        strcat(buff, "0");
    }
    
    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */

    strcat(buff, "*");
    memset(buf, 0, 20);
    itoa_user(sum, buf, 16);
    strcat(buff, buf);

    strcat(buff, "\r\n");

    return strlen(buff);
}

int print_nmea_ins(double *ep, char *buff)
{
    double dms1[3], dms2[3];
    char *p = buff, *q, sum;
    char str_ins_status[24] = {0};
    char str_position_type[16] = {0};
	double pos[3], vel[3], att[3];

	pos[0] = g_ptr_ins_sol->latitude;
	pos[1] = g_ptr_ins_sol->longitude;
	pos[2] = g_ptr_ins_sol->height;

	vel[0] = g_ptr_ins_sol->north_velocity;
	vel[1] = g_ptr_ins_sol->east_velocity;
	vel[2] = g_ptr_ins_sol->up_velocity;

	att[0] = g_ptr_ins_sol->roll;
	att[1] = g_ptr_ins_sol->pitch;
	att[2] = g_ptr_ins_sol->azimuth;

    switch (g_ptr_ins_sol->ins_status) {
        case INS_INACTIVE:
            strcpy(str_ins_status, "INS_INACTIVE");
            break;
        case INS_ALIGNING:
            strcpy(str_ins_status, "INS_ALIGNING");
            break;
        case INS_HIGH_VARIANCE:
            strcpy(str_ins_status, "INS_HIGH_VARIANCE");
            break;
        case INS_SOLUTION_GOOD:
            strcpy(str_ins_status, "INS_SOLUTION_GOOD");
            break;
        case INS_SOLUTION_FREE:
            strcpy(str_ins_status, "INS_SOLUTION_FREE");
            break;
        case INS_ALIGNMENT_COMPLETE:
            strcpy(str_ins_status, "INS_ALIGNMENT_COMPLETE");
            break;
        default:
            strcpy(str_ins_status, "NONE");
            break;
    }

    switch (g_ptr_ins_sol->pos_type) {
        case INS_NONE:
            strcpy(str_position_type, "INS_NONE");
            break;
        case INS_PSRSP:
            strcpy(str_position_type, "INS_PSRSP");
            break;
        case INS_PSRDIFF:
            strcpy(str_position_type, "INS_PSRDIFF");
            break;
        case INS_PROPAGATED:
            strcpy(str_position_type, "INS_PROPAGATED");
            break;
        case INS_RTKFIXED:
            strcpy(str_position_type, "INS_RTKFIXED");
            break;
        case INS_RTKFLOAT:
            strcpy(str_position_type, "INS_RTKFLOAT");
            break;
        default:
            strcpy(str_position_type, "NONE");
            break;
    }

    deg2dms(fabs(pos[0]), dms1, 7);
    deg2dms(fabs(pos[1]), dms2, 7);
    p += sprintf(p, "$GNINS,%02.0f%02.0f%05.2f,%s,%s,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                 ep[3], ep[4], ep[5], str_ins_status, str_position_type, dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
                 dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", pos[2], vel[0], vel[1], vel[2], att[0], att[1], att[2]);
    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
    return p - (char *)buff;
}

static int print_nmea_rmc(double *ep, double *pos, double heading, double speed, int fixID, char *buff)
{
    double dms1[3], dms2[3], amag = 0.0;
    char *p = buff, *q, sum, *emag = "E";

    if (fixID <= 0)
    {
        p += sprintf(p, "$GPRMC,,,,,,,,,,,,");
        for (q = (char *)buff + 1, sum = 0; *q; q++)
            sum ^= *q;
        p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
        return p - (char *)buff;
    }

    deg2dms(fabs(pos[0]) * RAD2DEG, dms1, 7);
    deg2dms(fabs(pos[1]) * RAD2DEG, dms2, 7);
    p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
                 ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
                 dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", speed / 0.514444444, heading,
                 ep[2], ep[1], (int)ep[0] % 100, amag, emag,
                 fixID == 4 || fixID == 5 ? "D" : "A");
    //    sol->stat==SOLQ_DGPS||sol->stat==SOLQ_FLOAT||sol->stat==SOLQ_FIX?"D":"A");
    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
    return p - (char *)buff;
}

int print_nmea_vtg(char *buff)
{
    char *p = buff, *q, sum;

    double heading = g_ptr_ins_sol->azimuth;
    double speed = sqrt(g_ptr_ins_sol->east_velocity * g_ptr_ins_sol->east_velocity + g_ptr_ins_sol->north_velocity * g_ptr_ins_sol->north_velocity);
    
    p += sprintf(p, "$GPVTG,%06.2f,T,,M,%.2f,N,%.2f,K", 
            heading, speed / 0.514444444, speed * 3.6);

    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
    return p - (char *)buff;
}

static unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for (j = 8; j > 0; j--)
    {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount,   /* Number of bytes in the data block */
                                  unsigned char *ucBuffer) /* Data block */
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while (ulCount-- != 0)
    {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return (ulCRC);
}

int writeRawImuMsg(const int gps_update, const int16_t week1, const uint32_t sec1, const int32_t week2, const uint32_t sec2,const ImuData* p_ImuData)
{
	long crc = 0;
	imuStr.header.sync1 = 0xAA;
    imuStr.header.sync2 = 0x44;
    imuStr.header.sync3 = 0x12;
    imuStr.header.header_length = 28;
    imuStr.header.message_id = 268;
    imuStr.header.message_type.format = BINARY;
    imuStr.header.message_type.response = ORIGINAL_MESSAGE;
    imuStr.header.port_address = 0;
    imuStr.header.message_length = 40; //
    imuStr.header.sequence = 0;
    imuStr.header.idle = 0;
	imuStr.header.time_status = 7;
    if (gps_update == 1)
    {
        imuStr.header.gps_week = week2;
        imuStr.header.gps_millisecs = sec2;
    }
    else
    {
        imuStr.header.gps_week = 0;
        imuStr.header.gps_millisecs = 0;
    }

    imuStr.header.status = 0;
    imuStr.header.Reserved = 0;
    imuStr.header.version = 0;

    imuStr.gps_week = week1;
    imuStr.gps_millisecs = sec1 ;
    //imuStr.imuStatus
    imuStr.z_acceleration = p_ImuData->Accz/grav_WGS84;
    imuStr.y_acceleration_neg = p_ImuData->Accy/grav_WGS84;
    imuStr.x_acceleration = p_ImuData->Accx/grav_WGS84;
    imuStr.z_gyro_rate = p_ImuData->Gyroz * RAD2DEG;
    imuStr.y_gyro_rate_neg = p_ImuData->Gyroy * RAD2DEG;
    imuStr.x_gyro_rate = p_ImuData->Gyrox * RAD2DEG;
    crc = CalculateBlockCRC32(imuStr.header.header_length + imuStr.header.message_length, (unsigned char *)&imuStr);
    memcpy((int8_t *)imuStr.crc, (int8_t *)&crc, 4);
	return 1;
}

int writePositionMsg(const int week, const uint32_t gps_millisecs,const GnssData* p_GnssData)
{
	long crc;
	positionStr.header.sync1 = 0xAA;
	positionStr.header.sync2 = 0x44;
	positionStr.header.sync3 = 0x12;
	positionStr.header.header_length = 28;
	positionStr.header.message_id = 42;
	positionStr.header.message_type.format = BINARY;
	positionStr.header.message_type.response = ORIGINAL_MESSAGE;
	positionStr.header.port_address = 0;
	positionStr.header.message_length = 72; //
	positionStr.header.sequence = 0;
	positionStr.header.idle = 0;
	positionStr.header.time_status = 0;
	positionStr.header.gps_week = week;
	positionStr.header.gps_millisecs = gps_millisecs;
	positionStr.header.status = 0;
	positionStr.header.Reserved = 0;
	positionStr.header.version = 0;

	positionStr.solution_status = 0;
	positionStr.position_type = p_GnssData->gpsFixType;


	positionStr.latitude = p_GnssData->latitude * RAD2DEG;
	positionStr.longitude = p_GnssData->longitude * RAD2DEG;
	positionStr.height = p_GnssData->altitude;
	positionStr.undulation = 0;
	positionStr.datum_id = 0;
	positionStr.latitude_standard_deviation = p_GnssData->latitude_std;
	positionStr.longitude_standard_deviation = p_GnssData->longitude_std;
	positionStr.height_standard_deviation = p_GnssData->altitude_std;
	// positionStr.base_station_id[4];RAD2DEG
	positionStr.differential_age = 0;
	positionStr.solution_age = p_GnssData->sol_age;
	positionStr.number_of_satellites = 0;
	positionStr.number_of_satellites_in_solution = p_GnssData->numSatellites;
	positionStr.num_gps_plus_glonass_l1 = 0;
	positionStr.num_gps_plus_glonass_l2 = 0;
	positionStr.reserved = 0;
	positionStr.extended_solution_status = 0;
	positionStr.reserved2 = 0;
	positionStr.signals_used_mask = 0;


	crc = CalculateBlockCRC32(positionStr.header.header_length + positionStr.header.message_length, (unsigned char *)&positionStr);
	memcpy((int8_t *)positionStr.crc, (int8_t *)&crc, 4);
	return 1;

}

int writeVelocityMsg(const int week, const uint32_t gps_millisecs, const GnssData* p_GnssData)
{
	long crc;
	velocityStr.header.sync1 = 0xAA;
	velocityStr.header.sync2 = 0x44;
	velocityStr.header.sync3 = 0x12;
	velocityStr.header.header_length = 28;
	velocityStr.header.message_id = 99;
	velocityStr.header.message_type.format = BINARY;
	velocityStr.header.message_type.response = ORIGINAL_MESSAGE;
	velocityStr.header.port_address = 0;
	velocityStr.header.message_length = 44; //
	velocityStr.header.sequence = 0;
	velocityStr.header.idle = 0;
	velocityStr.header.time_status = 0;
	velocityStr.header.gps_week = week;
	velocityStr.header.gps_millisecs = gps_millisecs;
	velocityStr.header.status = 0;
	velocityStr.header.Reserved = 0;
	velocityStr.header.version = 0;

	velocityStr.latency = 0;
	velocityStr.age = 0;
	velocityStr.horizontal_speed = sqrt(p_GnssData->north_velocity * p_GnssData->north_velocity + p_GnssData->east_velocity * p_GnssData->east_velocity);
	velocityStr.track_over_ground = atan2(p_GnssData->east_velocity, p_GnssData->north_velocity) * RAD2DEG;
	velocityStr.vertical_speed = - p_GnssData->up_velocity;
	crc = CalculateBlockCRC32(velocityStr.header.header_length + velocityStr.header.message_length, (unsigned char *)&velocityStr);
	memcpy((int8_t *)velocityStr.crc, (int8_t *)&crc, 4);
	return 1;
}

int writeGnssRawMsg(const int week, const uint32_t gps_millisecs,const GnssData* p_GnssData)
{
	writePositionMsg(week, gps_millisecs, p_GnssData);
	writeVelocityMsg(week, gps_millisecs, p_GnssData);
	return 1;
}



int writeGGAMsg(int week ,double time,const GnssInsSystem* p_gnssInsSystem)
{
	double ep[6];
    double horizontal_speed; 
	uint8_t type = 0;
	double blh[3];

    if (p_gnssInsSystem->mlc_STATUS == 4) {

        blh[0] = p_gnssInsSystem->outNav.lat;
        blh[1] = p_gnssInsSystem->outNav.lon;
        blh[2] = p_gnssInsSystem->outNav.height;

        gtime_t gpstime = gpst2time(week, time);
        gtime_t utctime = gpst2utc_2(gpstime);
        time2epoch(utctime, ep);

	    // gga type need to be changed
        if (g_ptr_ins_sol->pos_type == 1 || g_ptr_ins_sol->pos_type == 4 || g_ptr_ins_sol->pos_type == 5){
            type = g_ptr_ins_sol->pos_type + 5;
        } else {
            type = 7;
        }
        print_nmea_gga(ep, blh, p_gnssInsSystem->mGnssData.numSatellites, type, p_gnssInsSystem->mGnssData.HDOP, p_gnssInsSystem->mGnssData.sol_age, ggaBuff_bt);
        print_nmea_pashr_bt(ep, pashrBuff_bt);

        horizontal_speed = sqrt(p_gnssInsSystem->outNav.vn * p_gnssInsSystem->outNav.vn + p_gnssInsSystem->outNav.ve * p_gnssInsSystem->outNav.ve);

        print_nmea_rmc(ep, blh, p_gnssInsSystem->outNav.heading * RAD2DEG, horizontal_speed, g_ptr_ins_sol->pos_type, rmcBuff);
    }

	//outnmea_gga(ggaBuff_bt, ep, 2, blh, p_gnssInsSystem->mGnssData.numSatellites, p_gnssInsSystem->mGnssData.HDOP, p_gnssInsSystem->mGnssData.sol_age);

	return 1;
}

int writeOdoDataMsg(const OdoData* p_OdoData)
{
    long crc = 0;
	odoStr.header.sync1 = 0xAA;
    odoStr.header.sync2 = 0x44;
    odoStr.header.sync3 = 0x12;
    odoStr.header.header_length = 28;
    odoStr.header.message_id = 177;
    odoStr.header.message_type.format = BINARY;
    odoStr.header.message_type.response = ORIGINAL_MESSAGE;
    odoStr.header.port_address = 0;
    odoStr.header.message_length = 30; //16
    odoStr.header.sequence = 0;
    odoStr.header.idle = 0;
	odoStr.header.time_status = 7;
    odoStr.header.gps_week = p_OdoData->week;
    odoStr.header.gps_millisecs = (int)(p_OdoData->timestampd * 1000);

    odoStr.header.status = 0;
    odoStr.header.Reserved = 0;
    odoStr.header.version = 0;

    odoStr.gps_millisecs = (int)(p_OdoData->timestamp * 1000);
    odoStr.speed = p_OdoData->vehicle_speed;
    odoStr.mode = p_OdoData->mode;
    odoStr.week = p_OdoData->week;
    odoStr.fwd = p_OdoData->fwd;
    odoStr.wheel_tick = p_OdoData->wheel_tick;
    crc = CalculateBlockCRC32(odoStr.header.header_length + odoStr.header.message_length, (unsigned char *)&odoStr);
    memcpy((int8_t *)odoStr.crc, (int8_t *)&crc, 4);
	return 1;
}
int writeINSPVAXMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem)
{
	long crc;
	int32_t n = p_gnssInsSystem->mKalmanStruct.n;
	inspvaxstr.header.sync1 = 0xAA;
	inspvaxstr.header.sync2 = 0x44;
	inspvaxstr.header.sync3 = 0x12;
	inspvaxstr.header.header_length = 28;
	inspvaxstr.header.message_id = 1465;
	inspvaxstr.header.message_type.format = BINARY;
	inspvaxstr.header.message_type.response = ORIGINAL_MESSAGE;
	inspvaxstr.header.port_address = 0;
	inspvaxstr.header.message_length = 126; //
	inspvaxstr.header.sequence = 0;
	inspvaxstr.header.idle = 0;
	inspvaxstr.header.time_status = 0;
	inspvaxstr.header.gps_week = week;
	inspvaxstr.header.gps_millisecs = itow;
	inspvaxstr.ins_status = (int)p_gnssInsSystem->ins_status;
	inspvaxstr.pos_type = (int)p_gnssInsSystem->ins_positin_type;
	inspvaxstr.latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	inspvaxstr.longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	inspvaxstr.height = p_gnssInsSystem->outNav.height;
	inspvaxstr.north_velocity = p_gnssInsSystem->outNav.vn;
	inspvaxstr.east_velocity = p_gnssInsSystem->outNav.ve;
	inspvaxstr.up_velocity = -p_gnssInsSystem->outNav.vd;
	inspvaxstr.roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	inspvaxstr.pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	inspvaxstr.azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	inspvaxstr.latitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[0]);
	inspvaxstr.longitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[n+1]);
	inspvaxstr.altitude_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[2*n+2]);
	inspvaxstr.north_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[3*n+3]);
	inspvaxstr.east_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[4*n+4]);
	inspvaxstr.up_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[5*n+5]);
	inspvaxstr.roll_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[6*n+6])* RAD2DEG;
	inspvaxstr.pitch_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[7*n+7])* RAD2DEG;
	inspvaxstr.azimuth_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[8*n+8])* RAD2DEG;
	inspvaxstr.Ext_sol_stat = 0;
	inspvaxstr.time_since_update = (int16_t)p_gnssInsSystem->GNSSLOSETIME1;
	crc = CalculateBlockCRC32(inspvaxstr.header.header_length + inspvaxstr.header.message_length, (unsigned char *)&inspvaxstr);
	memcpy((int8_t *)inspvaxstr.crc, (int8_t *)&crc, 4);
	return 1;
}

int writeINSPVAMsg(int32_t week, uint32_t itow, const GnssInsSystem* p_gnssInsSystem)
{
	long crc;
	inspvastr.header.sync1 = 0xAA;
	inspvastr.header.sync2 = 0x44;
	inspvastr.header.sync3 = 0x12;
	inspvastr.header.header_length = 28;
	inspvastr.header.message_id = 507;
	inspvastr.header.message_type.format = BINARY;
	inspvastr.header.message_type.response = ORIGINAL_MESSAGE;
	inspvastr.header.port_address = 0;
	inspvastr.header.message_length = 88; //
	inspvastr.header.sequence = 0;
	inspvastr.header.idle = 0;
	inspvastr.header.time_status = 0;
	inspvastr.header.gps_week = week;
	inspvastr.header.gps_millisecs = itow;
	inspvastr.gps_week = week;
	inspvastr.gps_millisecs = itow;
	inspvastr.latitude = p_gnssInsSystem->outNav.lat * RAD2DEG;
	inspvastr.longitude = p_gnssInsSystem->outNav.lon * RAD2DEG;
	inspvastr.height = p_gnssInsSystem->outNav.height;
	inspvastr.north_velocity = p_gnssInsSystem->outNav.vn;
	inspvastr.east_velocity = p_gnssInsSystem->outNav.ve;
	inspvastr.up_velocity = -p_gnssInsSystem->outNav.vd;
	inspvastr.roll = p_gnssInsSystem->outNav.roll * RAD2DEG;
	inspvastr.pitch = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	inspvastr.azimuth = p_gnssInsSystem->outNav.heading * RAD2DEG;
	inspvastr.status = p_gnssInsSystem->mlc_STATUS;
	crc = CalculateBlockCRC32(inspvastr.header.header_length + inspvastr.header.message_length, (unsigned char *)&inspvastr);
	memcpy((int8_t *)inspvastr.crc, (int8_t *)&crc, 4);
	return 1;
}