/*------------------------------------------------------------------------------
* Orientation.c : Matrix operation
*
*          Copyright (C) 2018 by ZhengpengXu, All rights reserved.
*
* version : $Revision: 1.0 $ $Date: 2018/10/19 20:00:00 $
* history : 2018/10/19 1.0 new
*-----------------------------------------------------------------------------*/
#include "orientation.h"
#include <math.h>
#include <stdio.h>

//euler angles to direction cosine matrix  
void euler2dcm(const double eular[3], double dc[3][3])
{
	double roll = eular[0];
	double  pitch = eular[1];
	double  heading = eular[2];
	double  cr, cp, ch, sr, sp, sh;
	cr = cos(roll); cp = cos(pitch); ch = cos(heading);
	sr = sin(roll); sp = sin(pitch); sh = sin(heading);

	dc[0][0] = cp * ch;
	dc[0][1] = -cr * sh + sr * sp*ch;
	dc[0][2] = sr * sh + cr * sp*ch;

	dc[1][0] = cp * sh;
	dc[1][1] = cr * ch + sr * sp*sh;
	dc[1][2] = -sr * ch + cr * sp * sh;

	dc[2][0] = -sp;
	dc[2][1] = sr * cp;
	dc[2][2] = cr * cp;
}
//euler angles to quaternions  
void  euler2quat(const double eular[3], double q[4])
{
	double c_r = cos(eular[0] / 2);
	double c_p = cos(eular[1] / 2);
	double c_h = cos(eular[2] / 2);
	double s_r = sin(eular[0] / 2);
	double s_p = sin(eular[1] / 2);
	double s_h = sin(eular[2] / 2);
	q[0] = c_r * c_p * c_h + s_r * s_p * s_h;
	q[1] = s_r * c_p * c_h - c_r * s_p * s_h;
	q[2] = c_r * s_p * c_h + s_r * c_p * s_h;
	q[3] = c_r * c_p * s_h - s_r * s_p * c_h;
}
//direction cosine matrix to euler angles
void dcm2euler(const double dc[3][3], double eular[3])
{
	double roll, pitch, heading;
	pitch = atan(-dc[2][0] / sqrt(dc[2][1] * dc[2][1] + dc[2][2] * dc[2][2]));
	if (dc[2][0] <= -0.999)
	{
		roll = atan2(dc[2][1], dc[2][2]);
		heading = atan2((dc[1][2] - dc[0][1]), (dc[0][2] + dc[1][1]));
	}
	else if (dc[2][0] >= 0.999)
	{
		roll = atan2(dc[2][1], dc[2][2]);
		heading = PI + atan2((dc[1][2] + dc[0][1]), (dc[0][2] - dc[1][1]));
	}
	else
	{
		roll = atan2(dc[2][1], dc[2][2]);
		heading = atan2(dc[1][0], dc[0][0]);
	}
	if (heading < 0)// heading 0-360
	{
		heading = 2 * PI + heading;
	}
	else if (heading > 2 * PI) heading -= 2 * PI;
	eular[0] = roll;
	eular[1] = pitch;
	eular[2] = heading;
}
//direction cosine matrix to quaternions
void dcm2quat(const double dc[3][3], double q[4])
{

}
//quaternions to euler angles
void quat2euler()
{

}
//quaternions to direction cosine matrix 
void quat2dcm(const double q[4], double dc[3][3])
{
	dc[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	dc[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	dc[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);

	dc[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
	dc[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	dc[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);

	dc[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	dc[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
	dc[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

//referance frames and transformation
//Compute the DCM (C_ne) representing the attitude of the navigation frame from the latitude and longitude.
void pos2dcm(double lat, double lon, double C_ne[3][3])
{
	double sinB = sin(lat);
	double sinL = sin(lon);
	double cosB = cos(lat);
	double cosL = cos(lon);

	C_ne[0][0] = -sinB * cosL;
	C_ne[0][1] = -sinL;
	C_ne[0][2] = -cosB * cosL;

	C_ne[1][0] = -sinB * sinL;
	C_ne[1][1] = cosL;
	C_ne[1][2] = -cosB * sinL;

	C_ne[2][0] = cosB;
	C_ne[2][1] = 0;
	C_ne[2][2] = -sinB;
}
//Compute the quaternion (q_ne) representing the attitude of the navigation frame.
void pos2quat(double lat, double lon, double q[4])
{
	double cos_L = cos(lon / 2);
	double sin_L = sin(lon / 2);
	double cos_B = cos(-PI / 4 - lat / 2);
	double sin_B = sin(-PI / 4 - lat / 2);
	q[0] = cos_B * cos_L;
	q[1] = -sin_B * sin_L;
	q[2] = sin_B * cos_L;
	q[3] = cos_B * sin_L;
}
// Position error to rotation vector conversion
// void dpos2rvec(double delta[3], double rv[3])
//{
//	rv[0] = delta[2] * cos(delta[0]);
//	rv[1] = -delta[1];
//	rv[2] = -delta[2] * sin(delta[0]);
//}
// Position error to rotation vector conversion
void dpos2rvec(double lat, double delta_lat, double delta_lon, double rv[3])
{
	rv[0] = delta_lon * cos(lat);
	rv[1] = -delta_lat;
	rv[2] = -delta_lon * sin(lat);
}


