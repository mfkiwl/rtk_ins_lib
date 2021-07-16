/*------------------------------------------------------------------------------
* Orientation.c : Matrix operation
*
*          Copyright (C) 2018 by ZhengpengXu, All rights reserved.
*
* version : $Revision: 1.0 $ $Date: 2018/10/19 20:00:00 $
* history : 2018/10/19 1.0 new
*-----------------------------------------------------------------------------*/
#ifndef		__ORIENTATION_H__
#define		__ORIENTATION_H__
#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif
#include <math.h>
#include <stdio.h>

	//euler angles to direction cosine matrix  
void euler2dcm(const double eular[3], double dc[3][3]);
	//euler angles to quaternions  
void  euler2quat(const double eular[3], double q[4]);
	//direction cosine matrix to euler angles
void dcm2euler(const double dc[3][3], double eular[3]);
	//direction cosine matrix to quaternions
void dcm2quat(const double dc[3][3], double q[4]);
	//quaternions to euler angles
void quat2euler();
	//quaternions to direction cosine matrix 
void quat2dcm(const double q[4], double dc[3][3]);

	//referance frames and transformation
	//Compute the DCM (C_ne) representing the attitude of the navigation frame from the latitude and longitude.
void pos2dcm(double lat, double lon, double C_ne[3][3]);
	//Compute the quaternion (q_ne) representing the attitude of the navigation frame.
void pos2quat(double lat, double lon, double q[4]);
	// Position error to rotation vector conversion
	// void dpos2rvec(double delta[3], double rv[3])
	//{
	//	rv[0] = delta[2] * cos(delta[0]);
	//	rv[1] = -delta[1];
	//	rv[2] = -delta[2] * sin(delta[0]);
	//}
	// Position error to rotation vector conversion
void dpos2rvec(double lat, double delta_lat, double delta_lon, double rv[3]);


#endif 