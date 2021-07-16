#ifndef		__EARTH_H__
#define		__ERATH_H__
#include <stdint.h>

typedef  struct EarthParameter
{
	double a ;       //Ellipsoid long axis
	double b ;       //Ellipsoid short axis
	double f;       //Ellipsoidal oblate 
	double e;       //first Eccentricity of Elliopsoid 
	double e2;
	//double ep;
	//double ep2;     //second Eccentricity of Elliopsoid 
	double wie;     //rotational angular velocity of the earths  
	double GM;      //geocentric gravitational constant 
} EarthParameter;

typedef struct GravityParameter
{
	double g0;
	double g1;
	double g2;
	double g3;
	double g4;
	double g5;
} GravityParameter;



uint8_t UpdateMN(const double *BLH, double *M, double *N);
uint8_t UpdateW(const double *BLH, const double M, const double N, const double *vn, double *wnie, double *wnen);
uint8_t UpdateWnie(const double *BLH, const double M, const double N, double *wnie);
uint8_t UpdateWnen(const double *BLH, const double M, const double N, const double *vn, double *wnen);


uint8_t UpdateGravity(const double *BLH, double* ng);

//Earth-Centered, Earth-Fixed
typedef struct ECEF
{
	double x;
	double y;
	double z;
} ECEF;
//Geographic coordinates 
typedef struct Geo
{
	double lat;
	double lon;
	double height;
} Geo;

int8_t llh2ecef(const Geo* mGeo, ECEF* mECEF);
int8_t ecef2llh(const ECEF* mECEF,Geo* mGeo);


extern const  GravityParameter normg; 
extern const  EarthParameter WGS84;



#endif
