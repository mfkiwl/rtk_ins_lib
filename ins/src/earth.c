#include "earth.h"
#include "math.h"


const GravityParameter normg = { 9.7803267715 ,0.0052790414 ,0.0000232718,-0.000003087691089,0.00000000439773 , 0.000000000000721 };
const  EarthParameter WGS84 = { 6378137.0, 6356752.3142, 0.0033528106643315515,0.081819190837555025,0.0066943799893122479 ,  7.2922115147e-5,398600441800000.00 };


uint8_t UpdateMN(const double *BLH, double *M, double *N)
{
	double sinB = sin(*BLH);
	double temp = 1 - WGS84.e2 * sinB * sinB;
	double sqrttemp = sqrt(temp);
	*M = WGS84.a * (1 - WGS84.e2) / (sqrttemp*temp);
	*N = WGS84.a / sqrttemp;
	return 1;
};
uint8_t UpdateW(const double *BLH, const double M, const double N, const double *vn, double *wnie, double *wnen)
{
	double cosB = cos(*BLH);
	double sinB = sin(*BLH);
	double tanB = tan(*BLH);
	wnie[0] = WGS84.wie * cosB;
	wnie[1] = 0.0;
	wnie[2] = -WGS84.wie * sinB;
	wnen[0] = vn[1] / (N + BLH[2]);
	wnen[1] = -vn[0] / (M + BLH[2]);
	wnen[2] = -vn[1] * tanB / (N + BLH[2]);
	return 1;
};
uint8_t UpdateWnie(const double *BLH, const double M, const double N, double *wnie)
{
	double cosB = cos(*BLH);
	double sinB = sin(*BLH);
	wnie[0] = WGS84.wie * cosB;
	wnie[1] = 0.0;
	wnie[2] = -WGS84.wie * sinB;
	return 1;
};
uint8_t UpdateWnen(const double *BLH, const double M, const double N, const double *vn, double *wnen)
{
	double tanB = tan(*BLH);
	wnen[0] = vn[1] / (N + BLH[2]);
	wnen[1] = -vn[0] / (M + BLH[2]);
	wnen[2] = -vn[1] * tanB / (N + BLH[2]);
	return 1;
};

uint8_t UpdateGravity(const double *BLH, double* ng)
{
	double sinB = sin(*BLH);
	double s2 = sinB * sinB;
	double s4 = s2 * s2;
	*ng = normg.g0 * (1.0f + normg.g1 * s2 + normg.g2 * s4) + (normg.g3 + normg.g4 * s2) * BLH[2] + normg.g5 * BLH[2] * BLH[2];
	return 1;
}

int8_t llh2ecef(const Geo* mGeo, ECEF* mECEF)
{
	double M, N;
	UpdateMN(&(mGeo->lat), &M, &N);
	double c_lat, s_lat, c_lon, s_lon, Rn_h;
	c_lat = cos(mGeo->lat);
	s_lat = sin(mGeo->lat);
	c_lon = cos(mGeo->lon);
	s_lon = sin(mGeo->lon);
	Rn_h = N + mGeo->height;
	mECEF->x = Rn_h * c_lat*c_lon;
	mECEF->y = Rn_h * c_lat*s_lon;
	mECEF->z = (N*(1 - WGS84.e2) + mGeo->height)*s_lat;
	return 1;
}
int8_t ecef2llh(const ECEF* mECEF, Geo* mGeo)
{
	double M, N;
	mGeo->lon = atan2(mECEF->y, mECEF->x);
	double p = sqrt(mECEF->x*mECEF->x + mECEF->y * mECEF->y);
	mGeo->lat = 0.0f;
	UpdateMN(&(mGeo->lat), &M, &N);
	mGeo->height = p / cos(mGeo->lat) - N;
	mGeo->lat = atan2(mECEF->z, p * (1 - WGS84.e2 * N / (N + mGeo->height)));
	for (int i = 0; i < 4; i++)
	{
		UpdateMN(&(mGeo->lat), &M, &N);
		mGeo->height = p / cos(mGeo->lat) - N;
		mGeo->lat = atan2(mECEF->x, p * (1 - WGS84.e2 * N / (N + mGeo->height)));
	}
	return 1;
}