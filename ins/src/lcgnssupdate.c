#include <math.h>
#include <string.h>
#include "lcgnssupdate.h"
#include "lcsystem.h"
#include "lcstruct.h"
#include "earth.h"
#include "orientation.h"
#include "cmatrix.h"
#include "zuptdetect.h"
#include "lcinsupdate.h"

#ifndef NHCSTARTTIME
#define  NHCSTARTTIME (20)
#endif // !LIMITACCNORMAL
#ifndef NHCTIMEINTERVAL1
#define  NHCTIMEINTERVAL1 (0.3)
#endif // !LIMITACCNORMAL
#ifndef NHCTIMEINTERVAL2
#define  NHCTIMEINTERVAL2 (0.8)
#endif // !LIMITACCNORMAL
#ifndef NHCTIME
#define  NHCTIME (0.5)
#endif // !LIMITACCNORMAL

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif
extern LCSetting mLCSetting;
extern GnssInsSystem mGnssInsSystem;
extern UpdataStruct mUpdataStruct;
extern int8_t isEFKFinished; 
extern int8_t PositionInitflag;


#define RE_WGS84    6378137.0               /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563)     /* earth flattening (WGS84) */

static int lastflag = 0;
static double eye33[3][3] = {{1,0,0},{0,1,0},{0,0,1} };

#ifndef POSINNOTHRESHLDRED1
#define POSINNOTHRESHLDRED1 12
#endif // !POSINNOTHRESHLDRED1
#ifndef POSINNOTHRESHLDRED2
#define POSINNOTHRESHLDRED2 6
#endif // !POSINNOTHRESHLDRED1
#ifndef POSINNOTHRESHLDRED3
#define POSINNOTHRESHLDRED3 2.5
#endif // !POSINNOTHRESHLDRED1

//10s 内GNSS固定解算且速度大于5m/s ， INS组合解算结果同GNSS差异 位置差异大于1m，平均值，系统hardreset 120S



static void blh2C_en(const double *blh, double C_en[3][3])
{
	/* blh => C_en */
	double lat = blh[0], lon = blh[1]; /*, ht = blh[2];*/
	C_en[0][0] = -sin(lat) * cos(lon);
	C_en[1][0] = -sin(lat) * sin(lon);
	C_en[2][0] = cos(lat);
	C_en[0][1] = -sin(lon);
	C_en[1][1] = cos(lon);
	C_en[2][1] = 0.0;
	C_en[0][2] = -cos(lat) * cos(lon);
	C_en[1][2] = -cos(lat) * sin(lon);
	C_en[2][2] = -sin(lat);
	return;
}
static void pos2ecef(const double *pos, double *r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]);
	double cosl = cos(pos[1]);
	double e2 = FE_WGS84 * (2.0 - FE_WGS84);
	double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

	r[0] = (v + pos[2]) * cosp * cosl;
	r[1] = (v + pos[2]) * cosp * sinl;
	r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}
static void blhdiff(double *blh, double *blh_ref, double *ned)
{
	double C_en[3][3] = { 0 };
	double xyz[3] = { 0 };
	double xyz_ref[3] = { 0 };
	double dxyz[3] = { 0 };

	blh2C_en(blh_ref, C_en);
	pos2ecef(blh_ref, xyz_ref);
	pos2ecef(blh, xyz);
	dxyz[0] = xyz[0] - xyz_ref[0];
	dxyz[1] = xyz[1] - xyz_ref[1];
	dxyz[2] = xyz[2] - xyz_ref[2];
	ned[0] = C_en[0][0] * dxyz[0] + C_en[1][0] * dxyz[1] + C_en[2][0] * dxyz[2];
	ned[1] = C_en[0][1] * dxyz[0] + C_en[1][1] * dxyz[1] + C_en[2][1] * dxyz[2];
	ned[2] = C_en[0][2] * dxyz[0] + C_en[1][2] * dxyz[1] + C_en[2][2] * dxyz[2];
	return;
}
static int8_t MotionAlign(const GnssData* premGnssData, const GnssData* mGnssData, uint8_t GnssType, Nav *mNav)
{
	int8_t retal = 0;
	double Vel_NED[3] = {0.0,0.0,0.0};
	double vehchile_speed = 0.0;
	// double vehchile_speed_M = 4.0;//mLCSetting.imuSensorType == 2 ? 10 : 5;
	// double vehchile_speed_M = 0;//mLCSetting.imuSensorType == 2 ? 10 : 5;
	// double vehchile_speed_M = 2;//mLCSetting.imuSensorType == 2 ? 10 : 5;
	double vehchile_speed_M = 2;//mLCSetting.imuSensorType == 2 ? 10 : 5;
	//double vehchile_speed_M = mLCSetting.imuSensorType == 2 ? 10 : 2;

	double dt = mGnssData->timestamp - premGnssData->timestamp;
	if (premGnssData->timestamp < 0) return 0;
	if (1 == GnssType )
	{
		if (mGnssData->Mode >= 4 && premGnssData->Mode >= 4 && dt < 2.0 && dt > 0.4)
		{
		    double prepos[3] = { premGnssData->latitude, premGnssData->longitude ,premGnssData->altitude };
		    double pos[3] = { mGnssData->latitude, mGnssData->longitude ,mGnssData->altitude };

		    double M, N;
		    UpdateMN(pos, &M, &N);

		    Vel_NED[0] = (M + pos[2])*(pos[0] - prepos[0]) / dt;
		    Vel_NED[1] = (N + pos[2])*(pos[1] - prepos[1])*cos(pos[0]) / dt;
		    Vel_NED[2] = -(pos[2] - prepos[2]) / dt;
		    vehchile_speed = sqrt(Vel_NED[0] * Vel_NED[0] + Vel_NED[1] * Vel_NED[1]);
		}
		else
		{
			lastflag = 0;
		}
	}
	if (3 == GnssType && mLCSetting.gnssDataRate != 10)
	{
		if (mGnssData->Mode >= 4  && premGnssData->Mode >= 4 && dt < 2.0 && dt > 0.4)
		{
			Vel_NED[0] = mGnssData->north_velocity;
			Vel_NED[1] = mGnssData->east_velocity;
			Vel_NED[2] = -mGnssData->up_velocity;
			vehchile_speed = sqrt(Vel_NED[0] * Vel_NED[0] + Vel_NED[1] * Vel_NED[1]);
		}
		else
		{
			lastflag = 0;
		}
	}
	else if (3 == GnssType && mLCSetting.gnssDataRate == 10)
	{
		if (mGnssData->Mode >= 4 && premGnssData->Mode >= 4)
		{
			Vel_NED[0] = mGnssData->north_velocity;
			Vel_NED[1] = mGnssData->east_velocity;
			Vel_NED[2] = -mGnssData->up_velocity;
			vehchile_speed = sqrt(Vel_NED[0] * Vel_NED[0] + Vel_NED[1] * Vel_NED[1]);
		}
		else
		{
			lastflag = 0;
		}
	}

	int8_t is_line_motion = IsOnLine();
	if (vehchile_speed > vehchile_speed_M && is_line_motion) 
	{
		if (lastflag == 1 && (mGnssData->Mode == 4 || mLCSetting.gnssDataRate == 10))
		{
			mNav->lon = mGnssData->longitude;
			mNav->lat = mGnssData->latitude;
			mNav->height = mGnssData->altitude;
			mNav->vn = Vel_NED[0];
			mNav->ve = Vel_NED[1];
			mNav->vd = Vel_NED[2];
			mNav->roll = 0.0;
			mNav->pitch = 0.0;
			double heading = atan2(mNav->ve, mNav->vn);
			if (heading < 0)
			{
				heading += 2 * PI;
			}
			if (heading > 2 * PI)
			{
				heading -= 2 * PI;
			}
			mNav->heading = heading;
			retal = 1;
			lastflag = 0;
		}
		lastflag = 1;
	}
	else
	{
		lastflag = 0;
	}
	return retal;
}
static int8_t DualantennaAlign(const GnssData* premGnssData, const GnssData* mGnssData, uint8_t GnssType, Nav *mNav)
{
	int8_t retal = 0;
	double Vel_NED[3];
	double vehchile_speed = 0.0;
	
	if (GnssType == 0)
	{
		double prepos[3] = { premGnssData->latitude, premGnssData->longitude ,premGnssData->altitude };
		double pos[3] = { mGnssData->latitude, mGnssData->longitude ,mGnssData->altitude };

		double M, N;
		UpdateMN(pos, &M, &N);
		double dt = (mGnssData->timestamp - premGnssData->timestamp);
		if (dt < 1.1 && dt > 0.9)
		{
			Vel_NED[0] = (M + pos[2])*(pos[0] - prepos[0]) / dt;
			Vel_NED[1] = (N + pos[2])*(pos[1] - prepos[1])*cos(pos[0]) / dt;
			Vel_NED[2] = -(pos[2] - prepos[2]) / dt;
			vehchile_speed = sqrt(Vel_NED[0] * Vel_NED[0] + Vel_NED[1] * Vel_NED[1]);
		}
	}
	if (GnssType == 1)
	{
		Vel_NED[0] = mGnssData->north_velocity;
		Vel_NED[1] = mGnssData->east_velocity;
		Vel_NED[2] = -mGnssData->up_velocity;
		vehchile_speed = sqrt(Vel_NED[0] * Vel_NED[0] + Vel_NED[1] * Vel_NED[1]);
	}
	if (0)
	{
		mNav->lon = mGnssData->longitude;
		mNav->lat = mGnssData->latitude;
		mNav->height = mGnssData->altitude;
		mNav->vn = Vel_NED[0];// Vel_NED[0];
		mNav->ve = Vel_NED[1]; //Vel_NED[1];
		mNav->vd = Vel_NED[2];// Vel_NED[2];
		mNav->roll = 0.0;
		mNav->pitch = 0.0;
		double heading = mGnssData->heading; //mGnssData->heading;
		if (heading < 0)
		{
			heading += 2 * PI;
		}
		if (heading >= 2 * PI)
		{
			heading -= 2 * PI;
		}
		mNav->heading = heading;
		retal = 1;
	}
	return retal;
}
static int8_t ekf_measurement_update(float *x, float* P, const float *H, const int *L, float z, float R, int n, int m)
{
	int i, j;
	double PHt[16];
	for (i = 0; i < n; i++)
	{
		PHt[i] = 0.0;
		for (j = 0; j < m; ++j)
		{
			PHt[i] += P[i*n + L[j]] * H[j];
		}
	}

	float R_ = 0.0;

		for (i = 0; i < m; ++i)
		{
			R_ += H[i] * PHt[L[i]];
		}
	
	float P_inov = R_ + R;

	float z_ = 0.0;
	for (i = 0; i < m; ++i)
	{
		z_ += H[i] * x[L[i]];
	}
	float inno = 0.0;
	inno = z - z_;
	/* P_inov = R + H*P*H' */
	/* inov = z-H*x */
	/* x = x + K * inov = x + P*H'*inv(P_inov)*inov */
	for (i = 0; i < n; ++i)
	{
		x[i] += PHt[i] * inno / P_inov;
	}
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j < n; ++j)
		{
			P[i*n+j] =(float)((double)P[i*n+j] - (PHt[i] * PHt[j])/ (double)P_inov);
		}
	}
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j < i; ++j)
		{
			P[i*n+j] = 0.5*(P[i*n + j]+ P[j*n + i]);
			P[j*n+i] = P[i*n+j];
		}
	}



	return 1;
}

static int8_t SetGNSSPOSMeasZ(const GnssData *mGnssData, float* Z)
{
	int ret = 1;
	ECEF r_gps_E;
	Geo r_gps;

	r_gps.lat = mGnssData->latitude;
	r_gps.lon = mGnssData->longitude;
	r_gps.height = mGnssData->altitude;
	llh2ecef(&r_gps, &r_gps_E);
	double r_gps_e[3] = { r_gps_E.x, r_gps_E.y, r_gps_E.z };


	Geo r_ins;
	ECEF r_ins_E;
	r_ins.lat = mUpdataStruct.mPVA.latitude;
	r_ins.lon = mUpdataStruct.mPVA.longitude;
	r_ins.height = mUpdataStruct.mPVA.altitude;
	llh2ecef(&r_ins, &r_ins_E);
	double r_ins_e[3] = { r_ins_E.x, r_ins_E.y, r_ins_E.z };

	double Cne[3][3];
	double Cen[3][3];
	pos2dcm(r_gps.lat, r_gps.lon, Cne);
	MatrixTranspose(*Cne, 3, 3, *Cen);

	double  larm_rn[3];


	MatrixMutiply(*mUpdataStruct.C_bn, mGnssInsSystem.leverarm_b, 3, 3, 1, larm_rn);

	double diff_r_e[3], diff_r_n[3],Z1[3];
	MatrixSub(r_ins_e, r_gps_e, 3, 1, diff_r_e);
	MatrixMutiply(*Cen, diff_r_e, 3, 3, 1, diff_r_n);
	MatrixAdd(diff_r_n, larm_rn, 3, 1, Z1);
	Z[0] = (float)Z1[0];
	Z[1] = (float)Z1[1];
	Z[2] = (float)Z1[2];
	ret = 1;
	return ret;
}

static int8_t SetGNSSVELMeaz(const GnssData *mGnssData, float* Z)
{
	int ret = 1;
	double Vel_NED[3] = { mUpdataStruct.mPVA.northVelocity, mUpdataStruct.mPVA.eastVelocity,  mUpdataStruct.mPVA.downVelocity };
	double Vel_GNSS[3] = { mGnssData->north_velocity,  mGnssData->east_velocity,   -mGnssData->up_velocity};
	double la_r[3],w_in[3],C1[3][3],Z1[3];
	MatrixMutiply(*mUpdataStruct.C_bn, mGnssInsSystem.leverarm_b, 3, 3, 1, la_r);
	MatrixAdd(mUpdataStruct.mPVA.w_ie, mUpdataStruct.mPVA.w_en, 3, 1, w_in);
	double temp1[3];
	CrossProduct(w_in, la_r, temp1);
	double temp2[3], temp3[3];
	CrossProduct(mGnssInsSystem.leverarm_b, mUpdataStruct.mPVA.w_ib, temp2);
	MatrixMutiply(*mUpdataStruct.C_bn, temp2, 3, 3, 1, temp3);
	double temp4[3];
	MatrixAdd(temp1, temp3, 3, 1, temp4);
	double temp5[3];
	MatrixSub(Vel_NED, Vel_GNSS,3,1, temp5);
	MatrixSub(temp5, temp4, 3, 1, Z1);
	GetSkewSymmetricMatrixOfVector(mUpdataStruct.mPVA.wnb_n, *C1);   //C1 Mwnbn
	MatrixMutiply(*C1, mGnssInsSystem.odoleverarm_b, 3, 3, 1, temp1);  //temp1 Mbv

	double INS_Vv_Odo[3];
	MatrixAdd(Vel_GNSS, temp1, 3, 1, Vel_NED); //Vel_NED INS_Vn_Odo

	double C_vn[3][3], C_nv[3][3];
	memcpy(C_vn, eye33, 9 * sizeof(double));
	memcpy(C_nv, eye33, 9 * sizeof(double));

	if (mLCSetting.isUseMisAlignment)
	{
		MatrixMutiply(*mUpdataStruct.C_bn, *mGnssInsSystem.C_InstallationAngle, 3, 3, 3, *C_vn);
		MatrixTranspose(*C_vn, 3, 3, *C_nv);
	}
	else
	{
		MatrixTranspose(*mUpdataStruct.C_bn, 3, 3, *C_nv);
	}

	MatrixMutiply(*C_nv, Vel_NED, 3, 3, 1, INS_Vv_Odo);


	if (fabs(INS_Vv_Odo[1]) > 1.5 || fabs(INS_Vv_Odo[2]) > 1)  
	{
		ret = 0;
		return ret;
	}
	Z[0] = (float)Z1[0];
	Z[1] = (float)Z1[1];
	Z[2] = (float)Z1[2];

	ret = 1;
	return ret;

}

static int8_t SetZuptMeasZ(float *Z)
{
	int ret = -1;
	Z[0] = mUpdataStruct.mPVA.northVelocity;
	Z[1] = mUpdataStruct.mPVA.eastVelocity;
	Z[2] = mUpdataStruct.mPVA.downVelocity;
	ret = 1;
	return ret;
}

static int8_t GNSSVELupdate()
{
	int8_t ret = -1;
	double temp1[3];
	double C1[3][3];
	GetSkewSymmetricMatrixOfVector(mUpdataStruct.mPVA.wnb_n, *C1);   //C1 Mwnbn
	MatrixMutiply(*C1, mGnssInsSystem.odoleverarm_b, 3, 3, 1, temp1);  //temp1 Mbv

	double Vel_NED[3] = { mUpdataStruct.mPVA.northVelocity, mUpdataStruct.mPVA.eastVelocity,  mUpdataStruct.mPVA.downVelocity };
	double INS_Vv_Odo[3];
	MatrixAdd(Vel_NED, temp1, 3, 1, Vel_NED); //Vel_NED INS_Vn_Odo

	double  C_vn[3][3], C_nv[3][3], C_bv[3][3];

	memcpy(C_vn, eye33, 9 * sizeof(double));
	memcpy(C_nv, eye33, 9 * sizeof(double));
	memcpy(C_bv, eye33, 9 * sizeof(double));

	if (mLCSetting.isUseMisAlignment)
	{
		MatrixMutiply(*mUpdataStruct.C_bn, *mGnssInsSystem.C_InstallationAngle, 3, 3, 3, *C_vn);
		MatrixTranspose(*C_vn, 3, 3, *C_nv);
	}
	else
	{
		MatrixTranspose(*mUpdataStruct.C_bn, 3, 3, *C_nv);
	}

	MatrixMutiply(*C_nv, Vel_NED, 3, 3, 1, INS_Vv_Odo);
	double v = sqrt(mGnssInsSystem.mGnssData.north_velocity * mGnssInsSystem.mGnssData.north_velocity
		+ mGnssInsSystem.mGnssData.east_velocity * mGnssInsSystem.mGnssData.east_velocity
		+ mGnssInsSystem.mGnssData.up_velocity * mGnssInsSystem.mGnssData.up_velocity);

	double A_psi[3][3], A_g[3][3];
	GetSkewSymmetricMatrixOfVector(Vel_NED, *C1);
	MatrixMutiply(*C_nv, *C1, 3, 3, 3, *A_psi);


	GetSkewSymmetricMatrixOfVector(mGnssInsSystem.odoleverarm_b, *C1);
	if (mLCSetting.isUseMisAlignment)
	{
		MatrixTranspose(*mGnssInsSystem.C_InstallationAngle, 3, 3, *C_bv);
	}
	MatrixMutiply(*C_bv, *C1, 3, 3, 3, *A_g);


	for (int i = 0; i < 3; ++i)
	{
		float NHCmeasZ = INS_Vv_Odo[i];
		if (i == 0)
		{
			NHCmeasZ = INS_Vv_Odo[0] - v;
		}

		memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n*sizeof(float));
		memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n*sizeof(int));
		mGnssInsSystem.mKalmanStruct.H[0] = C_nv[i][0];
		mGnssInsSystem.mKalmanStruct.H[1] = C_nv[i][1];
		mGnssInsSystem.mKalmanStruct.H[2] = C_nv[i][2];
		mGnssInsSystem.mKalmanStruct.H[3] = -A_psi[i][0];
		mGnssInsSystem.mKalmanStruct.H[4] = -A_psi[i][1];
		mGnssInsSystem.mKalmanStruct.H[5] = -A_psi[i][2];
		mGnssInsSystem.mKalmanStruct.H[6] = -A_g[i][0];
		mGnssInsSystem.mKalmanStruct.H[7] = -A_g[i][1];
		mGnssInsSystem.mKalmanStruct.H[8] = -A_g[i][2];
		mGnssInsSystem.mKalmanStruct.L[0] = 3;
		mGnssInsSystem.mKalmanStruct.L[1] = 4;
		mGnssInsSystem.mKalmanStruct.L[2] = 5;
		mGnssInsSystem.mKalmanStruct.L[3] = 6;
		mGnssInsSystem.mKalmanStruct.L[4] = 7;
		mGnssInsSystem.mKalmanStruct.L[5] = 8;
		mGnssInsSystem.mKalmanStruct.L[6] = 9;
		mGnssInsSystem.mKalmanStruct.L[7] = 10;
		mGnssInsSystem.mKalmanStruct.L[8] = 11;

		int m = 9;
		float NHCR = 0.04;
		if (i == 1)NHCR = 0.01;
		if (i == 2)NHCR = 0.01;
		ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
			NHCmeasZ, NHCR, mGnssInsSystem.mKalmanStruct.n, m);
		for (int j = 0; j < mGnssInsSystem.mKalmanStruct.n; ++j)
		{
			if (mUpdataStruct.P[j*mGnssInsSystem.mKalmanStruct.n+j] < 0)
			{
				InitP(mGnssInsSystem.mGnssData, 0, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mUpdataStruct.P);
			}

		}

	}
	ret = 1;
	return ret;
}
static int8_t Odoupdate()
{
	int8_t ret = -1;
	double temp1[3], temp2[3];
	double C1[3][3];
	GetSkewSymmetricMatrixOfVector(mUpdataStruct.mPVA.wnb_b, *C1);   //C1 Mwnbn
	MatrixMutiply(*C1, mGnssInsSystem.odoleverarm_b, 3, 3, 1, temp2);  //temp1 Mbv
	MatrixMutiply(*mUpdataStruct.C_bn, temp2, 3, 3, 1, temp1);

	double Vel_NED[3] = { mUpdataStruct.mPVA.northVelocity, mUpdataStruct.mPVA.eastVelocity,  mUpdataStruct.mPVA.downVelocity };
	double INS_Vv_Odo[3];
	MatrixAdd(Vel_NED, temp1, 3, 1, Vel_NED); //Vel_NED INS_Vn_Odo

	double  C_vn[3][3], C_nv[3][3], C_bv[3][3];

	memcpy(C_vn, eye33, 9 * sizeof(double));
	memcpy(C_nv, eye33, 9 * sizeof(double));
	memcpy(C_bv, eye33, 9 * sizeof(double));

	if (mLCSetting.isUseMisAlignment)
	{
		MatrixMutiply(*mUpdataStruct.C_bn, *mGnssInsSystem.C_InstallationAngle, 3, 3, 3, *C_vn);
		MatrixTranspose(*C_vn, 3, 3, *C_nv);
	}
	else
	{
		MatrixTranspose(*mUpdataStruct.C_bn, 3, 3, *C_nv);
	}

	MatrixMutiply(*C_nv, Vel_NED, 3, 3, 1, INS_Vv_Odo);
	double v = mGnssInsSystem.mOdoData.vehicle_speed * mGnssInsSystem.mNav.Odo_scale * mLCSetting.odoScale;

	double A_psi[3][3], A_g[3][3];
	GetSkewSymmetricMatrixOfVector(Vel_NED, *C1);
	MatrixMutiply(*C_nv, *C1, 3, 3, 3, *A_psi);


	GetSkewSymmetricMatrixOfVector(mGnssInsSystem.odoleverarm_b, *C1);
	if (mLCSetting.isUseMisAlignment)
	{
		MatrixTranspose(*mGnssInsSystem.C_InstallationAngle, 3, 3, *C_bv);
	}
	MatrixMutiply(*C_bv, *C1, 3, 3, 3, *A_g);
	/*Go straight*/
	if (fabs(v) > 4 && mGnssInsSystem.mOdoData.fwd != -1)
	{
		/*Excessive lateral or Vertical velocity*/
		if (fabs(INS_Vv_Odo[1]) > fabs(INS_Vv_Odo[0]) || fabs(INS_Vv_Odo[2]) > fabs(INS_Vv_Odo[0]))
		{
			mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
			mGnssInsSystem.InitNavi = 0;
			mGnssInsSystem.mpreImuData.timestamp = -1;
			mGnssInsSystem.Isfirstfusionimu = 1;
			ret = -1;
			return ret;
		}
	}
	/*Can bus speed,It may be 0 or negative when reversing*/
	if (1 == mGnssInsSystem.Odomode)
	{
		/*negative when reversing*/
		if (fabs(fabs(v) - fabs(INS_Vv_Odo[0])) < 0.2 && INS_Vv_Odo[0] < 0)
		{
			v = -fabs(v);
		}

		if (mGnssInsSystem.mOdoData.fwd == -1)
		{
			mGnssInsSystem.mOdoData.fwd = 0;
			/*Static but not determine*/
			if (INS_Vv_Odo[0] > 0)
			{
				v = 0.0;
			}
			/*0 when reversing*/
			else
			{
				ret = 2;
			}
		}

		if (fabs(v) < 0.11)
		{
			ret = 2;
		}
	}

	double wnb_n = sqrt(mUpdataStruct.mPVA.wnb_n[2] * mUpdataStruct.mPVA.wnb_n[2] + mUpdataStruct.mPVA.wnb_n[1] * mUpdataStruct.mPVA.wnb_n[1]
		+ mUpdataStruct.mPVA.wnb_n[0] * mUpdataStruct.mPVA.wnb_n[0]);
	double scale = 1.0; //(1.0 + 10 * wnb_n) * (1.0 + 10 * wnb_n) *(5.0 + fabs(INS_Vv_Odo[0])) / 10.0;
	double scale2 = 1.0; //(1.0 + 10 * fabs(mUpdataStruct.mPVA.wnb_n[1])) * (1.0 + 10 * fabs(mUpdataStruct.mPVA.wnb_n[1]));

	double NHCmeasZ[3] = { 0.0 };
	 NHCmeasZ[0] = INS_Vv_Odo[0] - v;
	 NHCmeasZ[1] = INS_Vv_Odo[1];
	 NHCmeasZ[2] = INS_Vv_Odo[2];

	if (fabs(NHCmeasZ[0]) > 12 * sqrt(mUpdataStruct.P[3* mGnssInsSystem.mKalmanStruct.n + 3] + 
		mUpdataStruct.P[4* mGnssInsSystem.mKalmanStruct.n+ 4] + mUpdataStruct.P[5* mGnssInsSystem.mKalmanStruct.n+5] + 0.25))
	{
		ret = 0;
		return ret;
	}

	if (ret != 2) { ret = 1; };

	if (ret == 1)
	{
		mGnssInsSystem.right_odocout++;
		if (mGnssInsSystem.right_odocout > 10)
		{
			mGnssInsSystem.isUseOdo = 1;
		}
		if(mGnssInsSystem.error_odocout > 0)mGnssInsSystem.error_odocout--;
	}
	if (mGnssInsSystem.isUseOdo == 1)
	{
		for (int i = 1; i < 3; ++i)
		{
			memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
			memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
			mGnssInsSystem.mKalmanStruct.H[0] = C_nv[i][0];
			mGnssInsSystem.mKalmanStruct.H[1] = C_nv[i][1];
			mGnssInsSystem.mKalmanStruct.H[2] = C_nv[i][2];
			mGnssInsSystem.mKalmanStruct.H[3] = -A_psi[i][0];
			mGnssInsSystem.mKalmanStruct.H[4] = -A_psi[i][1];
			mGnssInsSystem.mKalmanStruct.H[5] = -A_psi[i][2];
			mGnssInsSystem.mKalmanStruct.H[6] = -A_g[i][0];
			mGnssInsSystem.mKalmanStruct.H[7] = -A_g[i][1];
			mGnssInsSystem.mKalmanStruct.H[8] = -A_g[i][2];
			mGnssInsSystem.mKalmanStruct.L[0] = 3;
			mGnssInsSystem.mKalmanStruct.L[1] = 4;
			mGnssInsSystem.mKalmanStruct.L[2] = 5;
			mGnssInsSystem.mKalmanStruct.L[3] = 6;
			mGnssInsSystem.mKalmanStruct.L[4] = 7;
			mGnssInsSystem.mKalmanStruct.L[5] = 8;
			mGnssInsSystem.mKalmanStruct.L[6] = 9;
			mGnssInsSystem.mKalmanStruct.L[7] = 10;
			mGnssInsSystem.mKalmanStruct.L[8] = 11;

			int m = 9;
			float NHCR = 0.04 * scale;
			if (i == 1)NHCR = 0.04 * scale;
			if (i == 2)NHCR = 0.09 * scale2;


			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
				NHCmeasZ[i], NHCR, mGnssInsSystem.mKalmanStruct.n, m);
			for (int j = 0; j < mGnssInsSystem.mKalmanStruct.n; ++j)
			{
				if (mUpdataStruct.P[j*mGnssInsSystem.mKalmanStruct.n + j] < 0)
				{
					InitP(mGnssInsSystem.mGnssData, 0, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mUpdataStruct.P);
				}

			}

		}
		for (int i = 0; i < 1; ++i)
		{
			memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
			memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
			mGnssInsSystem.mKalmanStruct.H[0] = C_nv[i][0];
			mGnssInsSystem.mKalmanStruct.H[1] = C_nv[i][1];
			mGnssInsSystem.mKalmanStruct.H[2] = C_nv[i][2];
			mGnssInsSystem.mKalmanStruct.H[3] = -A_psi[i][0];
			mGnssInsSystem.mKalmanStruct.H[4] = -A_psi[i][1];
			mGnssInsSystem.mKalmanStruct.H[5] = -A_psi[i][2];
			mGnssInsSystem.mKalmanStruct.H[6] = -A_g[i][0];
			mGnssInsSystem.mKalmanStruct.H[7] = -A_g[i][1];
			mGnssInsSystem.mKalmanStruct.H[8] = -A_g[i][2];
			mGnssInsSystem.mKalmanStruct.H[9] = v;
			mGnssInsSystem.mKalmanStruct.L[0] = 3;
			mGnssInsSystem.mKalmanStruct.L[1] = 4;
			mGnssInsSystem.mKalmanStruct.L[2] = 5;
			mGnssInsSystem.mKalmanStruct.L[3] = 6;
			mGnssInsSystem.mKalmanStruct.L[4] = 7;
			mGnssInsSystem.mKalmanStruct.L[5] = 8;
			mGnssInsSystem.mKalmanStruct.L[6] = 9;
			mGnssInsSystem.mKalmanStruct.L[7] = 10;
			mGnssInsSystem.mKalmanStruct.L[8] = 11;
			mGnssInsSystem.mKalmanStruct.L[9] = 15;
			int m = 9;
			if (mGnssInsSystem.mImuData.timestamp - mGnssInsSystem.lastGNSSLCTime < 1)
			{
				 m = 10;
			}
			float NHCR = 0.001 * scale;


			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
				NHCmeasZ[i], NHCR, mGnssInsSystem.mKalmanStruct.n, m);
			for (int j = 0; j < mGnssInsSystem.mKalmanStruct.n; ++j)
			{
				if (mUpdataStruct.P[j*mGnssInsSystem.mKalmanStruct.n + j] < 0)
				{
					InitP(mGnssInsSystem.mGnssData, 0, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mUpdataStruct.P);
				}

			}

		}
		

		
	}
	return ret;
}

static int8_t NHCupdate()
{
	int8_t ret = -1;
	double temp1[3];
	double C1[3][3];
	GetSkewSymmetricMatrixOfVector(mUpdataStruct.mPVA.wnb_n, *C1);   //C1 Mwnbn

	MatrixMutiply(*C1, mGnssInsSystem.odoleverarm_b, 3, 3, 1, temp1);  //temp1 Mbv

	double Vel_NED[3] = { mUpdataStruct.mPVA.northVelocity, mUpdataStruct.mPVA.eastVelocity,  mUpdataStruct.mPVA.downVelocity };
	double INS_Vv_Odo[3];
	MatrixAdd(Vel_NED, temp1, 3, 1, Vel_NED); //Vel_NED INS_Vn_Odo

	double C_bv[3][3], C_vb[3][3], C_vn[3][3], C_nv[3][3];
	memcpy(C_bv, eye33, 9 * sizeof(double));
	memcpy(C_vb, eye33, 9 * sizeof(double));
	memcpy(C_vn, eye33, 9 * sizeof(double));
	memcpy(C_nv, eye33, 9 * sizeof(double));

	if (mLCSetting.isUseMisAlignment)
	{
		MatrixMutiply(*mUpdataStruct.C_bn, *mGnssInsSystem.C_InstallationAngle, 3, 3, 3, *C_vn);
		MatrixTranspose(*C_vn, 3, 3, *C_nv);
	}
	else
	{
		MatrixTranspose(*mUpdataStruct.C_bn, 3, 3, *C_nv);
	}

	MatrixMutiply(*C_nv, Vel_NED, 3, 3, 1, INS_Vv_Odo);
	/*Go straight*/
	double v = sqrt(INS_Vv_Odo[0] * INS_Vv_Odo[0] + INS_Vv_Odo[1] * INS_Vv_Odo[1] + INS_Vv_Odo[2] * INS_Vv_Odo[2]);
	if (fabs(v) > 4 && mGnssInsSystem.mOdoData.fwd != -1)
	{
		/*Excessive lateral or Vertical velocity*/
		if (fabs(INS_Vv_Odo[1]) > fabs(INS_Vv_Odo[0]) || fabs(INS_Vv_Odo[2]) > fabs(INS_Vv_Odo[0]))
		{
			mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
			mGnssInsSystem.mErrorType = INSSystemExp1; // Course speed mismatch 
			ErrorReset(mGnssInsSystem.mErrorType);

			ret = -1;
			return ret;
		}
	}

	double A_psi[3][3], A_g[3][3];
	GetSkewSymmetricMatrixOfVector(Vel_NED, *C1);
	MatrixMutiply(*C_nv, *C1, 3, 3, 3, *A_psi);


	GetSkewSymmetricMatrixOfVector(mGnssInsSystem.odoleverarm_b, *C1);
	if (mLCSetting.isUseMisAlignment)
	{
		MatrixTranspose(*mGnssInsSystem.C_InstallationAngle, 3, 3, *C_bv);
	}
	MatrixMutiply(*C_bv, *C1, 3, 3, 3, *A_g);
	double wnb_n = sqrt(mUpdataStruct.mPVA.wnb_n[2] * mUpdataStruct.mPVA.wnb_n[2] /*+ mUpdataStruct.mPVA.wnb_n[1] * mUpdataStruct.mPVA.wnb_n[1]
		+ mUpdataStruct.mPVA.wnb_n[0] * mUpdataStruct.mPVA.wnb_n[0]*/);
	double scale = 1.0; //(1.0 + 10 * wnb_n) * (1.0 + 10 * wnb_n) *(5.0 + fabs(INS_Vv_Odo[0])) / 10.0;
	double scale2 = 1.0; //(1.0 + 10 * fabs(mUpdataStruct.mPVA.wnb_n[1])) * (1.0 + 10 * fabs(mUpdataStruct.mPVA.wnb_n[1]));


	for (int i = 1; i < 3; ++i)
	{
		float NHCmeasZ = (float)INS_Vv_Odo[i];
		memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n*sizeof(float));
		memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n*sizeof(int));
		mGnssInsSystem.mKalmanStruct.H[0] = (float)C_nv[i][0];
		mGnssInsSystem.mKalmanStruct.H[1] = (float)C_nv[i][1];
		mGnssInsSystem.mKalmanStruct.H[2] = (float)C_nv[i][2];
		mGnssInsSystem.mKalmanStruct.H[3] = -(float)A_psi[i][0];
		mGnssInsSystem.mKalmanStruct.H[4] = -(float)A_psi[i][1];
		mGnssInsSystem.mKalmanStruct.H[5] = -(float)A_psi[i][2];
		mGnssInsSystem.mKalmanStruct.H[6] = -(float)A_g[i][0];
		mGnssInsSystem.mKalmanStruct.H[7] = -(float)A_g[i][1];
		mGnssInsSystem.mKalmanStruct.H[8] = -(float)A_g[i][2];
		mGnssInsSystem.mKalmanStruct.L[0] = 3;
		mGnssInsSystem.mKalmanStruct.L[1] = 4;
		mGnssInsSystem.mKalmanStruct.L[2] = 5;
		mGnssInsSystem.mKalmanStruct.L[3] = 6;
		mGnssInsSystem.mKalmanStruct.L[4] = 7;
		mGnssInsSystem.mKalmanStruct.L[5] = 8;
		mGnssInsSystem.mKalmanStruct.L[6] = 9;
		mGnssInsSystem.mKalmanStruct.L[7] = 10;
		mGnssInsSystem.mKalmanStruct.L[8] = 11;

		int m = 9;
		float NHCR = 0.04 * scale;
		if (i == 2)NHCR = 0.09 * scale2;

		ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
			NHCmeasZ, NHCR, mGnssInsSystem.mKalmanStruct.n, m);
		for (int j = 0; j < mGnssInsSystem.mKalmanStruct.n; ++j)
		{
			if (mUpdataStruct.P[j*mGnssInsSystem.mKalmanStruct.n + j] < 0)
			{
				InitP(mGnssInsSystem.mGnssData, 0, mGnssInsSystem.mIMUSensor, mGnssInsSystem.mKalmanStruct.n, mUpdataStruct.P);
			}

		}
	}
	ret = 1;
	return ret;

}

/*   ODO  Updata
	args:  double systemtime               ODO RECEIVETME
	return:status ( -2:error reject
	                -1:reject
					 1: OK
					 2: Zupt
					 3: systemerror
					)

*/
int8_t OdoObsUpdata(double systemtime)
{
	int ret = -1;

	if (mUpdataStruct.IsUseZUPT)
	{
		mGnssInsSystem.mMeasUpdataType = ZuptUpdate;
		float ZuptMeasZ[3] = { 0.0 };
		SetZuptMeasZ(ZuptMeasZ);
		for (int i = 0; i < 3; i++)
		{
			memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
			memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
			int m = 1;
			mGnssInsSystem.mKalmanStruct.H[0] = 1;
			mGnssInsSystem.mKalmanStruct.L[0] = i + 3;
			float ZuptR = 0.0001;
			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, ZuptMeasZ[i], ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
		}
		memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
		memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
		if (mUpdataStruct.IsUseZUPTA)  //mUpdataStruct.IsUseZUPTA
		{
			int m = 3;
			mGnssInsSystem.mKalmanStruct.H[0] = 0;
			mGnssInsSystem.mKalmanStruct.H[1] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*sin(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
			mGnssInsSystem.mKalmanStruct.H[2] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*cos(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
			mGnssInsSystem.mKalmanStruct.L[0] = 9;
			mGnssInsSystem.mKalmanStruct.L[1] = 10;
			mGnssInsSystem.mKalmanStruct.L[2] = 11;

			float ZuptR = (0.05 * PI / 180.0) *(0.05 * PI / 180.0);
			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, mUpdataStruct.DiffHEAD, ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
		}
		mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
		mGnssInsSystem.lastZuptTime = systemtime;
		isEFKFinished = 1;
		ret = 2;
		return ret;
	}


	double speed_ins = sqrt(mUpdataStruct.mPVA.northVelocity *mUpdataStruct.mPVA.northVelocity +
		mUpdataStruct.mPVA.eastVelocity * mUpdataStruct.mPVA.eastVelocity);

	if (mLCSetting.isUseNHC)
	{
			mGnssInsSystem.mMeasUpdataType = OdoUpdate;
			int8_t status = Odoupdate();
			if (status == -1)
			{
				ret = 3;
				return ret;
			}
			else if(status == 0)
			{
				ret = -2;
				mGnssInsSystem.right_odocout = 0;
				mGnssInsSystem.error_odocout++;
				if (mGnssInsSystem.error_odocout > 5)
				{
					mGnssInsSystem.isUseOdo = 0;
				}
				return ret;
			}
			else if (status == 2)
			{
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
			}
			else
			{
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | NHCUpdate;
			}
			mGnssInsSystem.lastNHCLCTime = mUpdataStruct.LCTime;
			isEFKFinished = 1;
		
	}
	ret = 1;
	return ret;
}

int8_t VirtualObsUpdata(double systemtime)
{
	int ret = -1;

	if (mUpdataStruct.IsUseZUPT)
	{
		mGnssInsSystem.mMeasUpdataType = ZuptUpdate;
		float ZuptMeasZ[3] = { 0.0 };
		SetZuptMeasZ(ZuptMeasZ);
		for (int i = 0; i < 3; i++)
		{
			memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
			memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
			int m = 1;
			mGnssInsSystem.mKalmanStruct.H[0] = 1;
			mGnssInsSystem.mKalmanStruct.L[0] = i + 3;
			float ZuptR = 0.0001;
			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, ZuptMeasZ[i], ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
		}
		memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
		memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
		if (mUpdataStruct.IsUseZUPTA)  //mUpdataStruct.IsUseZUPTA
		{
			int m = 3;
			mGnssInsSystem.mKalmanStruct.H[0] = 0;
			mGnssInsSystem.mKalmanStruct.H[1] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*sin(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
			mGnssInsSystem.mKalmanStruct.H[2] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*cos(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
			mGnssInsSystem.mKalmanStruct.L[0] = 9;
			mGnssInsSystem.mKalmanStruct.L[1] = 10;
			mGnssInsSystem.mKalmanStruct.L[2] = 11;

			float ZuptR = (0.05 * PI / 180.0) *(0.05 * PI / 180.0);
			ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, mUpdataStruct.DiffHEAD, ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
		}


		mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
		mGnssInsSystem.lastZuptTime = systemtime;
		isEFKFinished = 1;
		ret = 2;
		return ret;
	}


	double speed_ins = sqrt(mUpdataStruct.mPVA.northVelocity *mUpdataStruct.mPVA.northVelocity +
		mUpdataStruct.mPVA.eastVelocity * mUpdataStruct.mPVA.eastVelocity);



	if (mLCSetting.isUseNHC)
	{
			mGnssInsSystem.mMeasUpdataType = NHCUpdate;
			if (NHCupdate() == 1)
			{
				ret = 1;
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
				isEFKFinished = 1;
			}
			else
			{
				ret = -1;
			}
			mGnssInsSystem.lastNHCLCTime = mUpdataStruct.LCTime;		
	}
	return ret;
}




int8_t CheckSystemStability(double* data, int8_t type)
{
	int8_t ret = 0;
	return ret;
}

/*   GNSS  Updata 
	args:  double systemtime               GNSS RECEIVETME
	       GnssData *mGnssData             GINS gnss data struct
	return:status ( -3 : SystemError
					-2 : The frequency is too high
					-1 : unkonw result reject
					 1: OK
					 2: Direct location migration
					)

*/

int8_t ObsUpdata(double systemtime, GnssData mGnssData)
{
	int ret = -1;

	// GNSS USE RATE 
	mGnssInsSystem.GNSSREJRCTType = 0;

	if (mLCSetting.gnssDataRate != 0)
	{
		if (mGnssData.timestamp - mGnssInsSystem.lastObstime < 0.8 / mLCSetting.useGNSSRate)
		{
			mGnssInsSystem.is_gnss_pos_valid = 0;
			ret = -2;
			return ret;
		}
	}
	else
	{
		if (mGnssData.timestamp - mGnssInsSystem.lastObstime < 0.48)
		{
			mGnssInsSystem.is_gnss_pos_valid = 0;
			ret = -2;
			return ret;
		}
	}

	mGnssInsSystem.lastObstime = mGnssData.timestamp; //avoid gnss  arrival time is greater than 0.5S

	/*GNSS中断 判定隧道*/
	int8_t Outstatus = 0;
	if (mGnssData.Mode != 0)
	{
		if ((mGnssInsSystem.premGnssData.timestamp > 0 && mGnssData.timestamp - mGnssInsSystem.premGnssData.timestamp > 60))
		{
			mGnssInsSystem.OutOftunnalTime = mGnssData.timestamp;
		}
		if (mGnssInsSystem.OutOftunnalTime > 0.0001)
		{
			if (mGnssData.timestamp - mGnssInsSystem.OutOftunnalTime < 10)
			{
				Outstatus = 1;
			}
			else if (mGnssData.timestamp - mGnssInsSystem.OutOftunnalTime >= 10
				&& mGnssData.timestamp - mGnssInsSystem.OutOftunnalTime < 60)
			{
				Outstatus = 0;
				mGnssInsSystem.OutOftunnalTime = 0;
			}
			else if (mGnssData.timestamp - mGnssInsSystem.OutOftunnalTime >= 60)
			{
				Outstatus = 0;
				mGnssInsSystem.OutOftunnalTime = 0;
			}
		}
	}


	//降低使用float，特别是在高架下的飞点
	if (mGnssData.Mode == 5)
	{
		if (mGnssInsSystem.premGnssData.gpsFixType < 4)
		{
			mGnssData.Mode = 0;
			mGnssInsSystem.GNSSREJRCTType = SolutionSataChange;
		}
		else if (mGnssData.timestamp - mGnssInsSystem.premGnssData.timestamp > 5)
		{
			mGnssData.Mode = 0;
			mGnssInsSystem.GNSSREJRCTType = SolutionSataChange;
		}
	}
	if (mGnssData.Mode == 1)
	{
		if (mGnssInsSystem.premGnssData.Mode != 1)
		{
			mGnssData.Mode = 0;
			mGnssInsSystem.GNSSREJRCTType = SolutionSataChange;
		}
		else if (mGnssData.timestamp - mGnssInsSystem.premGnssData.timestamp > 5)
		{
			mGnssData.Mode = 0;
			mGnssInsSystem.GNSSREJRCTType = SolutionSataChange;
		}
	}

	if (mLCSetting.isUseHeadOnline && mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime > 120
		&& mGnssInsSystem.firstGNSSUseTime > 0)
	{
		{
			if (mGnssData.gpsFixType > 0) {
				if (mGnssInsSystem.premGnssData.gpsFixType > 0) {

					float gnss_speed = sqrt(mGnssData.east_velocity*mGnssData.east_velocity + mGnssData.north_velocity * mGnssData.north_velocity);
					float dt = mGnssData.timestamp - mGnssInsSystem.premGnssData.timestamp;
					int8_t dnc = 0;
					if (mGnssInsSystem.cnt_bad_gnss == 0 && dt > 5) dnc = 1;
					if (mGnssInsSystem.cnt_bad_gnss == 0)
					{
						mGnssInsSystem.isheadonline = 1;
					}

					if (mGnssInsSystem.cnt_bad_gnss != 0)
					{
						if (!(IsOnLine()))
						{
							mGnssInsSystem.isheadonline = 0;
						}
					}

					if (IsOnLine() && gnss_speed > 6 && !dnc
						&& mGnssInsSystem.isheadonline) {

						double blh_prev[3] = { mGnssInsSystem.premGnssData.latitude, mGnssInsSystem.premGnssData.longitude, mGnssInsSystem.premGnssData.altitude };
						double blh_curr[3] = { mGnssData.latitude, mGnssData.longitude, mGnssData.altitude };
						double ned[3] = { 0.0 };
						blhdiff(blh_curr, blh_prev, ned);
						float gnss_heading = atan2(ned[1], ned[0]);
						float heading_diff = gnss_heading - mGnssInsSystem.outNav.heading;
						if (heading_diff > PI) heading_diff -= 2 * PI;
						if (heading_diff < -PI) heading_diff += 2 * PI;

						float thr_dev = (15.0 - mGnssInsSystem.cnt_bad_gnss*dt);
						if (thr_dev <= 3) {
							thr_dev = 3;
						}
						if (fabs(heading_diff) > thr_dev * PI / 180.0) {
							mGnssData.Mode = 0;
							mGnssInsSystem.GNSSREJRCTType = HeadOnline;
							mGnssInsSystem.is_gnss_pos_valid = 0;
							mGnssInsSystem.cnt_bad_gnss += 1;
							//if (mGnssInsSystem.cnt_bad_gnss == 14) {
							//	mGnssInsSystem.cnt_bad_gnss = 1;
							//}
						}
						else {
							mGnssInsSystem.cnt_bad_gnss = 0;
							mGnssInsSystem.isheadonline = 1;
						}

					}
					else
					{
						mGnssInsSystem.isheadonline = 1;
					}
				}
			}
			else {
				mGnssInsSystem.is_gnss_pos_valid = 0;
			}


		}
	}
	if (fabs(mUpdataStruct.LCTime - mGnssData.timestamp) > 0.08)
	{
		ret = -1;
		mGnssInsSystem.GNSSREJRCTType = GNSSUNKONWN;
		return ret;
	}

	float GNSSPOSMeasZ[3] = { 0.0 };
	float GNSSPOSmeasR[3] = { 0.0 };
	float GNSSVELMeasZ[3] = { 0.0 };
	float GNSSVELmeasR[3] = { 0.0 };
	if (mGnssData.Mode != 0)
	{
		if (!mGnssData.useNativeGNSSPOS)
		{
			GNSSPOSmeasR[0] = mLCSetting.GNSSScale*mGnssData.latitude_std * mGnssData.latitude_std;
			GNSSPOSmeasR[1] = mLCSetting.GNSSScale*mGnssData.longitude_std * mGnssData.longitude_std;
			GNSSPOSmeasR[2] = mLCSetting.GNSSScale  * mGnssData.altitude_std * mGnssData.altitude_std; //4

			if (4 == mGnssData.Mode || 5 == mGnssData.Mode)
			{
				if (GNSSPOSmeasR[2] > 4 * GNSSPOSmeasR[0] || GNSSPOSmeasR[2] > 4 * GNSSPOSmeasR[1])
				{
					GNSSPOSmeasR[2] = GNSSPOSmeasR[0] + GNSSPOSmeasR[1];
				}
			}

			if (1 == mGnssData.Mode)
			{
				double scale = 1.0; // 0.25
				GNSSPOSmeasR[0] = scale * mLCSetting.GNSSScale * mGnssData.latitude_std * mGnssData.latitude_std;
				GNSSPOSmeasR[1] = scale * mLCSetting.GNSSScale * mGnssData.longitude_std * mGnssData.longitude_std;
				GNSSPOSmeasR[2] = scale * mLCSetting.GNSSScale * mGnssData.altitude_std * mGnssData.altitude_std;
			}


			if (mGnssData.Mode == 4)
			{
				if (GNSSPOSmeasR[0] < 0.01 * 0.01)
				{
					GNSSPOSmeasR[0] = 0.01 * 0.01;
				}
				if (GNSSPOSmeasR[1] < 0.01 * 0.01)
				{
					GNSSPOSmeasR[1] = 0.01 * 0.01;
				}					
				if (GNSSPOSmeasR[2] < 0.01 * 0.01)
				{
					GNSSPOSmeasR[2] = 0.01 * 0.01;
				}

			}
			if (mGnssData.Mode == 5)
			{
				if (GNSSPOSmeasR[0] < 0.3 * 0.3)
				{
					GNSSPOSmeasR[0] = 0.3 * 0.3;
				}
				if (GNSSPOSmeasR[1] < 0.3 * 0.3)
				{
					GNSSPOSmeasR[1] = 0.3 * 0.3;
				}
				if (GNSSPOSmeasR[2] < 0.3 * 0.3)
				{
					GNSSPOSmeasR[2] = 0.3 * 0.3;
				}
			}
			if (mGnssData.Mode == 2)
			{
				if (GNSSPOSmeasR[0] < 0.5 * 0.5)
				{
					GNSSPOSmeasR[0] = 0.5 * 0.5;
				}
				if (GNSSPOSmeasR[1] < 0.5 * 0.5)
				{
					GNSSPOSmeasR[1] = 0.5 * 0.5;
				}
				if (GNSSPOSmeasR[2] < 0.5 * 0.5)
				{
					GNSSPOSmeasR[2] = 0.5 * 0.5;
				}
			}
			if (mGnssData.Mode == 1)
			{
				if (GNSSPOSmeasR[0] < 1.0 * 1.0)
				{
					GNSSPOSmeasR[0] = 1.0 * 1.0;
				}
				if (GNSSPOSmeasR[1] < 1.0 * 1.0)
				{
					GNSSPOSmeasR[1] = 1.0 * 1.0;
				}
				if (GNSSPOSmeasR[2] < 1.0 * 1.0)
				{
					GNSSPOSmeasR[2] = 1.0 * 1.0;
				}
			}
			////if (4 == mGnssData.Mode)
			//{

			//	if (mGnssData.latitude_std > 0.1) { mGnssData.latitude_std = 0.1; };
			//	if (mGnssData.longitude_std > 0.1) { mGnssData.longitude_std = 0.1; };
			//	if (mGnssData.altitude_std > 0.1) { mGnssData.altitude_std = 0.1; };

			//	double scale = 1.0; // 0.25
			//	GNSSPOSmeasR[0] = scale * mLCSetting.GNSSScale*mGnssData.latitude_std * mGnssData.latitude_std;
			//	GNSSPOSmeasR[1] = scale * mLCSetting.GNSSScale* mGnssData.longitude_std * mGnssData.longitude_std;
			//	GNSSPOSmeasR[2] = scale * mLCSetting.GNSSScale * mGnssData.altitude_std * mGnssData.altitude_std;
			//}
			}
			else
			{
				GNSSPOSmeasR[0] = 0.09 * mLCSetting.GNSSScale*mGnssData.latitude_std * mGnssData.latitude_std;
				GNSSPOSmeasR[1] = 0.09 * mLCSetting.GNSSScale*mGnssData.longitude_std * mGnssData.longitude_std;
				GNSSPOSmeasR[2] = 0.09 * mLCSetting.GNSSScale* mGnssData.altitude_std * mGnssData.altitude_std;
			}
			if (mLCSetting.gnssSensorType != 0)
			{
				if (GNSSPOSmeasR[0] < 0.0122 *  mLCSetting.GNSSScale
					&& GNSSPOSmeasR[1] < 0.0122 *  mLCSetting.GNSSScale
					&& GNSSPOSmeasR[2] < 0.0122 *  mLCSetting.GNSSScale)
				{
					mGnssData.Mode = 4;
				}
			}
		}

		mGnssInsSystem.GNSSLOSETIME1 = mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime;
		mGnssInsSystem.GNSSVelflag = 0;
		mGnssInsSystem.GNSSPOSHORFLAG = 0;
		mGnssInsSystem.GNSSPOSALTFLAG = 0;
		float gnss_speed = 0; /*calculate by gnss or odo*/
		if (mLCSetting.isUseOdo == 0)
		{
			gnss_speed = sqrt(mGnssData.east_velocity*mGnssData.east_velocity + mGnssData.north_velocity * mGnssData.north_velocity);
		}
		else if (mLCSetting.isUseOdo == 1)
		{
			gnss_speed = mGnssInsSystem.mOdoData.vehicle_speed;
		}

		if (mLCSetting.isUseGNSSVel && mGnssData.Mode != 0)
		{
			mGnssInsSystem.GNSSVelflag = SetGNSSVELMeaz(&mGnssData, GNSSVELMeasZ);
			double inov_v_threshold = 6 * sqrt(mUpdataStruct.P[3 * mGnssInsSystem.mKalmanStruct.n + 3] + mUpdataStruct.P[4 * mGnssInsSystem.mKalmanStruct.n + 4] + mUpdataStruct.P[5 * mGnssInsSystem.mKalmanStruct.n + 5] + 0.25);
		    if (fabs(GNSSVELMeasZ[0]) > inov_v_threshold
				|| fabs(GNSSVELMeasZ[1]) > inov_v_threshold
				|| fabs(GNSSVELMeasZ[2]) > inov_v_threshold)
			{
				mGnssInsSystem.GNSSVelflag = 0;
			}
			/*Assumed velocity affects position*/
			/*
			if (gnss_speed < 5 && mGnssInsSystem.GNSSVelflag == 0 && mGnssData.Mode !=4)
			{
			    mGnssInsSystem.GNSSflag = 0;
			}
			if (mGnssInsSystem.GNSSVelflag == 0 && mGnssData.Mode ==1)
			{
				mGnssInsSystem.GNSSflag = 0;
			}
			*/
		}

		if (!mGnssData.useNativeGNSSPOS && mLCSetting.isUseGNSSVel)
		{
			/*GNSS position inaccuracy under vulgar experience*/

			/*GNSS no fix, under 2m/s ,gnss result is bad*/
			/*
			if (mGnssData.Mode != 0)
			{
				if ((GNSSPOSmeasR[0] > 0.09 *  mLCSetting.GNSSScale|| GNSSPOSmeasR[1] > 0.09  *  mLCSetting.GNSSScale || GNSSPOSmeasR[2] > 0.09 *  mLCSetting.GNSSScale) 
					&& gnss_speed < 2.0)
				{
					mGnssInsSystem.GNSSflag = 0;
				}
			}
			*/

			/*system start GNSS bad float or signal, under 0.5m/s ,gnss result is bad, avoid The system is unstable*/

			if (mGnssData.timestamp - mGnssInsSystem.AlignCTime < 120)
			{
				/*
				if (GNSSPOSmeasR[0] > 0.16 || GNSSPOSmeasR[1] > 0.16 || GNSSPOSmeasR[2] > 0.16)
				{
					mGnssInsSystem.GNSSflag = 0;
				}
				*/
				if ((GNSSPOSmeasR[0] > 0.09   
					|| GNSSPOSmeasR[1] > 0.09 
					|| GNSSPOSmeasR[2] > 0.09 )
					&& gnss_speed < 0.5)
				{
					mGnssInsSystem.GNSSREJRCTType = LowSpeed;
					mGnssInsSystem.GNSSflag = 0;
				}
			}
		}
		
		if (mGnssData.Mode != 0)
		{
			mGnssInsSystem.GNSSPOSHORFLAG = 1;
			mGnssInsSystem.GNSSPOSALTFLAG = 1;
			SetGNSSPOSMeasZ(&mGnssData, GNSSPOSMeasZ);

			/*GNSS is good experience,but The deviation is large,system reset */
			{
				
				int systemGood = SetInnoDetectData(GNSSPOSMeasZ);
				if (mGnssInsSystem.BiasEstStab == 1)
				{
					mGnssInsSystem.SystemStab = 1;
				}
			
				if (mGnssInsSystem.IsGnssGood == 1)
				{

					if (systemGood == 1)
					{
					}
					else if (systemGood == 0)
					{
						// if (mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime > 60 && Outstatus != 1)
						// {
						//	mGnssInsSystem.mErrorType = GnssInnoError;
						//	ErrorReset(mGnssInsSystem.mErrorType);
						//	return -3;
						// }
					}
				}


			}

			int8_t deleteGNSSfalg = 1;
			int8_t scaleGNSSfalg = 1;
			int8_t scaleGNSSfalgHor = 1;
			int8_t scaleGNSSfalgAlt = 1;
			if (mLCSetting.isUseExpQC)
			{

				if (mGnssInsSystem.lastGNSSLCTime > 0 && (mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime > 360))
				{
					if (mGnssData.Mode >= 4 
						&& GNSSPOSmeasR[0] < 0.001  * mLCSetting.GNSSScale 
						&& GNSSPOSmeasR[1] < 0.001  * mLCSetting.GNSSScale 
						&& GNSSPOSmeasR[2] < 0.001  * mLCSetting.GNSSScale)
					{
						//Position Init
						PositionInitFlag = 1;
						mGnssInsSystem.lastGNSSLCTime = mGnssData.timestamp;
						ret = 2;
						return ret;
					}
				}
				if (mLCSetting.gnssSensorType != 0)
				{
					if (mLCSetting.isUseGNSSVel)
					{
						if (fabs(gnss_speed) < 5 && !mGnssData.useNativeGNSSPOS)
						{
							if (GNSSPOSmeasR[0] > 0.05  * mLCSetting.GNSSScale
								|| GNSSPOSmeasR[1] > 0.05  * mLCSetting.GNSSScale)
							{
								mGnssData.Mode = 0;
							}
						}
					}
					if (mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime > 120 && !mGnssData.useNativeGNSSPOS)
					{
						if ((fabs(GNSSPOSMeasZ[0]) > 6 * sqrt(mUpdataStruct.P_r[0])
							|| fabs(GNSSPOSMeasZ[1]) > 6 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n +1])
							|| fabs(GNSSPOSMeasZ[2]) > 6 * sqrt(mUpdataStruct.P_r[2* mGnssInsSystem.mKalmanStruct.n+2]))
							&& (GNSSPOSmeasR[0] > 0.99 * mLCSetting.GNSSScale || GNSSPOSmeasR[1] > 0.99 * mLCSetting.GNSSScale)
							)
						{
							mGnssInsSystem.GNSSflag = 0;
						}
					}
					if (mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime > 120 && !mGnssData.useNativeGNSSPOS)
					{
						if ((fabs(GNSSPOSMeasZ[0]) > 50
							|| fabs(GNSSPOSMeasZ[1]) > 50
							|| fabs(GNSSPOSMeasZ[2]) > 50)
							&& (GNSSPOSmeasR[0] > 0.99 * mLCSetting.GNSSScale || GNSSPOSmeasR[1] > 0.99 * mLCSetting.GNSSScale) //|| GNSSPOSmeasR[2] > 100)
							)
						{
							mGnssInsSystem.GNSSflag = 0;
						}
					}
					if (GNSSPOSmeasR[0] < 0.0122 * mLCSetting.GNSSScale
						&& GNSSPOSmeasR[1] < 0.0122 * mLCSetting.GNSSScale
						&& GNSSPOSmeasR[2] < 0.0122 * mLCSetting.GNSSScale)
					{
						scaleGNSSfalg = 0;
					}
					if (GNSSPOSmeasR[0] < 0.0122 * mLCSetting.GNSSScale
						&& GNSSPOSmeasR[1] < 0.0122 * mLCSetting.GNSSScale)
					{
						scaleGNSSfalgHor = 0;
					}
					if (GNSSPOSmeasR[2] < 0.0122 * mLCSetting.GNSSScale)
					{
						scaleGNSSfalgAlt = 0;
					}
					if (GNSSPOSmeasR[0] < 0.1 * mLCSetting.GNSSScale
						&& GNSSPOSmeasR[1] < 0.1 * mLCSetting.GNSSScale
						&& GNSSPOSmeasR[2] < 0.1 * mLCSetting.GNSSScale)
					{
						deleteGNSSfalg = 0;
					}
				}
			}
			
				double Tre_gnsslc = 2.0;
		if (1 == mLCSetting.isUseOdo)
		{
			Tre_gnsslc = 60.0;
		}
			if (1)
			//if (mGnssData.Mode != 4
			//	|| (mGnssData.Mode == 4 && sqrt(GNSSPOSmeasR[0] + GNSSPOSmeasR[1]) > 0.2))
			{
				if ((mUpdataStruct.P_r[0] < 9 * GNSSPOSmeasR[0] &&
					mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n +1] < 9 * GNSSPOSmeasR[1] &&
					mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n * 2 + 2] < 9 * GNSSPOSmeasR[2])
					|| (mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime < Tre_gnsslc && mGnssInsSystem.lastNHCLCTime > 0 ))//&& mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime-mGnssInsSystem.zupttime2 > 1200))
				{
					mGnssInsSystem.zupttime2 = 0;
					if (deleteGNSSfalg && Outstatus == 0)
					{
						if (fabs(GNSSPOSMeasZ[0]) > POSINNOTHRESHLDRED1 * sqrt(mUpdataStruct.P_r[0] + GNSSPOSmeasR[0]+0.01)
							|| fabs(GNSSPOSMeasZ[1]) > POSINNOTHRESHLDRED1 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n+1] + GNSSPOSmeasR[1]+0.01))
						{
							mGnssInsSystem.GNSSPOSHORFLAG = 0;
							mGnssInsSystem.GNSSflag = 0;
						}
						if (fabs(GNSSPOSMeasZ[2]) > POSINNOTHRESHLDRED1 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n * 2 + 2] + GNSSPOSmeasR[2]+0.01))
						{
							mGnssInsSystem.GNSSPOSALTFLAG = 0;
							GNSSPOSmeasR[2] *= 10000;
						}

					}
					if (scaleGNSSfalgHor && (Outstatus == 0 || Outstatus == 2))
					{
						if (fabs(GNSSPOSMeasZ[0]) > POSINNOTHRESHLDRED3 * sqrt(mUpdataStruct.P_r[0] + GNSSPOSmeasR[0]))
						{
							double scale = fabs(GNSSPOSMeasZ[0]) / sqrt(mUpdataStruct.P_r[0] + GNSSPOSmeasR[0]);
							GNSSPOSmeasR[0] *= scale;
						}

						if (fabs(GNSSPOSMeasZ[1]) > POSINNOTHRESHLDRED3 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n  + 1] + GNSSPOSmeasR[1]))
						{
							double scale = fabs(GNSSPOSMeasZ[1]) / sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n + 1] + GNSSPOSmeasR[1]);
							GNSSPOSmeasR[1] *= scale;
						}
					}
					if (scaleGNSSfalgAlt && (Outstatus == 0 || Outstatus == 2))
					{
					 if (fabs(GNSSPOSMeasZ[2]) > POSINNOTHRESHLDRED3 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n*2 + 2] + GNSSPOSmeasR[2])
						 && fabs(GNSSPOSMeasZ[2]) < POSINNOTHRESHLDRED1 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n * 2 + 2] + GNSSPOSmeasR[2]))
					    {
						double scale = fabs(GNSSPOSMeasZ[2]) / sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n * 2 + 2] + GNSSPOSmeasR[2]);
						GNSSPOSmeasR[2] *= scale;
					    }
					 if (fabs(GNSSPOSMeasZ[2]) > 12 * sqrt(mUpdataStruct.P_r[mGnssInsSystem.mKalmanStruct.n * 2 + 2] + GNSSPOSmeasR[2]))//&& !mGnssData.useNativeGNSSPOS)
					 {
						 GNSSPOSmeasR[2] *= 10000000;
					 }
					}
				}
			}
		}

		/*Assumed velocity affects position*/
		if (mGnssInsSystem.GNSSflag == 0 || mGnssData.Mode == 0)
		{
			mGnssInsSystem.GNSSVelflag = 0;
		}
        
		if (1 == mLCSetting.isUseGNSSZupt)
		{
			if (fabs(GetZuptGNSStime() - mGnssData.timestamp) < 0.001 && mUpdataStruct.IsUseZUPT != 1)
			{
				mUpdataStruct.IsUseZUPT = 1;
			}
		}
		if (mGnssInsSystem.PerMeasUpdata & ZuptUpdate)
		{
			if (mGnssData.Mode != 4) mGnssData.Mode = 0;
		}
		else if (mUpdataStruct.IsUseZUPT)
		{
			mGnssInsSystem.mMeasUpdataType = ZuptUpdate;
			float ZuptMeasZ[3] = { 0.0 };
			SetZuptMeasZ(ZuptMeasZ);
			for (int i = 0; i < 3; i++)
			{
				memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
				memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
				int m = 1;
				mGnssInsSystem.mKalmanStruct.H[0] = 1;
				mGnssInsSystem.mKalmanStruct.L[0] = i + 3;
				float ZuptR = 0.0001;
				ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, ZuptMeasZ[i], ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
			}
			memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
			memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
			if (mUpdataStruct.IsUseZUPTA)  //mUpdataStruct.IsUseZUPTA
			{
				int m = 3;
				mGnssInsSystem.mKalmanStruct.H[0] = 0;
				mGnssInsSystem.mKalmanStruct.H[1] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*sin(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
				mGnssInsSystem.mKalmanStruct.H[2] = (float)((1.0 / cos(mUpdataStruct.mPVA.pitch))*cos(mUpdataStruct.mPVA.roll)* mUpdataStruct.zuptdifftime);
				mGnssInsSystem.mKalmanStruct.L[0] = 9;
				mGnssInsSystem.mKalmanStruct.L[1] = 10;
				mGnssInsSystem.mKalmanStruct.L[2] = 11;

				float ZuptR = (0.05 * PI / 180.0) *(0.05 * PI / 180.0);
				ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L, mUpdataStruct.DiffHEAD, ZuptR, mGnssInsSystem.mKalmanStruct.n, m);
			}


			mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
			mGnssInsSystem.lastZuptTime = mGnssData.timestamp;
			if (fabs(mGnssData.up_velocity) > 2.0
			|| (fabs(mGnssData.up_velocity) > 1.0 && fabs(mGnssData.up_velocity) > 2 * fabs(gnss_speed)))
			{
				mGnssData.Mode = 0;
			}
			if (mGnssData.Mode != 4) mGnssData.Mode = 0;
			isEFKFinished = 1;
		}


		double speed_ins = sqrt(mUpdataStruct.mPVA.northVelocity *mUpdataStruct.mPVA.northVelocity +
			mUpdataStruct.mPVA.eastVelocity * mUpdataStruct.mPVA.eastVelocity);

		if (mLCSetting.isUseGNSSVel && mLCSetting.useGNSSRate == 1 && mLCSetting.isUseNHC)
		{
			if (mGnssInsSystem.GNSSVelflag  && mGnssInsSystem.GNSSflag == 0
				&& !mUpdataStruct.IsUseZUPT)
/*				&& IsOnLine2()
				&& speed_ins > 1.0
				&& (!(!IsOnLine3() && speed_ins > 10)))*/
			{
				mGnssInsSystem.mMeasUpdataType = GNSSVelocityUpdate;
				GNSSVELupdate();
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | NHCUpdate;
				mGnssInsSystem.lastNHCLCTime = mUpdataStruct.LCTime;
				mGnssInsSystem.lastVelLCTime = mUpdataStruct.LCTime;
				isEFKFinished = 1;
			}
		}
		else if (mLCSetting.isUseGNSSVel)
		{
			if (mGnssInsSystem.GNSSflag == 1 && mGnssData.gpsFixType >= 2 && mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime > 0.5
				&& !mUpdataStruct.IsUseZUPT
				&& IsOnLine2()
				&& mGnssInsSystem.GNSSVelflag
				&& speed_ins > 1.0)
			{
				mGnssInsSystem.mMeasUpdataType = GNSSVelocityUpdate;
				for (int i = 0; i < 3; i++)
				{
					GNSSVELmeasR[i] = 0.04;
					memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
					memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
					int m = 1;
					mGnssInsSystem.mKalmanStruct.H[0] = 1;
					mGnssInsSystem.mKalmanStruct.L[0] = i + 3;
					ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
						GNSSVELMeasZ[i], GNSSVELmeasR[i], mGnssInsSystem.mKalmanStruct.n, m);
				}
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
				mGnssInsSystem.lastVelLCTime = mUpdataStruct.LCTime;
			}
		}
/*
		if (mGnssInsSystem.PerMeasUpdata & NHCUpdate)
		{
		}
		else if (mLCSetting.isUseNHC && !mLCSetting.isUseOdo)
		{
			if (mGnssData.timestamp - mGnssInsSystem.firstGNSSUseTime > NHCSTARTTIME
				&& (mGnssData.timestamp - mGnssInsSystem.lastVelLCTime > 0.5 || mLCSetting.gnssDataRate == 0)
				&& mGnssInsSystem.firstGNSSUseTime > 0
				&& IsOnLine2()
				&& mGnssData.Mode < 2
				&& !mUpdataStruct.IsUseZUPT
				&& speed_ins > 1.0
				&& (!(!IsOnLine3() && speed_ins > 10)))
			{
				mGnssInsSystem.mMeasUpdataType = NHCUpdate;
				NHCupdate();
				mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
				mGnssInsSystem.lastNHCLCTime = mUpdataStruct.LCTime;
				isEFKFinished = 1;
			}
		}
		*/


		if (mGnssInsSystem.GNSSflag == 1 
			&& (mGnssData.Mode >= 1 || (mGnssData.useNativeGNSSPOS == 1 && mGnssData.Mode >=1))
			&& mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime > 0.8 / mLCSetting.useGNSSRate)
		{
			mGnssInsSystem.mMeasUpdataType = GNSSPoitionUpdate;
	
				for (int i = 0; i < 3; i++)
				{
					memset(mGnssInsSystem.mKalmanStruct.L, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(int));
					memset(mGnssInsSystem.mKalmanStruct.H, 0, mGnssInsSystem.mKalmanStruct.n * sizeof(float));
					int m = 1;
					mGnssInsSystem.mKalmanStruct.H[0] = 1;
					mGnssInsSystem.mKalmanStruct.L[0] = i;
					ekf_measurement_update(mUpdataStruct.X, mUpdataStruct.P, mGnssInsSystem.mKalmanStruct.H, mGnssInsSystem.mKalmanStruct.L,
						GNSSPOSMeasZ[i], GNSSPOSmeasR[i], mGnssInsSystem.mKalmanStruct.n, m);

				}

			mGnssInsSystem.PerMeasUpdata = mGnssInsSystem.PerMeasUpdata | mGnssInsSystem.mMeasUpdataType;
			mGnssInsSystem.lastGNSSLCTime = mUpdataStruct.LCTime;
			mGnssInsSystem.lastGNSSLCType = mGnssData.Mode;
			mGnssInsSystem.GNSSflag = 0;
			if (mGnssInsSystem.firstGNSSUseTime < 0.5)
			{
				mGnssInsSystem.firstGNSSUseTime = mUpdataStruct.LCTime;
			}

			isEFKFinished = 1;
		}
		ret = 1;
		mGnssInsSystem.GNSSflag = 0;
		return ret;
	
}
/*  ADD origin GNSS data and Updata process
	args:  GnssData *mGnssData              origin gnss data struct
	return:status ( -4 :system init
	                -3 :system error
	                -2 :Position Data unchanged but not zupt
	                -1 : error GNSS Data
	                )

*/
int8_t ADDGNSSDATA(const GnssData mGnssData)
{
	int8_t ret = -1;

	if (mGnssData.week > 1024 && mGnssData.week < 3072)
	{
		if (mGnssInsSystem.firstGNSSTime == -1 && mGnssData.Mode > 0)
		{
			mGnssInsSystem.firstGNSSTime = mGnssData.timestamp;
		}
	}
	else
	{
		mGnssInsSystem.GNSSflag = 0;
		ret = -1;
		return ret;
	}
		////Prevent GNSS data from abnormal
	if ((mGnssData.latitude_std >= 0 && mGnssData.latitude_std < 450)
		&& (mGnssData.longitude_std >= 0 && mGnssData.longitude_std < 450)
		&& (mGnssData.altitude_std >= 0 && mGnssData.altitude_std < 450)
		&& (mGnssData.longitude > -180 * PI / 180 && mGnssData.longitude < 180 * PI / 180)
		&& (mGnssData.latitude > -90 * PI / 180 && mGnssData.latitude < 90 * PI / 180)
		&& (mGnssData.altitude > -10000 && mGnssData.altitude < 10000))
	{
	}
	else
	{
		mGnssInsSystem.GNSSflag = 0;
		ret = -1;
		// return ret;
	}
	if (fabs(mGnssData.latitude - mGnssInsSystem.mGnssData.latitude) < 0.000000001
		&& fabs(mGnssData.longitude - mGnssInsSystem.mGnssData.longitude) < 0.000000001)
	{
		if (mLCSetting.gnssDataType == 3)
		{
			if (fabs(mGnssData.north_velocity) > 0.1 || fabs(mGnssData.east_velocity) > 0.1)
			{
				mGnssInsSystem.GNSSflag = 0;
				ret = -2;
				return ret;
			}
		}
	}

	memcpy(&(mGnssInsSystem.mGnssData), &(mGnssData), sizeof(mGnssData));
	mGnssInsSystem.is_gnss_pos_valid = 1; //For GNSS QC line 

	// Use GNSS Position and Velicity to help detect Vehicle Zupt 
	//  1, When Vehicle Zupt is detected by GNSS ,Cal IMU Std threshold
	//  2, when IMU Std threshold is very different from IMU Std threshold default ,replace it 
	//  3, Control flexible opening
	if (mLCSetting.isUseGNSSZupt)
	{
		if (SetGNSSZuptData(&(mGnssInsSystem.mGnssData), &(mGnssInsSystem.premGnssData)) == 1)
		{
			mGnssInsSystem.GNSSZuptFlag = 1;
		};
	}

	 //Axial inspection and installation angle estimation
     //roll pictch use Mean All
     //GNSS direct  GNSS A > 0.5 m/s2      cal  roll picth

	if (mLCSetting.isOnlineMisAlignmentEst)
	{
		if (mGnssInsSystem.mlc_STATUS > INSDATA_NREADY)
		{
			int8_t MisAlignmentAiax[3] = { 0,0,0 };
			double misAlignment[3] = { 0.0,0.0,0.0 };
			int8_t MisAl_status = SetGNSSFlexDetectData(&(mGnssInsSystem.mGnssData), &(mGnssInsSystem.premGnssData), MisAlignmentAiax, misAlignment);
			if (MisAl_status == 2)
			{
				//打印debug 看安装角估计曲线
#ifdef DEBUGMESSAGE
				tracemis(&(mGnssInsSystem.mImuData), MisAlignmentAiax, misAlignment);
#endif
			}
			if (mGnssInsSystem.mlc_STATUS == INSDATA_READY)
			{
				mGnssInsSystem.mlc_STATUS = MISALIGNMENT_ING;
			}
			else if (mGnssInsSystem.mlc_STATUS == MISALIGNMENT_ING)
			{
				if (MisAl_status == 1)
				{
					mGnssInsSystem.mlc_STATUS = MISALIGNMENT_ING;
				}
				if (MisAl_status == 2)
				{
					mGnssInsSystem.mlc_STATUS = MISALIGNMENT_COMPLETE;
					if (mLCSetting.MisAlignmentAiax[0] != MisAlignmentAiax[0]
						|| mLCSetting.MisAlignmentAiax[1] != MisAlignmentAiax[1]
						|| mLCSetting.MisAlignmentAiax[2] != MisAlignmentAiax[2])
					{
						InitSoftModeAxix(MisAlignmentAiax); //跟最原始的设置有区别
					}
					memcpy(mLCSetting.MisAlignmentAiax, MisAlignmentAiax ,3 * sizeof(int8_t));
					memcpy(mLCSetting.misAlignment, misAlignment, 3 * sizeof(double));
				}
			}
			else
			{
				if (MisAl_status == 2)
				{
					//被移动动初始化，但是这个太晚了，应该是在过程中被判断。 
					if (mLCSetting.MisAlignmentAiax[0] != MisAlignmentAiax[0]
						|| mLCSetting.MisAlignmentAiax[1] != MisAlignmentAiax[1]
						|| mLCSetting.MisAlignmentAiax[2] != MisAlignmentAiax[2])
					{
						mGnssInsSystem.mErrorType = MisAligmentChange;
						InitHardModeAxix(MisAlignmentAiax);
						ErrorReset(mGnssInsSystem.mErrorType);
						//设备被移动，状态重新初始化 IMU 数据无需初始化
						mGnssInsSystem.mlc_STATUS = MISALIGNMENT_COMPLETE;
						memcpy(mLCSetting.MisAlignmentAiax, MisAlignmentAiax, 3 * sizeof(int8_t));
						memcpy(mLCSetting.misAlignment, misAlignment, 3 * sizeof(double));					
					}
					else if (fabs(mLCSetting.misAlignment[0] - misAlignment[0]) > 5 * PI/180
						|| fabs(mLCSetting.misAlignment[1] - misAlignment[1]) > 5 * PI / 180
						|| fabs(mLCSetting.misAlignment[2] - misAlignment[2]) > 5 * PI / 180)
					{
						mGnssInsSystem.mErrorType = MisAligmentChange;
						InitHardModeAxix(MisAlignmentAiax);
						ErrorReset(mGnssInsSystem.mErrorType);

						//设备安装角度朕动，状态重新初始化
						mGnssInsSystem.mlc_STATUS = MISALIGNMENT_COMPLETE;
						memcpy(mLCSetting.MisAlignmentAiax, MisAlignmentAiax, 3 * sizeof(int8_t));
						memcpy(mLCSetting.misAlignment, misAlignment, 3 * sizeof(double));
					}
				}

			}
		}
	}

	//int8_t DectctGNSSStatus()
	{
		//about 10 second V > 5m/s and Gnss is straght and Gnss is fixed

		mGnssInsSystem.IsGnssGood = SetGNSSDetectData(&(mGnssInsSystem.mGnssData), &(mGnssInsSystem.premGnssData));
		
	}



	switch (mGnssInsSystem.mlc_STATUS)
	{
	case INSDATA_NREADY:
	{
	}break;
	case INSDATA_READY:
	{
		if (!mLCSetting.isOnlineMisAlignmentEst)
		{
			mGnssInsSystem.mlc_STATUS = MISALIGNMENT_COMPLETE;
		}
	}break;
	case MISALIGNMENT_ING:
	{
	}break;
	case MISALIGNMENT_COMPLETE:
	{
		mGnssInsSystem.ins_status = INS_ALIGNING;
		switch (mLCSetting.initialAttitudeMode)
		{
		case 0:
		{
			int8_t AlignMentflag = MotionAlign(&mGnssInsSystem.premGnssData, &mGnssData, mLCSetting.gnssDataType, &mGnssInsSystem.mNav);
			if (AlignMentflag)
			{
				mGnssInsSystem.mlc_STATUS = ALIGNMENT_COMPLETE;
				mGnssInsSystem.ins_status = INS_ALIGNMENT_COMPLETE;
				mGnssInsSystem.AlignCTime = mGnssData.timestamp;
#ifdef DEBUGMESSAGE
				printf("INS FUSION :INS_ALIGNMENT_COMPLETE , gnss_week: %d, gnss_time0fweek: %.4f\n", mGnssData.week, mGnssData.timestamp);
#endif // DEBUGMESSAGE
				initsystemSoftreset();
			    initsystemfromGNSS();
				mGnssInsSystem.mlc_STATUS = INS_FUSING;
			}
		}break;
		case 1:
		{
			if (fabs(mGnssData.timestamp - mGnssInsSystem.premGnssData.timestamp) < 1.5)
			{
				int8_t AlignMentflag = DualantennaAlign(&mGnssInsSystem.premGnssData, &mGnssData, mLCSetting.gnssDataRate, &mGnssInsSystem.mNav);

				if (AlignMentflag)
				{
					mGnssInsSystem.mlc_STATUS = ALIGNMENT_COMPLETE;
					mGnssInsSystem.AlignCTime = mGnssData.timestamp;

					initsystemSoftreset();
					initsystemfromGNSS();

					mGnssInsSystem.mlc_STATUS = INS_FUSING;
				}
			}


		}break;
		default:
			ret = -3;
			return ret;
		}
	}break;
	case ALIGNMENT_COMPLETE:
	{
		mGnssInsSystem.mlc_STATUS = INS_FUSING;
	}break;
	case INS_FUSING:
	{
		if (mLCSetting.isInsFreInit == 1)
		{
			if (mGnssInsSystem.mGnssData.Mode > 0) 
			{
				if (mGnssInsSystem.lastGNSSLCTime > 0 && mGnssData.timestamp - mGnssInsSystem.lastGNSSLCTime > mLCSetting.insFreTre)
				{
					mGnssInsSystem.mErrorType = GNSSInterruptMax;
					ErrorReset(mGnssInsSystem.mErrorType);
					mGnssInsSystem.mlc_STATUS = INSDATA_NREADY;
					mGnssInsSystem.InitNavi = 0;
					mGnssInsSystem.Isfirstfusionimu = 1;
					mGnssInsSystem.GNSSflag = 0;
					ret = -4;
					return ret;
				}
			}
		}
		int8_t suc = ObsUpdata(mGnssData.timestampd, mGnssData);
		if (1 != suc)
		{
			mGnssInsSystem.mErrorType = GNSSREJECT;
		}
	}break;
	default:
		ret = -3;
		return ret;
	}
	
	if (mGnssInsSystem.is_gnss_pos_valid) {
		memcpy(&(mGnssInsSystem.premGnssData), &(mGnssData), sizeof(mGnssData));
	}
	mGnssInsSystem.GNSSflag = 0;
	ret = 1;
	return ret;
}

