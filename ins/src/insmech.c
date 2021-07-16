#include "insmech.h"

#include <stdio.h>
#include<string.h>
#include "lcstruct.h"
#include "cmatrix.h"
#include "earth.h"
#include "orientation.h"
#include "matrix_wu.h"


#include "stdio.h"
#include "stdlib.h"
#include "time.h"
static double temp1[3] = {0.0,0.0,0.0};
static double temp2[3] = {0.0,0.0,0.0};
static double temp3[3] = {0.0,0.0,0.0};
static double q1[4] = {0.0,0.0,0.0,0.0};
static double q2[4] = {0.0,0.0,0.0,0.0};
static double q3[4] = {0.0,0.0,0.0,0.0};
static double C1[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double C2[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double C3[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
static double eye33[3][3] = { {1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0} };
static float eye33float[3][3] = {{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0}};

static double dvel_b_prev[3] ;
static double dtheta_b_prev[3] ;
static double dvel_b_cur[3];
static double dtheta_b_cur[3];

int8_t KF_feedback(Par* mPar, float x[15], Nav* mNav, int IsBiasFeedBack, GnssInsSystem *mGnssInsSystem)
{
	int ret = -1;
	int i = 0;
	double d_lat = (double)(x[0]) / ((double)(mNav->height) + mPar->Rm);
	double d_lon = (double)(x[1]) / (double)(mPar->Rn + mNav->height) / cos(mNav->lat);
	dpos2rvec(mNav->lat, d_lat, d_lon, temp1);
	for (i = 0; i < 3; i++)
	{
		temp2[i] = -temp1[i];
	}
	rvec2quat(temp2, q1);
	quatprod(mNav->q_ne, q1, q2);
	quat2pos(q2, temp3);
	mNav->lat = temp3[0];
	mNav->lon = temp3[1];
	mNav->height = mNav->height + x[2];
	memcpy(mNav->q_ne, q2, 4*sizeof(double));

	GetSkewSymmetricMatrixOfVector(temp1, *C1);
	MatrixAdd(*eye33, *C1, 3, 3, *C2);


	temp2[0] = (double)(mNav->vn - x[3]);
	temp2[1] = (double)(mNav->ve - x[4]);
	temp2[2] = (double)(mNav->vd - x[5]);

	MatrixMutiply(*C2, temp2, 3, 3, 1, temp3);
	mNav->vn = (float)temp3[0];
	mNav->ve = (float)temp3[1];
	mNav->vd = (float)temp3[2];
	temp2[0] = (double)x[6];
	temp2[1] = (double)x[7];
	temp2[2] = (double)x[8] ;

	MatrixAdd(temp2, temp1, 3, 1, temp3);
	rvec2quat(temp3, q1);
	quatprod(q1, mNav->q_bn, q2);
	memcpy(mNav->q_bn, q2, 4 * sizeof(double));

	quat2dcm(mNav->q_bn, mNav->c_bn);
	dcm2euler(mNav->c_bn, temp3);
	mNav->roll = (float)temp3[0];
	mNav->pitch = (float)temp3[1];
	mNav->heading = (float)temp3[2];

	if (IsBiasFeedBack)
	{
		mNav->sensorbias.bias_gyro_x += x[9];
		mNav->sensorbias.bias_gyro_y += x[10];
		mNav->sensorbias.bias_gyro_z += x[11];
		mNav->sensorbias.bias_acc_x += x[12];
		mNav->sensorbias.bias_acc_y += x[13];
		mNav->sensorbias.bias_acc_z += x[14];
	}

	
	if (16 == mGnssInsSystem->mKalmanStruct.n)
	{
		if (mGnssInsSystem-> mImuData.timestamp - mGnssInsSystem->lastGNSSLCTime < 1.2
			&& fabs(mGnssInsSystem->mOdoData.vehicle_speed) > 5.0
			&& fabs(mGnssInsSystem->mNav.wnb_n[2]) < 0.03
			&&mGnssInsSystem->premGnssData.Mode == 4
			&&mGnssInsSystem->mGnssData.Mode == 4)
		{
			mNav->Odo_scale += x[15];
		}
	}

	memset(x, 0, mGnssInsSystem->mKalmanStruct.n * sizeof(float));
    ret = 1;
	return ret;
}

int8_t DataChangeLC(const ImuData* mImuData, INS *cins)
{
	int ret = -1;
	cins->timestamp = mImuData->timestamp;
	cins->timestamped = mImuData->timestamped;
	cins->gyro_x = mImuData->Gyrox;
	cins->gyro_y = mImuData->Gyroy;
	cins->gyro_z = mImuData->Gyroz;
	cins->acc_x = mImuData->Accx;
	cins->acc_y = mImuData->Accy;
	cins->acc_z = mImuData->Accz;
	ret = 1;
	return ret;
}

int8_t compensate( const Sensorbias* bias, INS* ins)
{
	ins->acc_x -= bias->bias_acc_x;
	ins->acc_y -= bias->bias_acc_y;
	ins->acc_z -= bias->bias_acc_z;
	ins->gyro_x -= bias->bias_gyro_x;
	ins->gyro_y -= bias->bias_gyro_y;
	ins->gyro_z -= bias->bias_gyro_z;
	return 1;
}

static double zeta[3];
static double mid_v[3] = { 0.0 };
static double dv_fb[3] = { 0.0 };
static double dv_fn[3] = { 0.0 };

int8_t INS_MECH(const INS* ins_pre, const INS* ins_cur, const Nav* nav, Nav* nav1, Par* mPar)
{
	unsigned char i;
	//Calculat earthhparents  use navparements
	double dt = ins_cur->timestamp - ins_pre->timestamp;
	//Date prepare
	dvel_b_prev[0] = ins_pre->acc_x * dt;
	dvel_b_prev[1] = ins_pre->acc_y * dt ;
	dvel_b_prev[2] = ins_pre->acc_z * dt ;
	dtheta_b_prev[0] = ins_pre->gyro_x* dt;
	dtheta_b_prev[1] = ins_pre->gyro_y * dt;
	dtheta_b_prev[2] = ins_pre->gyro_z* dt ;
	dvel_b_cur[0] = ins_cur->acc_x * dt;
	dvel_b_cur[1] =  ins_cur->acc_y * dt;
	dvel_b_cur[2] =  ins_cur->acc_z * dt ;
	dtheta_b_cur[0] = ins_cur->gyro_x* dt;
	dtheta_b_cur[1] = ins_cur->gyro_y* dt;
	dtheta_b_cur[2] = ins_cur->gyro_z * dt ;


	CrossProduct(dtheta_b_cur, dvel_b_cur, temp1);
	CrossProduct(dtheta_b_prev, dvel_b_cur, temp2);
	CrossProduct(dvel_b_prev, dtheta_b_cur, temp3);
	for (int i = 0; i < 3; i++)
	{
		dv_fb[i] = dvel_b_cur[i] + 0.5*temp1[i] + (temp2[i] + temp3[i]) / 12;
	}

	mPar->g[0] = 0.0;
	mPar->g[1] = 0.0;
	UpdateGravity(&(nav->lat), &(mPar->g[2]));
	UpdateMN(&(nav->lat), &mPar->Rm, &mPar->Rn);

	//V*************************Velcocity Updata***************************************
	//position extrapolation
	//earth and angular rate updateing
	mid_v[0] =(double)(nav->vn) + 0.5 * nav->dv_n[0];
	mid_v[1] =(double)(nav->ve) + 0.5 * nav->dv_n[1];
	mid_v[2] =(double)(nav->vd) + 0.5 * nav->dv_n[2];

	mPar->w_ie[0] = WGS84.wie * cos(nav->lat);
	mPar->w_ie[1] = 0.0;
	mPar->w_ie[2] = -WGS84.wie * sin(nav->lat);

	mPar->w_en[0] = (double)(nav->ve) / (mPar->Rn + nav->height);
	mPar->w_en[1] = -(double)(nav->vn) / (mPar->Rm + nav->height);
	mPar->w_en[2] = -(double)(nav->ve) * tan(nav->lat) / (mPar->Rn + (double)(nav->height));
	// navigation frame rotation vector

	for (i = 0; i < 3; i++)
	{
		temp1[i] = 0.5*(mPar->w_ie[i] + mPar->w_en[i]) * dt;
	}
	GetSkewSymmetricMatrixOfVector(temp1, *C1);
	MatrixSub(*eye33, *C1, 3, 3, *C2);
	MatrixMutiply(*C2, *nav->c_bn, 3, 3, 3, *C3);
	MatrixMutiply(*C3, dv_fb, 3, 3, 1, dv_fn);




	for (i = 0; i < 3; i++)
	{
		temp1[i] = 2 * mPar->w_ie[i] + mPar->w_en[i];
	}
	CrossProduct(temp1, mid_v, temp2);
	for (i = 0; i < 3; i++)
	{
		nav1->dv_n[i] = dv_fn[i] + (mPar->g[i] - temp2[i])*dt;
		nav1->a_n[i] = nav1->dv_n[i] / dt;
	}
	nav1->vn = nav->vn + (float)(nav1->dv_n[0]);
	nav1->ve = nav->ve + (float)(nav1->dv_n[1]);
	nav1->vd = nav->vd + (float)(nav1->dv_n[2]);

	//**************************Postion Updata**************************
	mid_v[0] = 0.5*(double)(nav1->vn + nav->vn);
	mid_v[1] = 0.5*(double)(nav1->ve + nav->ve);
	mid_v[2] = 0.5*(double)(nav1->vd + nav->vd);
	mPar->w_en[0] = mid_v[1] / (mPar->Rn + (double)(nav->height));
	mPar->w_en[1] = -mid_v[0] / (mPar->Rm + (double)(nav->height));
	mPar->w_en[2] = -mid_v[1] * tan(nav->lat) / (mPar->Rn + (double)(nav->height));
	for (i = 0; i < 3; i++)
	{
		zeta[i] = (mPar->w_ie[i] + mPar->w_en[i]) * dt;
	}
	rvec2quat(zeta, q1);  
	temp3[0] = 0.0;
	temp3[1] = 0.0;
	temp3[2] = -WGS84.wie*dt;

	rvec2quat(temp3, q2);
	quatprod(nav->q_ne, q1, q3);
	quatprod(q2, q3, nav1->q_ne);
	norm_quat(nav1->q_ne);
	quat2pos(nav1->q_ne, temp1);  

	nav1->lat = temp1[0];
	nav1->lon = temp1[1];
	nav1->height = nav->height - mid_v[2] * dt;
	//*************************Attitude Updata*****************************

	//Calculate rotational and sculling motion
	CrossProduct(dtheta_b_prev, dtheta_b_cur, temp1);
	for (i = 0; i < 3; ++i)
	{
		temp2[i] = dtheta_b_cur[i] + temp1[i] / 12;
	}
	rvec2quat(temp2, q2); 

	for (i = 0; i < 3; i++)
	{
		temp3[i] = -zeta[i];
	}
	rvec2quat(temp3, q1);  

	quatprod(nav->q_bn, q2, q3);
	quatprod(q1, q3, nav1->q_bn);
	norm_quat(nav1->q_bn);
	quat2dcm(nav1->q_bn, nav1->c_bn);
	dcm2euler(nav1->c_bn, temp1);
	MatrixTranspose(*nav1->c_bn, 3, 3, *nav1->c_nb);

	nav1->roll = (float)temp1[0];
	nav1->pitch = (float)temp1[1];
	nav1->heading = (float)temp1[2];

	for (i = 0; i < 3; i++)
	{
		mPar->f_n[i] = dv_fn[i] / dt;
		mPar->f_b[i] = dvel_b_cur[i] / dt;
		mPar->w_b[i] = dtheta_b_cur[i] / dt;
		temp2[i] = (mPar->w_ie[i] + mPar->w_en[i]);
	}
	MatrixMutiply(*(nav1->c_bn), mPar->w_b, 3, 3, 1, temp1);
	for (i = 0; i < 3; i++)
	{
		nav1->wnb_n[i] = temp1[i] - temp2[i];
		MatrixMutiply(*nav1->c_nb, nav1->wnb_n, 3, 3, 1, nav1->wnb_b);
	}

	return 1;
}

/***************************************************************************************
Function: KF_predict_16PHI
Description: ;Calculate PHI(transition matrix)
Input :dt  time increment
	   nav  Navigation information
Output:PHI  transition matrix
Return :
Others:
********************************************************************************************/
int8_t KF_predict_16PHI(const float dt, const Nav *mNav, const Par *mPar, const ImuSensor *mImuSensor, const int16_t n,float* PHI)
{
	double R = sqrt(mPar->Rm* mPar->Rn);

	//PHI = 15x15/16*16 transition matrix
	memset(PHI, 0, n * n * sizeof(float));
	//Postion error dynamics
	//pos to pos
	for (int i = 0; i < 3; i++)
	{
		temp1[i] = -mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp1, *C1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			PHI[i*n + j] =(float)(eye33[i][j] + C1[i][j] * dt);
		}
	}
	//pos to vel
	for (int i = 0; i < 3; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			PHI[i*n + j] = (float)(eye33[i][j - 3] * dt);
		}
	}
	//Velocity error dynamics
	//vel to pose
	PHI[3*n + 0] = -(float)(mPar->g[2] / (R + (double)mNav->height)*dt);
	PHI[4*n + 1] = PHI[3 * n];
	PHI[5*n + 2] = -2 * PHI[3 * n];
	//vel to vel
	for (int i = 0; i < 3; i++)
	{
		temp2[i] = -2 * mPar->w_ie[i] - mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp2, *C2);
	for (int i = 3; i < 6; i++)
	{
		for (int j = 3; j < 6; j++)
		{
			PHI[i*n + j] = (float)(eye33[i - 3][j - 3] + C2[i - 3][j - 3] * dt);
		}
	}
	//vel to att
	GetSkewSymmetricMatrixOfVector(mPar->f_n, *C3);
	for (int i = 3; i < 6; i++)
	{
		for (int j = 6; j < 9; j++)
		{
			PHI[i*n + j] = (float)(C3[i - 3][j - 6] * dt);
		}
	}
	//vel to acc bias
	for (int i = 3; i < 6; i++)
	{
		for (int j = 12; j < 15; j++)
		{
			PHI[i*n + j] = (float)(mNav->c_bn[i - 3][j - 12] * dt);
		}
	}
	//attitude error dynamic
	for (int i = 0; i < 3; i++)
	{
		temp3[i] = -mPar->w_ie[i] - mPar->w_en[i];
	}
	GetSkewSymmetricMatrixOfVector(temp3, *C3);
	for (int i = 6; i < 9; i++)
	{
		for (int j = 6; j < 9; j++)
		{
			PHI[i*n + j] = (float)(eye33[i - 6][j - 6] + C3[i - 6][j - 6] * dt);
		}
	}
	//att to gyro bias
	for (int i = 6; i < 9; i++)
	{
		for (int j = 9; j < 12; j++)
		{
			PHI[i*n + j] = -(float)(mNav->c_bn[i - 6][j - 9] * dt);
		}
	}

	//gyro bias
	PHI[9 * n + 9] = mImuSensor->bg_model[0];
	PHI[10 * n + 10] = mImuSensor->bg_model[1];
	PHI[11 * n + 11] = mImuSensor->bg_model[2];
	//acc bias
	PHI[12*n+12] = mImuSensor->ba_model[0];
	PHI[13*n+13] = mImuSensor->ba_model[1];
	PHI[14*n+14] = mImuSensor->ba_model[2];
	if (16 == n)
	{
		PHI[15 * n + 15] = 1;
	}
	return 1;
}
/*******************************************************************************************
Function: KF_predict
Description: ;
Input :x    state vect,old
	   P    Covariance,old
	   PHI  transition matrix
	   Q    the continuous-time system noise
	   dt   time increment
Output:x1  state vect new
	   P1  Covariance new
Return :
Others:
*************************************************************************************************/
int8_t  KF_predict(const int16_t n,const float* PHI, const float* Q, const float dt, float* x, float* P, UpdataStruct* updataStruct)
{
	float Qd[StateX2];
	float M1[StateX2],M2[StateX2];

		PHI_Q(n,PHI, Q, M1);//PHI*Q
		PHIQ_QPHIT(n, M1, dt, Qd);
		//if (n == 16) { Qd[255] = Q[15]; };
	
		PHI_P(n, PHI, (updataStruct->Q), M1); //PHI*Q
		PHIP_PHIT(n, M1, PHI, M2);  //PHI*Q*PHI_T
		MatrixAddfloat(M2, Qd, n, n, (updataStruct->Q));

	

		PHI_P(n, PHI, (updataStruct->PHI), M1);
		memcpy(updataStruct->PHI, M1, n * n * sizeof(float));



		PHI_P(n, PHI, P, M1);
		PHIP_PHIT(n, M1, PHI, M2);
		MatrixAddfloat(M2, Qd, n, n, P);
	
	return 1;
}



