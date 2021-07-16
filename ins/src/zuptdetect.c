#include <math.h>
#include<string.h>
#include "zuptdetect.h"

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

#ifndef DEG2RAD
#define DEG2RAD (0.017453292519943)
#endif // !DEG2RAD
#ifndef RAD2DEG
#define RAD2DEG (57.295779513082323)
#endif // !DEG2RAD

#ifndef RE_WGS84
#define RE_WGS84    6378137.0               /* earth semimajor axis (WGS84) (m) */
#endif
#ifndef FE_WGS84
#define FE_WGS84    (1.0/298.257223563)     /* earth flattening (WGS84) */
#endif // !1


static double ZuptdetectData[50][6];
static int16_t DateLength = 0;

static double ZuptdetectGNSS[10][7];
static int16_t DateLengthGNSS = 0;
static double Rezupttime = -999.99;

static double lastZuptDetTime = -999.99;

static int16_t ZuptValue1 = 0;
static int16_t lastZuptValue1 = 0;
static int16_t ZuptValue2 = 0;
static int16_t ZuptValue3 = 0;

static double ZUPTThreShold[6] = { 0.0 };
static double MISCHANGEThreShold[12] = { 0.0 };


static double  GZmean[8] = { 0.0 };
static double Rawdatamean[8][6] = { 0,0 };
static double THR_STRT_LINE = 0.0873; // 5 deg
static int16_t DateLength_1 = 0;

static double Max[6] = { 0.0 };
static double Mean[6] = { 0.0 };
static double LastMean1[6] = { 0.0 };
static double LastMean2[6] = { 0.0 };

static double Var[6] = { 0.0 };
static double Std[6] = { 0.0 };

//
static double Amean[6] = { 0.0 };  //Stable IMU state and calculate the average value of IMU output
static int64_t date_length = 0.0;

static int8_t GNSSZuptFlag = 0;
static int16_t DateLengthFlex2 = 0;



//Zupt内变量初始化  1、纯静态  2,IMU低通，3,判断转弯直行等 2、GNSS静态 3 GNSS安装角估计
//本质上是无需初始化
int8_t InitOriginalZupt()
{
	int8_t ret = 0;

	lastZuptDetTime = -999.99;
	ZuptValue1 = 0;
	lastZuptValue1 = 0;
	ZuptValue2 = 0;

	DateLength = 0;   //Zupt 数组 
	memset(&ZuptdetectData[0][0], 0, 50 * 6 * sizeof(double)); 

	memset(LastMean2, 0, 6 * sizeof(double));
	memset(LastMean1, 0, 6 * sizeof(double));
	memset(Max, 0, 6 * sizeof(double));
	memset(Mean, 0, 6 * sizeof(double));
	memset(Var, 0, 6 * sizeof(double));
	memset(Std, 0, 6 * sizeof(double));
	ret = 1;
	return ret;
}

int8_t InitMeanBias()
{
	int8_t ret = 0;
	date_length = 0;
	memset(Amean, 0, 6 * sizeof(double));
	ret = 1;
	return ret;
}

static int8_t Zaxix = 3;
int8_t InitPatternRecognition(const int8_t zaxix)
{
	int8_t ret = 0;
	DateLength_1 = 0;
	Zaxix = zaxix;
	memset(&Rawdatamean[0][0], 0,8* 6 * sizeof(double));
	memset(GZmean, 0, 6 * sizeof(double));
	ret = 1;
	return ret;
}

int8_t InitGNSSZupt()
{
	int8_t ret = 0;
	DateLengthGNSS = 0;
	GNSSZuptFlag = 0;
	memset(&ZuptdetectGNSS[0][0], 0, 10 * 7 *sizeof(double));
	ret = 1;
	return ret;
}



int8_t SetZUPTThreShold(int16_t imuenortype,int16_t imudatarate)
{
	int8_t ret = 0;
	enum ImuSensorType
	{
		SPAN_CPT = 0,
		ACEINNA,
		UBLOXM8
	};
	switch (imuenortype)
	{
	case SPAN_CPT:
	{
		ZUPTThreShold[0] = 0.0006;
		ZUPTThreShold[1] = 0.0006;
		ZUPTThreShold[2] = 0.0006;
		ZUPTThreShold[3] = 0.03;
		ZUPTThreShold[4] = 0.03;
		ZUPTThreShold[5] = 0.6;

	}break;
	case ACEINNA:
	{
		if (imudatarate == 50)
		{
			ZUPTThreShold[0] = 0.0015;
			ZUPTThreShold[1] = 0.0015;
			ZUPTThreShold[2] = 0.0015;
			ZUPTThreShold[3] = 0.012;
			ZUPTThreShold[4] = 0.012;
			ZUPTThreShold[5] = 0.012;
		}
		else
		{
			//ZUPTThreShold[0] = 0.0017;
			//ZUPTThreShold[1] = 0.0017;
			//ZUPTThreShold[2] = 0.0017;
			////ZUPTThreShold[3] = 0.012;
			////ZUPTThreShold[4] = 0.012;
			////ZUPTThreShold[5] = 0.012;
			////ZUPTThreShold[0] = 0.004;
			////ZUPTThreShold[1] = 0.004;
			////ZUPTThreShold[2] = 0.0017;

		 ////   ZUPTThreShold[3] = 0.25;
			//////ZUPTThreShold[3] = 0.07;
			////ZUPTThreShold[4] = 0.07;
			////ZUPTThreShold[5] = 0.25;

			////ZUPTThreShold[0] = 0.0017;
			////ZUPTThreShold[1] = 0.0017;
			////ZUPTThreShold[2] = 0.0017;

			//ZUPTThreShold[3] = 0.07;
			//ZUPTThreShold[4] = 0.07;
			//ZUPTThreShold[5] = 0.07;


			/*second* 0.0009 0.009*/
			ZUPTThreShold[0] = 0.0017;
			ZUPTThreShold[1] = 0.0017;
			ZUPTThreShold[2] = 0.0017;

			ZUPTThreShold[3] = 0.03;//007
			ZUPTThreShold[4] = 0.03;
			ZUPTThreShold[5] = 0.3;//

		}

	}break;
	case UBLOXM8:
	{
		if (imudatarate == 10)
		{
			ZUPTThreShold[0] = 0.015;
			ZUPTThreShold[1] = 0.015;
			ZUPTThreShold[2] = 0.015;
			ZUPTThreShold[3] = 0.05;
			ZUPTThreShold[4] = 0.05;
			ZUPTThreShold[5] = 0.05;
		}
		else
		{
			ZUPTThreShold[0] = 0.005;
			ZUPTThreShold[1] = 0.005;
			ZUPTThreShold[2] = 0.005;
			ZUPTThreShold[3] = 0.27;
			ZUPTThreShold[4] = 0.10;
			ZUPTThreShold[5] = 0.10;
		}
	}break;
	default:
		return ret;
	}

	MISCHANGEThreShold[0] = 5 * PI / 180;
	MISCHANGEThreShold[1] = 5 * PI / 180;
	MISCHANGEThreShold[2] = 50 * PI / 180;
	MISCHANGEThreShold[3] = 5 ;
	MISCHANGEThreShold[4] = 5;
	MISCHANGEThreShold[5] = 5;
	MISCHANGEThreShold[6] = 5 * PI / 180;
	MISCHANGEThreShold[7] = 5 * PI / 180;
	MISCHANGEThreShold[8] = 30 * PI / 180;
	MISCHANGEThreShold[9] = 3; 
	MISCHANGEThreShold[10] = 3;
	MISCHANGEThreShold[11] = 3;

	ret = 1;
	return ret;
}

int8_t reSetZUPTThreShold(const int8_t *MisAlignmentAiax)
{
	int8_t ret = 0;
	if (MisAlignmentAiax[0] == 1 && MisAlignmentAiax[1] == 2 && MisAlignmentAiax[2] == 3)
	{
		ret = 1;
	}
	else
	{
		double T[6] = { 0.0 };
		T[0] = ZUPTThreShold[abs(MisAlignmentAiax[0]) - 1];
		T[1] = ZUPTThreShold[abs(MisAlignmentAiax[1]) - 1];
		T[2] = ZUPTThreShold[abs(MisAlignmentAiax[2]) - 1];
		T[3] = ZUPTThreShold[abs(MisAlignmentAiax[0]) + 3 - 1];
		T[4] = ZUPTThreShold[abs(MisAlignmentAiax[1]) + 3 - 1];
		T[5] = ZUPTThreShold[abs(MisAlignmentAiax[2]) + 3 - 1];
		memcpy(ZUPTThreShold, T, 6 * sizeof(double));
		double T1[12] = { 0.0 };
		T1[0] = MISCHANGEThreShold[abs(MisAlignmentAiax[0]) - 1];
		T1[1] = MISCHANGEThreShold[abs(MisAlignmentAiax[1]) - 1];
		T1[2] = MISCHANGEThreShold[abs(MisAlignmentAiax[2]) - 1];
		T1[3] = MISCHANGEThreShold[abs(MisAlignmentAiax[0]) +3- 1];
		T1[4] = MISCHANGEThreShold[abs(MisAlignmentAiax[1]) +3- 1];
		T1[5] = MISCHANGEThreShold[abs(MisAlignmentAiax[2]) +3- 1];
		T1[6] = MISCHANGEThreShold[abs(MisAlignmentAiax[0]) +6- 1];
		T1[7] = MISCHANGEThreShold[abs(MisAlignmentAiax[1]) +6- 1];
		T1[8] = MISCHANGEThreShold[abs(MisAlignmentAiax[2]) +6- 1];
		T1[9] = MISCHANGEThreShold[abs(MisAlignmentAiax[0]) +9- 1];
		T1[10] = MISCHANGEThreShold[abs(MisAlignmentAiax[1]) +9- 1];
		T1[11] = MISCHANGEThreShold[abs(MisAlignmentAiax[2]) +9- 1];
		memcpy(MISCHANGEThreShold, T1, 12 * sizeof(double));


		ret = 1;
	}
	return ret;
}
static int8_t IMUMoveMode = 0;

//运行过程中 安装角发生改变 上机初始化可以
int8_t InitHardModeAxix(const int8_t* MisAlignmentAiax)
{
	int8_t ret = 0;
	IMUMoveMode = 0;
	DateLengthFlex2 = 0;

	ret &= reSetZUPTThreShold(MisAlignmentAiax);
	ret &= InitOriginalZupt();
	ret &= InitMeanBias();
	ret &= InitPatternRecognition(MisAlignmentAiax[2]);
	ret &= InitGNSSZupt();
	return ret;
}
//运行过程中 安装角估计完成
int8_t InitSoftModeAxix(const int8_t* MisAlignmentAiax)
{
	int8_t ret = 0;
	if (GNSSZuptFlag)
	{
	}
	else
	{
		ret &= reSetZUPTThreShold(MisAlignmentAiax);
	}
	ret &= InitPatternRecognition(MisAlignmentAiax[2]);
	return ret;
}
static double MaxImu[6] = { 0.0 };
static double MeanImu[6] = { 0.0 };
static double VarImu[6] = { 0.0 };
static double StdImu[6] = { 0.0 };
/*   ADD  imu data  for  Zupt fuchtion , MisAlignment function , Driving state and so on 

	args:  ImuData *imudata              ins process  imu data struct
	       int16_t imudataRata           ins process imu datarate
	return:status ( -1: Data too short
					 1: ok;
					 2: Data is short for Zupt fuchtion , MisAlignment function , Driving state and so on )

*/
int8_t SetZuptDetectData(const ImuData mImudata,const int16_t imudataRata)
{
	int8_t ret = -1;
	int32_t CheckDataLegth = (int32_t)floor(imudataRata/2);
	if (DateLength == CheckDataLegth)
	{
		for (int16_t i = 0; i < DateLength - 1; i++)
		{
			for (int16_t j = 0; j < 6; j++)
			{
				ZuptdetectData[i][j] = ZuptdetectData[i + 1][j];
			}
		}
		ZuptdetectData[DateLength - 1][0] = mImudata.Gyrox;
		ZuptdetectData[DateLength - 1][1] = mImudata.Gyroy;
		ZuptdetectData[DateLength - 1][2] = mImudata.Gyroz;
		ZuptdetectData[DateLength - 1][3] = mImudata.Accx;
		ZuptdetectData[DateLength - 1][4] = mImudata.Accy;
		ZuptdetectData[DateLength - 1][5] = mImudata.Accz;
	}
	if (DateLength < CheckDataLegth)
	{
		ZuptdetectData[DateLength][0] = mImudata.Gyrox;
		ZuptdetectData[DateLength][1] = mImudata.Gyroy;
		ZuptdetectData[DateLength][2] = mImudata.Gyroz;
		ZuptdetectData[DateLength][3] = mImudata.Accx;
		ZuptdetectData[DateLength][4] = mImudata.Accy;
		ZuptdetectData[DateLength][5] = mImudata.Accz;
		DateLength += 1;
		return ret;
	}
	double dt = mImudata.timestamp - floor(mImudata.timestamp);
	double Dt = 1.0/ imudataRata;
	if ((dt < Dt || (dt > 0.5 - Dt && dt < 0.5 + Dt)
		&& mImudata.timestamp > lastZuptDetTime + 0.45))
	{
		memcpy(LastMean2, LastMean1, 6 * sizeof(double));
		memcpy(LastMean1, Mean, 6 * sizeof(double));

		memset(Max, 0, 6 * sizeof(double));
        memset(Mean, 0, 6 * sizeof(double));
        memset(Var, 0, 6 * sizeof(double));
        memset(Std, 0, 6 * sizeof(double));
		for (int16_t i = 0; i < DateLength; i++)
		{
			for (int16_t j = 0; j < 6; j++)
			{
				Max[j] += ZuptdetectData[i][j];
			}
		}
		for (int16_t j = 0; j < 6; j++)
		{
			Mean[j] = Max[j] / DateLength;
		}
		for (int16_t i = 0; i < DateLength; i++)
		{
			for (int16_t j = 0; j < 6; j++)
			{
				Var[j] += (ZuptdetectData[i][j] - Mean[j])*(ZuptdetectData[i][j] - Mean[j]);
			}
		}
		for (int16_t j = 0; j < 6; j++)
		{
			Std[j] = sqrt(Var[j] / DateLength);
		}
		
		ZuptValue1 = 0;
		if (Std[0] < ZUPTThreShold[0] && Std[1] < ZUPTThreShold[1] && Std[2] < ZUPTThreShold[2]
			&&Std[3] < ZUPTThreShold[3] && Std[4] < ZUPTThreShold[4] && Std[5] < ZUPTThreShold[5])
		{
			ZuptValue1 = 1;
		}
		if (ZuptValue1 && lastZuptValue1)
		{
			ZuptValue2 += 1;
		}
		if (ZuptValue2 > 5)
		{
			ZuptValue2 = 5;
		}
		if (!ZuptValue1 && lastZuptValue1)
		{
			ZuptValue2 = (ZuptValue2 - 4 > 0) ? (ZuptValue2 - 4) : 0;
		}
		if (!ZuptValue1 && !lastZuptValue1)
		{
			ZuptValue2 = 0;
		}

		lastZuptValue1 = ZuptValue1;
		lastZuptDetTime = mImudata.timestamp;

		if (fabs(Mean[0]) < 5 *PI/180 && fabs(Mean[1]) < 5 * PI / 180 && fabs(Mean[2]) < 5 * PI / 180)
		{
			date_length += 1;
			Amean[0] = (Amean[0] * (date_length - 1) + Mean[0]) / (date_length);
			Amean[1] = (Amean[1] * (date_length - 1) + Mean[1]) / (date_length);
			Amean[2] = (Amean[2] * (date_length - 1) + Mean[2]) / (date_length);
			Amean[3] = (Amean[3] * (date_length - 1) + Mean[3]) / (date_length);
			Amean[4] = (Amean[4] * (date_length - 1) + Mean[4]) / (date_length);
			Amean[5] = (Amean[5] * (date_length - 1) + Mean[5]) / (date_length);
		}
		if (DateLength_1 == 8)
		{
			for (int16_t i = 0; i < DateLength_1 - 1; i++)
			{
				for (int16_t j = 0; j < 6; j++)
				{
					Rawdatamean[i][j] = Rawdatamean[i + 1][j];
				}
				GZmean[i] = Zaxix/abs(Zaxix) * Rawdatamean[i][abs(Zaxix) - 1];
			}
			for (int16_t j = 0; j < 6; j++)
			{
				Rawdatamean[DateLength_1 - 1][j] = Mean[j];
			}
			GZmean[DateLength_1 - 1] = Zaxix / abs(Zaxix) * Rawdatamean[DateLength_1 - 1][abs(Zaxix) - 1];
		}
		if (DateLength_1 < 8)
		{
			for (int16_t j = 0; j < 6; j++)
			{
				Rawdatamean[DateLength_1][j] = Mean[j];
			}
			GZmean[DateLength_1] = Zaxix / abs(Zaxix) * Rawdatamean[DateLength_1][abs(Zaxix) - 1];
			DateLength_1 += 1;
			ret = 2;
			return ret;
		}
		if (DateLength_1 == 8)
		{
			ret = 1;
			//如何判断移动 变化起伏太大 陀螺 50deg/s 加表3G，以及STD > 20deg/s 2g 减去mean
			memset(MaxImu, 0, 6 * sizeof(double));
			memset(MeanImu, 0, 6 * sizeof(double));
			memset(VarImu, 0, 6 * sizeof(double));
			memset(StdImu, 0, 6 * sizeof(double));
			//Re cal zupt
			for (int16_t i = 0; i < DateLength_1; i++)
			{
				for (int16_t j = 0; j < 6; j++)
				{
					MaxImu[j] += Rawdatamean[i][j];
				}
			}
			for (int16_t j = 0; j < 6; j++)
			{
				MeanImu[j] = MaxImu[j] / DateLength_1;
			}
			for (int16_t i = 0; i < DateLength_1; i++)
			{
				for (int16_t j = 0; j < 6; j++)
				{
					VarImu[j] += (Rawdatamean[i][j] - MeanImu[j])*(Rawdatamean[i][j] - MeanImu[j]);
				}
			}
			for (int16_t j = 0; j < 6; j++)
			{
				StdImu[j] = sqrt(VarImu[j] / DateLength_1);
			}
			if (MeanImu[0] - Amean[0] > MISCHANGEThreShold[0]//5 * PI / 180
				|| MeanImu[1] - Amean[1] > MISCHANGEThreShold[1]//5 * PI / 180
				|| MeanImu[2] - Amean[2] > MISCHANGEThreShold[2])//50 * PI / 180)
			{
				//IMU移动
				IMUMoveMode = 1;
			}
			if (MeanImu[3] - Amean[3] > MISCHANGEThreShold[3]//5
				|| MeanImu[4] - Amean[4] > MISCHANGEThreShold[4]//5
				|| MeanImu[5] - Amean[5] > MISCHANGEThreShold[5])//5)
			{
				//IMU移动
				IMUMoveMode = 1;
			}
			if (StdImu[0]  > MISCHANGEThreShold[6]//5 * PI / 180
				|| StdImu[1]  > MISCHANGEThreShold[7]//5 * PI / 180
				|| StdImu[2]   > MISCHANGEThreShold[8])//30 * PI / 180)   //是不是可以检查ZHENGDONG
			{
				//IMU移动
				IMUMoveMode = 1;
			}
			if (StdImu[3]  > MISCHANGEThreShold[9]//3
				|| StdImu[4] > MISCHANGEThreShold[10]//3
				|| StdImu[5]   > MISCHANGEThreShold[11])//3)
			{
				//IMU移动
				IMUMoveMode = 1;
			}
		}
		return ret;
	}
	if (DateLength_1 == 8)
	{
		ret = 1;
	}
	else
	{
		ret = 2;
	}
	return ret;
}

int8_t GetIMUMoveMode()
{
	return IMUMoveMode;
}

int8_t IsOnLine()    //last 4seconde
{
	int8_t ret = 1;

	if (DateLength_1 == 8)   
	{
		for (int i = 0; i < 8; i++)
		{
			if (fabs(GZmean[i]) >  THR_STRT_LINE)
			{
				ret = 0;
			}
		}	
	}
	else
	{
		ret = 0;
	}
	return ret;
}
int8_t IsOnLine2()    //last 2 seconde
{
	int8_t ret = 1;

	if (DateLength_1 == 8)
	{
		for (int i = 0; i < 4; i++)
		{
			if (fabs(GZmean[i+4]) > 5 * 3.14 / 180)
			{
				ret = 0;
			}
		}
	}
	else
	{
		ret = 0;
	}
	return ret;
}
int8_t IsOnLine3()    //last 4 seconde
{
	int8_t ret = 1;
	//判断转弯
	double Max = 0;
	double Mean = 0;

	int ret1 = 0;
	if (DateLength_1 == 8)
	{
		for (int i = 0; i < 8; i++)
		{
			Max += GZmean[i];
		}
		Mean = Max / 8;
		if (Mean > 1.0 * 3.14 / 180)  //存在转弯 
		{
			ret1 = 1;
			for (int i = 0; i < 8; i++)
			{
				if (GZmean[i] < 1.0 * 3.14 / 180)
				{
					ret1 = 0;
				}
			}
		}
		if (Mean < -1.0 * 3.14 / 180)
		{
			ret1 = 1;
			for (int i = 0; i < 8; i++)
			{
				if (GZmean[i] > -1.0 * 3.14 / 180)
				{
					ret1 = 0;
				}
			}
		}
	}
	//判断是不是一直一个方向的小转弯 如果是ret1 = 0，返回1.
	return !ret1;
}

static double zuptmean[6];
int8_t SetZUPTMeanFirst()
{
	memcpy(zuptmean, Mean, 6 * sizeof(double));
	return 1;
}
int8_t SetZUPTMean(const ImuData mImudata)
{
	zuptmean[0] = 0.96*zuptmean[0] + 0.04*mImudata.Gyrox;
	zuptmean[1] = 0.96*zuptmean[1] + 0.04*mImudata.Gyroy;
	zuptmean[2] = 0.96*zuptmean[2] + 0.04*mImudata.Gyroz;
	zuptmean[3] = 0.96*zuptmean[3] + 0.04*mImudata.Accx;
	zuptmean[4] = 0.96*zuptmean[4] + 0.04*mImudata.Accy;
	zuptmean[5] = 0.96*zuptmean[5] + 0.04*mImudata.Accz;
	return 1;
}
int8_t GetZUPTMean(double *zupt)
{
	memcpy(zupt, zuptmean, 6 * sizeof(double));
	return 1;
}


int8_t GetZuptVal()
{
	int8_t ret = 0;

	 double CurMax[6] = { 0.0 };
	 double CurMean[6] = { 0.0 };
	 double CurVar[6] = { 0.0 };
	 double CurStd[6] = { 0.0 };
	
	for (int16_t i = 0; i < DateLength; i++)
	{
		for (int16_t j = 0; j < 6; j++)
		{
			CurMax[j] += ZuptdetectData[i][j];
		}
	}
	for (int16_t j = 0; j < 6; j++)
	{
		CurMean[j] = CurMax[j] / DateLength;
	}
	for (int16_t i = 0; i < DateLength; i++)
	{
		for (int16_t j = 0; j < 6; j++)
		{
			CurVar[j] += (ZuptdetectData[i][j] - CurMean[j])*(ZuptdetectData[i][j] - CurMean[j]);
		}
	}
	for (int16_t j = 0; j < 6; j++)
	{
		CurStd[j] = sqrt(CurVar[j] / DateLength);
	}
	ZuptValue3 = 0;

	if (CurStd[0] < ZUPTThreShold[0] && CurStd[1] < ZUPTThreShold[1] && CurStd[2] < ZUPTThreShold[2]
		&&CurStd[3] < ZUPTThreShold[3] && CurStd[4] < ZUPTThreShold[4] && Std[5] < ZUPTThreShold[5]) 
	{
		ZuptValue3 = 1;
	}

	if (ZuptValue3 && ZuptValue2 - 2 > 0 )
	{
		ret = 1;
	}

	return ret;
}

static double MaxG[6] = { 0.0 };
static double MeanG[6] = { 0.0 };
static double VarG[6] = { 0.0 };
static double StdG[6] = { 0.0 };
int8_t SetGNSSZuptData(const GnssData *pGnssData, const GnssData *pPreGnssData)
{
	int8_t ret = -1;
	double CurZupt = 0;
	double CurZuptReal = 0;
	double ReclZupt = 0;

	if (pGnssData->timestamp - pPreGnssData->timestamp > 0.999
		&& pGnssData->timestamp - pPreGnssData->timestamp < 1.001
		&& pGnssData->Mode >= 4
		&& pPreGnssData->Mode >=4)
	{
		double ZuptPostion = 0;
		if (pGnssData->latitude - pPreGnssData->latitude < 0.0000001 * DEG2RAD
			&&pGnssData->longitude - pPreGnssData->longitude < 0.0000001 * DEG2RAD
			&&pGnssData->altitude - pPreGnssData->altitude < 0.03)
		{
			ZuptPostion = 1;
		}
		double ZuptVel = 0;
		if (fabs(pGnssData->north_velocity)  < 0.015
			&&fabs(pGnssData->east_velocity)  < 0.015
			&&fabs(pGnssData->up_velocity)  < 0.03)
		{
			ZuptVel = 1;
		}
		if (ZuptPostion && ZuptVel)
		{
			CurZupt = 1;
		}
	}
	if (CurZupt && lastZuptDetTime > 0)
	{
		int32_t CheckDataLegth = 10;


		if (DateLengthGNSS == CheckDataLegth)
		{
			for (int16_t i = 0; i < DateLengthGNSS - 1; i++)
			{
				for (int16_t j = 0; j < 7; j++)
				{
					ZuptdetectGNSS[i][j] = ZuptdetectGNSS[i + 1][j];
				}
			}
			ZuptdetectGNSS[DateLengthGNSS - 1][0] = pGnssData->timestamp;
			ZuptdetectGNSS[DateLengthGNSS - 1][1] = Std[0];
			ZuptdetectGNSS[DateLengthGNSS - 1][2] = Std[1];
			ZuptdetectGNSS[DateLengthGNSS - 1][3] = Std[2];
			ZuptdetectGNSS[DateLengthGNSS - 1][4] = Std[3];
			ZuptdetectGNSS[DateLengthGNSS - 1][5] = Std[4];
			ZuptdetectGNSS[DateLengthGNSS - 1][6] = Std[5];

		}
		if (DateLengthGNSS < CheckDataLegth)
		{
			ZuptdetectGNSS[DateLengthGNSS][0] = pGnssData->timestamp;
			ZuptdetectGNSS[DateLengthGNSS][1] = Std[0];
			ZuptdetectGNSS[DateLengthGNSS][2] = Std[1];
			ZuptdetectGNSS[DateLengthGNSS][3] = Std[2];
			ZuptdetectGNSS[DateLengthGNSS][4] = Std[3];
			ZuptdetectGNSS[DateLengthGNSS][5] = Std[4];
			ZuptdetectGNSS[DateLengthGNSS][6] = Std[5];
			DateLengthGNSS += 1;
			return 1;
		}


		
	
	//持续5s钟静止

		if (CurZupt && DateLengthGNSS == 10)
		{
			if (ZuptdetectGNSS[DateLengthGNSS - 1][0] - ZuptdetectGNSS[5][0] > 3.9999
				&& ZuptdetectGNSS[DateLengthGNSS - 1][0] - ZuptdetectGNSS[5][0] < 4.0001)
			{
				Rezupttime = ZuptdetectGNSS[DateLengthGNSS - 1][0];
			}
			if (ZuptdetectGNSS[DateLengthGNSS - 1][0] - ZuptdetectGNSS[0][0] > 8.9999
				&& ZuptdetectGNSS[DateLengthGNSS - 1][0] - ZuptdetectGNSS[0][0] < 9.0001)
			{
				CurZuptReal = 1;
				ReclZupt = 1;
			}
			if (CurZuptReal == 1 && ReclZupt == 1)
			{
				memset(MaxG, 0, 6 * sizeof(double));
				memset(MeanG, 0, 6 * sizeof(double));
				memset(VarG, 0, 6 * sizeof(double));
				memset(StdG, 0, 6 * sizeof(double));
				//Re cal zupt
				for (int16_t i = 0; i < DateLengthGNSS; i++)
				{
					for (int16_t j = 0; j < 6; j++)
					{
						MaxG[j] += ZuptdetectGNSS[i][j + 1];
					}
				}
				for (int16_t j = 0; j < 6; j++)
				{
					MeanG[j] = MaxG[j] / DateLengthGNSS;
				}
				for (int16_t i = 0; i < DateLengthGNSS; i++)
				{
					for (int16_t j = 0; j < 6; j++)
					{
						VarG[j] += (ZuptdetectGNSS[i][j + 1] - MeanG[j])*(ZuptdetectGNSS[i][j + 1] - MeanG[j]);
					}
				}
				for (int16_t j = 0; j < 6; j++)
				{
					StdG[j] = sqrt(VarG[j] / DateLengthGNSS);
				}

				//两个阈值之间的差异
				for (int16_t j = 0; j < 6; j++)
				{
					double recZ = MeanG[j] + 3 * StdG[j];
					/*if (ZUPTThreShold[j] > 1.05 *(MeanG[j] + 6 * StdG[j])
						|| ZUPTThreShold[j] < 0.95 *(MeanG[j] + 6 * StdG[j]))*/
					{
						ret = 1;
						GNSSZuptFlag = 1;
						ZUPTThreShold[j] = MeanG[j] + 6 * StdG[j] > 3* MeanG[j]? MeanG[j] + 6 * StdG[j]: 3 * MeanG[j];
					}
				}

			}
		}
	}
	return ret;
}

double GetZuptGNSStime()
{
	return Rezupttime;
}


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

int8_t leastSquareLinearFit(const float x[],const float y[], const int num, float* a, float*b,float *r)
{
	float sum_x2 = 0.0;
	float sum_y2 = 0.0;
	float sum_y = 0.0;
	float sum_x = 0.0;
	float sum_xy = 0.0;


	for (int i = 0; i < num; ++i)
	{
		sum_x2 += x[i] * x[i];
		sum_y2 += y[i] * y[i];
		sum_y += y[i];
		sum_x += x[i];
		sum_xy += x[i] * y[i];
	}



	*a = (num*sum_xy - sum_x * sum_y) / (num*sum_x2 - sum_x * sum_x);
	*b = (sum_x2*sum_y - sum_x * sum_xy) / (num*sum_x2 - sum_x * sum_x);
	*r = (num*sum_xy - sum_x * sum_y) / sqrt((num*sum_x2 - sum_x * sum_x)*(num*sum_y2 - sum_y * sum_y));

	return 1;


}

static double FlexdetectGNSS[15][7];
static int16_t DateLengthFlex = 0;
static float lastgnss_heading = -999.99;

static double FlexdetectGNSS1[15][12];
static int16_t DateLengthFlex1 = 0;

static float FlexdetectGNSS2[15][12];

static double lastned[3] = {-999,-999,-999};
static double lastv = -999;
int8_t SetGNSSFlexDetectDataArryFit(const double *pdata,int8_t *axix,double *misAlignment)
{
	int32_t CheckDataLegth = 10;
	if (DateLengthFlex2 == CheckDataLegth)
	{
		for (int16_t i = 0; i < DateLengthFlex2 - 1; i++)
		{
			for (int16_t j = 0; j < 12; j++)
			{
				FlexdetectGNSS2[i][j] = FlexdetectGNSS2[i+1][j];
			}
		}

		for (int16_t j = 0; j < 12; j++)
		{
			FlexdetectGNSS2[DateLengthFlex2 - 1][j] = (float)pdata[j];
		}


	}
	if (DateLengthFlex2 < CheckDataLegth)
	{
		for (int16_t j = 0; j < 12; j++)
		{
			FlexdetectGNSS2[DateLengthFlex2][j] = (float)pdata[j];
		}
		DateLengthFlex2 += 1;
		if (DateLengthFlex2 != 10)
		{
			return 0;
		}
	}
	//拟合
	float x[4][15] = { 0.0 };
	float a[3], b[3], r[3];
	for (int j = 0; j < 15; j++)
	{
		x[3][j] = FlexdetectGNSS2[j][7];
	}
	for (int i = 0; i < 3; i++)
	{

			for (int j = 0; j < 15; j++)
			{
				x[i][j] = FlexdetectGNSS2[j][i+4];
			}

		leastSquareLinearFit(x[3],x[i], DateLengthFlex2, &(a[i]), &(b[i]), &(r[i]));
	}


	

	//轴向， 安装角度判定条件

	//判定Z 轴  1,2,3
	int Z_axix = 0;
	int Z[3] = { 0,0,0 };
	double Z_axix_mean = fabs(Amean[3]);
	for (int i = 0; i < 2; i++)
	{
		if (Z_axix_mean < fabs(Amean[i + 4]))
		{
			Z_axix_mean = fabs(Amean[i + 4]);
			Z_axix = i + 1;
		}
	}
	Z_axix += 1;
	Z[Z_axix - 1] = 1;
	if (Amean[Z_axix-1+3] > 0)
	{
		Z_axix = -Z_axix;
		Z[Z_axix - 1] = -1;
	}
	//确定X轴 1,2,3
	int X_axix = 0;
	int X[3] = { 0,0,0 };
	double X_axix_Angle = fabs(fabs(a[0]));
	for (int i = 0; i < 2; i++)
	{
		if (X_axix_Angle < fabs(a[i+1]))
		{
			X_axix_Angle = fabs(a[i+1]);
			X_axix = i + 1;
		}
	}
	X_axix = X_axix + 1;
	X[abs(X_axix) - 1] = 1;

	if (a[X_axix-1] < 0)
	{
		X_axix = -X_axix;
		X[abs(X_axix) - 1] = -1;
	}

	//
		//剔除异常点 残差。
	if (fabs(r[abs(X_axix) - 1]) > 0.8)
	{
		//if (abs(X[abs(X_axix) - 1]) > 0)
		{
			for (int i1 = 0; i1 < DateLengthFlex2 - 1; i1++)
			{
				if (fabs(x[abs(X_axix) - 1][i1] - a[abs(X_axix) - 1]*x[3][i1]) > 0.3)
				{
					for (int16_t i = i1; i < DateLengthFlex2 - 1; i++)
					{
						for (int16_t j = 0; j < 12; j++)
						{
							FlexdetectGNSS2[i][j] = FlexdetectGNSS2[i + 1][j];
						}
					}
					DateLengthFlex2 -= 1;
				}
			}
		}
	}
	// 剔除倒车点
	if (fabs(r[abs(X_axix) - 1]) > 0.8)
	{
		if ((X[abs(X_axix) - 1]) > 0)
		{
			for (int i1 = 0; i1 < DateLengthFlex2 - 1; i1++)
			{
				if (x[abs(X_axix) - 1][i1] * x[3][i1] < 0)
				{
					for (int16_t i = i1; i < DateLengthFlex2 - 1; i++)
					{
						for (int16_t j = 0; j < 12; j++)
						{
							FlexdetectGNSS2[i][j] = FlexdetectGNSS2[i + 1][j];
						}
					}
					DateLengthFlex2 -= 1;
				}
			}
		}
		if ((X[abs(X_axix) - 1]) < 0)
		{
			for (int i1 = 0; i1 < DateLengthFlex2 - 1; i1++)
			{
				if (x[abs(X_axix) - 1][i1] * x[3][i1] > 0)
				{
					for (int16_t i = i1; i < DateLengthFlex2 - 1; i++)
					{
						for (int16_t j = 0; j < 12; j++)
						{
							FlexdetectGNSS2[i][j] = FlexdetectGNSS2[i + 1][j];
						}
					}
					DateLengthFlex2 -= 1;
				}
			}
		}
	}
	if (DateLengthFlex2 != CheckDataLegth)
	{
		return 1;
	}

	
	if (DateLengthFlex2 != CheckDataLegth)
	{
		return 1;
	}
	int Y_axix = 0;
	int Y[3] = { 0,0,0 };
	if (fabs(r[abs(X_axix) - 1]) > 0.8)
	{
		//轴向探测完成，安装角估计完成
		Y_axix = 6 - fabs(X_axix) - fabs(Z_axix);
		//符合右手坐标系 直接矩阵相乘
		Y[2] = -( X[0] * Z[1] - X[1] * Z[0]);
		Y[1] = -( X[2] * Z[0] - X[0] * Z[2]);
		Y[0] = -(X[1] * Z[2] - X[2] * Z[1]);
		if (Y[2] + Y[1] + Y[0] < 0)
		{
			Y_axix = -Y_axix;
		}
	}

	//计算安装角度
	//完善 基于重力加速度还是加速度度计算roll，picth
	double roll, pitch, heading;
	if (fabs(r[abs(X_axix) - 1]) > 0.8)
	{
		//根据安装方向重新排列组合
		double g = sqrt(Amean[3] * Amean[3] + Amean[4] * Amean[4] + Amean[5] * Amean[5]);
		//A 归一化 但并不一定正确
		double a1[3] = { 0.0 };
		double a1s = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		for (int i = 0; i < 3; i++)
		{
			a1[i] = a[i]/ a1s;// / a1s;
		}
		//初步计算roll，pitch
		//roll = asin(abs(Y_axix) / Y_axix * Amean[abs(Y_axix) - 1 + 3], g);
		//pitch = asin(abs(X_axix) / X_axix * Amean[abs(X_axix) - 1 + 3] / g);
		pitch = asin(abs(X_axix) / X_axix * Amean[abs(X_axix) - 1 + 3] / g);
		roll  = asin(-abs(Y_axix)/ Y_axix *Amean[abs(Y_axix) - 1 + 3]/g/cos(pitch));
		//double pitch = asin(a1[2]);
		
		//double temp = abs(Y_axix) / Y_axix * a1[Y_axix - 1] / cos(pitch);
		//if (temp > 1) { temp = 1; }
		//else if (temp < -1) { temp = -1; };

		//heading = -asin(temp);  //只有大小没有方向

		double temp1 = abs(X_axix) / X_axix * a1[abs(X_axix) - 1] / cos(pitch);
		if (a[abs(X_axix) - 1] > 0.999)
		{
			temp1 = 1;
		}
		if (a[abs(X_axix) - 1] < -0.999)
		{
			temp1 = -1;
		}
		if (temp1 > 1) { temp1 = 1; }
		else if (temp1 < -1) { temp1 = -1; };
		double heading = acos(fabs(temp1));  //只有大小没有方向
		if (fabs(r[abs(Y_axix) - 1]) < 0.8 && fabs(a[abs(Y_axix) - 1]) > 0.4)
		{
			return 1;
		}
		if (abs(Y_axix) / Y_axix * a[abs(Y_axix) - 1] > 0)
		{
			heading = -fabs(heading);
		}
		axix[0] = X_axix;
		axix[1] = Y_axix;
		axix[2] = Z_axix;
		misAlignment[0] = roll;
		misAlignment[1] = pitch;
		misAlignment[2] = heading;

		double aa[11] = { 0.0 };
		aa[0] = FlexdetectGNSS2[CheckDataLegth-1][0];//时间
		aa[1] = FlexdetectGNSS2[CheckDataLegth-1][3];//陀螺角速度
		aa[2] = FlexdetectGNSS2[CheckDataLegth-1][10];//GNSS角速度
		aa[3] = r[abs(X_axix) - 1];   //X相关性系数        
		aa[4] = r[abs(Y_axix) - 1];   //Y相关性系数
		aa[5] = r[abs(Z_axix) - 1];   //Z相关性系数

		aa[6] = misAlignment[0];
		aa[7] = misAlignment[1];
		aa[8] = misAlignment[2];
		aa[9] = FlexdetectGNSS2[CheckDataLegth-1][11];// pitch1;

		aa[10] = FlexdetectGNSS2[CheckDataLegth-1][7]; //GNSS加速度
#ifdef DEBUGMESSAGE

		tracemis1(aa);
#endif
		return 2;
	}
	else
	{
		return 1;
	}
}

int8_t SetGNSSFlexDetectDataArry(const GnssData *pPreGnssData,const GnssData *pGnssData,const double *ned ,const double diff_head, int8_t *axix, double *misAlignment)
{
	int8_t ret = 0;
	int32_t CheckDataLegth = 10;
	double v = {0.0};
	double as = 0.5, bs = 0.5, cs = 0.0;
	if (lastned[0] != -999)
	{
		double a[3] = {0.0};
	    double v1 = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
		v = sqrt(pGnssData->north_velocity * pGnssData->north_velocity + pGnssData->east_velocity * pGnssData->east_velocity//
			 + pGnssData->up_velocity * pGnssData->up_velocity);
		double diffv = v - v1;
		//a[0] = v - sqrt(lastned[0] * lastned[0] + lastned[1] * lastned[1]);
		lastv = sqrt(pPreGnssData->north_velocity * pPreGnssData->north_velocity + pPreGnssData->east_velocity * pPreGnssData->east_velocity);//
			//+ pPreGnssData->up_velocity * pPreGnssData->up_velocity);
		a[0] = v - lastv;

		a[1] = ned[2] - lastned[2];

		//a[1] = v - sqrt(lastned[0] * lastned[0] + lastned[1] * lastned[1]);

		if (DateLengthFlex1 == CheckDataLegth)
		{
			for (int16_t i = 0; i < DateLengthFlex1 - 1; i++)
			{
				for (int16_t j = 0; j < 12; j++)
				{
					FlexdetectGNSS1[i][j] = FlexdetectGNSS1[i + 1][j];
				}
			}
			FlexdetectGNSS1[DateLengthFlex1 - 1][0] = pGnssData->timestamp;
			FlexdetectGNSS1[DateLengthFlex1 - 1][1] = as * LastMean1[0] + bs * Mean[0] + cs * LastMean2[0] - Amean[0];
			FlexdetectGNSS1[DateLengthFlex1 - 1][2] = as * LastMean1[1] + bs * Mean[1]+ cs * LastMean2[1] - Amean[1];
			FlexdetectGNSS1[DateLengthFlex1 - 1][3] = as * LastMean1[2] + bs * Mean[2]+ cs * LastMean2[2] - Amean[2];
			FlexdetectGNSS1[DateLengthFlex1 - 1][4] = as * LastMean1[3] + bs * Mean[3]+ cs * LastMean2[3] - Amean[3];
			FlexdetectGNSS1[DateLengthFlex1 - 1][5] = as * LastMean1[4] + bs * Mean[4]+ cs * LastMean2[4] - Amean[4];
			FlexdetectGNSS1[DateLengthFlex1 - 1][6] = as * LastMean1[5] + bs * Mean[5]+ cs * LastMean2[5] - Amean[5];
			FlexdetectGNSS1[DateLengthFlex1 - 1][7] = a[0];
			FlexdetectGNSS1[DateLengthFlex1 - 1][8] = diffv;
			FlexdetectGNSS1[DateLengthFlex1 - 1][9] = 1;
			FlexdetectGNSS1[DateLengthFlex1 - 1][10] = diff_head;
			FlexdetectGNSS1[DateLengthFlex1 - 1][11] = v;

		}
		if (DateLengthFlex1 < CheckDataLegth)
		{
			FlexdetectGNSS1[DateLengthFlex1][0] = pGnssData->timestamp;
			FlexdetectGNSS1[DateLengthFlex1][1] = as * LastMean1[0] + bs * Mean[0]+ cs * LastMean2[0] - Amean[0];
			FlexdetectGNSS1[DateLengthFlex1][2] = as * LastMean1[1] + bs * Mean[1]+ cs * LastMean2[1] - Amean[1];
			FlexdetectGNSS1[DateLengthFlex1][3] = as * LastMean1[2] + bs * Mean[2]+ cs * LastMean2[2] - Amean[2];
			FlexdetectGNSS1[DateLengthFlex1][4] = as * LastMean1[3] + bs * Mean[3]+ cs * LastMean2[3] - Amean[3];
			FlexdetectGNSS1[DateLengthFlex1][5] = as * LastMean1[4] + bs * Mean[4]+ cs * LastMean2[4] - Amean[4];
			FlexdetectGNSS1[DateLengthFlex1][6] = as * LastMean1[5] + bs * Mean[5]+ cs * LastMean2[5] - Amean[5];
			FlexdetectGNSS1[DateLengthFlex1][7] = a[0];
			FlexdetectGNSS1[DateLengthFlex1][8] = diffv;
			FlexdetectGNSS1[DateLengthFlex1][9] = 1;
			FlexdetectGNSS1[DateLengthFlex1][10] = diff_head;
			FlexdetectGNSS1[DateLengthFlex1][11] = v;


			DateLengthFlex1 += 1;
			//return 1;
		}
		//4s 内均直行，且航向变化均值小于2deg 判定为直行
		if (DateLengthFlex1 > 4)
		{
			double mean_diff_head_2 = 0.0;
			double mean_diff_head_4 = 0.0;
			double mean_diff_head_10 = 0.0;

			for (int i = DateLengthFlex1 - 2; i < DateLengthFlex1; i++)
			{
				mean_diff_head_2 += FlexdetectGNSS1[i][10];
			}
			mean_diff_head_2 = mean_diff_head_2 / 4;

			for (int i = DateLengthFlex1 - 4; i < DateLengthFlex1; i++)
			{
				mean_diff_head_4 += FlexdetectGNSS1[i][10];
			}
			mean_diff_head_4 = mean_diff_head_4 / 4;


			// static or move 
			double thr_flex_gnss = sqrt(FlexdetectGNSS1[DateLengthFlex1 - 1][3] * FlexdetectGNSS1[DateLengthFlex1 - 1][3]
				+ FlexdetectGNSS1[DateLengthFlex1 - 1][2] * FlexdetectGNSS1[DateLengthFlex1 - 1][2]
				+ FlexdetectGNSS1[DateLengthFlex1 - 1][1] * FlexdetectGNSS1[DateLengthFlex1 - 1][1]);
			if (thr_flex_gnss < 1 * PI / 180
				/*&& fabs(ned[2]) < 0.06*/)
			{
				if (v > 1.5 && fabs(a[0]) > 0.75 && lastv >1.5 && 
					thr_flex_gnss * v < 5 * PI / 180
					&& fabs(pGnssData->up_velocity) < 0.2)
				{
					if ( 1/*FlexdetectGNSS1[DateLengthFlex1-1][4] * FlexdetectGNSS1[DateLengthFlex1-1][4] +
						FlexdetectGNSS1[DateLengthFlex1-1][5] * FlexdetectGNSS1[DateLengthFlex1-1][5] +
						FlexdetectGNSS1[DateLengthFlex1-1][6] * FlexdetectGNSS1[DateLengthFlex1-1][6] > 0.8*(FlexdetectGNSS1[DateLengthFlex1-1][7] * FlexdetectGNSS1[DateLengthFlex1 - 1][7])*/)
					{
						if (fabs(FlexdetectGNSS1[DateLengthFlex1 - 1][9] - 1) < 0.01)
						{
							FlexdetectGNSS1[DateLengthFlex1 - 1][9] = 0;
							ret = SetGNSSFlexDetectDataArryFit(FlexdetectGNSS1[DateLengthFlex1 - 1], axix, misAlignment);
						}
					}
				}
			}
			//else if (fabs(FlexdetectGNSS1[DateLengthFlex1 - 1][10]) < 2 * PI / 180)
			//{
			//	if (FlexdetectGNSS1[DateLengthFlex1 - 1][0] - FlexdetectGNSS1[DateLengthFlex1 - 2][0] < 1.001)
			//	{
			//		if (mean_diff_head_2 < 1 * PI / 180)
			//		{
			//			for (int i = 0; i < 2; i++)
			//			{

			//				if (FlexdetectGNSS1[DateLengthFlex1 - i - 1][11] > 0.5 && fabs(FlexdetectGNSS1[DateLengthFlex1 - i - 1][7]) > 0.5)
			//				{
			//					if (FlexdetectGNSS1[DateLengthFlex1-i - 1][4] * FlexdetectGNSS1[DateLengthFlex1-i - 1][4] +
			//						FlexdetectGNSS1[DateLengthFlex1-i - 1][5] * FlexdetectGNSS1[DateLengthFlex1-i - 1][5] +
			//						FlexdetectGNSS1[DateLengthFlex1-i - 1][6] * FlexdetectGNSS1[DateLengthFlex1-i - 1][6] > 
			//						0.8*(FlexdetectGNSS1[DateLengthFlex1 - 1][7] * FlexdetectGNSS1[DateLengthFlex1 - 1][7])
			//						&& FlexdetectGNSS1[DateLengthFlex1 - i - 1][4] * FlexdetectGNSS1[DateLengthFlex1 - i - 1][4] +
			//						FlexdetectGNSS1[DateLengthFlex1 - i - 1][5] * FlexdetectGNSS1[DateLengthFlex1 - i - 1][5] +
			//						FlexdetectGNSS1[DateLengthFlex1 - i - 1][6] * FlexdetectGNSS1[DateLengthFlex1 - i - 1][6] <
			//						1.2*(FlexdetectGNSS1[DateLengthFlex1 - 1][7] * FlexdetectGNSS1[DateLengthFlex1 - 1][7]))
			//					{
			//						if (fabs(FlexdetectGNSS1[DateLengthFlex1-i - 1][9] - 1) < 0.01)
			//						{
			//							FlexdetectGNSS1[DateLengthFlex1-i - 1][9] = 0;
			//							ret = SetGNSSFlexDetectDataArryFit(FlexdetectGNSS1[DateLengthFlex1-i - 1], axix, misAlignment);
			//						}
			//					}
			//				}
			//			}
			//		}
			//	}
			//}
			//else if (fabs(mean_diff_head_4) < 2 * PI / 180 
			//	&& FlexdetectGNSS1[DateLengthFlex1 - 1][0] - FlexdetectGNSS1[DateLengthFlex1 - 4][0] < 3.001 )
			//{
			//	if (v > 0.5 && fabs(a[0]) > 0.5)
			//	{
			//		if (FlexdetectGNSS1[DateLengthFlex1 - 1][4] * FlexdetectGNSS1[DateLengthFlex1 - 1][4] +
			//			FlexdetectGNSS1[DateLengthFlex1 - 1][5] * FlexdetectGNSS1[DateLengthFlex1 - 1][5] +
			//			FlexdetectGNSS1[DateLengthFlex1 - 1][6] * FlexdetectGNSS1[DateLengthFlex1 - 1][6] > 0.8*(a[0] * a[0]))
			//		{
			//			if (fabs(FlexdetectGNSS1[DateLengthFlex1 - 1][9] - 1) < 0.01)
			//			{
			//				ret = SetGNSSFlexDetectDataArryFit(FlexdetectGNSS1[DateLengthFlex1 - 1], axix, misAlignment);
			//			}
			//		}
			//	}
			//}
		}
	}
	memcpy(lastned, ned,3 * sizeof(double));
	lastv = v;
	return ret;

}
int8_t SetGNSSFlexDetectData(const GnssData *pGnssData, const GnssData *pPreGnssData, int8_t *axix, double *misAlignment)
{
	int8_t ret = 0;

	if (pGnssData->timestamp - pPreGnssData->timestamp > 0.999
		&& pGnssData->timestamp - pPreGnssData->timestamp < 1.001
		&& pGnssData->Mode == 4
		&& pPreGnssData->Mode == 4)
	{
		// 判定直行
		double blh_prev[3] = { pPreGnssData->latitude, pPreGnssData->longitude, pPreGnssData->altitude };
		double blh_curr[3] = { pGnssData->latitude, pGnssData->longitude, pGnssData->altitude };
		double ned[3] = { 0.0 };
		blhdiff(blh_curr, blh_prev, ned);
		float gnss_heading = atan2(ned[1], ned[0]);
		if (lastgnss_heading > -10)
		{
			float diff_heading = gnss_heading - lastgnss_heading;
			if (diff_heading > PI)
			{
				diff_heading -= 2 * PI;
			}
			else if (diff_heading < -PI)
			{
				diff_heading += 2 * PI;
			}

			if (1)//fabs(diff_heading) < 5 * PI / 180) //comment
			{
				ret = SetGNSSFlexDetectDataArry(pPreGnssData, pGnssData, ned, diff_heading, axix, misAlignment);
			}
		}
		lastgnss_heading = gnss_heading;
	}
	else
	{
		lastgnss_heading = -99;
	}
	return ret;
}

static double GnssStatusDect[15][12];
static int16_t DateLengthGnssStatus = 0;
static double lastned1[3] = { -999,-999,-999 };
static double lastv1 = -999;
static float lastgnss_heading1 = -999.99;

int8_t SetGNSSDetectDataArry(const GnssData *pPreGnssData, const GnssData *pGnssData, const double *ned, const double diff_head)
{
	int8_t ret = 0;
	int32_t CheckDataLegth = 10;
	double v = { 0.0 };
	double as = 0.5, bs = 0.5, cs = 0.0;
	if (lastned1[0] != -999)
	{
		double a[3] = { 0.0 };
		double v1 = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
		v = sqrt(pGnssData->north_velocity * pGnssData->north_velocity + pGnssData->east_velocity * pGnssData->east_velocity//
			+ pGnssData->up_velocity * pGnssData->up_velocity);
		double diffv = v - v1;
		//a[0] = v - sqrt(lastned[0] * lastned[0] + lastned[1] * lastned[1]);
		lastv1 = sqrt(pPreGnssData->north_velocity * pPreGnssData->north_velocity + pPreGnssData->east_velocity * pPreGnssData->east_velocity);//
			//+ pPreGnssData->up_velocity * pPreGnssData->up_velocity);
		a[0] = v - lastv1;

		a[1] = ned[2] - lastned1[2];

		//a[1] = v - sqrt(lastned[0] * lastned[0] + lastned[1] * lastned[1]);

		if (DateLengthGnssStatus == CheckDataLegth)
		{
			for (int16_t i = 0; i < DateLengthGnssStatus - 1; i++)
			{
				for (int16_t j = 0; j < 12; j++)
				{
					GnssStatusDect[i][j] = GnssStatusDect[i + 1][j];
				}
			}
			GnssStatusDect[DateLengthGnssStatus - 1][0] = pGnssData->timestamp;
			GnssStatusDect[DateLengthGnssStatus - 1][1] = as * LastMean1[0] + bs * Mean[0] + cs * LastMean2[0] - Amean[0];
			GnssStatusDect[DateLengthGnssStatus - 1][2] = as * LastMean1[1] + bs * Mean[1] + cs * LastMean2[1] - Amean[1];
			GnssStatusDect[DateLengthGnssStatus - 1][3] = as * LastMean1[2] + bs * Mean[2] + cs * LastMean2[2] - Amean[2];
			GnssStatusDect[DateLengthGnssStatus - 1][4] = as * LastMean1[3] + bs * Mean[3] + cs * LastMean2[3] - Amean[3];
			GnssStatusDect[DateLengthGnssStatus - 1][5] = as * LastMean1[4] + bs * Mean[4] + cs * LastMean2[4] - Amean[4];
			GnssStatusDect[DateLengthGnssStatus - 1][6] = as * LastMean1[5] + bs * Mean[5] + cs * LastMean2[5] - Amean[5];
			GnssStatusDect[DateLengthGnssStatus - 1][7] = a[0];
			GnssStatusDect[DateLengthGnssStatus - 1][8] = diffv;
			GnssStatusDect[DateLengthGnssStatus - 1][9] = 1;
			GnssStatusDect[DateLengthGnssStatus - 1][10] = diff_head;
			GnssStatusDect[DateLengthGnssStatus - 1][11] = v;

		}
		if (DateLengthGnssStatus < CheckDataLegth)
		{
			GnssStatusDect[DateLengthGnssStatus][0] = pGnssData->timestamp;
			GnssStatusDect[DateLengthGnssStatus][1] = as * LastMean1[0] + bs * Mean[0] + cs * LastMean2[0] - Amean[0];
			GnssStatusDect[DateLengthGnssStatus][2] = as * LastMean1[1] + bs * Mean[1] + cs * LastMean2[1] - Amean[1];
			GnssStatusDect[DateLengthGnssStatus][3] = as * LastMean1[2] + bs * Mean[2] + cs * LastMean2[2] - Amean[2];
			GnssStatusDect[DateLengthGnssStatus][4] = as * LastMean1[3] + bs * Mean[3] + cs * LastMean2[3] - Amean[3];
			GnssStatusDect[DateLengthGnssStatus][5] = as * LastMean1[4] + bs * Mean[4] + cs * LastMean2[4] - Amean[4];
			GnssStatusDect[DateLengthGnssStatus][6] = as * LastMean1[5] + bs * Mean[5] + cs * LastMean2[5] - Amean[5];
			GnssStatusDect[DateLengthGnssStatus][7] = a[0];
			GnssStatusDect[DateLengthGnssStatus][8] = diffv;
			GnssStatusDect[DateLengthGnssStatus][9] = 1;
			GnssStatusDect[DateLengthGnssStatus][10] = diff_head;
			GnssStatusDect[DateLengthGnssStatus][11] = v;


			DateLengthGnssStatus += 1;
			//return 1;
		}
		//4s 内均直行，且航向变化均值小于2deg 判定为直行
		int8_t gnssstatus = 0;

		if (DateLengthGnssStatus > 4)
		{
			double mean_diff_head_2 = 0.0;
			double mean_diff_head_4 = 0.0;
			double mean_diff_head_10 = 0.0;

			for (int i = DateLengthGnssStatus - 2; i < DateLengthGnssStatus; i++)
			{
				mean_diff_head_2 += GnssStatusDect[i][10];
			}
			mean_diff_head_2 = mean_diff_head_2 / 4;

			for (int i = DateLengthGnssStatus - 4; i < DateLengthGnssStatus; i++)
			{
				mean_diff_head_4 += GnssStatusDect[i][10];
			}
			mean_diff_head_4 = mean_diff_head_4 / 4;

			if (DateLengthGnssStatus == 10)
			{
				ret = 0;
				int8_t start = 4;
				for (int i = start; i < DateLengthGnssStatus; i++)
				{
					mean_diff_head_10 += GnssStatusDect[i][10];
				}
				mean_diff_head_10 = mean_diff_head_10 / 4;

				{

					if (mean_diff_head_10 < 2 * PI / 180)
					{
						gnssstatus = 1;
						for (int i = start; i < DateLengthGnssStatus; i++)
						{
							if (sqrt(GnssStatusDect[i][3] * GnssStatusDect[i][3]
								+ GnssStatusDect[i][2] * GnssStatusDect[i][2]
								+ GnssStatusDect[i][1] * GnssStatusDect[i][1]) < 2 * PI / 180)
							{
							}
							else
							{
								gnssstatus = 0;
							}
							if (GnssStatusDect[i][11] > 5)
							{

							}
							else
							{
								gnssstatus = 0;
							}
						}
						if (GnssStatusDect[DateLengthGnssStatus - 1][0] - GnssStatusDect[start][0] > DateLengthGnssStatus - start - 1 + 0.1)
						{
							gnssstatus = 0;
						}
						else
						{

						}

					}
				}


			}

		}
		if (gnssstatus == 1)
		{
			ret = 1;
		}


	}
	memcpy(lastned1, ned, 3 * sizeof(double));
	lastv1 = v;
	return ret;
}

int8_t  SetGNSSDetectData(const GnssData *pGnssData, const GnssData *pPreGnssData)
{
	int8_t ret = -1;
	if (pGnssData->timestamp - pPreGnssData->timestamp > 0.999
		&& pGnssData->timestamp - pPreGnssData->timestamp < 1.001
		&& pGnssData->Mode == 4
		&& pPreGnssData->Mode == 4)
	{
		// 判定直行
		double blh_prev[3] = { pPreGnssData->latitude, pPreGnssData->longitude, pPreGnssData->altitude };
		double blh_curr[3] = { pGnssData->latitude, pGnssData->longitude, pGnssData->altitude };
		double ned[3] = { 0.0 };
		blhdiff(blh_curr, blh_prev, ned);
		float gnss_heading = atan2(ned[1], ned[0]);
		if (lastgnss_heading1 > -10)
		{
			float diff_heading = gnss_heading - lastgnss_heading1;
			if (diff_heading > PI)
			{
				diff_heading -= 2 * PI;
			}
			else if (diff_heading < -PI)
			{
				diff_heading += 2 * PI;
			}

			if (1)//fabs(diff_heading) < 5 * PI / 180) //comment
			{
				ret = SetGNSSDetectDataArry(pPreGnssData, pGnssData, ned, diff_heading);
			}
		}
		lastgnss_heading1 = gnss_heading;
	}
	else
	{
		lastgnss_heading1 = -99;
	}
	return ret;

}



static int InnoDateLength = 0;
static int InnoCheckDataLegth = 5;
static float InnodetectData[5][3] = { {0.0} };
static float InnoMax[3] = { 0.0 };
static float InnoMean[3] = { 0.0 };
static float InnoVar[3] = { 0.0 };
static float InnoStd[3] = { 0.0 };
static float InnoThreShold[3] = {1,1,1};
static float InnoThreSholdMAX[3] = { 5,5,5 };

int8_t SetInnoDetectData(const float *inno)
{
	int8_t ret = -1;
	if (InnoDateLength == InnoCheckDataLegth)
	{
		for (int16_t i = 0; i < InnoDateLength - 1; i++)
		{
			for (int16_t j = 0; j < 3; j++)
			{
				InnodetectData[i][j] = InnodetectData[i + 1][j];
			}
		}
		InnodetectData[InnoDateLength - 1][0] = *inno;
		InnodetectData[InnoDateLength - 1][1] = *(inno + 1);
		InnodetectData[InnoDateLength - 1][2] = *(inno + 2);

	}
	if (InnoDateLength < InnoCheckDataLegth)
	{
		InnodetectData[InnoDateLength][0] = *(inno);
		InnodetectData[InnoDateLength][1] = *(inno + 1);
		InnodetectData[InnoDateLength][2] = *(inno + 2);

		InnoDateLength += 1;
		ret = -1;
		return ret;
	}
	memset(InnoMax, 0, 3 * sizeof(float));
	memset(InnoMean, 0, 3 * sizeof(float));
	memset(InnoVar, 0, 3 * sizeof(float));
	memset(InnoStd, 0, 3 * sizeof(float));
	for (int16_t i = 0; i < InnoDateLength; i++)
	{
		for (int16_t j = 0; j < 3; j++)
		{
			InnoMax[j] += InnodetectData[i][j];
		}
	}
	for (int16_t j = 0; j < 3; j++)
	{
		InnoMean[j] = InnoMax[j] / InnoDateLength;
	}
	for (int16_t i = 0; i < InnoDateLength; i++)
	{
		for (int16_t j = 0; j < 3; j++)
		{
			InnoVar[j] += (InnodetectData[i][j] - InnoMean[j])*(InnodetectData[i][j] - InnoMean[j]);
		}
	}
	for (int16_t j = 0; j < 3; j++)
	{
		InnoStd[j] = sqrt(InnoVar[j] / InnoDateLength);
	}
	if (fabs(InnoMean[0]) < InnoThreShold[0] && fabs(InnoMean[1]) < InnoThreShold[1] && fabs(InnoMean[2]) < InnoThreShold[2]
		&& InnoStd[0] < InnoThreShold[0] && InnoStd[1] < InnoThreShold[1] && InnoStd[2] < InnoThreShold[2])
	{
		ret = 1;
	}
	else 	if (fabs(InnoMean[0]) < InnoThreSholdMAX[0] && fabs(InnoMean[1]) < InnoThreSholdMAX[1] 
		&& InnoStd[0] < InnoThreSholdMAX[0] && InnoStd[1] < InnoThreSholdMAX[1])
	{
		ret = 2;
	}
	else
	{
		ret = 0;
	}
	return ret;

}

int8_t GetGNSSStatus()
{
	int ret = -1;
	return  ret;

}

int8_t InitInno()
{
	int8_t ret = -1;
	InnoDateLength = 0;
	ret = 1;
	return ret;
}
int8_t InitGnssStatus()
{
	int8_t ret = -1;
	DateLengthGnssStatus = 0;
	ret = 1;
	return ret;
}
int8_t InitInnoErrorDect()
{
	int8_t ret = -1;
	ret = 1;
	ret &= InitInno();
	ret &= InitGnssStatus();
	return ret;
}