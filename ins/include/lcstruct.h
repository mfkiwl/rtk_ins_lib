#ifndef _INS_LCSTRUCT_
#define _INS_LCSTRUCT_

#include <stdint.h>
#include "datatype.h"
#define  StateX   16
#define  State2X  32
#define  StateX2  256
#ifdef __cplusplus
extern "C" {
#endif
		typedef struct  INS//ǰ����
		{
			double timestamp;
			double timestamped;
			double gyro_x;      //���ݼ��ٶ����� rad/s
			double gyro_y;
			double gyro_z;
			double acc_x;       //�������ٶ����� m/s
			double acc_y;
			double acc_z;
		}INS;
		typedef struct Par
		{
			double Rm;
			double Rn;
			double w_ie[3];  //earth rotation rate in n-frame;
			double w_en[3];  //trans rate in n-frame
			double g[3];
			double f_n[3];   //specific force 
			double f_b[3];   //specific force 
			double w_b[3];   //Bϵ�����ݹ۲���
		}Par;

		typedef struct SensorOut
		{
			double f_n[3];
			double f_b[3];
			double w_n[3];
			double w_b[3];
		}SensorOut;
		typedef struct Sensorbias
		{
			float bias_gyro_x;
			float bias_gyro_y;
			float bias_gyro_z;
			float bias_acc_x;
			float bias_acc_y;
			float bias_acc_z;

		}Sensorbias;
		typedef struct Nav
		{
			double lat;
			double lon;
			double height;
			float vn;
			float ve;
			float vd;
			float roll;
			float pitch;
			float heading;

			
			double dv_n[3];
			double wnb_n[3];   //������ٶȣ���nϵ��b�����nϵ�Ľ��ٶȣ�rad/s
			double wnb_b[3];
			double a_n[3];

			double w_b[3];
			double a_b[3];  

			double c_bn[3][3];
			double c_nb[3][3];
			double q_bn[4];
			double q_ne[4];

			Sensorbias sensorbias;
			double Odo_scale;
		}Nav;
		enum MeasUpdataType
		{
			MeasNone = 0,
			GNSSPoitionUpdate = 1 << 0,
			GNSSVelocityUpdate = 1 << 1,
			ZuptUpdate = 1 << 2,
			NHCUpdate = 1 << 3,
			OdoUpdate = 1 << 4,
			GNSSYawUpdate = 1 << 5,
			ZuptLock = 1 << 6,
		};
		enum GNSSREJRCTType
		{
			REJRCTTNone = 0,
			SolutionSataChange= 1 << 0,
			HeadOnline = 1<<1,
			GNSSUNKONWN =1<<2,
			LowSpeed = 1 <<3,
		};
		typedef struct PVA
		{
			double longitude;
			double latitude;
			double altitude;

			float northVelocity;
			float eastVelocity;
			float downVelocity;

			float roll;
			float pitch;
			float yaw;

			double wnb_n[3];
			double wnb_b[3]; 
			double a_n[3]; 

			double w_ie[3];
			double w_en[3];
			double w_ib[3];
		}PVA;
		enum LC_STATUS
		{
			INSDATA_NREADY = 0,  //0
			INSDATA_READY,
			ALIGNMENT_ING,
			ALIGNMENT_COMPLETE,
			INS_FUSING,
			MISALIGNMENT_ING,
			MISALIGNMENT_COMPLETE
		};
		enum INS_STATUS
		{
			INS_INACTIVE,
			INS_ALIGNING,
			INS_HIGH_VARIANCE,
			INS_SOLUTION_GOOD,
			INS_SOLUTION_FREE,
			INS_ALIGNMENT_COMPLETE,
			DETERMINING_ORIENTATION,
			WAITING_INITIALPOS,
			WAITING_AZIMUTH,
			INITIALIZING_BIASES
		};
		enum INS_POSITION_TYPE
		{
			INS_NONE,
			INS_PSRSP,
			INS_PSRDIFF,
			INS_PROPAGATED,
			INS_RTKFIXED,
			INS_RTKFLOAT
		};

		typedef struct ImuSensor
		{
			uint16_t IMUDataRate;

			float Gyr_bias_std[3];//deg/h
			float Acc_bias_std[3];//mGal
			float Gyr_scale_std[3];//ppm
			float Acc_scale_std[3];//ppm
			float Gyr_noise_arw;//deg/sqrt(h)
			float Acc_noise_vrw;//m/s/sqrt(h)
			float Gyr_bias_CorTime;//h
			float Acc_bias_CorTime;//h
			float Gyr_scale_CorTime;//h
			float Acc_scale_CorTime;//h

			float bg_model[3];
			float ba_model[3];

			float q_bg[3];
			float q_ba[3];

			float Gyr_bias_rept;
			float Acc_bias_rept;
		}ImuSensor;
		enum ErrorType
		{
			ErrorNONE = 0,
			IMUInterruptWarning = 1,
			IMUInterruptS = 2,
			IMUInterrupt2S = 3,
			IMUInterrupt5S = 4,
			GNSSInterruptMax = 5,
			IMUDataExp = 6,
			INSSystemExp1 = 7, // Course speed mismatch 
			INSSystemExp2 = 8, //bias estimation
			MisAligmentChange  = 9,
			GNSSREJECT =10,
			GnssInnoError = 11,
		};
		typedef struct GnssInsLC
		{
			uint16_t ProcType;
			double Leverarm[3];
			double OdoLeverarm[3];
			double INSRotationRvb[3];
			ImuSensor mIMUSensor;
			double OdoRate;
			uint8_t   IsCar;
			double CarLeverarm[3];
			uint8_t   IsUseZUPT;
			uint8_t   IsUseNHC;
			uint8_t   IsUseOdo;
			uint16_t   IMUDataRate;
			uint8_t   InitialAttitudeMode;//0:motion Alignment  1:Dual antenna
			uint8_t    GnssDataType;

			double g_bias[3];
			double g_scale[3];
			double a_bias[3];
			double a_scale[3];
			double Odo_scale;
		}GnssInsLC;
		typedef struct UpdataStruct
		{
			double LCTime;
			PVA mPVA;
			double C_bn[3][3];
			float X[StateX];
			float P[StateX2];
			float P_r[StateX2];


			float PHI[StateX2];
			float Q[StateX2];

			int8_t IsUseZUPT;
			int8_t IsUseZUPTA;
			int8_t Zuptlockflag;
			float DiffHEAD;
			double zuptdifftime;
		}UpdataStruct;
		typedef struct KalmanStruct
		{
			int16_t n;
			float P[StateX2];
			float X[StateX];
			float H[StateX];
			int L[StateX];
		}KalmanStruct;
		typedef struct posfliterstruct
		{
			double curx[3];
			double totalx[3];
			double dx[3];
			double d_lat;
			double d_lon;
			double d_height;
			int count;
		}posfliterstruct;
		typedef struct GnssInsSystem
		{
			enum LC_STATUS mlc_STATUS;
			enum INS_STATUS ins_status;
			enum INS_POSITION_TYPE ins_positin_type;
			enum ErrorType mErrorType; //��Ȼ����ȼ���ͬ�����չ��Ǵ���
			ImuData  mImuData;
			ImuData  mpreImuData;

			OdoData mOdoData;

			INS  mInsData;
			INS  mpreInsData;

			Nav mNav;
			Nav mpreNav;
			Nav outNav;

			Par mPar;

			GnssData mGnssData;
			GnssData premGnssData;

			ImuSensor mIMUSensor;

			KalmanStruct mKalmanStruct;

			uint8_t GNSSflag;                    //Is the current epoch used
			uint8_t GNSSPOSHORFLAG;
			uint8_t GNSSPOSALTFLAG;

			uint8_t GNSSREJRCTType;

			uint8_t GNSSVelflag;
			uint8_t is_gnss_pos_valid;
			uint16_t cnt_bad_gnss;
			int16_t IsUseNHC;
			int16_t IsUseOdo;
			int16_t Odomode;

			double nextLCTime;


			float Q[16];    
			double AlignCTime;
			double firstGNSSTime;

			int8_t InitNavi ;
			int8_t InitSensor;

			int8_t CurIsUseZupt;
			enum MeasUpdataType mMeasUpdataType;
			int16_t PerMeasUpdata;
			int16_t outPerMeasUpdata;

			int8_t INSPROCESSMODE;// 1 INS MECHINE 2 EKF FEED BACK 3 OBS UPDATA NO FEED BACK
			int8_t GNSSZuptFlag;

			int8_t IsIMUBiasFeedBack;

			double firstimutime;
			double firstGNSSUseTime ;
			double lastGNSSLCTime;
			enum INS_POSITION_TYPE lastGNSSLCType;
			double lastVelLCTime;
			double lastNHCLCTime;
			double lastZuptTime ;
			double lastObstime;
			double GNSSLOSETIME1;   //NOT DELETE ZUPT TIME
			double GNSSLOSETIME2;   //DELETE ZUPT TIME
			int8_t Isfirstfusionimu;
			double nextNHCLCTime;

			double leverarm_b[3];
			double userleverarm_b[3];
			double odoleverarm_b[3];
			double OutOftunnalTime;
			int isheadonline;


			int firstzupt;
			Nav zuptNav;
			double lastzuptdetcttime;
			int ISUSE_MISALIGMENT;
			double C_InstallationAngle[3][3]; // Rotation matrix from  vehicle frame to IMU frame C_vb


			double RotationCBV[3][3];

			int32_t error_odocout;
			int32_t right_odocout;
			int32_t isUseOdo;
			double zupttime1; // continuity zupt
			double zupttime2; // All zupt

			int8_t BiasEstStab;
			int8_t IsGnssGood;
			int8_t SystemStab;
			posfliterstruct posfliter;

			double lastfeedbacktime;
			double timesincegnssfb;
		}GnssInsSystem;



#ifdef __cplusplus
}
#endif





#endif // !



