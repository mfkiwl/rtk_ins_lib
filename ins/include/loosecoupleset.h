#ifndef LOOSECOUPLESET_H_
#define LOOSECOUPLESET_H_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
	enum FILEOUTPUTTRPE
	{
		FILEOUTPUTNONE = 0,
		GNSSOUTPUT = 1 << 0,
		INSOUTPUT = 1 << 1,
		RAWMEAS = 1 << 2,
		MISALIGMENT = 1<<3,
	};
	enum PROFLIETRPE
	{
		GGA = 1 << 0,
		INSPVA_LOG = 1 << 1,
		INSPVAX_LOG = 1 << 2,
		INSDEBUG1 = 1 << 3,
		INSDEBUG2 = 1 << 4,
		INSDEBUGMESSAGE = 1 << 5,
		TESLAGNSSCSV = 1 << 6,
		TESLAREFCSV = 1 << 7,
		CORRIMU = 1 << 8,
	};
	typedef struct LCSetting
	{
		int16_t procType;
		char procfileNme[255];

		char insfileName[255], gnssfileNme[255], odofileName[255];

		int16_t initialAttitudeMode;                                   //0-MOTION 1-GIVEN  2 STATIC			
		double attitueRPH[3];                                          // SystemInit by Given attitue (deg)

		int16_t imuSensorType;                                         //IMU type
		int16_t imuDataRate;
		int16_t gnssSensorType;
		int16_t gnssDataType;                                          // 0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading
		int16_t gnssDataRate;
		int16_t odoDataRate;



		double priLeverArm[3];                                          //Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.
		int8_t isUseDuaAnt;
		double secLeverArm[3];                                          //Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.

		double rotationRBV[3];

		int8_t isUseMisAlignment;
		int8_t MisAlignmentAiax[3];
		double misAlignment[3];                                          //Rotation from the vehicle frame to the IMU body frame.
		int8_t isOnlineMisAlignmentEst;

		int8_t useGNSSRate;
		int8_t isUseNHC;
		int8_t useNHCRate;
		double odoLeverArm[3];                                          //Offset from the IMU center of navigation to the  center of the Odometer.
		int8_t isUseGNSSVel;
		int8_t isUseOdo;
		int8_t useOdoRate;
		double odoScale;

		int8_t isUseZUPT;
		int8_t isUseZUPTLOCK;

		double GNSSScale;
		int8_t isUseHeadOnline;
		int8_t isUseExpQC;
		int8_t isInsFreInit;
		double insFreTre;
		int8_t isUseGNSSZupt;

		double accBias[3];                                             //mGal
		double gyroBias[3];                                            //deg/h
		double accScale[3];                                            //ppm
		double gyroScale[3];                                           //ppm

		int8_t  isUserOutPut;
		double  userLeverArm[3];//Offset from the IMU center of navigation to the  center of the output user.
		int32_t fileOutputType;
		int32_t profiletype;
		int32_t isKmlOutput;
		int32_t kmlOutputDateRate;
		int32_t gnssOutputDataRate;
		int32_t insOutputDataRate;
		int8_t isusefliter;
	}LCSetting;
	extern LCSetting mLCSetting;
#ifdef __cplusplus
	 }
#endif

#endif