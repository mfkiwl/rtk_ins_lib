#include "loosecoupleset.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifdef  __cplusplus
	LCSetting mLCSetting =
	{
		mLCSetting.procType = 0,
		mLCSetting.initialAttitudeMode = 0,
		mLCSetting.attitueRPH[0] = 0,
		mLCSetting.attitueRPH[1] = 0,
		mLCSetting.attitueRPH[2] = 0,

		mLCSetting.imuSensorType = 1,                                         /*IMU type*/
		mLCSetting.imuDataRate = 100,
		mLCSetting.gnssSensorType = 0,
		mLCSetting.gnssDataType = 3,                                          /*0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading*/
		mLCSetting.gnssDataRate = 1,
		mLCSetting.odoDataRate = 0,

		mLCSetting.priLeverArm[0] = 0,                                   /*Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.*/
		mLCSetting.priLeverArm[1] = 0,
		mLCSetting.priLeverArm[2] = 0,
		mLCSetting.isUseDuaAnt = 0,
		mLCSetting.secLeverArm[0] = 0,                                       /*Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.*/
		mLCSetting.secLeverArm[1] = 0,
		mLCSetting.secLeverArm[2] = 0,

		mLCSetting.rotationRBV[0] = 0,
		mLCSetting.rotationRBV[1] = 0,
		mLCSetting.rotationRBV[2] = 0,

		mLCSetting.isUseMisAlignment = 0,
		mLCSetting.MisAlignmentAiax[0] = 1,
		mLCSetting.MisAlignmentAiax[1] = 2,
		mLCSetting.MisAlignmentAiax[2] = 3,
		mLCSetting.misAlignment[0] = 0,                                     /* Rotation from the vehicle frame to the IMU body frame.*/
		mLCSetting.misAlignment[1] = 0,
		mLCSetting.misAlignment[2] = 0,
		mLCSetting.isOnlineMisAlignmentEst = 0,

		mLCSetting.useGNSSRate = 1,
		mLCSetting.isUseNHC = 1,
		mLCSetting.useNHCRate = 1,
		mLCSetting.odoLeverArm[0] = 0,                                          /*Offset from the IMU center of navigation to the  center of the Odometer.*/
		mLCSetting.odoLeverArm[1] = 0,
		mLCSetting.odoLeverArm[2] = 0,
		mLCSetting.isUseGNSSVel = 0,
		mLCSetting.isUseOdo = 0,
		mLCSetting.useOdoRate = 1,
		mLCSetting.odoScale = 1,

		mLCSetting.GNSSScale = 1,
		mLCSetting.isUseHeadOnline = 0,
		mLCSetting.isUseExpQC = 0,
		mLCSetting.isInsFreInit = 1,
		mLCSetting.insFreTre = 300,

		mLCSetting.isUseZUPT = 1,
		mLCSetting.isUseZUPTLOCK = 1,
	    mLCSetting.isUseGNSSZupt = 0,

		mLCSetting.accBias[0] = 0,                                            /*mGal*/
		mLCSetting.accBias[1] = 0,
		mLCSetting.accBias[2] = 0,
		mLCSetting.gyroBias[0] = 0,                                          /*deg/h*/
		mLCSetting.gyroBias[1] = 0,
		mLCSetting.gyroBias[2] = 0,
		mLCSetting.accScale[0] = 0,                                           /*ppm*/
		mLCSetting.accScale[1] = 0,
		mLCSetting.accScale[2] = 0,
		mLCSetting.gyroScale[0] = 0,                                         /*ppm*/
		mLCSetting.gyroScale[1] = 0,
		mLCSetting.gyroScale[2] = 0,

		mLCSetting.isUserOutPut = 1,
		mLCSetting.userLeverArm[0] = 0, /*Offset from the IMU center of navigation to the  center of the output user.*/
		mLCSetting.userLeverArm[1] = 0,
		mLCSetting.userLeverArm[2] = 0,

		mLCSetting.fileOutputType = 0,
		mLCSetting.profiletype = 0,
		mLCSetting.isKmlOutput = 0,
		mLCSetting.kmlOutputDateRate = 0,
		mLCSetting.gnssOutputDataRate = 0,
		mLCSetting.insOutputDataRate = 0,
		mLCSetting.isusefliter =0,
	};
#else
	LCSetting mLCSetting =
	{
	   .procType = 0,
	   .initialAttitudeMode = 0,
	   .attitueRPH = {0, 0, 0},

	   .imuSensorType = 1,                                         /*IMU type*/
	   .imuDataRate = 100,
	   .gnssSensorType = 0,
	   .gnssDataType = 3,                                          /*0 none ;1 << 0: gnss position; 1 << 1: gnss vel; 1 << 2: gnss heading*/
	   .gnssDataRate = 1,
	   .odoDataRate = 0,

	   .priLeverArm = {0, 0, 0},                                   /*Offset from the IMU center of navigation to the phase center of the primary GNSS antenna.*/
	   .isUseDuaAnt = 0,
	   .secLeverArm = {0, 0, 0},                                       /*Offset from the IMU center of navigation to the phase center of the secondary GNSS antenna.*/

	   .rotationRBV = {0, 0, 0},

	   .isUseMisAlignment = 0,
	   .MisAlignmentAiax = {1,2,3},
	   .misAlignment = {0, 0, 0},                                      /* Rotation from the vehicle frame to the IMU body frame.*/
	   .isOnlineMisAlignmentEst = 0,

	   .useGNSSRate = 1,
	   .isUseNHC = 1,
	   .useNHCRate = 1,
	   .odoLeverArm = {0, 0, 0},                                          /*Offset from the IMU center of navigation to the  center of the Odometer.*/
	   .isUseGNSSVel = 0,
	   .isUseOdo = 0,
	   .useOdoRate = 1,
	   .odoScale = 1,

	   .GNSSScale = 1,
	   .isUseHeadOnline = 0,  //0
	   .isUseExpQC = 0,   //0
	   .isInsFreInit = 0,
	   .insFreTre = 180,

		  .isUseZUPT = 1,
		   .isUseZUPTLOCK = 1,
		   .isUseGNSSZupt = 0,

		   .accBias = {0, 0, 0},                                            /*mGal*/
		   .gyroBias = {0, 0, 0},                                          /*deg/h*/
		   .accScale = {0, 0, 0},                                           /*ppm*/
		   .gyroScale = {0, 0, 0},                                         /*ppm*/

		   .isUserOutPut = 1,
		   .userLeverArm = {0, 0, 0}, /*Offset from the IMU center of navigation to the  center of the output user.*/

		   .fileOutputType = 3,
		   .profiletype = 256,
		   .isKmlOutput = 1,
		   .kmlOutputDateRate = 1,
		   .gnssOutputDataRate = 1,
		   .insOutputDataRate = 10,
		   .isusefliter =0,
	};
#endif //  SYS_OS


#ifdef __cplusplus
	 }
#endif