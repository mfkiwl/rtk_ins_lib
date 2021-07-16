#ifndef _LC_SYSTEM_
#define _LC_SYSTEM_
#include <stdint.h>
#include <stdio.h>
#include "loosecoupleset.h"
#include "lcstruct.h"
#ifdef __cplusplus
extern "C" {
#endif
extern UpdataStruct mUpdataStruct;
extern GnssInsSystem mGnssInsSystem;
extern int8_t isEFKFinished;
extern int8_t IsFeedback;
extern int8_t PositionInitFlag;

extern int KFStatus;// = 0; // 0��oempty 1��oUpdata 2��ofeedback


extern int32_t gps_start_week;

int8_t InitP(GnssData mGnssData, float delay, ImuSensor mIMUSensor, const int16_t n, float* P);

int8_t SetImuSensorParameter(ImuSensor* imusensor, const int16_t mImuSensorType, const int16_t IMUDataRate);

int8_t initsystemfromcfg(const LCSetting lcSetting);

int8_t initsystemfromGNSS( );

int8_t initsystemfromGNSS_STATUS1();

int8_t ErrorReset(int32_t error_type);

int8_t initsystemSoftreset();

int8_t SetLCtime(double lctime);

#ifdef __cplusplus
}
#endif

#endif