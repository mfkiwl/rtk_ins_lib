#ifndef _Zupt_Detect_
#define _Zupt_Detect_
#include <stdint.h>
#include "datatype.h"


int8_t InitHardModeAxix(const int8_t* MisAlignmentAiax);
int8_t InitSoftModeAxix(const int8_t* MisAlignmentAiax);
int8_t SetZUPTThreShold(int16_t imuenortype, int16_t imudatarate);
int8_t reSetZUPTThreShold(const int8_t *MisAlignmentAiax);

int8_t SetZuptDetectData(const ImuData mImudata, const int16_t imudataRata);
int8_t GetZuptVal();
int8_t IsOnLine();
int8_t IsOnLine2();
int8_t IsOnLine3();



int8_t SetZUPTMeanFirst();
int8_t SetZUPTMean(const ImuData mImudata);
int8_t GetZUPTMean(double *mean);


int8_t SetGNSSZuptData(const GnssData *pGnssData, const GnssData *pPreGnssData);

double GetZuptGNSStime();

int8_t SetGNSSFlexDetectData(const GnssData *pGnssData, const GnssData *pPreGnssData, int8_t *axix, double *misAlignment);

int8_t  SetGNSSDetectData(const GnssData *pGnssData, const GnssData *pPreGnssData);


int8_t GetGNSSStatus();

int8_t SetInnoDetectData(const float *inno);

int8_t InitInnoErrorDect();





#endif