#ifndef _LC_INSUPDATE_
#define _LC_INSUPDATE_
#include <stdint.h>
#include "datatype.h"
#include "lcstruct.h"


int8_t AddIMUData(const ImuData mImudata);

int8_t GetLCData(Nav* mNav, double LCTime,int type);


#endif