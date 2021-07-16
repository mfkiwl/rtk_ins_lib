#ifndef _LC_GNSS_UPDATE_
#define _LC_GNSS_UPDATE_
#include <stdint.h>
#include "datatype.h"
int8_t OdoObsUpdata(double systemtime);
int8_t  ADDGNSSDATA(const GnssData msg);
int8_t VirtualObsUpdata(double systemtime);


#endif