#ifndef  _POSITION_FILTER
#define _POSITION_FILTER
#include "lcstruct.h"
void positionfilter_init(posfliterstruct *postionfliter);
void positionfilter_set(const float* x, posfliterstruct *positionfliter, int isgnssupdate, int iscurzupt, double RM, double RN, double height, double lat);
void positionfilter_compensent(double* postion, posfliterstruct *positionfliter, double RM, double RN, double height, double lat);




#endif // ! _POSITION_FILTER
