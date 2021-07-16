#ifndef _INS_MECH_
#define _INS_MECH_


#include "lcstruct.h"



int8_t KF_feedback(Par* mPar, float x[15], Nav* mNav, int IsBiasFeedBack, GnssInsSystem *mGnssInsSystem);

int8_t DataChangeLC(const ImuData* mImuData, INS *cins);

int8_t compensate(const Sensorbias* bias, INS* ins);

int8_t INS_MECH(const INS* ins_pre, const INS* ins_cur, const Nav* nav, Nav* nav1, Par* mPar);

/***************************************************************************************
Function: KF_predict_16PHI
Description: ;Calculate PHI(transition matrix)
Input :dt  time increment
	   nav  Navigation information
Output:PHI  transition matrix
Return :
Others:
********************************************************************************************/
int8_t KF_predict_16PHI(const float dt, const Nav *mNav, const Par *mPar, const ImuSensor *mImuSensor, const int16_t n,float* PHI);
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
int8_t  KF_predict(const int16_t n,const float* PHI, const float* Q, const float dt, float* x, float* P, UpdataStruct* updataStruct);

 

#endif // _INS_MECH_
