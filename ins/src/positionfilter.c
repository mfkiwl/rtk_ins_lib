#include "positionfilter.h"
#include "math.h"
#ifndef D_EPOCH_POS_THRESHOLD
#define D_EPOCH_POS_THRESHOLD (0.05)
#endif
#ifndef D_SECOND_POS_THRESHOLD
#define D_SECOND_POS_THRESHOLD (1.0)
#endif
#ifndef D_IMU_NUMBER_THRESHOLD
#define D_IMU_NUMBER_THRESHOLD (100)
#endif // 
#ifndef D_CHANGE_POS_THRESHOLD
#define D_CHANGE_POS_THRESHOLD (0.1)
#endif // 
void positionfilter_init(posfliterstruct *postionfliter)
{
	postionfliter->count = 0;
	for (int i = 0; i < 3; i++)
	{
		postionfliter->curx[i] = 0.0;
	}
	for (int i = 0; i < 3; i++)
	{
		postionfliter->totalx[i] = 0.0;
	}
	for (int i = 0; i < 3; i++)
	{
		postionfliter->dx[i] = 0.0;
	}
	postionfliter->d_lat = 0.0;
	postionfliter->d_lon = 0.0;
	postionfliter->d_height = 0.0;
}
void positionfilter_set(const float* x, posfliterstruct *positionfliter, int isgnssupdate,int iscurzupt, double RM, double RN,double height,double lat)
{
	for (int i = 0; i < 3; i++)
	{
		positionfliter->curx[i] = (double)(x[i]);
	}
	for (int i = 0; i < 3; i++)
	{
		positionfliter->totalx[i] += positionfliter->curx[i];
	}
	if (fabs(positionfliter->totalx[0]) < D_CHANGE_POS_THRESHOLD
		&& fabs(positionfliter->totalx[1]) < D_CHANGE_POS_THRESHOLD
		&& fabs(positionfliter->totalx[2]) < D_CHANGE_POS_THRESHOLD)
	{
		positionfilter_init(positionfliter);
	}
	else /*if (1 == isgnssupdate)*/
	{
		positionfliter->count = 0;
		if (iscurzupt != 1)
		{
			for (int i = 0; i < 3; i++)
			{

				if (fabs(positionfliter->totalx[i]) < D_SECOND_POS_THRESHOLD)
				{
					positionfliter->dx[i] = (1.0 / D_IMU_NUMBER_THRESHOLD)*positionfliter->totalx[i];
				}
				else
				{
					if (positionfliter->totalx[i] > 0)
					{
						positionfliter->dx[i] = (1.0 / D_IMU_NUMBER_THRESHOLD)*D_SECOND_POS_THRESHOLD;
					}
					else
					{
						positionfliter->dx[i] = -(1.0 / D_IMU_NUMBER_THRESHOLD)*D_SECOND_POS_THRESHOLD;
					}
				}
			}
			positionfliter->d_lat = positionfliter->dx[0] / (RM + height);
			positionfliter->d_lon = positionfliter->dx[1] / (RN + height) / cos(lat);
			positionfliter->d_height = positionfliter->dx[2];
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				positionfliter->dx[i] = 0.0;
			}
			positionfliter->d_lat = 0.0;
			positionfliter->d_lon = 0.0;
			positionfliter->d_height = 0.0;
		}
	}	
}
void positionfilter_compensent(double* postion, posfliterstruct *positionfliter , double RM, double RN, double height, double lat)
{
	if (fabs(positionfliter->totalx[0]) < D_EPOCH_POS_THRESHOLD
		&& fabs(positionfliter->totalx[1]) < D_EPOCH_POS_THRESHOLD
		&& fabs(positionfliter->totalx[2]) < D_EPOCH_POS_THRESHOLD)
	{
		positionfilter_init(positionfliter);
	}
	else
	{
		/* Set to 0 after compensation is completed*/
		if (fabs(positionfliter->totalx[0]) < D_EPOCH_POS_THRESHOLD)

		{
			positionfliter->totalx[0] = 0.0;
			positionfliter->curx[0] = 0.0;
			positionfliter->dx[0] = 0.0;
			positionfliter->d_lat = 0.0;
		}
		if (fabs(positionfliter->totalx[1]) < D_EPOCH_POS_THRESHOLD)
		{
			positionfliter->totalx[1] = 0.0;
			positionfliter->curx[1] = 0.0;
			positionfliter->dx[1] = 0.0;
			positionfliter->d_lon = 0.0;
		}
		if (fabs(positionfliter->totalx[2]) < D_EPOCH_POS_THRESHOLD)

		{
			positionfliter->totalx[2] = 0.0;
			positionfliter->curx[2] = 0.0;
			positionfliter->dx[2] = 0.0;
			positionfliter->d_height = 0.0;
		}
		/*for future use*/
		positionfliter->count += 1;
		if (positionfliter->count > 100)
		{
			positionfliter->count = 0;
		}
		double d_uncompensated_lat = (double)(positionfliter->totalx[0]) / ((double)(height) + RM);
		double d_uncompensated_lon = (double)(positionfliter->totalx[1]) / (double)(RN + height) / cos(lat);
		double d_uncompensated_height = (double)(positionfliter->totalx[2]);

		postion[0] = postion[0] + d_uncompensated_lat - positionfliter->d_lat;
		postion[1] = postion[1] + d_uncompensated_lon - positionfliter->d_lon;
		postion[2] = postion[2] - d_uncompensated_height + positionfliter->d_height;

		positionfliter->totalx[0] -= positionfliter->dx[0];
		positionfliter->totalx[1] -= positionfliter->dx[1];
		positionfliter->totalx[2] -= positionfliter->dx[2];
	}

}
