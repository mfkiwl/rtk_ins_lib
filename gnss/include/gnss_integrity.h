#ifndef _GNSS_INTEGRITY_H_
#define _GNSS_INTEGRITY_H_

#include <stdio.h>
#include "gnss_datatype.h"

//double perct[4] = {4.42,4.89,5.32,5.73}; //10-5, 10-6, 10-7, 10-8

#ifdef __cplusplus
extern "C" {
#endif

double hor_pos_alert_limit_determination();
double ver_pos_alert_limit_determination();
double lateral_pos_alert_limit_determination();
double longitu_pos_alert_limit_determination();

double lateral_vel_alert_limit_determination();
double longitu_vel_alert_limit_determination();
double hor_vel_alert_limit_determination();
double ver_vel_alert_limit_determination();

double hor_pos_protection_level_determination(rcv_rtk_t *rcv);
double ver_pos_protection_level_determination(rcv_rtk_t *rcv);
double lateral_pos_protection_level_determination(rcv_rtk_t *rcv);
double longitu_pos_protection_level_determination(rcv_rtk_t *rcv);

double hor_vel_protection_level_determination(rcv_rtk_t *rcv);
double ver_vel_protection_level_determination(rcv_rtk_t *rcv);
double lateral_vel_protection_level_determination(rcv_rtk_t *rcv);
double longitu_vel_protection_level_determination(rcv_rtk_t *rcv);

void rtk_integrity(rcv_rtk_t *rcv);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
