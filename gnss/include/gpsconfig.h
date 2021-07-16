/*****************************************************************************
   includes
*****************************************************************************/

#include "gnss_defs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

tInt  gnss_dsp_trk_task_priority       = 14;
tInt  gnss_tracker_task_priority       = 13;
tInt  gnss_dsp_acq_task_priority       = 12;
tInt  gnss_rtc_write_task_priority     = 12;
tInt  gnss_flash_erase_priority        = 11;
tInt  waas_task_priority               = 10;
tInt  gnss_measure_task_priority       = 9;
tInt  st_agps_ephmgr_task_priority     = 6;
tInt  nmea_cmdif_task_priority         = 8;
tInt  gnss_navigate_pps_task_priority  = 10;
tInt  gnss_navigate_task_priority      = 7;
tInt  gnss_navigate_data_task_priority = 6;
tInt  st_agps_task_priority            = 1;
tInt  gnss_dsp_ifm_task_priority       = 5;
tInt  nmea_outmsg2_task_priority       = 15;

/* Acquire constant parameters */
/* the defualt setting for the NCO  if not available in flash */
nco_t   startup_nco_max     = -37000;
nco_t   startup_nco_center  = -47000;
nco_t   startup_nco_min     = -57000;

/* nmea constant parameters */
tUInt         NMEA_msg_list[2]            = {0,0};
tUInt         NMEA_msg_list_1[2]          = {0,0};
tUInt         NMEA_msg_list_2[2]          = {0,0};
tUInt         NMEA_on_debug_msg_list[2]   = {0,0};
tUInt         NMEA_on_debug_msg_list_1[2] = {0,0};
tUInt         NMEA_on_debug_msg_list_2[2] = {0,0};
tU8           NMEA_msg_list_scaling       = 1;
tU8           NMEA_msg_list_scaling_1     = 1;
tU8           NMEA_msg_list_scaling_2     = 1;
tInt          nmea_outmsg_GGA_posdigit    = 5;
tInt          nmea_outmsg_RMC_posdigit    = 3;
tInt          nmea_outmsg_speed_digits    = 1;
tInt          nmea_outmsg_course_digits   = 1;
gpOS_clock_t  nmea_outmsg_delaytonextfix  = 0;

gnss_diff_source_t  diff_source = DIFF_SOURCE_AUTO;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

