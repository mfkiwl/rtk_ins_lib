/*****************************************************************************
   FILE:          main.c
   PROJECT:       STA8100 GPS application
   SW PACKAGE:    STA8100 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   The main application to run and test STA8100 GPS library
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25
*****************************************************************************/

#ifndef GNSSAPP_H
#define GNSSAPP_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gnss_defs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct gnssapp_startup_time_s
{
  gpOS_clock_t  nvm_start_cpu_time;
  gpOS_clock_t  sw_config_cpu_time;
  gpOS_clock_t  gnss_debug_cpu_time;
  gpOS_clock_t  xtal_cpu_time;
  gpOS_clock_t  gnss_lib_start_cpu_time;
  gpOS_clock_t  gnss_lib_set_timer_clock;
  gpOS_clock_t  gnss_module_start_cpu_time;
  gpOS_clock_t  nmea_start_cpu_start;
  gpOS_clock_t  as_start_cpu_start;
  gpOS_clock_t  gnssapp_init_end_cpu_time;
  gpOS_clock_t  suspend_restart_support_time;
  gpOS_clock_t  running_time_update_cpu_time;
  tUInt         running_time_ms;
  tUInt         MTU_timer_clock;
} gnssapp_startup_time_t;

typedef enum
{
  GNSSAPP_LOW_POWER_STANDBY_DISABLE,
  GNSSAPP_LOW_POWER_STANDBY_ENABLE
} gnss_app_lowpow_standby_type_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   gnssapp_init                    ( gpOS_partition_t *fast_part);
extern const tChar *  gnssapp_version                 ( tVoid);
extern const tChar *  gnssapp_binimg_version          ( tVoid);
extern tUInt          gnssapp_swcfg_version           ( tVoid);

extern tVoid          gnssapp_suspend                 ( tVoid);
extern tVoid          gnssapp_restart                 ( tVoid);

extern gpOS_error_t   gnssapp_imuselftestcmd          ( tInt, boolean_t *);

extern tVoid          gnssapp_swconfig_reload         ( tVoid);

extern boolean_t      gnssapp_sensors_presence        ( tUInt*) ;

extern gpOS_task_t *  gnssapp_get_ll_sensor_process_id(void) ;

extern gpOS_error_t   gnssapp_get_sens_config         (tUInt , tChar *, tInt *);

extern gnssapp_startup_time_t*  gnssapp_get_startup_time      ( tVoid);
extern tVoid                    gnssapp_reset_startup_time    ( tVoid);
extern tUInt                    gnssapp_update_running_time   ( tVoid);

#endif /* GNSSAPP_H */
