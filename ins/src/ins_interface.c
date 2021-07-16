#include <math.h>
#include "ins_interface_API.h"
#include "lcsystem.h"
#include "lcgnssupdate.h"
#include "lcinsupdate.h"
#include "loosecoupleset.h"
#include "insoutmsg.h"
#include "user_config.h"
#include "sensorsAPI.h"
#include "time_ref.h"
#include "rtklib_core.h"
#include "gpsAPI.h"
#include "user_config.h"
#include "taskRTK.h"
#include "sta9100_upgrade.h"


#ifndef SCALEFACTOR_ODO 
#define SCALEFACTOR_ODO (0.97 * 10 * 0.640 * 3.1415926/2000)
#endif

#ifndef RAD2DEG
#define  RAD2DEG (57.295779513082320)
#endif // !RAD2DEG
int32_t g_gnss_start_week = -1;

static uint16_t g_imu_week = 0;
static double g_imu_timestamp = 0.0;
static uint16_t g_imu_week_next = 0;
static double g_imu_timestamp_next = 0.0;

ImuData g_imu_data = {0};
OdoData g_odo_data = {0};

static GnssData g_gnss_data_temp = {0};
uint8_t g_gnss_obs_valid = 0;

ins_solution_t g_ins_sol = {0};
ins_solution_t *g_ptr_ins_sol = &g_ins_sol;
OdoData mOdoData;
uint32_t acquire_can_odo = 0;
int wheel_tick_update_flag = 0;
uint8_t odospeed_packet_rate   = 10;
uint8_t odospeed_packet_divide  = 0;


int8_t INSAddODOData(const OdoData odo_data);
void set_gnss_start_week(int32_t week)
{
	if (week > 1024 && week < 3072) {
		if (g_gnss_start_week == -1) {
			g_gnss_start_week = week;
		}
	}
}
uint8_t is_gnss_start_week_valid(void)
{
    if (g_gnss_start_week == -1) {
        return 0;
    } else {
        return 1;
    }
}
int32_t get_gnss_start_week(void)
{
    return g_gnss_start_week;
}

uint16_t get_imu_week(void)
{
    return g_imu_week;
}
double get_imu_timestamp(void)
{
    return g_imu_timestamp;
}
uint16_t get_imu_week_next(void)
{
    return g_imu_week_next;
}
double get_imu_timestamp_next(void)
{
    return g_imu_timestamp_next;
}

int8_t GetKFStatus(void)
{
	return KFStatus;
}

uint8_t get_gnss_obs_valid(void)
{
    return g_gnss_obs_valid;
}
void set_gnss_obs_valid(uint8_t value)
{
    g_gnss_obs_valid = value;
}

int32_t get_mGnssInsSystem_mlc_STATUS(void)
{
    return mGnssInsSystem.mlc_STATUS;
}

void set_wheel_tick_update_flag(int state)
{
    wheel_tick_update_flag = state;
}


static void _copy_ins_result(const GnssInsSystem* p_gnssInsSystem)
{
    int32_t n = p_gnssInsSystem->mKalmanStruct.n;

    g_ins_sol.gps_week          = get_imu_week();
    g_ins_sol.gps_millisecs     = (uint32_t)((get_imu_timestamp()+0.0001) * 1000);
	g_ins_sol.ins_status        = (uint32_t)p_gnssInsSystem->ins_status;
	g_ins_sol.pos_type          = (uint32_t)p_gnssInsSystem->ins_positin_type;
	g_ins_sol.latitude          = p_gnssInsSystem->outNav.lat * RAD2DEG;
	g_ins_sol.longitude         = p_gnssInsSystem->outNav.lon * RAD2DEG;
	g_ins_sol.height            = p_gnssInsSystem->outNav.height;
	g_ins_sol.north_velocity    = p_gnssInsSystem->outNav.vn;
	g_ins_sol.east_velocity     = p_gnssInsSystem->outNav.ve;
	g_ins_sol.up_velocity       = -p_gnssInsSystem->outNav.vd;
	g_ins_sol.roll              = p_gnssInsSystem->outNav.roll * RAD2DEG;
	g_ins_sol.pitch             = p_gnssInsSystem->outNav.pitch * RAD2DEG;
	g_ins_sol.azimuth           = p_gnssInsSystem->outNav.heading * RAD2DEG;
	g_ins_sol.latitude_std       = sqrt(p_gnssInsSystem->mKalmanStruct.P[0]);
	g_ins_sol.longitude_std      = sqrt(p_gnssInsSystem->mKalmanStruct.P[n+1]);
	g_ins_sol.altitude_std       = sqrt(p_gnssInsSystem->mKalmanStruct.P[2*n+2]);
	g_ins_sol.north_velocity_std = sqrt(p_gnssInsSystem->mKalmanStruct.P[3*n+3]);
	g_ins_sol.east_velocity_std  = sqrt(p_gnssInsSystem->mKalmanStruct.P[4*n+4]);
	g_ins_sol.up_velocity_std    = sqrt(p_gnssInsSystem->mKalmanStruct.P[5*n+5]);
	g_ins_sol.roll_std           = sqrt(p_gnssInsSystem->mKalmanStruct.P[6*n+6])* RAD2DEG;
	g_ins_sol.pitch_std          = sqrt(p_gnssInsSystem->mKalmanStruct.P[7*n+7])* RAD2DEG;
	g_ins_sol.azimuth_std        = sqrt(p_gnssInsSystem->mKalmanStruct.P[8*n+8])* RAD2DEG;
	g_ins_sol.time_since_update  = (uint16_t)p_gnssInsSystem->GNSSLOSETIME1;
    g_ins_sol.mlc_status         = p_gnssInsSystem->mlc_STATUS;
}

void ins_init(void)
{
    float32_t   pri_lever_arm[3];
    float32_t   vrp_lever_arm[3];
    float32_t   user_lever_arm[3];
    float32_t   rotation_rbv[3];

    get_rotation_rbv(rotation_rbv);
    mLCSetting.rotationRBV[0] = rotation_rbv[0] * D2R;
    mLCSetting.rotationRBV[1] = rotation_rbv[1] * D2R;
    mLCSetting.rotationRBV[2] = rotation_rbv[2] * D2R;

    get_pri_lever_arm(pri_lever_arm);
    mLCSetting.priLeverArm[0] = pri_lever_arm[0];
    mLCSetting.priLeverArm[1] = pri_lever_arm[1];
    mLCSetting.priLeverArm[2] = pri_lever_arm[2];
    // mLCSetting.priLeverArm[0] = 1.19;
    // mLCSetting.priLeverArm[1] = -0.45;
    // mLCSetting.priLeverArm[2] = -0.9;

    get_vrp_lever_arm(vrp_lever_arm);
    mLCSetting.odoLeverArm[0] = vrp_lever_arm[0];
    mLCSetting.odoLeverArm[1] = vrp_lever_arm[1];
    mLCSetting.odoLeverArm[2] = vrp_lever_arm[2];

    get_user_lever_arm(user_lever_arm);
    mLCSetting.userLeverArm[0] = user_lever_arm[0];
    mLCSetting.userLeverArm[1] = user_lever_arm[1];
    mLCSetting.userLeverArm[2] = user_lever_arm[2];
    // mLCSetting.userLeverArm[0] = 1.19;
    // mLCSetting.userLeverArm[1] = -0.45;
    // mLCSetting.userLeverArm[2] = -0.9;

    mLCSetting.imuDataRate = 100;
    uint8_t test_buff[200];
    sprintf(test_buff,"priLeverArmx: %f,priLeverArmy: %f,priLeverArmz: %f,userLeverArmx: %f,userLeverArmy: %f,userLeverArmz: %f",\
    mLCSetting.priLeverArm[0],
    mLCSetting.priLeverArm[1],
    mLCSetting.priLeverArm[2],
    mLCSetting.userLeverArm[0],
    mLCSetting.userLeverArm[1],
    mLCSetting.userLeverArm[2]);
    // uart7_write_dma(test_buff, strlen(test_buff));
    initsystemfromcfg(mLCSetting);
}

/*  ADD origin GNSS data and Updata process
	args:  GnssData *mGnssData              origin gnss data struct
	return:status ()

*/
int8_t INSADDGNSSDATA(const GnssData mGnssData)
{
	int8_t ret = -1;
	mGnssInsSystem.GNSSflag = 1;
	if (mGnssInsSystem.mImuData.timestamp + 0.2/mLCSetting.imuDataRate >= mGnssData.timestamp ) {
		KFStatus = 1;
		ret = ADDGNSSDATA(mGnssData);

        if (isEFKFinished) {
            KFStatus = 2;
        } else {
            KFStatus = 0;
        }
        g_gnss_obs_valid = 1;
    } else {
        ret = 0;
        memcpy(&g_gnss_data_temp, &mGnssData, sizeof(GnssData));
    }
    return ret;
}

static int8_t INSAddIMUData(const ImuData mImuData)
{
	int8_t ret = -1;
    static int wheel_tick_div = 0;
	if (1 == mGnssInsSystem.GNSSflag
		&& 0 == KFStatus
		&& mGnssInsSystem.mImuData.timestamp + 0.2 / mLCSetting.imuDataRate >= g_gnss_data_temp.timestamp)
	{
		KFStatus = 1;
		ADDGNSSDATA(g_gnss_data_temp);

        if (isEFKFinished) {
            KFStatus = 2;
        } else {
            KFStatus = 0;
        }
        g_gnss_obs_valid = 1;
    }
	ret = AddIMUData(mImuData);

    // wheel_tick_div++;
    // if (wheel_tick_div == 10)
    // {
    //     uint8_t ret = car_get_wheel_speed(&mOdoData.vehicle_speed, (uint8_t*)&mOdoData.fwd, (uint32_t*)&mOdoData.week, &mOdoData.timestamp);
    //     if (ret)
    //     {
    //         mOdoData.mode = 0;
    //         acquire_can_odo++;
    //         if (acquire_can_odo >= 50)
    //         {
    //             acquire_can_odo = 50;
    //         }
    //         if (mLCSetting.isUseOdo == 1)
    //         {
    //             INSAddODOData(mOdoData);
    //             set_wheel_tick_update_flag(1);
    //         }
    //         else
    //         {
    //             if (acquire_can_odo >= 50)
    //             {
    //                 mLCSetting.isUseOdo = 1;
    //                 mLCSetting.isInsFreInit = 0;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         if (acquire_can_odo > 0)
    //             acquire_can_odo--;
    //         if (mLCSetting.isUseOdo == 1)
    //         {
    //             if (acquire_can_odo == 0)
    //             {
    //                 if(mOdoData.wheel_tick < 1000)
    //                 {
    //                     mLCSetting.isUseOdo = 0;
    //                     mLCSetting.isInsFreInit = 1;
    //                 }
    //             }
    //         }
    //     }
    //     wheel_tick_div = 0;
    // }

    _copy_ins_result(&mGnssInsSystem);
	return ret;
}

void ins_gnss_time_update(void)
{
    mcu_time_base_t imu_time = g_MCU_time;
    uint16_t start_week = get_gnss_start_week();
    uint8_t test_buff[100];
    gtime_t time;
    time.time = imu_time.time;
    time.sec = (double)imu_time.msec / 1000 + 0.0001;
    int week = 0;
    g_imu_data.timestamp = time2gpst(time, &week);
    // sprintf(test_buff,"week = %d,g_imu_data.timestamp = %lf\r\n",week,g_imu_data.timestamp);
    // uart7_write_dma(test_buff,strlen(test_buff));
    if (week > 1024 && !is_gnss_start_week_valid()) {
        set_gnss_start_week(week);
    }
    if (is_gnss_start_week_valid()) {
        gtime_t time;
        time.time = imu_time.time;
        time.sec = (double)imu_time.msec / 1000;
        int week = 0;
        g_imu_data.timestamp = time2gpst(time, &week);

        if (g_ptr_gnss_sol->gnss_update == 1) {
            mcu_time_base_t next_obs_time;
            // next_obs_time = IMU_start_time;
            // next_obs_time.msec = (IMU_start_time.msec + 100) / 100 * 100;
            // if (next_obs_time.msec == 1000)
            // {
            //     next_obs_time.msec = 0;
            //     next_obs_time.time++;
            // }
            next_obs_time.time = imu_time.time + 1;
            next_obs_time.msec = 0;

            gtime_t time_next;
            int week_next = 0;
            time_next.time = next_obs_time.time;
            time_next.sec = (double)next_obs_time.msec / 1000;
            g_imu_data.timestamped = time2gpst(time_next, &week_next);
            // sprintf(test_buff,"0g_imu_data.week = %d,g_imu_data.timestamp = %lf\r\n",g_imu_data.week,g_imu_data.timestamp);
            // uart7_write_dma(test_buff,strlen(test_buff));
            g_imu_week_next = week_next;
            g_imu_timestamp_next = g_imu_data.timestamped;

            g_imu_data.timestamped += (week_next - start_week) * SECONDS_IN_WEEK;
            // sprintf(test_buff,"1g_imu_data.week = %d,week_next = %d,start_week = %d,g_imu_data.timestamp = %lf\r\n",\
            // g_imu_data.week,week_next,start_week,g_imu_data.timestamp);
            // uart7_write_dma(test_buff,strlen(test_buff));
        } else {
            g_imu_week_next = 0;
            g_imu_timestamp_next = 0.0;
            g_imu_data.timestamped = 0;
        }
        
        g_imu_week = week;
        g_imu_timestamp = g_imu_data.timestamp;

        g_imu_data.week = start_week;
        g_imu_data.timestamp += (week - start_week) * SECONDS_IN_WEEK;
        // sprintf(test_buff,"4g_imu_data.week = %d,g_imu_data.timestamp = %lf\r\n",\
        // g_imu_data.week,g_imu_data.timestamp);
        // uart7_write_dma(test_buff,strlen(test_buff));
    } else {
        g_imu_data.week = 0;
        g_imu_data.timestamp = imu_time.time + (imu_time.msec * 0.001);
        g_imu_data.timestamped = 0;

        g_imu_timestamp = g_imu_data.timestamp;
        // sprintf(test_buff,"2g_imu_data.week = %d,g_imu_data.timestamp = %lf\r\n",g_imu_data.week,g_imu_data.timestamp);
        // uart7_write_dma(test_buff,strlen(test_buff));
    }
    g_odo_data.week = g_imu_data.week;
    g_odo_data.timestamp = g_imu_data.timestamp;
    //UART7_Printf("g_imu_data.week:%d,g_imu_data.timestamp:%lf\r\n",g_imu_data.week,g_imu_data.timestamp);
}

void ins_fusion(void)
{
    if (odospeed_packet_rate != 0) {
        if (odospeed_packet_divide >= odospeed_packet_rate) {
            uint8_t ret = car_get_wheel_speed(&g_odo_data.vehicle_speed, (uint8_t*)&g_odo_data.fwd, (uint32_t*)&g_odo_data.week, &g_odo_data.timestamp);
            if (ret) {
                g_odo_data.mode = 0;
                acquire_can_odo++;
                if (acquire_can_odo >= 50) {
                    acquire_can_odo = 50;
                }
                if (mLCSetting.isUseOdo == 1) {
                    g_odo_data.fwd = 1;
                    INSAddODOData(g_odo_data);
                    set_wheel_tick_update_flag(1);
                } else {
                    if (acquire_can_odo >= 50) {
                        mLCSetting.isUseOdo = 1;
                        mLCSetting.isInsFreInit = 0;
                    }
                }
            } else {
                if (acquire_can_odo > 0) 
                    acquire_can_odo--;
                if (mLCSetting.isUseOdo == 1) {
                    if (acquire_can_odo == 0) {
                        if (g_odo_data.wheel_tick < 1000) {
                            mLCSetting.isUseOdo = 0;
                            mLCSetting.isInsFreInit = 1;
                        }
                    }
                }
            }

            odospeed_packet_divide = 1;
        } else {
            odospeed_packet_divide++;
        }
    }

    double data[3] = {0.0};
    sens_GetAccelData_g(data);
    g_imu_data.Accx = data[0] * 9.7803267714;
    g_imu_data.Accy = data[1] * 9.7803267714;
    g_imu_data.Accz = data[2] * 9.7803267714;
    // Obtain rate-sensor data [rad/sec]
    sens_GetRateData_radPerSec(data);
    g_imu_data.Gyrox = data[0];
    g_imu_data.Gyroy = data[1];
    g_imu_data.Gyroz = data[2];
    INSAddIMUData(g_imu_data);

    // uint8_t test_buff[100];
    // sprintf(test_buff,"timestamp = %f,accx = %f,Accy = %f,Accz = %f\r\n",g_imu_data.timestamp,g_imu_data.Accx,g_imu_data.Accy,g_imu_data.Accz);
    // uart7_write_dma(test_buff,strlen(test_buff));
    // UART7_Printf("%s", test_buff);
    //printf("imu data :%13.4f,%10.6f\n",g_imu_data.timestamp,g_imu_data.Accz);
}

double* get_imu_acc(void)
{
    return &g_imu_data.Accx;
}

double* get_imu_gyro(void)
{
    return &g_imu_data.Gyrox;
}

int8_t INSAddODOData(const OdoData odo_data)
{
	int8_t ret = -1;
#ifndef DEBUGMESSAGE
	int32_t outtime_week = odo_data.week;
	double  outtime_second = odo_data.timestamp;
	uint32_t outtime_second_m = (uint32_t)(outtime_second * 1000);
	if (odo_data.timestamp >= SECONDSOFWEEK)
	{
		int32_t df_week = floor(odo_data.timestamp / SECONDSOFWEEK);
		outtime_second = odo_data.timestamp - df_week * SECONDSOFWEEK;
		outtime_second_m = (uint32_t)(outtime_second * 1000);
		outtime_week = odo_data.week + SECONDSOFWEEK;
	}
	writeOdoDataMsg(&odo_data);
#endif // DEBUGMESSAGE
	double curent_vehicle_speed = 0.0;

		switch (odo_data.mode)
		{
		case 0:
		{
			ret = 1;
			mGnssInsSystem.Odomode = 1;
			curent_vehicle_speed = odo_data.vehicle_speed;
		}break;
		case 1:
		{
			if (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp > 0 && mGnssInsSystem.mOdoData.timestamp > 0)
			{
				double vehicle_speed;
				if (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp > 0.99 && odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp < 1.01)
				{
					vehicle_speed = SCALEFACTOR_ODO * (odo_data.wheel_tick - mGnssInsSystem.mOdoData.wheel_tick);
				}
				else
				{
					vehicle_speed = SCALEFACTOR_ODO * (odo_data.wheel_tick - mGnssInsSystem.mOdoData.wheel_tick) * 0.1 / (odo_data.timestamp - mGnssInsSystem.mOdoData.timestamp);
				}
				 vehicle_speed = mGnssInsSystem.mOdoData.fwd == 1 ? vehicle_speed : -vehicle_speed;
				 if (fabs(vehicle_speed) < 0.01)
				 {
					 curent_vehicle_speed = 0;
				 }
				 else
				 {
					curent_vehicle_speed = vehicle_speed;
					// curent_vehicle_speed = 0.10 * mGnssInsSystem.mOdoData.vehicle_speed + 0.90 * vehicle_speed;
			      }	
				 ret = 1;
			}
			else
			{
				ret = 0;
				mGnssInsSystem.IsUseOdo = 0;
			}
		}break;
		default:
			break;
		}

	
#ifdef DEBUGMESSAGE
tracerawodo(&odo_data);
#endif // DEBUGMESSAGE
	mGnssInsSystem.mOdoData.week = odo_data.week;
	mGnssInsSystem.mOdoData.timestamp = odo_data.timestamp;
	mGnssInsSystem.mOdoData.vehicle_speed = curent_vehicle_speed;
	mGnssInsSystem.mOdoData.wheel_tick = odo_data.wheel_tick;
	mGnssInsSystem.mOdoData.fwd = odo_data.fwd;
	return ret;
}


int32_t get_wheel_tick_update_flag(void)
{
    return wheel_tick_update_flag;
}


uint16_t get_odo_gps_week(void)
{
    int32_t outtime_week = g_odo_data.week;
    double  outtime_second = g_odo_data.timestamp;
    if (outtime_second >= SECONDSOFWEEK) {
        int32_t df_week = floor(outtime_second / SECONDSOFWEEK);
        outtime_second = outtime_second - df_week * SECONDSOFWEEK;
        outtime_week = outtime_week + df_week;
    }
    return outtime_week;
}

double get_odo_gps_timestamp(void)
{
    int32_t outtime_week = g_odo_data.week;
    double  outtime_second = g_odo_data.timestamp;
    if (outtime_second >= SECONDSOFWEEK) {
        int32_t df_week = floor(outtime_second / SECONDSOFWEEK);
        outtime_second = outtime_second - df_week * SECONDSOFWEEK;
        outtime_week = outtime_week + df_week;
    }
    return outtime_second;
}

int8_t get_odo_mode(void)
{
    return g_odo_data.mode;
}


double get_odo_speed(void)
{
    return g_odo_data.vehicle_speed;
}

int8_t get_odo_fwd(void)
{
    return g_odo_data.fwd;
}

uint64_t get_odo_wheel_tick(void)
{
    return g_odo_data.wheel_tick;
}
