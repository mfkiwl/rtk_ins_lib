#include "rtk_eng.h"
/*--------------------------------------------------------*/
/* code from rtklib to decode RTCM3 */
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "model.h"
#include "print_nmea.h"
#include "rtkcmn.h"
#include "rtcm.h"
#include "taskRTK.h"
#include "rtklib_core.h"

#define KNOT2M 0.514444444 /* m/knot */
#ifdef RTK_ENABLE
extern epoch_t gRov;
#endif

RTK_RAM_CODE static void ats_strcpy(char* des, const char* src, unsigned int maxlen)
{
	unsigned int i = 0, len = 0;

	if ((des == NULL) || (src == NULL) || (maxlen == (unsigned int)0))
	{
		/* error return */
	}
	else
	{
		len = (unsigned int)strlen(src);
		if (len >= maxlen)
		{
			len = maxlen;
		}
		for (i = 0; i < len; i++)
		{
			des[i] = (char)src[i];
		}
	}
}

RTK_RAM_CODE static void ats_strcat(char* des, const char* src, unsigned int maxlen)
{
	unsigned int i = 0, deslen = 0, srclen = 0;

	if ((des == NULL) || (src == NULL) || (maxlen == (unsigned int)0))
	{
		/* error return */
	}
	else
	{
		deslen = (unsigned int)strlen(des);
		srclen = (unsigned int)strlen(src);
		if ((deslen + srclen) >= maxlen)
		{
			srclen = maxlen - deslen;
		}
		for (i = 0; i < srclen; i++)
		{
			des[deslen + i] = (char)src[i];
		}
	}
}

RTK_RAM_CODE static void ats_strset(char* des, const char c, unsigned int maxlen)
{
	unsigned int i = 0;
	if ((des == NULL) || (maxlen == (unsigned int)0))
	{
		/* error return */
	}
	else
	{
		for (i = 0; i < maxlen; i++)
		{
			des[i] = (char)c;
		}
	}
}


RTK_RAM_CODE static void RealToArray(double realx, char *a, unsigned char idx, unsigned char dd)
{
	unsigned long long temp1 = 0, temp2 = 0, atemp = 0;
	double tempf = 0.0, real = realx;
	unsigned char i = 0, id = idx;
	if (id == (unsigned char)0)
	{
		if (fabs(real) < 10.0)
		{
			id = 1;
		}
		else if (fabs(real) < 100.0)
		{
			id = 2;
		}
		else if (fabs(real) < 1000.0)
		{
			id = 3;
		}
		else if (fabs(real) < 10000.0)
		{
			id = 4;
		}
		else
		{
			id = 8;
		}
	}

	if (real >= 0.0)
	{
		tempf = real * pow(10.0, (double)dd);
		temp1 = (unsigned long long)tempf;
		temp2 = (unsigned long long)real * (unsigned long long)pow(10.0, (double)dd);
		temp2 = temp1 - temp2;
		temp1 = (unsigned long long)real;
		for (i = id; i > (unsigned char)0; i--)
		{
			atemp = (temp1 % (unsigned long long)10) + (unsigned long long)0x30;
			a[i - (unsigned char)1] = (char)atemp;
			temp1 = temp1 / (unsigned long long)10;
		}
		a[id] = (char)'.';
		a[id + dd + (unsigned char)1] = (char)'\0';
		for (i = (unsigned char)(id + dd); i > id; i--)
		{
			atemp = (temp2 % (unsigned long long)10) + (unsigned long long)0x30;
			a[i] = (char)atemp;
			temp2 = temp2 / (unsigned long long)10;
		}

	}
	else
	{
		real = 0.0 - real;
		tempf = real * pow(10.0, (double)dd);
		temp1 = (unsigned long long)tempf;
		temp2 = (unsigned long long)real * (unsigned long long)pow(10.0, (double)dd);
		temp2 = temp1 - temp2;
		temp1 = (unsigned long long)real;
		a[0] = (char)0x2D;
		for (i = id; i > (unsigned char)0; i--)
		{
			atemp = (temp1 % (unsigned long long)10) + (unsigned long long)0x30;
			a[i] = (char)atemp;
			temp1 = (temp1 / (unsigned long long)10);
		}
		a[id + (unsigned char)1] = (char)'.';
		a[id + dd + (unsigned char)2] = (char)'\0';
		for (i = id + dd + (unsigned char)1; i > (id + (unsigned char)1); i--)
		{
			atemp = (temp2 % (unsigned long long)10) + (unsigned long long)0x30;
			a[i] = (char)atemp;
			temp2 = temp2 / (unsigned long long)10;
		}
	}
	if (dd == 0)
	{
		a[id] = '\0';

	}

}

RTK_RAM_CODE static void itoa_user(int num, char *str, int radix)
{
	char index[] = "0123456789ABCDEF";
	unsigned int unum = 0;
	int i = 0, j = 0, k = 0;

	if ((radix == 10) && (num < 0))
	{
		unum = (unsigned int)-num;
		str[i] = (char)'-';
		i++;
	}
	else
	{
		unum = (unsigned int)num;
	}
	str[i] = index[unum % (unsigned int)radix];
	i++;
	unum /= (unsigned int)radix;
	while (unum != (unsigned int)0)
	{
		str[i] = index[unum % (unsigned int)radix];
		i++;
		unum /= (unsigned int)radix;
	}
	str[i] = (char)'\0';

	if (str[0] == (char)'-')
	{
		k = 1;
	}
	else
	{
		k = 0;
	}

	for (j = k; j <= ((i - 1) / 2); j++)
	{
		char temp = 0;
		temp = str[j];
		str[j] = (char)str[((i - 1) + k) - j];
		str[((i - 1) + k) - j] = (char)temp;
	}
}

RTK_RAM_CODE static int print_nmea_gga(const double* ep, const double* xyz, int nsat, int type,
	double dop, double geo_sep, double age, char* buff)
{
	double h = 0.0, pos[3] = { 0 }, dms1[3] = { 0 }, dms2[3] = { 0 };
	char* p = (char*)buff, *q = NULL;
	unsigned char sum = 0;
	char buf[20] = { 0 };
	int ret = 0;

	if (type < 1)
	{
		ats_strcpy(buff, "$GPGGA,,,,,,,,,,,,,,", (unsigned int)1024);
		sum = (unsigned char)0;
		for (q = &buff[1]; *q != 0; q++)
		{
			sum ^= (unsigned char)* q;
		}
		ats_strcat(buff, "*", (unsigned int)1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, "\r\n", (unsigned int)1024);
	}
	else
	{
		ecef2pos(xyz, pos);
		h = geo_sep;
		deg2dms(fabs(pos[0]) * R2D, dms1, 7);
		deg2dms(fabs(pos[1]) * R2D, dms2, 7);

		ats_strcpy(buff, "$GPGGA,", 1024);

		RealToArray((ep[3] * 10000.0) + (ep[4] * 100.0) + ep[5] + 0.001, buf, 6, 2);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray((dms1[0] * 100.0) + dms1[1] + (dms1[2] / 60.0), buf, 4, 7);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, (pos[0] >= 0.0) ? ",N," : ",S,", (unsigned int)1024);


		RealToArray((dms2[0] * 100.0) + dms2[1] + (dms2[2] / 60.0), buf, 5, 7);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, (pos[1] >= 0.0) ? ",E," : ",W,", (unsigned int)1024);


		itoa_user(type, buf, 10);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray((double)nsat, buf, 2, 0);
		buf[2] = 0;
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray(dop, buf, 0, 1);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray(pos[2] - h, buf, 0, 3);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",M,", (unsigned int)1024);


		RealToArray(h, buf, 0, 3);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",M,", (unsigned int)1024);


		RealToArray(age, buf, 0, 1);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);

		sum = (unsigned char)0;
		for (q = &buff[1]; *q != 0; q++)
		{
			sum ^= (unsigned char)* q; /* check-sum */
		}

		ats_strcat(buff, "*", (unsigned int)1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, (unsigned int)1024);

		ats_strcat(buff, "\r\n", (unsigned int)1024);
	}
	ret = (int)strlen(buff);
	return ret;
}

RTK_RAM_CODE extern int print_nmea_gst(const double* ep, const float* var_llh, char* buff)
{
	char* p = (char*)buff, *q = NULL;
	unsigned char sum = 0;
	int ret = 0;
	char buf[20] = { 0 };

	ats_strcpy(buff, "$GNGST,", (unsigned int)1024);

	RealToArray((ep[3] * 10000.0) + (ep[4] * 100.0) + ep[5] + 0.001, buf, 6, 2);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(0.0, buf, 1, 1);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(0.0, buf, 2, 1);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(0.0, buf, 2, 1);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(var_llh[1], buf, 2, 1);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(var_llh[0], buf, 2, 1);
	ats_strcat(buff, buf, (unsigned int)1024);
	ats_strcat(buff, ",", (unsigned int)1024);


	RealToArray(var_llh[2], buf, 2, 1);
	ats_strcat(buff, buf, (unsigned int)1024);

	sum = (unsigned char)0;
	for (q = &buff[1]; *q != (char)0; q++)
	{
		sum ^= (unsigned char)* q; /* check-sum */
	}

	ats_strcat(buff, "*", (unsigned int)1024);

	itoa_user((int)sum, buf, 16);
	ats_strcat(buff, buf, (unsigned int)1024);

	ats_strcat(buff, "\r\n", (unsigned int)1024);

	ret = (int)strlen(buff);
	return ret;
}

RTK_RAM_CODE extern int print_rmc(gtime_t gtime, const double* ecef, int fixID, char* buff)
{
	static double dirp = 0.0;
	gtime_t ut;
	double ep[6], pos[3], enuv[3], dms1[3], dms2[3], vel = 0.0, dir = 0.0, amag = 0.0;
	char* p = buff, *q = NULL;
	const char* emag = "E";
	unsigned char sum = 0;
	char buf[20] = { 0 };
	int ret = 0;
	int eptemp = 0;

	if (fixID < 1)
	{
		ats_strcpy(buff, "$GPRMC,,,,,,,,,,,,", (unsigned int)1024);
		sum = (unsigned char)0;
		for (q = &buff[1]; *q != (char)0; q++)
		{
			sum ^= (unsigned char)* q;
		}
		ats_strcat(buff, "*", 1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, "\r\n", 1024);
	}
	else
	{
		ut = gpst2utc(gtime);
		if (ut.sec >= 0.995)
		{
			ut.time++;
			ut.sec = 0.0;
		}
		time2epoch(ut, ep);
		ecef2pos(ecef, pos);
		ecef2enu(pos, &ecef[3], enuv);
		vel = s_norm(enuv, 3);
		if (vel >= 1.0)
		{
			dir = atan2(enuv[0], enuv[1]) * R2D;
			if (dir < 0.0)
			{
				dir += 360.0;
			}
			dirp = dir;
		}
		else
		{
			dir = dirp;
		}
		deg2dms(fabs(pos[0]) * R2D, dms1, 7);
		deg2dms(fabs(pos[1]) * R2D, dms2, 7);

		ats_strcpy(buff, "$GPRMC,", (unsigned int)1024);

		RealToArray((ep[3] * 10000.0) + (ep[4] * 100.0) + ep[5] + 0.001, buf, 6, 2);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",A,", (unsigned int)1024);


		RealToArray((dms1[0] * 100.0) + dms1[1] + (dms1[2] / 60.0), buf, 4, 7);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, (pos[0] >= 0.0) ? ",N," : ",S,", (unsigned int)1024);


		RealToArray((dms2[0] * 100.0) + dms2[1] + (dms2[2] / 60.0), buf, 5, 7);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, (pos[1] >= 0.0) ? ",E," : ",W,", (unsigned int)1024);


		RealToArray(vel / KNOT2M, buf, 2, 2);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray(dir, buf, 2, 2);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		eptemp = (int)ep[0] % 100;
		RealToArray((ep[2] * 10000.0) + (ep[1] * 100.0) + (double)eptemp, buf, 6, 0);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray(amag, buf, 1, 1);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",E,", (unsigned int)1024);
		ats_strcat(buff, ((fixID == 4) || (fixID == 5)) ? "D" : "A", (unsigned int)1024);

		sum = (unsigned char)0;
		for (q = &buff[1]; *q != (char)0; q++)
		{
			sum ^= (unsigned char)* q; /* check-sum */
		}

		ats_strcat(buff, "*", (unsigned int)1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, (unsigned int)1024);

		ats_strcat(buff, "\r\n", (unsigned int)1024);
	}

	ret = (int)strlen(buff);
	return ret;
}

RTK_RAM_CODE extern int print_zda(gtime_t gtime, char* buff)
{
	static double dirp = 0.0;
	gtime_t ut;
	double ep[6], vel = 0.0, dir = 0.0, amag = 0.0;
	char* p = buff, *q = NULL;
	const char* emag = "E";
	unsigned char sum = 0;
	char buf[20] = { 0 };
	int ret = 0;
	int eptemp = 0;

	{
		ut = gpst2utc(gtime);
		if (ut.sec >= 0.995)
		{
			ut.time++;
			ut.sec = 0.0;
		}
		time2epoch(ut, ep);

		ats_strcpy(buff, "$GPZDA,", (unsigned int)1024);

		RealToArray((ep[3] * 10000.0) + (ep[4] * 100.0) + ep[5] + 0.001, buf, 6, 2);
		ats_strcat(buff, buf, (unsigned int)1024);
		ats_strcat(buff, ",", (unsigned int)1024);


		RealToArray(ep[2], buf, 2, 0);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, ",", (unsigned int)1024);

		RealToArray(ep[1], buf, 2, 0);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, ",", (unsigned int)1024);

		RealToArray(ep[0], buf, 4, 0);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, ",", (unsigned int)1024);

		ats_strcat(buff, "00,", (unsigned int)1024);
		ats_strcat(buff, "00,", (unsigned int)1024);
		sum = (unsigned char)0;
		for (q = &buff[1]; *q != (char)0; q++)
		{
			sum ^= (unsigned char)* q; /* check-sum */
		}

		ats_strcat(buff, "*", (unsigned int)1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, (unsigned int)1024);

		ats_strcat(buff, "\r\n", (unsigned int)1024);
	}

	ret = (int)strlen(buff);
	return ret;
}

/* output solution in the form of nmea GSA sentences -------------------------*/
RTK_RAM_CODE extern int print_gsa(unsigned char* buff, int fixID, const vec_t* ssat, const obs_t* rov, double *dop)
{
	double azel[MAXSAT * 2];
	int i = 0, j = 0, isys = 0, nsat = 0, ret = 0, blen = 0;
	int prn[MAXSAT], sys = 0, sum = 0;
	char* p = (char*)buff, *q = NULL, *s = NULL, buf[20] = { 0 };
	unsigned char sys_id[NSYS] = { SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP };
	const unsigned char sys_char[NSYS][3] = { "$GP","$GL","$GA","$BD" };

	if (fixID <= 0)
	{
		ats_strcpy(buff, "$GPGSA,A,1,,,,,,,,,,,,,,,", (unsigned int)1024);
		sum = (unsigned char)0;
		for (q = &buff[1]; *q != (char)0; q++)
		{
			sum ^= (unsigned char)* q;
		}
		ats_strcat(buff, "*", 1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, "\r\n", 1024);
	}
	else
	{
		for (isys = 0; isys < NSYS; isys++)
		{
			/* GPGSA, GLGSA, GAGSA, BDGSA */
			nsat = 0;
			for (i = 0; i < MAXOBS; i++)
			{
				if (nsat >= 12)
				{
					break;
				}
				if ((rov->data[i].flag_used == (unsigned char)0) && (ssat[i].azel[1] > 0.0))
				{
					sys = satsys(ssat[i].sat, &prn[nsat]);
					if ((sys == sys_id[isys]))
					{
						for (j = 0; j < 2; j++)
						{
							azel[j + (nsat * 2)] = ssat[i].azel[j];
						}
						nsat++;
					}
				}
			}
			if (nsat > 0)
			{
				blen = (unsigned int)strlen(buff);
				s = &buff[blen];
				ats_strcpy(&buff[blen], sys_char[isys], (unsigned int)3);
				ats_strcat(buff, "GSA,A,", (unsigned int)1024);
				itoa_user((fixID <= 0) ? 1 : 3, buf, 10);
				ats_strcat(buff, buf, (unsigned int)1024);
				for (i = 0; i < 12; i++)
				{
					if (i < nsat)
					{

						ats_strcat(buff, ",", (unsigned int)1024);
						itoa_user((int)prn[i], buf, 10);
						ats_strcat(buff, buf, (unsigned int)1024);
					}
					else
					{
						ats_strcat(buff, ",", (unsigned int)1024);
					}
				}

				ats_strcat(buff, ",", (unsigned int)1024);

				RealToArray(dop[1], buf, 2, 1);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, ",", (unsigned int)1024);

				RealToArray(dop[2], buf, 2, 1);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, ",", (unsigned int)1024);

				RealToArray(dop[3], buf, 2, 1);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, ",1", (unsigned int)1024);
				sum = (unsigned char)0;
				for (q = &s[1]; *q != (char)0; q++)
				{

					sum ^= (unsigned char)* q; /* check-sum */
				}
				ats_strcat(buff, "*", 1024);

				itoa_user((int)sum, buf, 16);
				ats_strcat(buff, buf, 1024);
				ats_strcat(buff, "\r\n", 1024);
			}
		}
	}
	ret = (int)strlen(buff);
	return ret;
}

/* output solution in the form of nmea GSV sentence --------------------------*/
RTK_RAM_CODE extern int print_gsv(unsigned char* buff, int fixID, const vec_t* ssat, const obs_t* rov)
{
	double az = 0.0, el = 0.0, snr = 0.0;
	int i = 0, j = 0, k = 0, n = 0, nmsg = 0, isys = 0, ret = 0;
	unsigned int blen = 0;
	int sats[MAXSAT], prn = 0, sys = 0, sum = 0;
	char* p = (char*)buff, *q = NULL, *s = NULL, buf[20] = { 0 };
	unsigned char sys_id[NSYS] = { SYS_GPS, SYS_GLO, SYS_GAL, SYS_CMP };
	const unsigned char sys_char[NSYS][3] = { "$GP","$GL","$GA","$BD" };

	if (fixID <= 0)
	{
		ats_strcpy(buff, "$GPGSV,1,1,0,,,,,,,,,,,,,,,,", (unsigned int)1024);
		sum = 0;
		for (q = &buff[1]; *q != (char)0; q++)
		{
			sum ^= * q;
		}
		ats_strcat(buff, "*", 1024);

		itoa_user((int)sum, buf, 16);
		ats_strcat(buff, buf, 1024);
		ats_strcat(buff, "\r\n", 1024);
	}
	else
	{
		for (isys = 0; isys < NSYS; isys++)
		{
			/* GPGSV, GLGSV, GAGSV, BDGSV */
			n = 0;
			for (i = 0; i < (int)rov->n; i++)
			{
				if (n >= 12)
				{
					break;
				}
				sys = satsys(ssat[i].sat, &prn);
				if ((sys == sys_id[isys]))
				{
					if (ssat[i].azel[1] > 0.0)
					{
						sats[n] = i;
						n++;
					}
				}
			}
			if (n <= 0)
			{
				nmsg = 0;
			}
			else
			{
				nmsg = ((n - 1) / 4) + 1;
			}

			k = 0;
			for (i = 0; i < nmsg; i++)
			{
				blen = (unsigned int)strlen(buff);
				s = &buff[blen];
				ats_strcpy(&buff[blen], sys_char[isys], (unsigned int)3);
				ats_strcat(buff, "GSV,", (unsigned int)1024);

				itoa_user(nmsg, buf, 10);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, ",", (unsigned int)1024);

				itoa_user(i + 1, buf, 10);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, ",", (unsigned int)1024);

				itoa_user(n, buf, 10);
				ats_strcat(buff, buf, (unsigned int)1024);

				for (j = 0; j < 4; j++)
				{
					if (k < n)
					{
						az = ssat[sats[k]].azel[0] * R2D;
						if (az < 0.0)
						{
							az += 360.0;
						}
						el = ssat[sats[k]].azel[1] * R2D;
						snr = (double)rov->data[sats[k]].SNR[0] * 0.25;
						ats_strcat(buff, ",", (unsigned int)1024);

						itoa_user((int)prn, buf, 10);
						ats_strcat(buff, buf, (unsigned int)1024);
						ats_strcat(buff, ",", (unsigned int)1024);

						RealToArray((double)el, buf, 2, 0);
						buf[2] = '\0';
						ats_strcat(buff, buf, (unsigned int)1024);
						ats_strcat(buff, ",", (unsigned int)1024);

						RealToArray((double)az, buf, 3, 0);
						buf[3] = '\0';
						ats_strcat(buff, buf, (unsigned int)1024);
						ats_strcat(buff, ",", (unsigned int)1024);

						RealToArray((double)snr, buf, 2, 0);
						buf[2] = '\0';
						ats_strcat(buff, buf, (unsigned int)1024);
						k++;
					}
					else
					{
						blen = (unsigned int)strlen(buff);
						ats_strcpy(&buff[blen], ",,,,", (unsigned int)1024);
						//ats_strcpy(buff, ",,,,", (unsigned int)1024);
						k++;
					}
				}
				//blen = (unsigned int)strlen(buff);
				//ats_strcpy(&buff[blen], ",1", (unsigned int)1024);
				//ats_strcpy(buff, ",1", 1024);  /* L1C/A */
				sum = (unsigned char)0;
				for (q = &s[1]; *q != (char)0; q++)
				{
					sum ^= (unsigned char)* q; /* check-sum */
				}
				ats_strcat(buff, "*", (unsigned int)1024);

				itoa_user((int)sum, buf, 16);
				ats_strcat(buff, buf, (unsigned int)1024);
				ats_strcat(buff, "\r\n", (unsigned int)1024);
			}
		}
	}
	ret = (int)strlen(buff);

	return ret;
}

RTK_RAM_CODE extern int print_pos_gga(gtime_t gtime, const double* pos, unsigned char num_of_sat,
	unsigned char fixID, double *dop, double geo_sep, double age, char* gga)
{
	double ep[6] = { 0 };
	gtime_t ut;
	int ret = 1;

	if (gga != NULL)
	{
		if (fixID != (unsigned char)0)
		{
			ut = gpst2utc(gtime);
			time2epoch(ut, ep);

			(void)print_nmea_gga(ep, pos, (int)num_of_sat, (int)fixID, dop[2], geo_sep, age, gga);
		}
	}
#ifdef RTK_ENABLE
	memset(rmc_buff, 0, sizeof(unsigned char)*NMEA_S_LEN);
	memset(gsa_buff, 0, sizeof(unsigned char)*NMEA_M_LEN);
	memset(gsv_buff, 0, sizeof(unsigned char)*NMEA_M_LEN);
	memset(zda_buff, 0, sizeof(unsigned char)*NMEA_S_LEN);
	print_rmc(gtime, pos, fixID, rmc_buff);
	print_gsv((unsigned char*)gsv_buff, fixID, gRov.vec, &gRov.obs);
	print_gsa((unsigned char*)gsa_buff, fixID, gRov.vec, &gRov.obs, dop);
	print_zda(gtime, zda_buff);
	nema_update_flag = 1;
#endif
	return ret;
}
