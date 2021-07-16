#include "rtk_debug.h"
#include "rtklib_core.h"

extern void print_csv(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, 
	int *iref, int *irov, int nsat, int nsd, rcv_rtk_t *rcv, int type, double *dop, char *buff)
{
    int i, j;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 };
    ecef2pos(obs_rov->pos, blh);
    blh2C_en(blh, C_en);
    int week = 0;
    double time = time2gpst(obs_rov->time, &week);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        if (type == 4)
        {
            dXYZ[i] = rcv->x_fixed[i] - obs_ref->pos[i];
        }
        else
        {
            dXYZ[i] = rcv->x[i] - obs_ref->pos[i];
        }

        vxyz[i] = obs_rov->pos[i+3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)]  = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i+3, j+3)];
        }
    }
    xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);
    xyz2ned(C_en, vxyz, covvXYZ, vned, covvNED);

    for (i = 0; i < 3; i++)
    {
        if (covNED[SMI(i, i)] < 0.0)
        {
            covNED[SMI(i, i)] = fabs(covNED[SMI(i, i)]) + 1.0e-6;
        }
    }
    char EAC[1024] = { 0 };
    int loc = 0;
    int prn = 0;
    for (i = 0; i < nsd; ++i)
    {
        if (i > 0) loc += sprintf(EAC + loc, ",");
        loc += sprintf(EAC + loc, "%c%02i,%4.1f,%5.1f,%2i", satid(vec_rov[irov[i]].sat, &prn),
            prn, vec_rov[irov[i]].azel[1] * 180.0 / PI, vec_rov[irov[i]].azel[0] * 180.0 / PI,
            obs_rov->data[irov[i]].SNR[0] / 4);
    }
    loc = 0;
    loc += sprintf(buff + loc, "%04i,%10i,%14.10f,%14.10f,%10.4f,%8.4f,%8.4f,%8.4f,%7.3f,%7.3f,%2i,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%1i,%s\n"
        , week
        , (int)(time * 1000)
        , blh[0] * 180 / PI, blh[1] * 180 / PI, blh[2]
        , vned[0], vned[1], vned[2]
        , dop[2], dop[1]
        , nsat
        , sqrt(covNED[SMI(0, 0)]), sqrt(covNED[SMI(1, 1)]), sqrt(covNED[SMI(2, 2)])
        , sqrt(covvNED[SMI(0, 0)]), sqrt(covvNED[SMI(1, 1)]), sqrt(covvNED[SMI(2, 2)])
        , type
        , EAC
    );

    return;
}

extern void print_ins_input_csv(obs_t *obs_ref, obs_t *obs_rov, rcv_rtk_t *rcv, int type, char *buff)
{
	int i, j;
	double dXYZ[3] = { 0 };
	double covXYZ[SMD(3)] = { 0 };
	double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 };
	double C_en[3][3] = { 0 };
	double blh[3] = { 0 };
	ecef2pos(obs_rov->pos, blh);
	blh2C_en(blh, C_en);
	int week = 0;
	double time = time2gpst(obs_rov->time, &week);

	/* position in NED */
	for (i = 0; i < 3; ++i)
	{
		if (type == 4)
		{
			dXYZ[i] = rcv->x_fixed[i] - obs_ref->pos[i];
		}
		else
		{
			dXYZ[i] = rcv->x[i] - obs_ref->pos[i];
		}

		for (j = i; j < 3; ++j)
		{
			covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
		}
	}
	xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);

	for (i = 0; i < 3; i++)
	{
		if (covNED[SMI(i, i)] < 0.0)
		{
			covNED[SMI(i, i)] = fabs(covNED[SMI(i, i)]) + 1.0e-6;
		}
	}

	double vXYZ[3] = { 0.0 }, covVNED[SMD(3)] = { 0 };
	for (i = 0; i < 3; i++) {
		if (rcv->fixType > 0) {
			dXYZ[i] = rcv->x[i + 3];
		}
	}
	xyz2ned(C_en, dXYZ, covXYZ, dNED, covVNED);
	float heading = atan2(dNED[1], dNED[0]);
	
	int loc = 0;
	
	loc += sprintf(buff + loc, "%04i,%10.3f,%14.10f,%14.10f,%10.4f,%10.4f,%10.4f,%10.4f,%1i,%10.4f,%10.4f,%10.4f,%7.2f\n"
		, week
		, time
		, blh[0] * 180 / PI, blh[1] * 180 / PI, blh[2]
		, sqrt(covNED[SMI(1, 1)]), sqrt(covNED[SMI(0, 0)]), sqrt(covNED[SMI(2, 2)])
		, type
		, dNED[0], dNED[1], dNED[2]
		, heading*R2D
	);

	return;
}

extern void print_obs_info(obs_t *rov, obs_t *ref)
{
    unsigned int i, j;
    int sys, prn;
    double ep[6] = { 0 };

    time2epoch(rov->time, ep);
    printf("> |rov| %4.0f %02.0f %02.0f %02.0f %02.0f %09.6f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
//#if 0
    for (i = 0; i < rov->n; i++)
    {
        sys = satid(rov->data[i].sat, &prn);
        printf("%c%02d", sys, prn);
        for (j = 0; j < NFREQ; j++)
        {
            if (rov->data[i].code[j] <= 0)
                continue;

             printf("%14.3lf %02d %10.4lf %14.3lf %5.2f", rov->data[i].P[j], rov->data[i].code[j],
             	rov->data[i].D[j], rov->data[i].L[j], (double)(rov->data[i].SNR[j] / 4.0));
        }
         printf("\n");
    }
//#endif

    time2epoch(ref->time, ep);
    printf("  |ref| %4.0f %02.0f %02.0f %02.0f %02.0f %09.6f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
//#if 0
    for (i = 0; i < (int)ref->n; i++)
    {
        sys = satid(ref->data[i].sat, &prn);
        if (ref->data[i].code[0] != 25)
            continue;

        printf("%c%02d", sys, prn);
        for (j = 0; j < NFREQ; j++)
        {

            printf("%14.3lf %02d %10.4lf %14.3lf %5.2f", ref->data[i].P[j], ref->data[i].code[j],
            	ref->data[i].D[j], ref->data[i].L[j], (double)(ref->data[i].SNR[j] / 4.0));
        }
        printf("\n");
    }
//#endif
}

extern void print_rtk_info(obs_t *rov, rcv_rtk_t *rcv)
{
    int i, s, f;
    int ns = rcv->ns;
    int np = rcv->np;

    printf("posvel,%10.2f,%10.3f,%10.3f,%10.3f,%7.3f,%7.3f,%7.3f,%10.3f,%10.3f,%10.3f,%7.3f,%7.3f,%7.3f\n"
        , fmod(rcv->time,86400.0)-18.0
        , rov->pos[0], rov->pos[1], rov->pos[2]
        , sqrt(rcv->P[SMI(0, 0)]), sqrt(rcv->P[SMI(1, 1)]), sqrt(rcv->P[SMI(2, 2)])
        , rov->pos[3], rov->pos[4], rov->pos[5]
        , sqrt(rcv->P[SMI(3, 3)]), sqrt(rcv->P[SMI(4, 4)]), sqrt(rcv->P[SMI(5, 5)])
    );
#if 0
    printf("clk ,%10.2f,", fmod(rcv->time, 86400.0) - 18.0);
    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            idx = s * NFREQ + f;
            if (rcv->x[10 + idx] == 0.0)  continue;
            if (s == 0)
                printf("G%d,%10.3f,%7.3f,", f + 1, rcv->x[10 + idx], sqrt(rcv->P[SMI(10 + idx, 10 + idx)]));
            else if (s == 1)
                printf("E%d,%10.3f,%7.3f,", f + 1, rcv->x[10 + idx], sqrt(rcv->P[SMI(10 + idx, 10 + idx)]));
            else if (s == 2)
                printf("C%d,%10.3f,%7.3f,", f + 1, rcv->x[10 + idx], sqrt(rcv->P[SMI(10 + idx, 10 + idx)]));
            else if (s == 3)
                printf("R%d,%10.3f,%7.3f,", f + 1, rcv->x[10 + idx], sqrt(rcv->P[SMI(10 + idx, 10 + idx)]));

        }
    }
    printf("\n");

    printf("ramb,%10.2f,", fmod(rcv->time, 86400.0) - 18.0);
    for (s = 0; s < NSYS; s++)
    {
        for (f = 0; f < NFREQ; f++)
        {
            idx = s * NFREQ + f;
            if (rcv->x[18 + idx] == 0.0)  continue;
            if (s == 0)
                printf("G%d,%10.3f,%7.3f,", f + 1, rcv->x[18 + idx], sqrt(rcv->P[SMI(18 + idx, 18 + idx)]));
            else if (s == 1)
                printf("E%d,%10.3f,%7.3f,", f + 1, rcv->x[18 + idx], sqrt(rcv->P[SMI(18 + idx, 18 + idx)]));
            else if (s == 2)
                printf("C%d,%10.3f,%7.3f,", f + 1, rcv->x[18 + idx], sqrt(rcv->P[SMI(18 + idx, 18 + idx)]));
            else if (s == 3)
                printf("R%d,%10.3f,%7.3f,", f + 1, rcv->x[18 + idx], sqrt(rcv->P[SMI(18 + idx, 18 + idx)]));
        }
    }
    printf("\n");

    for (i = 0; i < (int)rcv->ns; i++)
    {
        int prn1 = 0, prn2 = 0;
        sys = satsys(rcv->tag[i].s1, &prn1);
        sys = satsys(rcv->tag[i].s2, &prn2);
        printf("amb,%10.2f,%c%2d,%c%2d,%2d,%10.3f,%7.3f\n", fmod(rcv->time, 86400.0) - 18.0, sys2char(sys), prn1
            , sys2char(sys), prn2, rcv->tag[i].f + 1, rcv->x[np + i]
            , sqrt(rcv->P[SMI(np + i, np + i)]));
    }
    printf("\n");
#endif
}

/* print raw measuerments */
extern void print_raw_meas(rcv_rtk_t *rcv, obs_t *rov, obs_t *ref, vec_t* vec_rov,
    vec_t* vec_ref, FILE* fOBS)
{
    int i, j, prn = 0, week;
    double time;
    if (fOBS)
    {
        time = time2gpst(ref->time, &week);
        if (time > rcv->lastTimeRef)
        {
            fprintf(fOBS, "$GNSS_EPOCH,1,0,0.0,%10.3f,%3i,%14.4f,%14.4f,%14.4f\n",
                time, ref->n, ref->pos[0], ref->pos[1], ref->pos[2]);
            for (i = 0; i < (int)ref->n; ++i)
            {
                //fprintf(fOBS, "%3i,%3i,%3i,%18.4f,%18.4f,%18.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%3i"
                //    , satidx(ref->data[i].sat, &prn), prn, ref->data[i].sat
                //    , vec_ref[i].rs[0], vec_ref[i].rs[1], vec_ref[i].rs[2]
                //    , vec_ref[i].rs[3], vec_ref[i].rs[4], vec_ref[i].rs[5]
                //    , vec_ref[i].dts[0] * CLIGHT, vec_ref[i].dts[1] * CLIGHT
                //    , NFREQ
                //);
                for (j = 0; j < NFREQ; ++j)
                {
                    fprintf(fOBS, "%3d,%3d,%14.4f,%14.4f,%14.4f,%3d,%20.16f"
                        , ref->data[i].sat, ref->data[i].code[j], ref->data[i].P[j]
                        , ref->data[i].L[j], ref->data[i].D[j], ref->data[i].SNR[j]
                        , satwavelen(ref->data[i].sat, ref->data[i].code[j])
                    );
                }
                fprintf(fOBS, "\n");
            }
            rcv->lastTimeRef = time;
        }

        time = time2gpst(rov->time, &week);
        fprintf(fOBS, "$GNSS_EPOCH,0,0,0.0,%10.3f,%3i,%14.4f,%14.4f,%14.4f\n",
            time, rov->n, rov->pos[0], rov->pos[1], rov->pos[2]);

        for (i = 0; i < (int)rov->n; ++i)
        {
            //fprintf(fOBS, "%3i,%3i,%3i,%18.4f,%18.4f,%18.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%3i"
            //    , satidx(rov->data[i].sat, &prn), prn, rov->data[i].sat
            //    , vec_rov[i].rs[0], vec_rov[i].rs[1], vec_rov[i].rs[2]
            //    , vec_rov[i].rs[3], vec_rov[i].rs[4], vec_rov[i].rs[5]
            //    , vec_rov[i].dts[0] * CLIGHT, vec_rov[i].dts[1] * CLIGHT
            //    , NFREQ
            //);
            for (j = 0; j < NFREQ; ++j)
            {
                fprintf(fOBS, "%3d,%3d,%3d,%14.4f,%3d,%14.4f,%3d,%14.4f,%20.16f"
                    , rov->data[i].sat, rov->data[i].code[j], rov->data[i].SNR[j] / 4
                    , rov->data[i].P[j], rov->data[i].qualP[j], rov->data[i].L[j]
                    , rov->data[i].qualL[j], rov->data[i].D[j]
                    , satwavelen(rov->data[i].sat, rov->data[i].code[j])
                );
            }
            fprintf(fOBS, "\n");
        }
    }
}

///* print cycle slip */
extern void print_cycslip(rcv_rtk_t *rcv, obs_t *rov, double time, int week, FILE* fLOG)
{
    if (fLOG)
    {
        int sat, prn, sys, f;
        double nslip = 0.0, nobs = 0.0;
        int day = (int)floor((time - week * 7 * 24 * 3600.0) / 86400.0);
        double sec = time - week * 7 * 24 * 3600.0 - day * 86400.0;
        for (int i = 0; i < (int)rov->n; ++i)
        {
            sat = rov->data[i].sat;
            sys = satsys(sat, &prn);
            int sat_id = find_sat_index(sat, rcv->slip);

            for (f = 0; f < NFREQ; ++f)
            {
                if (rov->data[i].P[f] == 0.0) continue;
                if (rcv->slip[sat_id].dph[f] > 0.3)
                {
                    nslip++;
                }
                nobs++;
                fprintf(fLOG, "%c%2d,%d,%3d,%14.3f,%14.3f,%14.3f,%2i\n"
                    , sys2char(sys), prn, f, rov->data[i].SNR[f], rcv->slip[sat_id].dph[f]
                    , rcv->slip[sat_id].L[f], rcv->slip[sat_id].D[f], rcv->slip[sat_id].slip[f]);
            }
        }
        fprintf(fLOG, "CSLIP:%8.2f,%2d,%2d,%6.2f\n"
            , sec, (int)nslip, (int)nobs, nslip / nobs * 100.0);
    }
}