#include "gnss_integrity.h"
#include "model.h"
#include "rtcm.h"

#define POS_SF0  9.0
#define POS_SIG0 0.5
#define HOR_POS_AL 0.50
#define VER_POS_AL 0.50
#define LAT_POS_AL 0.14
#define LON_POS_AL 0.48
#define HOR_VEL_AL 0.30
#define VER_VEL_AL 0.30
#define PITCH_AL   1.50
#define ROLL_AL    1.50
#define HEADING_AL 1.50


double hor_pos_alert_limit_determination()
{

}

double ver_pos_alert_limit_determination()
{

}

double lateral_pos_alert_limit_determination()
{

}

double longitu_pos_alert_limit_determination()
{


}

double lateral_vel_alert_limit_determination()
{

}
double longitu_vel_alert_limit_determination()
{

}


double hor_vel_alert_limit_determination()
{

}
double ver_vel_alert_limit_determination()
{

}

double hor_pos_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double hor_pos_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    blh2C_en(blh, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)]  = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
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

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covNED[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covNED[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covNED[SMI(2, 2)];

    hor_pos_pl = sqrt(scaled_posvar[0] + scaled_posvar[1]);

    return hor_pos_pl;
}


double ver_pos_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double ver_pos_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);


    blh2C_en(blh, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
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

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covNED[SMI(2, 2)];

    ver_pos_pl = sqrt(scaled_posvar[2]);

    return ver_pos_pl;
}

double lateral_pos_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double lateral_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double dLLD[3] = { 0 }, covLLD[SMD(3)] = { 0 }, covvLLD[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 }, vlld[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    
    ned2_Clateral_lon(rcv->dynamic_stat.heading_vel, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)]  = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
        }
    }

    xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);
    xyz2ned(C_en, vxyz, covvXYZ, vned, covvNED);

    ned2lld(C_en, dNED, covNED, dLLD, covLLD);
    ned2lld(C_en, vned, covvNED, vlld, covvLLD);

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(2, 2)];

    lateral_pl = sqrt(scaled_posvar[1]);

    return lateral_pl;
}
double longitu_pos_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double longi_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double dLLD[3] = { 0 }, covLLD[SMD(3)] = { 0 }, covvLLD[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 }, vlld[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);

    ned2_Clateral_lon(rcv->dynamic_stat.heading_vel, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)]  = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
        }
    }

    xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);
    xyz2ned(C_en, vxyz, covvXYZ, vned, covvNED);

    ned2lld(C_en, dNED, covNED, dLLD, covLLD);
    ned2lld(C_en, vned, covvNED, vlld, covvLLD);

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covLLD[SMI(2, 2)];

    longi_pl = sqrt(scaled_posvar[0]);

    return longi_pl;
}

double hor_vel_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double hor_vel_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    blh2C_en(blh, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
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

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(2, 2)];

    hor_vel_pl = sqrt(SQR(scaled_posvar[0]) + SQR(scaled_posvar[1]));

    return hor_vel_pl;
}

double ver_vel_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double ver_vel_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    blh2C_en(blh, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
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

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covvNED[SMI(2, 2)];

    ver_vel_pl = sqrt(scaled_posvar[2]);

    return ver_vel_pl;
}


double lateral_vel_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double lateral_vel_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double dLLD[3] = { 0 }, covLLD[SMD(3)] = { 0 }, covvLLD[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 }, vlld[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    ned2_Clateral_lon(rcv->dynamic_stat.heading_vel, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
        }
    }

    xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);
    xyz2ned(C_en, vxyz, covvXYZ, vned, covvNED);

    ned2lld(C_en, dNED, covNED, dLLD, covLLD);
    ned2lld(C_en, vned, covvNED, vlld, covvLLD);

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(2, 2)];

    lateral_vel_pl = scaled_posvar[1];

    return lateral_vel_pl;
}

double longitu_vel_protection_level_determination(rcv_rtk_t *rcv)
{
    int i, j;
    double longi_vel_pl = 0.0;
    double dXYZ[3] = { 0 };
    double covXYZ[SMD(3)] = { 0 }, covvXYZ[SMD(3)] = { 0 };
    double dNED[3] = { 0 }, covNED[SMD(3)] = { 0 }, covvNED[SMD(3)] = { 0 };
    double dLLD[3] = { 0 }, covLLD[SMD(3)] = { 0 }, covvLLD[SMD(3)] = { 0 };
    double C_en[3][3] = { 0 };
    double blh[3] = { 0 };
    double vxyz[3] = { 0 }, vned[3] = { 0 }, vlld[3] = { 0 };
    double scaled_posvar[3] = { 0 };
    double pos_dsf = 0.0;
    ecef2pos(rcv->x, blh);
    ned2_Clateral_lon(rcv->dynamic_stat.heading_vel, C_en);

    /* position in NED */
    for (i = 0; i < 3; ++i)
    {
        vxyz[i] = rcv->x[i + 3];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            covvXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
        }
    }

    xyz2ned(C_en, dXYZ, covXYZ, dNED, covNED);
    xyz2ned(C_en, vxyz, covvXYZ, vned, covvNED);

    ned2lld(C_en, dNED, covNED, dLLD, covLLD);
    ned2lld(C_en, vned, covvNED, vlld, covvLLD);

    pos_dsf = (rcv->scales.medres[0] / POS_SIG0);
    pos_dsf = (pos_dsf < 1.0) ? 0.0 : pos_dsf;

    scaled_posvar[0] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(0, 0)];
    scaled_posvar[1] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(1, 1)];
    scaled_posvar[2] = (POS_SF0 + SQR(pos_dsf))*covvLLD[SMI(2, 2)];

    longi_vel_pl = sqrt(scaled_posvar[0]);

    return longi_vel_pl;
}

void rtk_integrity(rcv_rtk_t *rcv)
{
    double hppl, vppl,latppl,lonppl, hvpl, vvpl, latvpl, lonvpl;
    hppl   = hor_pos_protection_level_determination(rcv);
    vppl   = ver_pos_protection_level_determination(rcv);
    hvpl   = hor_vel_protection_level_determination(rcv);
    vvpl   = ver_vel_protection_level_determination(rcv);
    latppl = lateral_pos_protection_level_determination(rcv);
    lonppl = longitu_pos_protection_level_determination(rcv);
    latvpl = lateral_vel_protection_level_determination(rcv);
    lonvpl = longitu_vel_protection_level_determination(rcv);

    rcv->pos_integrity.integrity_type = (horizontal|| vertical);

    rcv->pos_integrity.hor_pl = hppl;
    rcv->pos_integrity.ver_pl = vppl;
    rcv->vel_integrity.hor_pl = hvpl;
    rcv->vel_integrity.ver_pl = vvpl;

    if (hppl < HOR_POS_AL && vppl < VER_POS_AL)
    {
        rcv->pos_integrity.integrity_status = Available;
    }
    else
    {
        rcv->pos_integrity.integrity_status = Unavailable;
    }

    rcv->vel_integrity.integrity_type = (horizontal || vertical);
    if (hvpl < HOR_VEL_AL && vvpl < VER_VEL_AL)
    {
        rcv->vel_integrity.integrity_status = Available;
    }
    else
    {
        rcv->vel_integrity.integrity_status = Unavailable;
    }
}