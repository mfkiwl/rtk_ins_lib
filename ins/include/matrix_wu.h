#ifndef		__MATRIX_WU_H__
#define		__MATRIX_WU_H__

#include <string.h>

#define SMD(i) ((i)*((i)+1)/2)
#define SMI(i, j)	((i) > (j) ? (SMD(i) + (j)) : (SMD(j) + (i)))
#define NX_INS 15          //15 
#define IsOdo 0

//PHI*P
void PHI_P(const int16_t n,const float *PHI,const float *P, float *PHIP)
{
	unsigned char i, j;
	const float *phi;
	float *c;
	memset(PHIP, 0, sizeof(float)*n*n);

	phi = PHI;
	c = PHIP;
	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[0] * P[i] + phi[1] * P[i + n] + phi[2] * P[i + 2 * n] + phi[3 + j] * P[i + (3 + j)*n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[0] * P[i] + phi[1] * P[i + n] + phi[2] * P[i + 2 * n] + phi[3] * P[i + 3 * n] + phi[4] * P[i + 4 * n] + phi[5] * P[i + 5 * n]
				+ phi[6] * P[i + 6 * n] + phi[7] * P[i + 7 * n] + phi[8] * P[i + 8 * n]
				+ phi[12] * P[i + 12 * n] + phi[13] * P[i + 13 * n] + phi[14] * P[i + 14 * n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[6] * P[i + 6 * n] + phi[7] * P[i + 7 * n] + phi[8] * P[i + 8 * n]
				+ phi[9] * P[i + 9 * n] + phi[10] * P[i + 10 * n] + phi[11] * P[i + 11 * n];
		}
		phi += n;
		c += n;
	}

	for (j = 0; j < 6; ++j)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = phi[9 + j] * P[i + (9 + j)*n];
		}
		phi += n;
		c += n;
	}

	if (16 == n)
	{
		for (i = 0; i < n; ++i)
		{
			c[i] = P[i + (n-1) * n];
		}
	}
}

//PHI*Q
void PHI_Q(const int16_t n,const float *PHI,const float *Q, float *PHIQ)
{
	unsigned char j;
	const float *phi;
	float *c;

	memset(PHIQ, 0, sizeof(float)*n*n);
	phi = PHI;
	c = PHIQ;
	for (j = 0; j < 3; ++j)
	{
		c[0] = phi[0] * Q[0];
		c[1] = phi[1] * Q[1];
		c[2] = phi[2] * Q[2];
		c[3 + j] = phi[3 + j] * Q[3 + j];
		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		c[0] = phi[0] * Q[0];
		c[1] = phi[1] * Q[1];
		c[2] = phi[2] * Q[2];

		c[3] = phi[3] * Q[3];
		c[4] = phi[4] * Q[4];
		c[5] = phi[5] * Q[5];

		c[6] = phi[6] * Q[6];
		c[7] = phi[7] * Q[7];
		c[8] = phi[8] * Q[8];

		c[12] = phi[12] * Q[12];
		c[13] = phi[13] * Q[12];
		c[14] = phi[4] * Q[14];

		phi += n;
		c += n;
	}

	for (j = 0; j < 3; ++j)
	{
		c[6] = phi[6] * Q[6];
		c[7] = phi[7] * Q[7];
		c[8] = phi[8] * Q[8];

		c[9] = phi[9] * Q[9];
		c[10] = phi[10] * Q[10];
		c[11] = phi[11] * Q[11];

		phi += n;
		c += n;
	}

	for (j = 0; j < 6; ++j)
	{
		c[9 + j] = phi[j + 9] * Q[j + 9];
		phi += n;
		c += n;
	}

	if (16 == n)
	{
		c[15] = Q[15];
	}
}

//PHIP*PHI_T
void PHIP_PHIT(const int16_t n, const float *PHIP, const float *PHI, float *PHIPPHIT)
{
	unsigned char i, j;
	const float *phi, *phip;
	float *c;
	phi = PHI;
	c = PHIPPHIT;
	phip = PHIP;

	for (i = 0; i < n; ++i)
	{
		phi = PHI;
		for (j = 0; j < 3; ++j)
		{
			c[j] = phip[0] * phi[0] + phip[1] * phi[1] + phip[2] * phi[2] + phip[3 + j] * phi[3 + j];
			phi += n;
		}
		for (j = 3; j < 6; ++j)
		{
			c[j] = phip[0] * phi[0] + phip[1] * phi[1] + phip[2] * phi[2]
				+ phip[3] * phi[3] + phip[4] * phi[4] + phip[5] * phi[5]
				+ phip[6] * phi[6] + phip[7] * phi[7] + phip[8] * phi[8]
				+ phip[12] * phi[12] + phip[13] * phi[13] + phip[14] * phi[14];
			phi += n;
		}
		for (j = 6; j < 9; ++j)
		{
			c[j] = phip[9] * phi[9] + phip[10] * phi[10] + phip[11] * phi[11]
				+ phip[6] * phi[6] + phip[7] * phi[7] + phip[8] * phi[8];
			phi += n;
		}

		for (j = 9; j < 15; ++j)
		{
			c[j] = phip[j] * phi[j];
			phi += n;
		}

		if (16 == n)
		{
			c[15] = phip[15];
		}

		phip += n;
		c += n;
	}
}

//0.5*(PHI*Q+Q*PHIT)*dt
void PHIQ_QPHIT(const int16_t n,float *PHIQ, float dt, float *c)
{
	unsigned char i, j;
	dt = dt * 0.5;
	for (i = 0; i < n; ++i)
	{
		for (j = 0; j < n; ++j)
		{
			c[i + j * n] = dt * (PHIQ[i + j * n] + PHIQ[j + i * n]);
		}
	}
}

#endif