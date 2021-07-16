#include <math.h>
#include "cmatrix.h"

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif



	uint8_t MatrixAdd(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
		}
		return 1;
	}
	uint8_t MatrixSub(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) - *(matrix_b + i);
		}
		return 1;

	}
	uint8_t MatrixMutiply(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
	{
		double sum = 0;
		double median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}
	uint8_t MatrixTranspose(const double *matrix_a, const int matrix_a_row, const int matrix_a_column, double *matrix_result)
	{
		for (int i = 0; i < matrix_a_column; i++)
		{
			for (int j = 0; j < matrix_a_row; j++)
			{
				matrix_result[matrix_a_row*i + j] = matrix_a[matrix_a_column*j + i];
			}
		}
		return 1;

	}


	uint8_t MatrixAddfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
		}
		return 1;
	}
	uint8_t MatrixSubfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
		{
			*(matrix_result + i) = *(matrix_a + i) - *(matrix_b + i);
		}
		return 1;

	}
	uint8_t MatrixMutiplyfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, float *matrix_result)
	{
		float sum = 0;
		float median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}
	uint8_t MatrixTransposefloat(const float *matrix_a, const int matrix_a_row, const int matrix_a_column, float *matrix_result)
	{
		for (int i = 0; i < matrix_a_column; i++)
		{
			for (int j = 0; j < matrix_a_row; j++)
			{
				matrix_result[matrix_a_row*i + j] = matrix_a[matrix_a_column*j + i];
			}
		}
		return 1;

	}
	uint8_t MatrixMutiplyfloatd(const float *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
	{
		double sum = 0;
		double median = 0;
		for (int i = 0; i < matrix_a_row; i++)
		{
			for (int k = 0; k < matrix_b_column; k++)
			{
				for (int j = 0; j < matrix_a_column; j++)
				{
					median =(double)(matrix_a[matrix_a_column*i + j]) * matrix_b[matrix_b_column*j + k];
					sum = sum + median;
				}
				matrix_result[matrix_b_column*i + k] = sum;
				sum = 0;
			}
		}
		return 1;

	}

	/*************************************************
	Function: MatrixCholosky
	Description:����Cholosky�ֽ�
	Input:
	Output:lower_tri_matrix
	Return:
	Others:
	*************************************************/
	uint8_t MatrixCholosky(const double *matrix, const int matrix_row, double *lower_tri_matrix)
	{
		int i, j, k, l, u;
		double sum = 0.0;
		if (matrix[0] <= 0.0)
		{
#ifdef DEBUG
			printf(" The matrix can't be decomposed with Cholosky decomposition method.\n");
#endif
		}
		lower_tri_matrix[0] = sqrt(matrix[0]);

		for (i = 1; i < matrix_row; i++)
			lower_tri_matrix[i*matrix_row] = matrix[i*matrix_row] / lower_tri_matrix[0];


		for (k = 1; k < matrix_row; k++)
		{
			l = k * matrix_row + k;
			for (j = 0; j < k; j++)
				sum = sum + lower_tri_matrix[k*matrix_row + j] * lower_tri_matrix[k*matrix_row + j];

			if ((lower_tri_matrix[l] - sum) <= 0.0)
			{
#ifdef DEBUG
				printf(" The matrix can't be decomposed with Cholosky decomposition method.\n");
#endif
			}
			lower_tri_matrix[l] = sqrt(matrix[l] - sum);
			sum = 0.0;
			for (i = k + 1; i < matrix_row; i++)
			{
				u = i * matrix_row + k;
				for (j = 0; j < k; j++)
					sum = sum + lower_tri_matrix[i*matrix_row + j] * lower_tri_matrix[k*matrix_row + j];
				lower_tri_matrix[u] = (matrix[u] - sum) / lower_tri_matrix[l];
				sum = 0.0;
			}
		}

		for (i = 0; i < matrix_row - 1; i++)
		{
			for (j = i + 1; j < matrix_row; j++)
				lower_tri_matrix[i*matrix_row + j] = 0.0;
		}
		return 1;
	}
	/*************************************************
	Function: MatrixInverse
	Description:L��������
	Input:
	Output:matrix_result
	Return:
	Others:
	*************************************************/
//	uint8_t MatrixInverse(const double *lower_tri_matrix, const int matrix_row, double *matrix_result)
//	{
//		int ret = 0;
//		int i, j, k, l, u;
//		double sum;
//		for (i = 0; i < matrix_row; i++)
//		{
//			l = i * matrix_row + i;
//			if (lower_tri_matrix[l] <= 0)
//			{
//#ifdef DEBUG
//				printf(" The matrix does not exist inverse matrix.\n");
//				return -1;
//#endif
//			}
//			matrix_result[l] = 1.0 / lower_tri_matrix[l];
//		}
//
//		for (i = 1; i < matrix_row; i++)
//		{
//			sum = 0.0;
//			for (j = 0; j < i; j++)
//			{
//				for (k = j; k < i; k++)
//				{
//					l = i * matrix_row + k;
//					u = k * matrix_row + j;
//					sum = sum + lower_tri_matrix[l] * matrix_result[u];
//				}
//				matrix_result[i*matrix_row + j] = -matrix_result[i*matrix_row + i] * sum;
//				sum = 0.0;
//			}
//		}
//
//		for (i = 0; i < matrix_row - 1; i++)
//		{
//			for (j = i + 1; j < matrix_row; j++)
//			{
//				l = i * matrix_row + j;
//				matrix_result[l] = 0.0;
//			}
//		}
//		return 1;
//	}
	static unsigned short l, u;
	static char is[25], js[25];
	uint8_t MatrixInverse(uint8_t n, double* a)
	{
		int i, j, k;
		unsigned short v;
		double d, p;
		for (k = 0; k <= n - 1; k++)
		{
			d = 0.0;
			for (i = k; i <= n - 1; i++)
				for (j = k; j <= n - 1; j++)
				{
					l = i * n + j; p = fabs(a[l]);
					if (p > d) { d = p; is[k] = i; js[k] = j; }
				}
			if (is[k] != k)
				for (j = 0; j <= n - 1; j++)
				{
					u = k * n + j; v = is[k] * n + j;
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			if (js[k] != k)
				for (i = 0; i <= n - 1; i++)
				{
					u = i * n + k; v = i * n + js[k];
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			l = k * n + k;
			a[l] = 1.0 / a[l];
			for (j = 0; j <= n - 1; j++)
				if (j != k)
				{
					u = k * n + j; a[u] = a[u] * a[l];
				}
			for (i = 0; i <= n - 1; i++)
				if (i != k)
					for (j = 0; j <= n - 1; j++)
						if (j != k)
						{
							u = i * n + j;
							a[u] = a[u] - a[i*n + k] * a[k*n + j];
						}
			for (i = 0; i <= n - 1; i++)
				if (i != k)
				{
					u = i * n + k; a[u] = -a[u] * a[l];
				}
		}
		for (k = n - 1; k >= 0; k--)
		{
			if (js[k] != k)
				for (j = 0; j <= n - 1; j++)
				{
					u = k * n + j; v = js[k] * n + j;
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
			if (is[k] != k)
				for (i = 0; i <= n - 1; i++)
				{
					u = i * n + k; v = i * n + is[k];
					p = a[u]; a[u] = a[v]; a[v] = p;
				}
		}
		return 1;

	}
	// Cross product of two vectors(c = a x b)
	uint8_t CrossProduct(double a[3], double b[3], double c[3])
	{
		c[0] = a[1] * b[2] - b[1] * a[2];
		c[1] = b[0] * a[2] - a[0] * b[2];
		c[2] = a[0] * b[1] - b[0] * a[1];
		return 1;

	}
	uint8_t GetSkewSymmetricMatrixOfVector(const double  mvector[3], double *M)
	{
		M[0] = 0; M[1] = -mvector[2]; M[2] = mvector[1];
		M[3] = mvector[2]; M[4] = 0; M[5] = -mvector[0];
		M[6] = -mvector[1]; M[7] = mvector[0]; M[8] = 0;
		return 1;

	}
	uint8_t quatprod(const double p[4], const double q[4], double r[4])
	{
		r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
		r[1] = p[1] * q[0] + p[0] * q[1] - p[3] * q[2] + p[2] * q[3];
		r[2] = p[2] * q[0] + p[3] * q[1] + p[0] * q[2] - p[1] * q[3];
		r[3] = p[3] * q[0] - p[2] * q[1] + p[1] * q[2] + p[0] * q[3];
		return 1;

	}
	uint8_t quat2rvec(double q[4], double rot_vec[3])
	{
		int i;
		if (fabs(q[0]) > 1.0E-15)
		{
			double atan_05zata = atan2(sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), q[0]);
			double atan2_05zata = atan_05zata * atan_05zata;
			double atan4_05zata = atan2_05zata * atan2_05zata;
			double atan6_05zata = atan4_05zata * atan2_05zata;
			double f = 0.5*(1 - atan2_05zata / 6 + atan4_05zata / 120 - atan6_05zata / 5040);
			for (i = 0; i < 3; ++i)
			{
				rot_vec[i] = q[i + 1] / f;
			}
		}
		else
		{
			for (i = 0; i < 3; ++i)
			{
				rot_vec[i] = q[i + 1] * PI;
			}
		}
		return 1;

	}

	uint8_t rvec2quat(double rot_vec[3], double q[4])
	{
		double mag2, c, s;
		mag2 = rot_vec[0] * rot_vec[0] + rot_vec[1] * rot_vec[1] + rot_vec[2] * rot_vec[2];

		if (mag2 < PI*PI)
		{
			mag2 = 0.25*mag2;

			c = 1.0 - mag2 / 2.0*(1.0 - mag2 / 12.0*(1.0 - mag2 / 30.0));
			s = 1.0 - mag2 / 6.0*(1.0 - mag2 / 20.0*(1.0 - mag2 / 42.0));

			if (c < 0)
			{
				q[0] = -c;
				q[1] = -0.5*s*rot_vec[0];
				q[2] = -0.5*s*rot_vec[1];
				q[3] = -0.5*s*rot_vec[2];
			}
			else
			{
				q[0] = c;
				q[1] = 0.5*s*rot_vec[0];
				q[2] = 0.5*s*rot_vec[1];
				q[3] = 0.5*s*rot_vec[2];
			}
		}
		else
		{
			c = sqrt(mag2);
			s = sin(c / 2);
			mag2 = s / c;

			q[0] = cos(c / 2);
			q[1] = rot_vec[0] * mag2;
			q[2] = rot_vec[1] * mag2;
			q[3] = rot_vec[2] * mag2;
			if (q[0] < 0)
			{
				q[0] = -q[0];
				q[1] = -q[1];
				q[2] = -q[2];
				q[3] = -q[3];
			}
		}
		return 1;

	}
	uint8_t quat2pos(double q[4], double *r)
	{
		r[0] = -2 * atan(q[2] / q[0]) - PI / 2;
		r[1] = 2 * atan2(q[3], q[0]);
		return 1;

	}
	uint8_t norm_quat(double q[4])
	{
		double e = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1) / 2;

		q[0] = (1 - e)*q[0];
		q[1] = (1 - e)*q[1];
		q[2] = (1 - e)*q[2];
		q[3] = (1 - e)*q[3];
		return 1;

	}
	uint8_t quatinv(const double p[4], double q[4])
	{
		double a = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2] + p[3] * p[3]);
		if (a < 1.0E-15)
		{
			q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
			return -1;
		}
		q[0] = p[0] / a;
		q[1] = -p[1] / a;
		q[2] = -p[2] / a;
		q[3] = -p[3] / a;
		return 1;

	}

