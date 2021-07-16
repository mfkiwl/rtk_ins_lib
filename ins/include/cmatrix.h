
#ifndef		__MATRIX_H__
#define		__MATRIX_H__
#include <stdint.h>

	uint8_t MatrixAdd(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result);
	uint8_t MatrixSub(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result);
	uint8_t MatrixMutiply(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, const int matrix_b_column, double *matrix_result);
	uint8_t MatrixTranspose(const double *matrix_a, const int matrix_row, const int matrix_column, double *matrix_result);

	uint8_t MatrixAddfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result);
	uint8_t MatrixSubfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, float *matrix_result);
	uint8_t MatrixMutiplyfloat(const float *matrix_a, const float *matrix_b, const int matrix_a_row, const int matrix_a_colume, const int matrix_b_column, float *matrix_result);
	uint8_t MatrixMutiplyfloatd(const float *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, const int matrix_b_column, double *matrix_result);


	uint8_t MatrixTransposefloat(const float *matrix_a, const int matrix_row, const int matrix_column, float *matrix_result);
	uint8_t MatrixCholosky(const double *matrix, const int matrix_row, double *lower_tri_matrix);
	//uint8_t MatrixInverse(const double *lower_tri_matrix, const int matrix_row, double *matrix_result);
	uint8_t CrossProduct(double a[3], double b[3], double c[3]);
	uint8_t GetSkewSymmetricMatrixOfVector(const double  mvector[3], double *M);
	uint8_t quatprod(const double p[4], const double q[4], double r[4]);
	uint8_t rvec2quat(double rot_vect[3], double q[4]);
	uint8_t quat2pos(double q[4], double *r);
	uint8_t norm_quat(double q[4]);
	uint8_t quatinv(const double p[4], double q[4]);
	uint8_t quat2rvec(double q[4], double rot_vec[3]);
	uint8_t MatrixInverse(uint8_t n,double* a); //Matrix Inverse

#endif