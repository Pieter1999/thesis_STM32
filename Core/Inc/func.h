#ifndef FUNC_H
#define FUNC_H

#include <stdint.h>
#include "string.h"
#include "main.h"
#include "ecos.h"

#ifdef LDL_LONG
    typedef long idxint;
#else
    typedef int idxint;
#endif

typedef struct {
    double w, x, y, z;
} Quaternion;

typedef struct {
    double m[4][4];  // 4x4 matrix
} Matrix4x4;

extern UART_HandleTypeDef huart2;

void parseBuffer(uint8_t *buffer, uint16_t size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void PD_control(double *q, double *q_ref, double *omega, double *k_p, double *k_i, double *k_d, double *k_s, double dt, double *T_c);
void send3Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements) ;
void send4Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements) ;
void send6Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements);
void quaternion_error(const Quaternion* q, const Quaternion* q_ref, Quaternion* result);
void matrix_multiply_vector(const Matrix4x4* mat, const Quaternion* vec, Quaternion* result);
void cross_product(const double *v1, const double *v2, double *result);
void normalize_vector(double *v);
void reference_quaternion_paper(void);
void DCM_to_quaternion(double DCM_matrix[3][3]);
void quaternion_321_rotation(double qw, double qx, double qy, double qz, double R[3][3]);
double vector_norm(double v[3]);
void matrix_vector_dot_product(double matrix[3][3], double vector[3], double result[3]);
void GGMoon(void);
void matrix_inverse(double input[3][3], double output[3][3]);
void matrix_vector_multiply(double matrix[3][3], double vector[3], double result[3]);
void solve_thruster_problem(const double* T_matrix, idxint n_thrusters, double T_c[3], double F_SSP_max);
void updatePWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, double value);
double quaternion_dot_product(const double q1[4], const double q2[4]);
void flip_quaternion(double q[4]);


#endif // FUNC_H
