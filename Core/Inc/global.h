#ifndef GLOBAL_H
#define GLOBAL_H

extern double pos_CAP[3], pos_Sun[3]; // CAPSTONE and Sun position vectors
extern double q[4], q_ref[4], omega[3], k_p[3], k_i[3], k_d[3], k_s[3], dt;
extern double T_c[3];  // Control torque results

extern const double T_matrix_1[3][6];
extern const double T_matrix_2[3][6];
extern const double T_matrix_3[3][8];
extern const double T_matrix_4[3][12];

extern double thrust_array[6];
extern const double F_SSP_max;
extern const uint32_t timer_period;
extern const double minimum_impulse_bit;

#endif
