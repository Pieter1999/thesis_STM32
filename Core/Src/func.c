#include "func.h"
#include <math.h>
#include <string.h>
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <global.h>
#include "ecos.h"


// Constants
const double F_SSP_max = 0.0000008; //N, test 200microNewtons of thrust from SSP thrusters
const uint32_t timer_period = 100000; //microseconds, frequency = 1 / (timer_period / 1,000,000)
const double minimum_impulse_bit = 0.01; // microNewtonseconds

// note: matrices do not adjust correctly for difference in panel distance. Re-do all matrices in case of different set-up.
const double T_matrix_1[3][6] = {
		{0.0, 0.0, -0.15, 0.15, 0.0, 0.0},
		{0.0, 0.0, 0.0, 0.0, -0.1, 0.1},
		{0.15, -0.15, 0.0, 0.0, 0.0, 0.0}
};

const double T_matrix_2[3][6] = {
		{0.0, 0.0, -0.15, 0.15, 0.0, 0.0},
		{-0.1, 0.1, 0.0, 0.0, -0.1, 0.1},
		{0.15, -0.15, 0.0, 0.0, -0.15, 0.15}
};

const double T_matrix_3[3][8] = {
		{0.0, 0.0, -0.15, 0.15, 0.0, 0.0, 0.1, 0.1},
		{0.0, 0.0, 0.0, 0.0, -0.1, 0.1, 0.0, 0.0},
		{0.15, -0.15, 0.0, 0.0, 0.0, 0.0, 0.1, -0.1}
};

const double T_matrix_4[3][12] = {
		{-0.1, 0.0, 0.15, 0.0, -0.1, 0.15, 0.0, 0.1, -0.15, -0.15, 0.1, 0.0},
		{0.0, -0.1, 0.1, 0.1, 0.0, -0.1, 0.1, 0.0, -0.1, 0.1, 0.0, -0.1},
		{0.1, -0.15, 0.0, 0.15, -0.1, 0.0, -0.15, 0.1, 0.0, 0.0, -0.1, 0.15}
};

const double inertia_matrix[3][3] = {
    {1.009, 0.0, 0.0},
    {0.0, 0.251, 0.0},
    {0.0, 0.0, 0.916}
};

// Retrieve buffer from incoming signal
void parseBuffer(uint8_t *buffer, uint16_t size) {
    char *token;

    // Convert buffer to null-terminated string
    buffer[size] = '\0'; // Ensure there is a null at the end of the buffer

    token = strtok((char *)buffer, ",");
    if (!token) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Parse Error\n", 12, HAL_MAX_DELAY);
        return;
    }

    // Parsing arrays from the buffer
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) pos_CAP[i] = token ? atof(token): 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) pos_Sun[i] = token ? atof(token): 0;
    for (int i = 0; i < 4; i++, token = strtok(NULL, ",")) q[i] = token ? atof(token) : 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) omega[i] = token ? atof(token) : 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) k_p[i] = token ? atof(token) : 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) k_i[i] = token ? atof(token) : 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) k_d[i] = token ? atof(token) : 0;
    for (int i = 0; i < 3; i++, token = strtok(NULL, ",")) k_s[i] = token ? atof(token) : 0;
    token = strtok(NULL, ",");
    dt = token ? atof(token) : 0;


}


// Matrix multiplication standard
void matrix_multiply_vector(const Matrix4x4* mat, const Quaternion* vec, Quaternion* result) {
    // Adjusting order of assignments to match [qew, qe1, qe2, qe3]
    result->w = mat->m[3][0] * vec->w + mat->m[3][1] * vec->x + mat->m[3][2] * vec->y + mat->m[3][3] * vec->z;
    result->x = mat->m[0][0] * vec->w + mat->m[0][1] * vec->x + mat->m[0][2] * vec->y + mat->m[0][3] * vec->z;
    result->y = mat->m[1][0] * vec->w + mat->m[1][1] * vec->x + mat->m[1][2] * vec->y + mat->m[1][3] * vec->z;
    result->z = mat->m[2][0] * vec->w + mat->m[2][1] * vec->x + mat->m[2][2] * vec->y + mat->m[2][3] * vec->z;
}

// Quaternion error calculation function
void quaternion_error(const Quaternion* q, const Quaternion* q_ref, Quaternion* result) {
    // Extract quaternion components
    double qrw = q_ref->w, qr1 = q_ref->x, qr2 = q_ref->y, qr3 = q_ref->z;
    double qw = q->w, q1 = q->x, q2 = q->y, q3 = q->z;

    // Adjusted quaternion vector
    Quaternion adj_vec = {-q1, -q2, -q3, qw};

    // Construct the matrix
    Matrix4x4 adj_mat = {
        {
            {qrw, qr3, -qr2, qr1},
            {-qr3, qrw, qr1, qr2},
            {qr2, -qr1, qrw, qr3},
            {-qr1, -qr2, qr3, qrw}
        }
    };

    // Perform the matrix-vector multiplication
    matrix_multiply_vector(&adj_mat, &adj_vec, result);
}


// Function to convert float array to string and send via UART
void send3Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements) {
    uint8_t buffer[200];  // Adjust size based on the expected length of the message

    sprintf((char*) buffer, "%.20lf,%.20lf,%.20lf\n", vector[0], vector[1], vector[2]);

    // Send the formatted string via UART
    HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

// Function to convert float array to string and send via UART, for reference quaternion
void send4Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements) {
    uint8_t buffer[200];  // Adjust size based on the expected length of the message

    sprintf((char*) buffer, "%.20lf,%.20lf,%.20lf,%.20lf\n", vector[0], vector[1], vector[2], vector[3]);

    // Send the formatted string via UART
    HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

// Function to convert float array to string and send via UART, for reference quaternion
void send6Vector(UART_HandleTypeDef *huart, const double *vector, uint8_t num_elements) {
    uint8_t buffer[300];  // Adjust size based on the expected length of the message

    sprintf((char*) buffer, "%.20lf,%.20lf,%.20lf,%.20lf,%.20lf,%.20lf\n", vector[0], vector[1], vector[2], vector[3], vector[4], vector[5]);

    // Send the formatted string via UART
    HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}



// PD control loop, adapted for C
void PD_control(double *q, double *q_ref, double *omega, double *k_p, double *k_i, double *k_d, double *k_s, double dt, double *T_c) {
    Quaternion current_q = {q[0], q[1], q[2], q[3]};
    Quaternion reference_q = {q_ref[0], q_ref[1], q_ref[2], q_ref[3]};
    Quaternion error_q;

    // Calculate quaternion error
    quaternion_error(&current_q, &reference_q, &error_q);

    // Destructure error quaternion components
    double qew = error_q.w;
    double qex = error_q.x;
    double qey = error_q.y;
    double qez = error_q.z;

    // Angular velocity components
    double omega_x = omega[0], omega_y = omega[1], omega_z = omega[2];

    // PID gains
    double k_p_1 = k_p[0], k_p_2 = k_p[1], k_p_3 = k_p[2];
    double k_i_1 = k_i[0], k_i_2 = k_i[1], k_i_3 = k_i[2];
    double k_d_1 = k_d[0], k_d_2 = k_d[1], k_d_3 = k_d[2];
    double k_s_1 = k_s[0], k_s_2 = k_s[1], k_s_3 = k_s[2];

    // Assuming integral terms should be persistent or managed outside this function
    static double int_x = 0, int_y = 0, int_z = 0;  // Integral terms

    // Control torque calculations
    T_c[0] = k_s_1 * (k_p_1 * qex + k_i_1 * int_x - k_d_1 * omega_x);
    T_c[1] = k_s_2 * (k_p_2 * qey + k_i_2 * int_y - k_d_2 * omega_y);
    T_c[2] = k_s_3 * (k_p_3 * qez + k_i_3 * int_z - k_d_3 * omega_z);

    // Update integral terms for next iteration
    int_x += qex * dt;
    int_y += qey * dt;
    int_z += qez * dt;
}

// Helper function to compute the cross product of two 3D vectors
void cross_product(const double *v1, const double *v2, double *result) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Helper function to normalize a 3D vector
void normalize_vector(double *v) {
    double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    for (int i = 0; i < 3; i++) {
        v[i] /= norm;
    }
}

void reference_quaternion_paper(void) {
	double Sun_pointing_vector[3];
	double Moon_pointing_vector[3];
	double cross_1[3], cross_2[3];
	double x1[3], x2[3], x3[3];
	double A_d[3][3];


	// Calculate Sun pointing vector
	for (int i = 0; i < 3; i++) {
		Sun_pointing_vector[i] = pos_Sun[i] - pos_CAP[i];
	}

	normalize_vector(Sun_pointing_vector);

    // Calculate Moon pointing vector
    for (int i = 0; i < 3; i++) {
        Moon_pointing_vector[i] = -pos_CAP[i];
    }

    normalize_vector(Moon_pointing_vector);

	// x1 = Moon pointing vector
	memcpy(x1, Moon_pointing_vector, 3 * sizeof(double));

	// x2 = cross product of Sun_pointing vector and x1, normalized
	cross_product(Sun_pointing_vector, x1, cross_1);
	normalize_vector(cross_1);
	memcpy(x2, cross_1, 3 * sizeof(double));

	// x3 = cross product of x1 and x2, normalized
	cross_product(x1, x2, cross_2);
	normalize_vector(cross_2);
	memcpy(x3, cross_2, 3 * sizeof(double));

	// Construct DCM matrix A_d to convert to quaternion representation
	for (int i = 0; i < 3; i++) {
	    A_d[i][0] = x2[i];    // First column: x2
	    A_d[i][1] = -x1[i];   // Second column: -x1
	    A_d[i][2] = x3[i];    // Third column: x3
	}

	// Final DCM to quaternion conversion
	DCM_to_quaternion(A_d);

}

void DCM_to_quaternion(double DCM[3][3]) {
    double a11 = DCM[0][0];
    double a22 = DCM[1][1];
    double a33 = DCM[2][2];
    double trace = a11 + a22 + a33;

    double qw, qx, qy, qz;

    if (trace > 0) {
        qw = 0.5 * sqrt(1 + trace);
        qx = (DCM[2][1] - DCM[1][2]) / (4 * qw);
        qy = (DCM[0][2] - DCM[2][0]) / (4 * qw);
        qz = (DCM[1][0] - DCM[0][1]) / (4 * qw);
    } else if (a11 > a22 && a11 > a33) {
        qx = 0.5 * sqrt(1 + a11 - a22 - a33);
        qw = (DCM[2][1] - DCM[1][2]) / (4 * qx);
        qy = (DCM[0][1] + DCM[1][0]) / (4 * qx);
        qz = (DCM[0][2] + DCM[2][0]) / (4 * qx);
    } else if (a22 > a33) {
        qy = 0.5 * sqrt(1 + a22 - a11 - a33);
        qw = (DCM[0][2] - DCM[2][0]) / (4 * qy);
        qx = (DCM[0][1] + DCM[1][0]) / (4 * qy);
        qz = (DCM[1][2] + DCM[2][1]) / (4 * qy);
    } else {
        qz = 0.5 * sqrt(1 + a33 - a11 - a22);
        qw = (DCM[1][0] - DCM[0][1]) / (4 * qz);
        qx = (DCM[0][2] + DCM[2][0]) / (4 * qz);
        qy = (DCM[1][2] + DCM[2][1]) / (4 * qz);
    }

    q_ref[0] = qw;
    q_ref[1] = qx;
    q_ref[2] = qy;
    q_ref[3] = qz;

}


// Function to calculate the dot product of two quaternions
double quaternion_dot_product(const double q1[4], const double q2[4]) {
    return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
}

// Function to flip the sign of a quaternion
void flip_quaternion(double q[4]) {
    q[0] = -q[0];
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

// Function to calculate quaternion-based rotation matrix (3-2-1 rotation)
void quaternion_321_rotation(double qw, double qx, double qy, double qz, double R[3][3]) {
    R[0][0] = 1 - 2 * (qy * qy + qz * qz);
    R[0][1] = 2 * (qx * qy - qz * qw);
    R[0][2] = 2 * (qx * qz + qy * qw);

    R[1][0] = 2 * (qx * qy + qz * qw);
    R[1][1] = 1 - 2 * (qx * qx + qz * qz);
    R[1][2] = 2 * (qy * qz - qx * qw);

    R[2][0] = 2 * (qx * qz - qy * qw);
    R[2][1] = 2 * (qy * qz + qx * qw);
    R[2][2] = 1 - 2 * (qx * qx + qy * qy);
}

// Function to calculate the norm of a 3-element vector
double vector_norm(double v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Function to multiply a 3x3 matrix with a 3-element vector (dot product)
void matrix_vector_dot_product(double matrix[3][3], double vector[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = matrix[i][0] * vector[0] + matrix[i][1] * vector[1] + matrix[i][2] * vector[2];
    }
}

// Function to calculate the inverse of a 3x3 matrix
void matrix_inverse(double input[3][3], double output[3][3]) {
    double det = input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1])
               - input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0])
               + input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);

    if (det == 0) {
        // Matrix is singular, cannot be inverted
        printf("Error: Singular matrix, cannot compute inverse.\n");
        return;
    }

    double inv_det = 1.0 / det;

    output[0][0] = (input[1][1] * input[2][2] - input[1][2] * input[2][1]) * inv_det;
    output[0][1] = (input[0][2] * input[2][1] - input[0][1] * input[2][2]) * inv_det;
    output[0][2] = (input[0][1] * input[1][2] - input[0][2] * input[1][1]) * inv_det;

    output[1][0] = (input[1][2] * input[2][0] - input[1][0] * input[2][2]) * inv_det;
    output[1][1] = (input[0][0] * input[2][2] - input[0][2] * input[2][0]) * inv_det;
    output[1][2] = (input[0][2] * input[1][0] - input[0][0] * input[1][2]) * inv_det;

    output[2][0] = (input[1][0] * input[2][1] - input[1][1] * input[2][0]) * inv_det;
    output[2][1] = (input[0][1] * input[2][0] - input[0][0] * input[2][1]) * inv_det;
    output[2][2] = (input[0][0] * input[1][1] - input[0][1] * input[1][0]) * inv_det;
}

// Function to multiply a 3x3 matrix with a 3-element vector
void matrix_vector_multiply(double matrix[3][3], double vector[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}

void solve_thruster_problem(const double* T_matrix, idxint n_thrusters, double T_c[3], double F_SSP_max) {

    // Dynamically calculate the number of thrusters (columns of T_matrix)
    idxint n = n_thrusters;  // Number of variables (thrusters)
    idxint m = 2 * n;        // Number of inequalities (0 <= thrust <= F_SSP_max)
    idxint p = 3;            // Number of equality constraints (T_matrix * thrust == T_c)
    idxint l = 2 * n;            // Number of simple bounds (0 <= thrust)
    idxint ncones = 0;       // Number of second-order cones (none in this case)
    idxint nex = 0;          // Number of exponential cones (none in this case)

	// Objective vector (sum of thrust is minimized)
	double* c = (double*)malloc(n*sizeof(double));
	for (idxint i = 0; i < n; i++) {
		c[i] = 1.0;
	}

	// Define G matrix as sparse in compressed column format
	idxint* G_i = (idxint*)malloc(2 * n * sizeof(idxint)); // Row indices for G's non-zero entries
	idxint* G_j = (idxint*)malloc((n + 1) * sizeof(idxint)); // Column indices (compressed format)
	double* G_x = (double*)malloc(2 * n * sizeof(double)); // Non-zero values

	for (idxint i = 0; i < n; i++) {
	    G_i[2 * i] = i;        // Lower bound row index
	    G_x[2 * i] = -1.0;     // Represents thrust >= 0 (negative because inequalities are G * x ≤ h)

	    G_i[2 * i + 1] = n + i; // Upper bound row index
	    G_x[2 * i + 1] = 1.0;   // Represents thrust ≤ F_SSP_max

	    G_j[i] = 2 * i; // Start of each column in `G_x` (compressed column index)
	}
	// End marker for the last column
	G_j[n] = 2 * n;

	// Vector h (right-hand side of the inequality constraints)
	double* h = (double*)malloc(2*n*sizeof(double));
	for (idxint i = 0; i < n; i++) {
		h[i] = 0.0; // Lower bound 0 <= thrust
		h[n+i] = F_SSP_max; // Upper bound thrust <= F_SSP_max
	}

    // Define sparse matrix arrays
    double A_x[] = {0.15, -0.15, -0.15, 0.15, -0.1, 0.1};
    idxint A_i[] = {2, 2, 0, 0, 1, 1};
    idxint A_j[] = {0, 1, 2, 3, 4, 5, 6};

    // RHS vector (T_c)
    double* b = (double*)malloc(p * sizeof(double));
    for (idxint i = 0; i < p; i++) {
        b[i] = T_c[i];
    }

    // Sparse G and A matrix arrays in compressed column storage (ccs)
    // Gpr: Non-zero values of G
    // Gjc: Column index array of G
    // Gir: Row index array of G
    // Similar structure for A
    pwork *work = ECOS_setup(
        n,        // Number of variables (thrusters)
        m,        // Number of inequalities
        p,        // Number of equality constraints
        l,        // Dimension of the positive orthant
        ncones,   // Number of second-order cones
        NULL,     // No second-order cones (hence NULL)
        nex,      // Number of exponential cones
        G_x,      // Sparse G matrix data
        G_j,      // G matrix column index array (compressed)
        G_i,      // G matrix row index array (compressed)
        A_x,      // Sparse A matrix data
        A_j,      // A matrix column index array (compressed)
        A_i,      // A matrix row index array (compressed)
        c,        // Cost function
        h,        // RHS of inequalities
        b         // RHS of equalities
    );

    // Solve the problem
    idxint exitflag = ECOS_solve(work);

    // Buffer to hold the UART messages
    char uart_msg[100];

    // Check for optimal solution and store the result in thrust_array
    if (exitflag == ECOS_OPTIMAL) {
    	snprintf(uart_msg, sizeof(uart_msg), "Optimal solution found:\r\n");
//    	HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        for (idxint i = 0; i < n; i++) {
            thrust_array[i] = work->x[i];  // Store the thrust values in the global array
        }
    } else {
    	snprintf(uart_msg, sizeof(uart_msg), "ECOS failed with exitflag: %ld\r\n", exitflag);
//    	HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
    }

    // Clean up workspace
    ECOS_cleanup(work, 0);

    // Free dynamically allocated memory
    free(c);
    free(G_i);
    free(G_j);
    free(G_x);
    free(h);
    free(b);
}


void updatePWMFrequency(TIM_HandleTypeDef *htim, uint32_t channel, double value) {
    // Only the duty cycle actually has to be updated
    // Based on hydrazine varying thrust paper
    // This is a first assumption, later refinement will follow

    uint32_t duty_cycle;

    if (value < (double)(minimum_impulse_bit / timer_period)) {
        duty_cycle = 0;
    } else {
        // Calculate the duty cycle normally
        duty_cycle = (uint32_t)((value / F_SSP_max) * timer_period);
    }

    uint8_t buffer[50];  // Adjust size based on the expected length of the message

    // Format string to include timer period and duty cycle
    sprintf((char*) buffer, "%u,%u\n", timer_period, duty_cycle);

    // Send the formatted string via UART
    HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), HAL_MAX_DELAY);

    // Dynamically set the duty cycle for the specified channel
    switch (channel) {
        case 1:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty_cycle);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, duty_cycle);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, duty_cycle);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, duty_cycle);
            break;
        default:
            // Invalid channel
            sprintf((char*) buffer, "Error: Invalid Channel\n");
            HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
            break;
    }
}







