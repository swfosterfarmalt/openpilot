#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2179469339109170571) {
   out_2179469339109170571[0] = delta_x[0] + nom_x[0];
   out_2179469339109170571[1] = delta_x[1] + nom_x[1];
   out_2179469339109170571[2] = delta_x[2] + nom_x[2];
   out_2179469339109170571[3] = delta_x[3] + nom_x[3];
   out_2179469339109170571[4] = delta_x[4] + nom_x[4];
   out_2179469339109170571[5] = delta_x[5] + nom_x[5];
   out_2179469339109170571[6] = delta_x[6] + nom_x[6];
   out_2179469339109170571[7] = delta_x[7] + nom_x[7];
   out_2179469339109170571[8] = delta_x[8] + nom_x[8];
   out_2179469339109170571[9] = delta_x[9] + nom_x[9];
   out_2179469339109170571[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7956168666464883728) {
   out_7956168666464883728[0] = -nom_x[0] + true_x[0];
   out_7956168666464883728[1] = -nom_x[1] + true_x[1];
   out_7956168666464883728[2] = -nom_x[2] + true_x[2];
   out_7956168666464883728[3] = -nom_x[3] + true_x[3];
   out_7956168666464883728[4] = -nom_x[4] + true_x[4];
   out_7956168666464883728[5] = -nom_x[5] + true_x[5];
   out_7956168666464883728[6] = -nom_x[6] + true_x[6];
   out_7956168666464883728[7] = -nom_x[7] + true_x[7];
   out_7956168666464883728[8] = -nom_x[8] + true_x[8];
   out_7956168666464883728[9] = -nom_x[9] + true_x[9];
   out_7956168666464883728[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_514528688558190830) {
   out_514528688558190830[0] = 1.0;
   out_514528688558190830[1] = 0;
   out_514528688558190830[2] = 0;
   out_514528688558190830[3] = 0;
   out_514528688558190830[4] = 0;
   out_514528688558190830[5] = 0;
   out_514528688558190830[6] = 0;
   out_514528688558190830[7] = 0;
   out_514528688558190830[8] = 0;
   out_514528688558190830[9] = 0;
   out_514528688558190830[10] = 0;
   out_514528688558190830[11] = 0;
   out_514528688558190830[12] = 1.0;
   out_514528688558190830[13] = 0;
   out_514528688558190830[14] = 0;
   out_514528688558190830[15] = 0;
   out_514528688558190830[16] = 0;
   out_514528688558190830[17] = 0;
   out_514528688558190830[18] = 0;
   out_514528688558190830[19] = 0;
   out_514528688558190830[20] = 0;
   out_514528688558190830[21] = 0;
   out_514528688558190830[22] = 0;
   out_514528688558190830[23] = 0;
   out_514528688558190830[24] = 1.0;
   out_514528688558190830[25] = 0;
   out_514528688558190830[26] = 0;
   out_514528688558190830[27] = 0;
   out_514528688558190830[28] = 0;
   out_514528688558190830[29] = 0;
   out_514528688558190830[30] = 0;
   out_514528688558190830[31] = 0;
   out_514528688558190830[32] = 0;
   out_514528688558190830[33] = 0;
   out_514528688558190830[34] = 0;
   out_514528688558190830[35] = 0;
   out_514528688558190830[36] = 1.0;
   out_514528688558190830[37] = 0;
   out_514528688558190830[38] = 0;
   out_514528688558190830[39] = 0;
   out_514528688558190830[40] = 0;
   out_514528688558190830[41] = 0;
   out_514528688558190830[42] = 0;
   out_514528688558190830[43] = 0;
   out_514528688558190830[44] = 0;
   out_514528688558190830[45] = 0;
   out_514528688558190830[46] = 0;
   out_514528688558190830[47] = 0;
   out_514528688558190830[48] = 1.0;
   out_514528688558190830[49] = 0;
   out_514528688558190830[50] = 0;
   out_514528688558190830[51] = 0;
   out_514528688558190830[52] = 0;
   out_514528688558190830[53] = 0;
   out_514528688558190830[54] = 0;
   out_514528688558190830[55] = 0;
   out_514528688558190830[56] = 0;
   out_514528688558190830[57] = 0;
   out_514528688558190830[58] = 0;
   out_514528688558190830[59] = 0;
   out_514528688558190830[60] = 1.0;
   out_514528688558190830[61] = 0;
   out_514528688558190830[62] = 0;
   out_514528688558190830[63] = 0;
   out_514528688558190830[64] = 0;
   out_514528688558190830[65] = 0;
   out_514528688558190830[66] = 0;
   out_514528688558190830[67] = 0;
   out_514528688558190830[68] = 0;
   out_514528688558190830[69] = 0;
   out_514528688558190830[70] = 0;
   out_514528688558190830[71] = 0;
   out_514528688558190830[72] = 1.0;
   out_514528688558190830[73] = 0;
   out_514528688558190830[74] = 0;
   out_514528688558190830[75] = 0;
   out_514528688558190830[76] = 0;
   out_514528688558190830[77] = 0;
   out_514528688558190830[78] = 0;
   out_514528688558190830[79] = 0;
   out_514528688558190830[80] = 0;
   out_514528688558190830[81] = 0;
   out_514528688558190830[82] = 0;
   out_514528688558190830[83] = 0;
   out_514528688558190830[84] = 1.0;
   out_514528688558190830[85] = 0;
   out_514528688558190830[86] = 0;
   out_514528688558190830[87] = 0;
   out_514528688558190830[88] = 0;
   out_514528688558190830[89] = 0;
   out_514528688558190830[90] = 0;
   out_514528688558190830[91] = 0;
   out_514528688558190830[92] = 0;
   out_514528688558190830[93] = 0;
   out_514528688558190830[94] = 0;
   out_514528688558190830[95] = 0;
   out_514528688558190830[96] = 1.0;
   out_514528688558190830[97] = 0;
   out_514528688558190830[98] = 0;
   out_514528688558190830[99] = 0;
   out_514528688558190830[100] = 0;
   out_514528688558190830[101] = 0;
   out_514528688558190830[102] = 0;
   out_514528688558190830[103] = 0;
   out_514528688558190830[104] = 0;
   out_514528688558190830[105] = 0;
   out_514528688558190830[106] = 0;
   out_514528688558190830[107] = 0;
   out_514528688558190830[108] = 1.0;
   out_514528688558190830[109] = 0;
   out_514528688558190830[110] = 0;
   out_514528688558190830[111] = 0;
   out_514528688558190830[112] = 0;
   out_514528688558190830[113] = 0;
   out_514528688558190830[114] = 0;
   out_514528688558190830[115] = 0;
   out_514528688558190830[116] = 0;
   out_514528688558190830[117] = 0;
   out_514528688558190830[118] = 0;
   out_514528688558190830[119] = 0;
   out_514528688558190830[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6047950613379053758) {
   out_6047950613379053758[0] = dt*state[3] + state[0];
   out_6047950613379053758[1] = dt*state[4] + state[1];
   out_6047950613379053758[2] = dt*state[5] + state[2];
   out_6047950613379053758[3] = state[3];
   out_6047950613379053758[4] = state[4];
   out_6047950613379053758[5] = state[5];
   out_6047950613379053758[6] = dt*state[7] + state[6];
   out_6047950613379053758[7] = dt*state[8] + state[7];
   out_6047950613379053758[8] = state[8];
   out_6047950613379053758[9] = state[9];
   out_6047950613379053758[10] = state[10];
}
void F_fun(double *state, double dt, double *out_1077854140641575516) {
   out_1077854140641575516[0] = 1;
   out_1077854140641575516[1] = 0;
   out_1077854140641575516[2] = 0;
   out_1077854140641575516[3] = dt;
   out_1077854140641575516[4] = 0;
   out_1077854140641575516[5] = 0;
   out_1077854140641575516[6] = 0;
   out_1077854140641575516[7] = 0;
   out_1077854140641575516[8] = 0;
   out_1077854140641575516[9] = 0;
   out_1077854140641575516[10] = 0;
   out_1077854140641575516[11] = 0;
   out_1077854140641575516[12] = 1;
   out_1077854140641575516[13] = 0;
   out_1077854140641575516[14] = 0;
   out_1077854140641575516[15] = dt;
   out_1077854140641575516[16] = 0;
   out_1077854140641575516[17] = 0;
   out_1077854140641575516[18] = 0;
   out_1077854140641575516[19] = 0;
   out_1077854140641575516[20] = 0;
   out_1077854140641575516[21] = 0;
   out_1077854140641575516[22] = 0;
   out_1077854140641575516[23] = 0;
   out_1077854140641575516[24] = 1;
   out_1077854140641575516[25] = 0;
   out_1077854140641575516[26] = 0;
   out_1077854140641575516[27] = dt;
   out_1077854140641575516[28] = 0;
   out_1077854140641575516[29] = 0;
   out_1077854140641575516[30] = 0;
   out_1077854140641575516[31] = 0;
   out_1077854140641575516[32] = 0;
   out_1077854140641575516[33] = 0;
   out_1077854140641575516[34] = 0;
   out_1077854140641575516[35] = 0;
   out_1077854140641575516[36] = 1;
   out_1077854140641575516[37] = 0;
   out_1077854140641575516[38] = 0;
   out_1077854140641575516[39] = 0;
   out_1077854140641575516[40] = 0;
   out_1077854140641575516[41] = 0;
   out_1077854140641575516[42] = 0;
   out_1077854140641575516[43] = 0;
   out_1077854140641575516[44] = 0;
   out_1077854140641575516[45] = 0;
   out_1077854140641575516[46] = 0;
   out_1077854140641575516[47] = 0;
   out_1077854140641575516[48] = 1;
   out_1077854140641575516[49] = 0;
   out_1077854140641575516[50] = 0;
   out_1077854140641575516[51] = 0;
   out_1077854140641575516[52] = 0;
   out_1077854140641575516[53] = 0;
   out_1077854140641575516[54] = 0;
   out_1077854140641575516[55] = 0;
   out_1077854140641575516[56] = 0;
   out_1077854140641575516[57] = 0;
   out_1077854140641575516[58] = 0;
   out_1077854140641575516[59] = 0;
   out_1077854140641575516[60] = 1;
   out_1077854140641575516[61] = 0;
   out_1077854140641575516[62] = 0;
   out_1077854140641575516[63] = 0;
   out_1077854140641575516[64] = 0;
   out_1077854140641575516[65] = 0;
   out_1077854140641575516[66] = 0;
   out_1077854140641575516[67] = 0;
   out_1077854140641575516[68] = 0;
   out_1077854140641575516[69] = 0;
   out_1077854140641575516[70] = 0;
   out_1077854140641575516[71] = 0;
   out_1077854140641575516[72] = 1;
   out_1077854140641575516[73] = dt;
   out_1077854140641575516[74] = 0;
   out_1077854140641575516[75] = 0;
   out_1077854140641575516[76] = 0;
   out_1077854140641575516[77] = 0;
   out_1077854140641575516[78] = 0;
   out_1077854140641575516[79] = 0;
   out_1077854140641575516[80] = 0;
   out_1077854140641575516[81] = 0;
   out_1077854140641575516[82] = 0;
   out_1077854140641575516[83] = 0;
   out_1077854140641575516[84] = 1;
   out_1077854140641575516[85] = dt;
   out_1077854140641575516[86] = 0;
   out_1077854140641575516[87] = 0;
   out_1077854140641575516[88] = 0;
   out_1077854140641575516[89] = 0;
   out_1077854140641575516[90] = 0;
   out_1077854140641575516[91] = 0;
   out_1077854140641575516[92] = 0;
   out_1077854140641575516[93] = 0;
   out_1077854140641575516[94] = 0;
   out_1077854140641575516[95] = 0;
   out_1077854140641575516[96] = 1;
   out_1077854140641575516[97] = 0;
   out_1077854140641575516[98] = 0;
   out_1077854140641575516[99] = 0;
   out_1077854140641575516[100] = 0;
   out_1077854140641575516[101] = 0;
   out_1077854140641575516[102] = 0;
   out_1077854140641575516[103] = 0;
   out_1077854140641575516[104] = 0;
   out_1077854140641575516[105] = 0;
   out_1077854140641575516[106] = 0;
   out_1077854140641575516[107] = 0;
   out_1077854140641575516[108] = 1;
   out_1077854140641575516[109] = 0;
   out_1077854140641575516[110] = 0;
   out_1077854140641575516[111] = 0;
   out_1077854140641575516[112] = 0;
   out_1077854140641575516[113] = 0;
   out_1077854140641575516[114] = 0;
   out_1077854140641575516[115] = 0;
   out_1077854140641575516[116] = 0;
   out_1077854140641575516[117] = 0;
   out_1077854140641575516[118] = 0;
   out_1077854140641575516[119] = 0;
   out_1077854140641575516[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2024765830105387342) {
   out_2024765830105387342[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7891237798228026138) {
   out_7891237798228026138[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7891237798228026138[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7891237798228026138[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7891237798228026138[3] = 0;
   out_7891237798228026138[4] = 0;
   out_7891237798228026138[5] = 0;
   out_7891237798228026138[6] = 1;
   out_7891237798228026138[7] = 0;
   out_7891237798228026138[8] = 0;
   out_7891237798228026138[9] = 0;
   out_7891237798228026138[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_1046796656331890993) {
   out_1046796656331890993[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5982310217900809888) {
   out_5982310217900809888[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5982310217900809888[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5982310217900809888[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5982310217900809888[3] = 0;
   out_5982310217900809888[4] = 0;
   out_5982310217900809888[5] = 0;
   out_5982310217900809888[6] = 1;
   out_5982310217900809888[7] = 0;
   out_5982310217900809888[8] = 0;
   out_5982310217900809888[9] = 1;
   out_5982310217900809888[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5735942229984248599) {
   out_5735942229984248599[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2797736668572699440) {
   out_2797736668572699440[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[6] = 0;
   out_2797736668572699440[7] = 1;
   out_2797736668572699440[8] = 0;
   out_2797736668572699440[9] = 0;
   out_2797736668572699440[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5735942229984248599) {
   out_5735942229984248599[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2797736668572699440) {
   out_2797736668572699440[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2797736668572699440[6] = 0;
   out_2797736668572699440[7] = 1;
   out_2797736668572699440[8] = 0;
   out_2797736668572699440[9] = 0;
   out_2797736668572699440[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2179469339109170571) {
  err_fun(nom_x, delta_x, out_2179469339109170571);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7956168666464883728) {
  inv_err_fun(nom_x, true_x, out_7956168666464883728);
}
void gnss_H_mod_fun(double *state, double *out_514528688558190830) {
  H_mod_fun(state, out_514528688558190830);
}
void gnss_f_fun(double *state, double dt, double *out_6047950613379053758) {
  f_fun(state,  dt, out_6047950613379053758);
}
void gnss_F_fun(double *state, double dt, double *out_1077854140641575516) {
  F_fun(state,  dt, out_1077854140641575516);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2024765830105387342) {
  h_6(state, sat_pos, out_2024765830105387342);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7891237798228026138) {
  H_6(state, sat_pos, out_7891237798228026138);
}
void gnss_h_20(double *state, double *sat_pos, double *out_1046796656331890993) {
  h_20(state, sat_pos, out_1046796656331890993);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5982310217900809888) {
  H_20(state, sat_pos, out_5982310217900809888);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5735942229984248599) {
  h_7(state, sat_pos_vel, out_5735942229984248599);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2797736668572699440) {
  H_7(state, sat_pos_vel, out_2797736668572699440);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5735942229984248599) {
  h_21(state, sat_pos_vel, out_5735942229984248599);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2797736668572699440) {
  H_21(state, sat_pos_vel, out_2797736668572699440);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
