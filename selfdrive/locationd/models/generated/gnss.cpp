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
void err_fun(double *nom_x, double *delta_x, double *out_3903931754286709959) {
   out_3903931754286709959[0] = delta_x[0] + nom_x[0];
   out_3903931754286709959[1] = delta_x[1] + nom_x[1];
   out_3903931754286709959[2] = delta_x[2] + nom_x[2];
   out_3903931754286709959[3] = delta_x[3] + nom_x[3];
   out_3903931754286709959[4] = delta_x[4] + nom_x[4];
   out_3903931754286709959[5] = delta_x[5] + nom_x[5];
   out_3903931754286709959[6] = delta_x[6] + nom_x[6];
   out_3903931754286709959[7] = delta_x[7] + nom_x[7];
   out_3903931754286709959[8] = delta_x[8] + nom_x[8];
   out_3903931754286709959[9] = delta_x[9] + nom_x[9];
   out_3903931754286709959[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6187082929283308233) {
   out_6187082929283308233[0] = -nom_x[0] + true_x[0];
   out_6187082929283308233[1] = -nom_x[1] + true_x[1];
   out_6187082929283308233[2] = -nom_x[2] + true_x[2];
   out_6187082929283308233[3] = -nom_x[3] + true_x[3];
   out_6187082929283308233[4] = -nom_x[4] + true_x[4];
   out_6187082929283308233[5] = -nom_x[5] + true_x[5];
   out_6187082929283308233[6] = -nom_x[6] + true_x[6];
   out_6187082929283308233[7] = -nom_x[7] + true_x[7];
   out_6187082929283308233[8] = -nom_x[8] + true_x[8];
   out_6187082929283308233[9] = -nom_x[9] + true_x[9];
   out_6187082929283308233[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3389584686144192417) {
   out_3389584686144192417[0] = 1.0;
   out_3389584686144192417[1] = 0;
   out_3389584686144192417[2] = 0;
   out_3389584686144192417[3] = 0;
   out_3389584686144192417[4] = 0;
   out_3389584686144192417[5] = 0;
   out_3389584686144192417[6] = 0;
   out_3389584686144192417[7] = 0;
   out_3389584686144192417[8] = 0;
   out_3389584686144192417[9] = 0;
   out_3389584686144192417[10] = 0;
   out_3389584686144192417[11] = 0;
   out_3389584686144192417[12] = 1.0;
   out_3389584686144192417[13] = 0;
   out_3389584686144192417[14] = 0;
   out_3389584686144192417[15] = 0;
   out_3389584686144192417[16] = 0;
   out_3389584686144192417[17] = 0;
   out_3389584686144192417[18] = 0;
   out_3389584686144192417[19] = 0;
   out_3389584686144192417[20] = 0;
   out_3389584686144192417[21] = 0;
   out_3389584686144192417[22] = 0;
   out_3389584686144192417[23] = 0;
   out_3389584686144192417[24] = 1.0;
   out_3389584686144192417[25] = 0;
   out_3389584686144192417[26] = 0;
   out_3389584686144192417[27] = 0;
   out_3389584686144192417[28] = 0;
   out_3389584686144192417[29] = 0;
   out_3389584686144192417[30] = 0;
   out_3389584686144192417[31] = 0;
   out_3389584686144192417[32] = 0;
   out_3389584686144192417[33] = 0;
   out_3389584686144192417[34] = 0;
   out_3389584686144192417[35] = 0;
   out_3389584686144192417[36] = 1.0;
   out_3389584686144192417[37] = 0;
   out_3389584686144192417[38] = 0;
   out_3389584686144192417[39] = 0;
   out_3389584686144192417[40] = 0;
   out_3389584686144192417[41] = 0;
   out_3389584686144192417[42] = 0;
   out_3389584686144192417[43] = 0;
   out_3389584686144192417[44] = 0;
   out_3389584686144192417[45] = 0;
   out_3389584686144192417[46] = 0;
   out_3389584686144192417[47] = 0;
   out_3389584686144192417[48] = 1.0;
   out_3389584686144192417[49] = 0;
   out_3389584686144192417[50] = 0;
   out_3389584686144192417[51] = 0;
   out_3389584686144192417[52] = 0;
   out_3389584686144192417[53] = 0;
   out_3389584686144192417[54] = 0;
   out_3389584686144192417[55] = 0;
   out_3389584686144192417[56] = 0;
   out_3389584686144192417[57] = 0;
   out_3389584686144192417[58] = 0;
   out_3389584686144192417[59] = 0;
   out_3389584686144192417[60] = 1.0;
   out_3389584686144192417[61] = 0;
   out_3389584686144192417[62] = 0;
   out_3389584686144192417[63] = 0;
   out_3389584686144192417[64] = 0;
   out_3389584686144192417[65] = 0;
   out_3389584686144192417[66] = 0;
   out_3389584686144192417[67] = 0;
   out_3389584686144192417[68] = 0;
   out_3389584686144192417[69] = 0;
   out_3389584686144192417[70] = 0;
   out_3389584686144192417[71] = 0;
   out_3389584686144192417[72] = 1.0;
   out_3389584686144192417[73] = 0;
   out_3389584686144192417[74] = 0;
   out_3389584686144192417[75] = 0;
   out_3389584686144192417[76] = 0;
   out_3389584686144192417[77] = 0;
   out_3389584686144192417[78] = 0;
   out_3389584686144192417[79] = 0;
   out_3389584686144192417[80] = 0;
   out_3389584686144192417[81] = 0;
   out_3389584686144192417[82] = 0;
   out_3389584686144192417[83] = 0;
   out_3389584686144192417[84] = 1.0;
   out_3389584686144192417[85] = 0;
   out_3389584686144192417[86] = 0;
   out_3389584686144192417[87] = 0;
   out_3389584686144192417[88] = 0;
   out_3389584686144192417[89] = 0;
   out_3389584686144192417[90] = 0;
   out_3389584686144192417[91] = 0;
   out_3389584686144192417[92] = 0;
   out_3389584686144192417[93] = 0;
   out_3389584686144192417[94] = 0;
   out_3389584686144192417[95] = 0;
   out_3389584686144192417[96] = 1.0;
   out_3389584686144192417[97] = 0;
   out_3389584686144192417[98] = 0;
   out_3389584686144192417[99] = 0;
   out_3389584686144192417[100] = 0;
   out_3389584686144192417[101] = 0;
   out_3389584686144192417[102] = 0;
   out_3389584686144192417[103] = 0;
   out_3389584686144192417[104] = 0;
   out_3389584686144192417[105] = 0;
   out_3389584686144192417[106] = 0;
   out_3389584686144192417[107] = 0;
   out_3389584686144192417[108] = 1.0;
   out_3389584686144192417[109] = 0;
   out_3389584686144192417[110] = 0;
   out_3389584686144192417[111] = 0;
   out_3389584686144192417[112] = 0;
   out_3389584686144192417[113] = 0;
   out_3389584686144192417[114] = 0;
   out_3389584686144192417[115] = 0;
   out_3389584686144192417[116] = 0;
   out_3389584686144192417[117] = 0;
   out_3389584686144192417[118] = 0;
   out_3389584686144192417[119] = 0;
   out_3389584686144192417[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_9055272195158205822) {
   out_9055272195158205822[0] = dt*state[3] + state[0];
   out_9055272195158205822[1] = dt*state[4] + state[1];
   out_9055272195158205822[2] = dt*state[5] + state[2];
   out_9055272195158205822[3] = state[3];
   out_9055272195158205822[4] = state[4];
   out_9055272195158205822[5] = state[5];
   out_9055272195158205822[6] = dt*state[7] + state[6];
   out_9055272195158205822[7] = dt*state[8] + state[7];
   out_9055272195158205822[8] = state[8];
   out_9055272195158205822[9] = state[9];
   out_9055272195158205822[10] = state[10];
}
void F_fun(double *state, double dt, double *out_681731754219421529) {
   out_681731754219421529[0] = 1;
   out_681731754219421529[1] = 0;
   out_681731754219421529[2] = 0;
   out_681731754219421529[3] = dt;
   out_681731754219421529[4] = 0;
   out_681731754219421529[5] = 0;
   out_681731754219421529[6] = 0;
   out_681731754219421529[7] = 0;
   out_681731754219421529[8] = 0;
   out_681731754219421529[9] = 0;
   out_681731754219421529[10] = 0;
   out_681731754219421529[11] = 0;
   out_681731754219421529[12] = 1;
   out_681731754219421529[13] = 0;
   out_681731754219421529[14] = 0;
   out_681731754219421529[15] = dt;
   out_681731754219421529[16] = 0;
   out_681731754219421529[17] = 0;
   out_681731754219421529[18] = 0;
   out_681731754219421529[19] = 0;
   out_681731754219421529[20] = 0;
   out_681731754219421529[21] = 0;
   out_681731754219421529[22] = 0;
   out_681731754219421529[23] = 0;
   out_681731754219421529[24] = 1;
   out_681731754219421529[25] = 0;
   out_681731754219421529[26] = 0;
   out_681731754219421529[27] = dt;
   out_681731754219421529[28] = 0;
   out_681731754219421529[29] = 0;
   out_681731754219421529[30] = 0;
   out_681731754219421529[31] = 0;
   out_681731754219421529[32] = 0;
   out_681731754219421529[33] = 0;
   out_681731754219421529[34] = 0;
   out_681731754219421529[35] = 0;
   out_681731754219421529[36] = 1;
   out_681731754219421529[37] = 0;
   out_681731754219421529[38] = 0;
   out_681731754219421529[39] = 0;
   out_681731754219421529[40] = 0;
   out_681731754219421529[41] = 0;
   out_681731754219421529[42] = 0;
   out_681731754219421529[43] = 0;
   out_681731754219421529[44] = 0;
   out_681731754219421529[45] = 0;
   out_681731754219421529[46] = 0;
   out_681731754219421529[47] = 0;
   out_681731754219421529[48] = 1;
   out_681731754219421529[49] = 0;
   out_681731754219421529[50] = 0;
   out_681731754219421529[51] = 0;
   out_681731754219421529[52] = 0;
   out_681731754219421529[53] = 0;
   out_681731754219421529[54] = 0;
   out_681731754219421529[55] = 0;
   out_681731754219421529[56] = 0;
   out_681731754219421529[57] = 0;
   out_681731754219421529[58] = 0;
   out_681731754219421529[59] = 0;
   out_681731754219421529[60] = 1;
   out_681731754219421529[61] = 0;
   out_681731754219421529[62] = 0;
   out_681731754219421529[63] = 0;
   out_681731754219421529[64] = 0;
   out_681731754219421529[65] = 0;
   out_681731754219421529[66] = 0;
   out_681731754219421529[67] = 0;
   out_681731754219421529[68] = 0;
   out_681731754219421529[69] = 0;
   out_681731754219421529[70] = 0;
   out_681731754219421529[71] = 0;
   out_681731754219421529[72] = 1;
   out_681731754219421529[73] = dt;
   out_681731754219421529[74] = 0;
   out_681731754219421529[75] = 0;
   out_681731754219421529[76] = 0;
   out_681731754219421529[77] = 0;
   out_681731754219421529[78] = 0;
   out_681731754219421529[79] = 0;
   out_681731754219421529[80] = 0;
   out_681731754219421529[81] = 0;
   out_681731754219421529[82] = 0;
   out_681731754219421529[83] = 0;
   out_681731754219421529[84] = 1;
   out_681731754219421529[85] = dt;
   out_681731754219421529[86] = 0;
   out_681731754219421529[87] = 0;
   out_681731754219421529[88] = 0;
   out_681731754219421529[89] = 0;
   out_681731754219421529[90] = 0;
   out_681731754219421529[91] = 0;
   out_681731754219421529[92] = 0;
   out_681731754219421529[93] = 0;
   out_681731754219421529[94] = 0;
   out_681731754219421529[95] = 0;
   out_681731754219421529[96] = 1;
   out_681731754219421529[97] = 0;
   out_681731754219421529[98] = 0;
   out_681731754219421529[99] = 0;
   out_681731754219421529[100] = 0;
   out_681731754219421529[101] = 0;
   out_681731754219421529[102] = 0;
   out_681731754219421529[103] = 0;
   out_681731754219421529[104] = 0;
   out_681731754219421529[105] = 0;
   out_681731754219421529[106] = 0;
   out_681731754219421529[107] = 0;
   out_681731754219421529[108] = 1;
   out_681731754219421529[109] = 0;
   out_681731754219421529[110] = 0;
   out_681731754219421529[111] = 0;
   out_681731754219421529[112] = 0;
   out_681731754219421529[113] = 0;
   out_681731754219421529[114] = 0;
   out_681731754219421529[115] = 0;
   out_681731754219421529[116] = 0;
   out_681731754219421529[117] = 0;
   out_681731754219421529[118] = 0;
   out_681731754219421529[119] = 0;
   out_681731754219421529[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_1841729915697983286) {
   out_1841729915697983286[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8761235728672435821) {
   out_8761235728672435821[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8761235728672435821[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8761235728672435821[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8761235728672435821[3] = 0;
   out_8761235728672435821[4] = 0;
   out_8761235728672435821[5] = 0;
   out_8761235728672435821[6] = 1;
   out_8761235728672435821[7] = 0;
   out_8761235728672435821[8] = 0;
   out_8761235728672435821[9] = 0;
   out_8761235728672435821[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2924715623646539284) {
   out_2924715623646539284[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2535761186771767843) {
   out_2535761186771767843[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2535761186771767843[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2535761186771767843[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2535761186771767843[3] = 0;
   out_2535761186771767843[4] = 0;
   out_2535761186771767843[5] = 0;
   out_2535761186771767843[6] = 1;
   out_2535761186771767843[7] = 0;
   out_2535761186771767843[8] = 0;
   out_2535761186771767843[9] = 1;
   out_2535761186771767843[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1370555120028092744) {
   out_1370555120028092744[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5519331308243009628) {
   out_5519331308243009628[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[6] = 0;
   out_5519331308243009628[7] = 1;
   out_5519331308243009628[8] = 0;
   out_5519331308243009628[9] = 0;
   out_5519331308243009628[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1370555120028092744) {
   out_1370555120028092744[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5519331308243009628) {
   out_5519331308243009628[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5519331308243009628[6] = 0;
   out_5519331308243009628[7] = 1;
   out_5519331308243009628[8] = 0;
   out_5519331308243009628[9] = 0;
   out_5519331308243009628[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3903931754286709959) {
  err_fun(nom_x, delta_x, out_3903931754286709959);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6187082929283308233) {
  inv_err_fun(nom_x, true_x, out_6187082929283308233);
}
void gnss_H_mod_fun(double *state, double *out_3389584686144192417) {
  H_mod_fun(state, out_3389584686144192417);
}
void gnss_f_fun(double *state, double dt, double *out_9055272195158205822) {
  f_fun(state,  dt, out_9055272195158205822);
}
void gnss_F_fun(double *state, double dt, double *out_681731754219421529) {
  F_fun(state,  dt, out_681731754219421529);
}
void gnss_h_6(double *state, double *sat_pos, double *out_1841729915697983286) {
  h_6(state, sat_pos, out_1841729915697983286);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8761235728672435821) {
  H_6(state, sat_pos, out_8761235728672435821);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2924715623646539284) {
  h_20(state, sat_pos, out_2924715623646539284);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2535761186771767843) {
  H_20(state, sat_pos, out_2535761186771767843);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1370555120028092744) {
  h_7(state, sat_pos_vel, out_1370555120028092744);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5519331308243009628) {
  H_7(state, sat_pos_vel, out_5519331308243009628);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1370555120028092744) {
  h_21(state, sat_pos_vel, out_1370555120028092744);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5519331308243009628) {
  H_21(state, sat_pos_vel, out_5519331308243009628);
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
