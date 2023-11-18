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
void err_fun(double *nom_x, double *delta_x, double *out_2788921404904260909) {
   out_2788921404904260909[0] = delta_x[0] + nom_x[0];
   out_2788921404904260909[1] = delta_x[1] + nom_x[1];
   out_2788921404904260909[2] = delta_x[2] + nom_x[2];
   out_2788921404904260909[3] = delta_x[3] + nom_x[3];
   out_2788921404904260909[4] = delta_x[4] + nom_x[4];
   out_2788921404904260909[5] = delta_x[5] + nom_x[5];
   out_2788921404904260909[6] = delta_x[6] + nom_x[6];
   out_2788921404904260909[7] = delta_x[7] + nom_x[7];
   out_2788921404904260909[8] = delta_x[8] + nom_x[8];
   out_2788921404904260909[9] = delta_x[9] + nom_x[9];
   out_2788921404904260909[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6290624704054750684) {
   out_6290624704054750684[0] = -nom_x[0] + true_x[0];
   out_6290624704054750684[1] = -nom_x[1] + true_x[1];
   out_6290624704054750684[2] = -nom_x[2] + true_x[2];
   out_6290624704054750684[3] = -nom_x[3] + true_x[3];
   out_6290624704054750684[4] = -nom_x[4] + true_x[4];
   out_6290624704054750684[5] = -nom_x[5] + true_x[5];
   out_6290624704054750684[6] = -nom_x[6] + true_x[6];
   out_6290624704054750684[7] = -nom_x[7] + true_x[7];
   out_6290624704054750684[8] = -nom_x[8] + true_x[8];
   out_6290624704054750684[9] = -nom_x[9] + true_x[9];
   out_6290624704054750684[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4733939560862216063) {
   out_4733939560862216063[0] = 1.0;
   out_4733939560862216063[1] = 0;
   out_4733939560862216063[2] = 0;
   out_4733939560862216063[3] = 0;
   out_4733939560862216063[4] = 0;
   out_4733939560862216063[5] = 0;
   out_4733939560862216063[6] = 0;
   out_4733939560862216063[7] = 0;
   out_4733939560862216063[8] = 0;
   out_4733939560862216063[9] = 0;
   out_4733939560862216063[10] = 0;
   out_4733939560862216063[11] = 0;
   out_4733939560862216063[12] = 1.0;
   out_4733939560862216063[13] = 0;
   out_4733939560862216063[14] = 0;
   out_4733939560862216063[15] = 0;
   out_4733939560862216063[16] = 0;
   out_4733939560862216063[17] = 0;
   out_4733939560862216063[18] = 0;
   out_4733939560862216063[19] = 0;
   out_4733939560862216063[20] = 0;
   out_4733939560862216063[21] = 0;
   out_4733939560862216063[22] = 0;
   out_4733939560862216063[23] = 0;
   out_4733939560862216063[24] = 1.0;
   out_4733939560862216063[25] = 0;
   out_4733939560862216063[26] = 0;
   out_4733939560862216063[27] = 0;
   out_4733939560862216063[28] = 0;
   out_4733939560862216063[29] = 0;
   out_4733939560862216063[30] = 0;
   out_4733939560862216063[31] = 0;
   out_4733939560862216063[32] = 0;
   out_4733939560862216063[33] = 0;
   out_4733939560862216063[34] = 0;
   out_4733939560862216063[35] = 0;
   out_4733939560862216063[36] = 1.0;
   out_4733939560862216063[37] = 0;
   out_4733939560862216063[38] = 0;
   out_4733939560862216063[39] = 0;
   out_4733939560862216063[40] = 0;
   out_4733939560862216063[41] = 0;
   out_4733939560862216063[42] = 0;
   out_4733939560862216063[43] = 0;
   out_4733939560862216063[44] = 0;
   out_4733939560862216063[45] = 0;
   out_4733939560862216063[46] = 0;
   out_4733939560862216063[47] = 0;
   out_4733939560862216063[48] = 1.0;
   out_4733939560862216063[49] = 0;
   out_4733939560862216063[50] = 0;
   out_4733939560862216063[51] = 0;
   out_4733939560862216063[52] = 0;
   out_4733939560862216063[53] = 0;
   out_4733939560862216063[54] = 0;
   out_4733939560862216063[55] = 0;
   out_4733939560862216063[56] = 0;
   out_4733939560862216063[57] = 0;
   out_4733939560862216063[58] = 0;
   out_4733939560862216063[59] = 0;
   out_4733939560862216063[60] = 1.0;
   out_4733939560862216063[61] = 0;
   out_4733939560862216063[62] = 0;
   out_4733939560862216063[63] = 0;
   out_4733939560862216063[64] = 0;
   out_4733939560862216063[65] = 0;
   out_4733939560862216063[66] = 0;
   out_4733939560862216063[67] = 0;
   out_4733939560862216063[68] = 0;
   out_4733939560862216063[69] = 0;
   out_4733939560862216063[70] = 0;
   out_4733939560862216063[71] = 0;
   out_4733939560862216063[72] = 1.0;
   out_4733939560862216063[73] = 0;
   out_4733939560862216063[74] = 0;
   out_4733939560862216063[75] = 0;
   out_4733939560862216063[76] = 0;
   out_4733939560862216063[77] = 0;
   out_4733939560862216063[78] = 0;
   out_4733939560862216063[79] = 0;
   out_4733939560862216063[80] = 0;
   out_4733939560862216063[81] = 0;
   out_4733939560862216063[82] = 0;
   out_4733939560862216063[83] = 0;
   out_4733939560862216063[84] = 1.0;
   out_4733939560862216063[85] = 0;
   out_4733939560862216063[86] = 0;
   out_4733939560862216063[87] = 0;
   out_4733939560862216063[88] = 0;
   out_4733939560862216063[89] = 0;
   out_4733939560862216063[90] = 0;
   out_4733939560862216063[91] = 0;
   out_4733939560862216063[92] = 0;
   out_4733939560862216063[93] = 0;
   out_4733939560862216063[94] = 0;
   out_4733939560862216063[95] = 0;
   out_4733939560862216063[96] = 1.0;
   out_4733939560862216063[97] = 0;
   out_4733939560862216063[98] = 0;
   out_4733939560862216063[99] = 0;
   out_4733939560862216063[100] = 0;
   out_4733939560862216063[101] = 0;
   out_4733939560862216063[102] = 0;
   out_4733939560862216063[103] = 0;
   out_4733939560862216063[104] = 0;
   out_4733939560862216063[105] = 0;
   out_4733939560862216063[106] = 0;
   out_4733939560862216063[107] = 0;
   out_4733939560862216063[108] = 1.0;
   out_4733939560862216063[109] = 0;
   out_4733939560862216063[110] = 0;
   out_4733939560862216063[111] = 0;
   out_4733939560862216063[112] = 0;
   out_4733939560862216063[113] = 0;
   out_4733939560862216063[114] = 0;
   out_4733939560862216063[115] = 0;
   out_4733939560862216063[116] = 0;
   out_4733939560862216063[117] = 0;
   out_4733939560862216063[118] = 0;
   out_4733939560862216063[119] = 0;
   out_4733939560862216063[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3892250880040031115) {
   out_3892250880040031115[0] = dt*state[3] + state[0];
   out_3892250880040031115[1] = dt*state[4] + state[1];
   out_3892250880040031115[2] = dt*state[5] + state[2];
   out_3892250880040031115[3] = state[3];
   out_3892250880040031115[4] = state[4];
   out_3892250880040031115[5] = state[5];
   out_3892250880040031115[6] = dt*state[7] + state[6];
   out_3892250880040031115[7] = dt*state[8] + state[7];
   out_3892250880040031115[8] = state[8];
   out_3892250880040031115[9] = state[9];
   out_3892250880040031115[10] = state[10];
}
void F_fun(double *state, double dt, double *out_505639274279674341) {
   out_505639274279674341[0] = 1;
   out_505639274279674341[1] = 0;
   out_505639274279674341[2] = 0;
   out_505639274279674341[3] = dt;
   out_505639274279674341[4] = 0;
   out_505639274279674341[5] = 0;
   out_505639274279674341[6] = 0;
   out_505639274279674341[7] = 0;
   out_505639274279674341[8] = 0;
   out_505639274279674341[9] = 0;
   out_505639274279674341[10] = 0;
   out_505639274279674341[11] = 0;
   out_505639274279674341[12] = 1;
   out_505639274279674341[13] = 0;
   out_505639274279674341[14] = 0;
   out_505639274279674341[15] = dt;
   out_505639274279674341[16] = 0;
   out_505639274279674341[17] = 0;
   out_505639274279674341[18] = 0;
   out_505639274279674341[19] = 0;
   out_505639274279674341[20] = 0;
   out_505639274279674341[21] = 0;
   out_505639274279674341[22] = 0;
   out_505639274279674341[23] = 0;
   out_505639274279674341[24] = 1;
   out_505639274279674341[25] = 0;
   out_505639274279674341[26] = 0;
   out_505639274279674341[27] = dt;
   out_505639274279674341[28] = 0;
   out_505639274279674341[29] = 0;
   out_505639274279674341[30] = 0;
   out_505639274279674341[31] = 0;
   out_505639274279674341[32] = 0;
   out_505639274279674341[33] = 0;
   out_505639274279674341[34] = 0;
   out_505639274279674341[35] = 0;
   out_505639274279674341[36] = 1;
   out_505639274279674341[37] = 0;
   out_505639274279674341[38] = 0;
   out_505639274279674341[39] = 0;
   out_505639274279674341[40] = 0;
   out_505639274279674341[41] = 0;
   out_505639274279674341[42] = 0;
   out_505639274279674341[43] = 0;
   out_505639274279674341[44] = 0;
   out_505639274279674341[45] = 0;
   out_505639274279674341[46] = 0;
   out_505639274279674341[47] = 0;
   out_505639274279674341[48] = 1;
   out_505639274279674341[49] = 0;
   out_505639274279674341[50] = 0;
   out_505639274279674341[51] = 0;
   out_505639274279674341[52] = 0;
   out_505639274279674341[53] = 0;
   out_505639274279674341[54] = 0;
   out_505639274279674341[55] = 0;
   out_505639274279674341[56] = 0;
   out_505639274279674341[57] = 0;
   out_505639274279674341[58] = 0;
   out_505639274279674341[59] = 0;
   out_505639274279674341[60] = 1;
   out_505639274279674341[61] = 0;
   out_505639274279674341[62] = 0;
   out_505639274279674341[63] = 0;
   out_505639274279674341[64] = 0;
   out_505639274279674341[65] = 0;
   out_505639274279674341[66] = 0;
   out_505639274279674341[67] = 0;
   out_505639274279674341[68] = 0;
   out_505639274279674341[69] = 0;
   out_505639274279674341[70] = 0;
   out_505639274279674341[71] = 0;
   out_505639274279674341[72] = 1;
   out_505639274279674341[73] = dt;
   out_505639274279674341[74] = 0;
   out_505639274279674341[75] = 0;
   out_505639274279674341[76] = 0;
   out_505639274279674341[77] = 0;
   out_505639274279674341[78] = 0;
   out_505639274279674341[79] = 0;
   out_505639274279674341[80] = 0;
   out_505639274279674341[81] = 0;
   out_505639274279674341[82] = 0;
   out_505639274279674341[83] = 0;
   out_505639274279674341[84] = 1;
   out_505639274279674341[85] = dt;
   out_505639274279674341[86] = 0;
   out_505639274279674341[87] = 0;
   out_505639274279674341[88] = 0;
   out_505639274279674341[89] = 0;
   out_505639274279674341[90] = 0;
   out_505639274279674341[91] = 0;
   out_505639274279674341[92] = 0;
   out_505639274279674341[93] = 0;
   out_505639274279674341[94] = 0;
   out_505639274279674341[95] = 0;
   out_505639274279674341[96] = 1;
   out_505639274279674341[97] = 0;
   out_505639274279674341[98] = 0;
   out_505639274279674341[99] = 0;
   out_505639274279674341[100] = 0;
   out_505639274279674341[101] = 0;
   out_505639274279674341[102] = 0;
   out_505639274279674341[103] = 0;
   out_505639274279674341[104] = 0;
   out_505639274279674341[105] = 0;
   out_505639274279674341[106] = 0;
   out_505639274279674341[107] = 0;
   out_505639274279674341[108] = 1;
   out_505639274279674341[109] = 0;
   out_505639274279674341[110] = 0;
   out_505639274279674341[111] = 0;
   out_505639274279674341[112] = 0;
   out_505639274279674341[113] = 0;
   out_505639274279674341[114] = 0;
   out_505639274279674341[115] = 0;
   out_505639274279674341[116] = 0;
   out_505639274279674341[117] = 0;
   out_505639274279674341[118] = 0;
   out_505639274279674341[119] = 0;
   out_505639274279674341[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3534555043245932423) {
   out_3534555043245932423[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_5909739894810249108) {
   out_5909739894810249108[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5909739894810249108[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5909739894810249108[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5909739894810249108[3] = 0;
   out_5909739894810249108[4] = 0;
   out_5909739894810249108[5] = 0;
   out_5909739894810249108[6] = 1;
   out_5909739894810249108[7] = 0;
   out_5909739894810249108[8] = 0;
   out_5909739894810249108[9] = 0;
   out_5909739894810249108[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_1177420800671986150) {
   out_1177420800671986150[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2444663620237150446) {
   out_2444663620237150446[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2444663620237150446[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2444663620237150446[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2444663620237150446[3] = 0;
   out_2444663620237150446[4] = 0;
   out_2444663620237150446[5] = 0;
   out_2444663620237150446[6] = 1;
   out_2444663620237150446[7] = 0;
   out_2444663620237150446[8] = 0;
   out_2444663620237150446[9] = 1;
   out_2444663620237150446[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_391663488407230341) {
   out_391663488407230341[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5791096403187492688) {
   out_5791096403187492688[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[6] = 0;
   out_5791096403187492688[7] = 1;
   out_5791096403187492688[8] = 0;
   out_5791096403187492688[9] = 0;
   out_5791096403187492688[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_391663488407230341) {
   out_391663488407230341[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5791096403187492688) {
   out_5791096403187492688[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5791096403187492688[6] = 0;
   out_5791096403187492688[7] = 1;
   out_5791096403187492688[8] = 0;
   out_5791096403187492688[9] = 0;
   out_5791096403187492688[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2788921404904260909) {
  err_fun(nom_x, delta_x, out_2788921404904260909);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6290624704054750684) {
  inv_err_fun(nom_x, true_x, out_6290624704054750684);
}
void gnss_H_mod_fun(double *state, double *out_4733939560862216063) {
  H_mod_fun(state, out_4733939560862216063);
}
void gnss_f_fun(double *state, double dt, double *out_3892250880040031115) {
  f_fun(state,  dt, out_3892250880040031115);
}
void gnss_F_fun(double *state, double dt, double *out_505639274279674341) {
  F_fun(state,  dt, out_505639274279674341);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3534555043245932423) {
  h_6(state, sat_pos, out_3534555043245932423);
}
void gnss_H_6(double *state, double *sat_pos, double *out_5909739894810249108) {
  H_6(state, sat_pos, out_5909739894810249108);
}
void gnss_h_20(double *state, double *sat_pos, double *out_1177420800671986150) {
  h_20(state, sat_pos, out_1177420800671986150);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2444663620237150446) {
  H_20(state, sat_pos, out_2444663620237150446);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_391663488407230341) {
  h_7(state, sat_pos_vel, out_391663488407230341);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5791096403187492688) {
  H_7(state, sat_pos_vel, out_5791096403187492688);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_391663488407230341) {
  h_21(state, sat_pos_vel, out_391663488407230341);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5791096403187492688) {
  H_21(state, sat_pos_vel, out_5791096403187492688);
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
