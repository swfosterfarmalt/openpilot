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
void err_fun(double *nom_x, double *delta_x, double *out_284607730628362834) {
   out_284607730628362834[0] = delta_x[0] + nom_x[0];
   out_284607730628362834[1] = delta_x[1] + nom_x[1];
   out_284607730628362834[2] = delta_x[2] + nom_x[2];
   out_284607730628362834[3] = delta_x[3] + nom_x[3];
   out_284607730628362834[4] = delta_x[4] + nom_x[4];
   out_284607730628362834[5] = delta_x[5] + nom_x[5];
   out_284607730628362834[6] = delta_x[6] + nom_x[6];
   out_284607730628362834[7] = delta_x[7] + nom_x[7];
   out_284607730628362834[8] = delta_x[8] + nom_x[8];
   out_284607730628362834[9] = delta_x[9] + nom_x[9];
   out_284607730628362834[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7549528948775121586) {
   out_7549528948775121586[0] = -nom_x[0] + true_x[0];
   out_7549528948775121586[1] = -nom_x[1] + true_x[1];
   out_7549528948775121586[2] = -nom_x[2] + true_x[2];
   out_7549528948775121586[3] = -nom_x[3] + true_x[3];
   out_7549528948775121586[4] = -nom_x[4] + true_x[4];
   out_7549528948775121586[5] = -nom_x[5] + true_x[5];
   out_7549528948775121586[6] = -nom_x[6] + true_x[6];
   out_7549528948775121586[7] = -nom_x[7] + true_x[7];
   out_7549528948775121586[8] = -nom_x[8] + true_x[8];
   out_7549528948775121586[9] = -nom_x[9] + true_x[9];
   out_7549528948775121586[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_961775498725370832) {
   out_961775498725370832[0] = 1.0;
   out_961775498725370832[1] = 0;
   out_961775498725370832[2] = 0;
   out_961775498725370832[3] = 0;
   out_961775498725370832[4] = 0;
   out_961775498725370832[5] = 0;
   out_961775498725370832[6] = 0;
   out_961775498725370832[7] = 0;
   out_961775498725370832[8] = 0;
   out_961775498725370832[9] = 0;
   out_961775498725370832[10] = 0;
   out_961775498725370832[11] = 0;
   out_961775498725370832[12] = 1.0;
   out_961775498725370832[13] = 0;
   out_961775498725370832[14] = 0;
   out_961775498725370832[15] = 0;
   out_961775498725370832[16] = 0;
   out_961775498725370832[17] = 0;
   out_961775498725370832[18] = 0;
   out_961775498725370832[19] = 0;
   out_961775498725370832[20] = 0;
   out_961775498725370832[21] = 0;
   out_961775498725370832[22] = 0;
   out_961775498725370832[23] = 0;
   out_961775498725370832[24] = 1.0;
   out_961775498725370832[25] = 0;
   out_961775498725370832[26] = 0;
   out_961775498725370832[27] = 0;
   out_961775498725370832[28] = 0;
   out_961775498725370832[29] = 0;
   out_961775498725370832[30] = 0;
   out_961775498725370832[31] = 0;
   out_961775498725370832[32] = 0;
   out_961775498725370832[33] = 0;
   out_961775498725370832[34] = 0;
   out_961775498725370832[35] = 0;
   out_961775498725370832[36] = 1.0;
   out_961775498725370832[37] = 0;
   out_961775498725370832[38] = 0;
   out_961775498725370832[39] = 0;
   out_961775498725370832[40] = 0;
   out_961775498725370832[41] = 0;
   out_961775498725370832[42] = 0;
   out_961775498725370832[43] = 0;
   out_961775498725370832[44] = 0;
   out_961775498725370832[45] = 0;
   out_961775498725370832[46] = 0;
   out_961775498725370832[47] = 0;
   out_961775498725370832[48] = 1.0;
   out_961775498725370832[49] = 0;
   out_961775498725370832[50] = 0;
   out_961775498725370832[51] = 0;
   out_961775498725370832[52] = 0;
   out_961775498725370832[53] = 0;
   out_961775498725370832[54] = 0;
   out_961775498725370832[55] = 0;
   out_961775498725370832[56] = 0;
   out_961775498725370832[57] = 0;
   out_961775498725370832[58] = 0;
   out_961775498725370832[59] = 0;
   out_961775498725370832[60] = 1.0;
   out_961775498725370832[61] = 0;
   out_961775498725370832[62] = 0;
   out_961775498725370832[63] = 0;
   out_961775498725370832[64] = 0;
   out_961775498725370832[65] = 0;
   out_961775498725370832[66] = 0;
   out_961775498725370832[67] = 0;
   out_961775498725370832[68] = 0;
   out_961775498725370832[69] = 0;
   out_961775498725370832[70] = 0;
   out_961775498725370832[71] = 0;
   out_961775498725370832[72] = 1.0;
   out_961775498725370832[73] = 0;
   out_961775498725370832[74] = 0;
   out_961775498725370832[75] = 0;
   out_961775498725370832[76] = 0;
   out_961775498725370832[77] = 0;
   out_961775498725370832[78] = 0;
   out_961775498725370832[79] = 0;
   out_961775498725370832[80] = 0;
   out_961775498725370832[81] = 0;
   out_961775498725370832[82] = 0;
   out_961775498725370832[83] = 0;
   out_961775498725370832[84] = 1.0;
   out_961775498725370832[85] = 0;
   out_961775498725370832[86] = 0;
   out_961775498725370832[87] = 0;
   out_961775498725370832[88] = 0;
   out_961775498725370832[89] = 0;
   out_961775498725370832[90] = 0;
   out_961775498725370832[91] = 0;
   out_961775498725370832[92] = 0;
   out_961775498725370832[93] = 0;
   out_961775498725370832[94] = 0;
   out_961775498725370832[95] = 0;
   out_961775498725370832[96] = 1.0;
   out_961775498725370832[97] = 0;
   out_961775498725370832[98] = 0;
   out_961775498725370832[99] = 0;
   out_961775498725370832[100] = 0;
   out_961775498725370832[101] = 0;
   out_961775498725370832[102] = 0;
   out_961775498725370832[103] = 0;
   out_961775498725370832[104] = 0;
   out_961775498725370832[105] = 0;
   out_961775498725370832[106] = 0;
   out_961775498725370832[107] = 0;
   out_961775498725370832[108] = 1.0;
   out_961775498725370832[109] = 0;
   out_961775498725370832[110] = 0;
   out_961775498725370832[111] = 0;
   out_961775498725370832[112] = 0;
   out_961775498725370832[113] = 0;
   out_961775498725370832[114] = 0;
   out_961775498725370832[115] = 0;
   out_961775498725370832[116] = 0;
   out_961775498725370832[117] = 0;
   out_961775498725370832[118] = 0;
   out_961775498725370832[119] = 0;
   out_961775498725370832[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_543635440202532621) {
   out_543635440202532621[0] = dt*state[3] + state[0];
   out_543635440202532621[1] = dt*state[4] + state[1];
   out_543635440202532621[2] = dt*state[5] + state[2];
   out_543635440202532621[3] = state[3];
   out_543635440202532621[4] = state[4];
   out_543635440202532621[5] = state[5];
   out_543635440202532621[6] = dt*state[7] + state[6];
   out_543635440202532621[7] = dt*state[8] + state[7];
   out_543635440202532621[8] = state[8];
   out_543635440202532621[9] = state[9];
   out_543635440202532621[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2295083499474340999) {
   out_2295083499474340999[0] = 1;
   out_2295083499474340999[1] = 0;
   out_2295083499474340999[2] = 0;
   out_2295083499474340999[3] = dt;
   out_2295083499474340999[4] = 0;
   out_2295083499474340999[5] = 0;
   out_2295083499474340999[6] = 0;
   out_2295083499474340999[7] = 0;
   out_2295083499474340999[8] = 0;
   out_2295083499474340999[9] = 0;
   out_2295083499474340999[10] = 0;
   out_2295083499474340999[11] = 0;
   out_2295083499474340999[12] = 1;
   out_2295083499474340999[13] = 0;
   out_2295083499474340999[14] = 0;
   out_2295083499474340999[15] = dt;
   out_2295083499474340999[16] = 0;
   out_2295083499474340999[17] = 0;
   out_2295083499474340999[18] = 0;
   out_2295083499474340999[19] = 0;
   out_2295083499474340999[20] = 0;
   out_2295083499474340999[21] = 0;
   out_2295083499474340999[22] = 0;
   out_2295083499474340999[23] = 0;
   out_2295083499474340999[24] = 1;
   out_2295083499474340999[25] = 0;
   out_2295083499474340999[26] = 0;
   out_2295083499474340999[27] = dt;
   out_2295083499474340999[28] = 0;
   out_2295083499474340999[29] = 0;
   out_2295083499474340999[30] = 0;
   out_2295083499474340999[31] = 0;
   out_2295083499474340999[32] = 0;
   out_2295083499474340999[33] = 0;
   out_2295083499474340999[34] = 0;
   out_2295083499474340999[35] = 0;
   out_2295083499474340999[36] = 1;
   out_2295083499474340999[37] = 0;
   out_2295083499474340999[38] = 0;
   out_2295083499474340999[39] = 0;
   out_2295083499474340999[40] = 0;
   out_2295083499474340999[41] = 0;
   out_2295083499474340999[42] = 0;
   out_2295083499474340999[43] = 0;
   out_2295083499474340999[44] = 0;
   out_2295083499474340999[45] = 0;
   out_2295083499474340999[46] = 0;
   out_2295083499474340999[47] = 0;
   out_2295083499474340999[48] = 1;
   out_2295083499474340999[49] = 0;
   out_2295083499474340999[50] = 0;
   out_2295083499474340999[51] = 0;
   out_2295083499474340999[52] = 0;
   out_2295083499474340999[53] = 0;
   out_2295083499474340999[54] = 0;
   out_2295083499474340999[55] = 0;
   out_2295083499474340999[56] = 0;
   out_2295083499474340999[57] = 0;
   out_2295083499474340999[58] = 0;
   out_2295083499474340999[59] = 0;
   out_2295083499474340999[60] = 1;
   out_2295083499474340999[61] = 0;
   out_2295083499474340999[62] = 0;
   out_2295083499474340999[63] = 0;
   out_2295083499474340999[64] = 0;
   out_2295083499474340999[65] = 0;
   out_2295083499474340999[66] = 0;
   out_2295083499474340999[67] = 0;
   out_2295083499474340999[68] = 0;
   out_2295083499474340999[69] = 0;
   out_2295083499474340999[70] = 0;
   out_2295083499474340999[71] = 0;
   out_2295083499474340999[72] = 1;
   out_2295083499474340999[73] = dt;
   out_2295083499474340999[74] = 0;
   out_2295083499474340999[75] = 0;
   out_2295083499474340999[76] = 0;
   out_2295083499474340999[77] = 0;
   out_2295083499474340999[78] = 0;
   out_2295083499474340999[79] = 0;
   out_2295083499474340999[80] = 0;
   out_2295083499474340999[81] = 0;
   out_2295083499474340999[82] = 0;
   out_2295083499474340999[83] = 0;
   out_2295083499474340999[84] = 1;
   out_2295083499474340999[85] = dt;
   out_2295083499474340999[86] = 0;
   out_2295083499474340999[87] = 0;
   out_2295083499474340999[88] = 0;
   out_2295083499474340999[89] = 0;
   out_2295083499474340999[90] = 0;
   out_2295083499474340999[91] = 0;
   out_2295083499474340999[92] = 0;
   out_2295083499474340999[93] = 0;
   out_2295083499474340999[94] = 0;
   out_2295083499474340999[95] = 0;
   out_2295083499474340999[96] = 1;
   out_2295083499474340999[97] = 0;
   out_2295083499474340999[98] = 0;
   out_2295083499474340999[99] = 0;
   out_2295083499474340999[100] = 0;
   out_2295083499474340999[101] = 0;
   out_2295083499474340999[102] = 0;
   out_2295083499474340999[103] = 0;
   out_2295083499474340999[104] = 0;
   out_2295083499474340999[105] = 0;
   out_2295083499474340999[106] = 0;
   out_2295083499474340999[107] = 0;
   out_2295083499474340999[108] = 1;
   out_2295083499474340999[109] = 0;
   out_2295083499474340999[110] = 0;
   out_2295083499474340999[111] = 0;
   out_2295083499474340999[112] = 0;
   out_2295083499474340999[113] = 0;
   out_2295083499474340999[114] = 0;
   out_2295083499474340999[115] = 0;
   out_2295083499474340999[116] = 0;
   out_2295083499474340999[117] = 0;
   out_2295083499474340999[118] = 0;
   out_2295083499474340999[119] = 0;
   out_2295083499474340999[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5929645947376494773) {
   out_5929645947376494773[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_520461601455328205) {
   out_520461601455328205[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_520461601455328205[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_520461601455328205[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_520461601455328205[3] = 0;
   out_520461601455328205[4] = 0;
   out_520461601455328205[5] = 0;
   out_520461601455328205[6] = 1;
   out_520461601455328205[7] = 0;
   out_520461601455328205[8] = 0;
   out_520461601455328205[9] = 0;
   out_520461601455328205[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7758809767259933663) {
   out_7758809767259933663[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_898721150884741153) {
   out_898721150884741153[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_898721150884741153[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_898721150884741153[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_898721150884741153[3] = 0;
   out_898721150884741153[4] = 0;
   out_898721150884741153[5] = 0;
   out_898721150884741153[6] = 1;
   out_898721150884741153[7] = 0;
   out_898721150884741153[8] = 0;
   out_898721150884741153[9] = 1;
   out_898721150884741153[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_219142218163687136) {
   out_219142218163687136[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7272996389187845126) {
   out_7272996389187845126[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[6] = 0;
   out_7272996389187845126[7] = 1;
   out_7272996389187845126[8] = 0;
   out_7272996389187845126[9] = 0;
   out_7272996389187845126[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_219142218163687136) {
   out_219142218163687136[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7272996389187845126) {
   out_7272996389187845126[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7272996389187845126[6] = 0;
   out_7272996389187845126[7] = 1;
   out_7272996389187845126[8] = 0;
   out_7272996389187845126[9] = 0;
   out_7272996389187845126[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_284607730628362834) {
  err_fun(nom_x, delta_x, out_284607730628362834);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7549528948775121586) {
  inv_err_fun(nom_x, true_x, out_7549528948775121586);
}
void gnss_H_mod_fun(double *state, double *out_961775498725370832) {
  H_mod_fun(state, out_961775498725370832);
}
void gnss_f_fun(double *state, double dt, double *out_543635440202532621) {
  f_fun(state,  dt, out_543635440202532621);
}
void gnss_F_fun(double *state, double dt, double *out_2295083499474340999) {
  F_fun(state,  dt, out_2295083499474340999);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5929645947376494773) {
  h_6(state, sat_pos, out_5929645947376494773);
}
void gnss_H_6(double *state, double *sat_pos, double *out_520461601455328205) {
  H_6(state, sat_pos, out_520461601455328205);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7758809767259933663) {
  h_20(state, sat_pos, out_7758809767259933663);
}
void gnss_H_20(double *state, double *sat_pos, double *out_898721150884741153) {
  H_20(state, sat_pos, out_898721150884741153);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_219142218163687136) {
  h_7(state, sat_pos_vel, out_219142218163687136);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7272996389187845126) {
  H_7(state, sat_pos_vel, out_7272996389187845126);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_219142218163687136) {
  h_21(state, sat_pos_vel, out_219142218163687136);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7272996389187845126) {
  H_21(state, sat_pos_vel, out_7272996389187845126);
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
