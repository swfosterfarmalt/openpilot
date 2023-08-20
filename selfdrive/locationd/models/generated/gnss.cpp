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
void err_fun(double *nom_x, double *delta_x, double *out_7494899235073949606) {
   out_7494899235073949606[0] = delta_x[0] + nom_x[0];
   out_7494899235073949606[1] = delta_x[1] + nom_x[1];
   out_7494899235073949606[2] = delta_x[2] + nom_x[2];
   out_7494899235073949606[3] = delta_x[3] + nom_x[3];
   out_7494899235073949606[4] = delta_x[4] + nom_x[4];
   out_7494899235073949606[5] = delta_x[5] + nom_x[5];
   out_7494899235073949606[6] = delta_x[6] + nom_x[6];
   out_7494899235073949606[7] = delta_x[7] + nom_x[7];
   out_7494899235073949606[8] = delta_x[8] + nom_x[8];
   out_7494899235073949606[9] = delta_x[9] + nom_x[9];
   out_7494899235073949606[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_608002279011974407) {
   out_608002279011974407[0] = -nom_x[0] + true_x[0];
   out_608002279011974407[1] = -nom_x[1] + true_x[1];
   out_608002279011974407[2] = -nom_x[2] + true_x[2];
   out_608002279011974407[3] = -nom_x[3] + true_x[3];
   out_608002279011974407[4] = -nom_x[4] + true_x[4];
   out_608002279011974407[5] = -nom_x[5] + true_x[5];
   out_608002279011974407[6] = -nom_x[6] + true_x[6];
   out_608002279011974407[7] = -nom_x[7] + true_x[7];
   out_608002279011974407[8] = -nom_x[8] + true_x[8];
   out_608002279011974407[9] = -nom_x[9] + true_x[9];
   out_608002279011974407[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7002084543314776290) {
   out_7002084543314776290[0] = 1.0;
   out_7002084543314776290[1] = 0;
   out_7002084543314776290[2] = 0;
   out_7002084543314776290[3] = 0;
   out_7002084543314776290[4] = 0;
   out_7002084543314776290[5] = 0;
   out_7002084543314776290[6] = 0;
   out_7002084543314776290[7] = 0;
   out_7002084543314776290[8] = 0;
   out_7002084543314776290[9] = 0;
   out_7002084543314776290[10] = 0;
   out_7002084543314776290[11] = 0;
   out_7002084543314776290[12] = 1.0;
   out_7002084543314776290[13] = 0;
   out_7002084543314776290[14] = 0;
   out_7002084543314776290[15] = 0;
   out_7002084543314776290[16] = 0;
   out_7002084543314776290[17] = 0;
   out_7002084543314776290[18] = 0;
   out_7002084543314776290[19] = 0;
   out_7002084543314776290[20] = 0;
   out_7002084543314776290[21] = 0;
   out_7002084543314776290[22] = 0;
   out_7002084543314776290[23] = 0;
   out_7002084543314776290[24] = 1.0;
   out_7002084543314776290[25] = 0;
   out_7002084543314776290[26] = 0;
   out_7002084543314776290[27] = 0;
   out_7002084543314776290[28] = 0;
   out_7002084543314776290[29] = 0;
   out_7002084543314776290[30] = 0;
   out_7002084543314776290[31] = 0;
   out_7002084543314776290[32] = 0;
   out_7002084543314776290[33] = 0;
   out_7002084543314776290[34] = 0;
   out_7002084543314776290[35] = 0;
   out_7002084543314776290[36] = 1.0;
   out_7002084543314776290[37] = 0;
   out_7002084543314776290[38] = 0;
   out_7002084543314776290[39] = 0;
   out_7002084543314776290[40] = 0;
   out_7002084543314776290[41] = 0;
   out_7002084543314776290[42] = 0;
   out_7002084543314776290[43] = 0;
   out_7002084543314776290[44] = 0;
   out_7002084543314776290[45] = 0;
   out_7002084543314776290[46] = 0;
   out_7002084543314776290[47] = 0;
   out_7002084543314776290[48] = 1.0;
   out_7002084543314776290[49] = 0;
   out_7002084543314776290[50] = 0;
   out_7002084543314776290[51] = 0;
   out_7002084543314776290[52] = 0;
   out_7002084543314776290[53] = 0;
   out_7002084543314776290[54] = 0;
   out_7002084543314776290[55] = 0;
   out_7002084543314776290[56] = 0;
   out_7002084543314776290[57] = 0;
   out_7002084543314776290[58] = 0;
   out_7002084543314776290[59] = 0;
   out_7002084543314776290[60] = 1.0;
   out_7002084543314776290[61] = 0;
   out_7002084543314776290[62] = 0;
   out_7002084543314776290[63] = 0;
   out_7002084543314776290[64] = 0;
   out_7002084543314776290[65] = 0;
   out_7002084543314776290[66] = 0;
   out_7002084543314776290[67] = 0;
   out_7002084543314776290[68] = 0;
   out_7002084543314776290[69] = 0;
   out_7002084543314776290[70] = 0;
   out_7002084543314776290[71] = 0;
   out_7002084543314776290[72] = 1.0;
   out_7002084543314776290[73] = 0;
   out_7002084543314776290[74] = 0;
   out_7002084543314776290[75] = 0;
   out_7002084543314776290[76] = 0;
   out_7002084543314776290[77] = 0;
   out_7002084543314776290[78] = 0;
   out_7002084543314776290[79] = 0;
   out_7002084543314776290[80] = 0;
   out_7002084543314776290[81] = 0;
   out_7002084543314776290[82] = 0;
   out_7002084543314776290[83] = 0;
   out_7002084543314776290[84] = 1.0;
   out_7002084543314776290[85] = 0;
   out_7002084543314776290[86] = 0;
   out_7002084543314776290[87] = 0;
   out_7002084543314776290[88] = 0;
   out_7002084543314776290[89] = 0;
   out_7002084543314776290[90] = 0;
   out_7002084543314776290[91] = 0;
   out_7002084543314776290[92] = 0;
   out_7002084543314776290[93] = 0;
   out_7002084543314776290[94] = 0;
   out_7002084543314776290[95] = 0;
   out_7002084543314776290[96] = 1.0;
   out_7002084543314776290[97] = 0;
   out_7002084543314776290[98] = 0;
   out_7002084543314776290[99] = 0;
   out_7002084543314776290[100] = 0;
   out_7002084543314776290[101] = 0;
   out_7002084543314776290[102] = 0;
   out_7002084543314776290[103] = 0;
   out_7002084543314776290[104] = 0;
   out_7002084543314776290[105] = 0;
   out_7002084543314776290[106] = 0;
   out_7002084543314776290[107] = 0;
   out_7002084543314776290[108] = 1.0;
   out_7002084543314776290[109] = 0;
   out_7002084543314776290[110] = 0;
   out_7002084543314776290[111] = 0;
   out_7002084543314776290[112] = 0;
   out_7002084543314776290[113] = 0;
   out_7002084543314776290[114] = 0;
   out_7002084543314776290[115] = 0;
   out_7002084543314776290[116] = 0;
   out_7002084543314776290[117] = 0;
   out_7002084543314776290[118] = 0;
   out_7002084543314776290[119] = 0;
   out_7002084543314776290[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7089057631738940520) {
   out_7089057631738940520[0] = dt*state[3] + state[0];
   out_7089057631738940520[1] = dt*state[4] + state[1];
   out_7089057631738940520[2] = dt*state[5] + state[2];
   out_7089057631738940520[3] = state[3];
   out_7089057631738940520[4] = state[4];
   out_7089057631738940520[5] = state[5];
   out_7089057631738940520[6] = dt*state[7] + state[6];
   out_7089057631738940520[7] = dt*state[8] + state[7];
   out_7089057631738940520[8] = state[8];
   out_7089057631738940520[9] = state[9];
   out_7089057631738940520[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4393676974186888507) {
   out_4393676974186888507[0] = 1;
   out_4393676974186888507[1] = 0;
   out_4393676974186888507[2] = 0;
   out_4393676974186888507[3] = dt;
   out_4393676974186888507[4] = 0;
   out_4393676974186888507[5] = 0;
   out_4393676974186888507[6] = 0;
   out_4393676974186888507[7] = 0;
   out_4393676974186888507[8] = 0;
   out_4393676974186888507[9] = 0;
   out_4393676974186888507[10] = 0;
   out_4393676974186888507[11] = 0;
   out_4393676974186888507[12] = 1;
   out_4393676974186888507[13] = 0;
   out_4393676974186888507[14] = 0;
   out_4393676974186888507[15] = dt;
   out_4393676974186888507[16] = 0;
   out_4393676974186888507[17] = 0;
   out_4393676974186888507[18] = 0;
   out_4393676974186888507[19] = 0;
   out_4393676974186888507[20] = 0;
   out_4393676974186888507[21] = 0;
   out_4393676974186888507[22] = 0;
   out_4393676974186888507[23] = 0;
   out_4393676974186888507[24] = 1;
   out_4393676974186888507[25] = 0;
   out_4393676974186888507[26] = 0;
   out_4393676974186888507[27] = dt;
   out_4393676974186888507[28] = 0;
   out_4393676974186888507[29] = 0;
   out_4393676974186888507[30] = 0;
   out_4393676974186888507[31] = 0;
   out_4393676974186888507[32] = 0;
   out_4393676974186888507[33] = 0;
   out_4393676974186888507[34] = 0;
   out_4393676974186888507[35] = 0;
   out_4393676974186888507[36] = 1;
   out_4393676974186888507[37] = 0;
   out_4393676974186888507[38] = 0;
   out_4393676974186888507[39] = 0;
   out_4393676974186888507[40] = 0;
   out_4393676974186888507[41] = 0;
   out_4393676974186888507[42] = 0;
   out_4393676974186888507[43] = 0;
   out_4393676974186888507[44] = 0;
   out_4393676974186888507[45] = 0;
   out_4393676974186888507[46] = 0;
   out_4393676974186888507[47] = 0;
   out_4393676974186888507[48] = 1;
   out_4393676974186888507[49] = 0;
   out_4393676974186888507[50] = 0;
   out_4393676974186888507[51] = 0;
   out_4393676974186888507[52] = 0;
   out_4393676974186888507[53] = 0;
   out_4393676974186888507[54] = 0;
   out_4393676974186888507[55] = 0;
   out_4393676974186888507[56] = 0;
   out_4393676974186888507[57] = 0;
   out_4393676974186888507[58] = 0;
   out_4393676974186888507[59] = 0;
   out_4393676974186888507[60] = 1;
   out_4393676974186888507[61] = 0;
   out_4393676974186888507[62] = 0;
   out_4393676974186888507[63] = 0;
   out_4393676974186888507[64] = 0;
   out_4393676974186888507[65] = 0;
   out_4393676974186888507[66] = 0;
   out_4393676974186888507[67] = 0;
   out_4393676974186888507[68] = 0;
   out_4393676974186888507[69] = 0;
   out_4393676974186888507[70] = 0;
   out_4393676974186888507[71] = 0;
   out_4393676974186888507[72] = 1;
   out_4393676974186888507[73] = dt;
   out_4393676974186888507[74] = 0;
   out_4393676974186888507[75] = 0;
   out_4393676974186888507[76] = 0;
   out_4393676974186888507[77] = 0;
   out_4393676974186888507[78] = 0;
   out_4393676974186888507[79] = 0;
   out_4393676974186888507[80] = 0;
   out_4393676974186888507[81] = 0;
   out_4393676974186888507[82] = 0;
   out_4393676974186888507[83] = 0;
   out_4393676974186888507[84] = 1;
   out_4393676974186888507[85] = dt;
   out_4393676974186888507[86] = 0;
   out_4393676974186888507[87] = 0;
   out_4393676974186888507[88] = 0;
   out_4393676974186888507[89] = 0;
   out_4393676974186888507[90] = 0;
   out_4393676974186888507[91] = 0;
   out_4393676974186888507[92] = 0;
   out_4393676974186888507[93] = 0;
   out_4393676974186888507[94] = 0;
   out_4393676974186888507[95] = 0;
   out_4393676974186888507[96] = 1;
   out_4393676974186888507[97] = 0;
   out_4393676974186888507[98] = 0;
   out_4393676974186888507[99] = 0;
   out_4393676974186888507[100] = 0;
   out_4393676974186888507[101] = 0;
   out_4393676974186888507[102] = 0;
   out_4393676974186888507[103] = 0;
   out_4393676974186888507[104] = 0;
   out_4393676974186888507[105] = 0;
   out_4393676974186888507[106] = 0;
   out_4393676974186888507[107] = 0;
   out_4393676974186888507[108] = 1;
   out_4393676974186888507[109] = 0;
   out_4393676974186888507[110] = 0;
   out_4393676974186888507[111] = 0;
   out_4393676974186888507[112] = 0;
   out_4393676974186888507[113] = 0;
   out_4393676974186888507[114] = 0;
   out_4393676974186888507[115] = 0;
   out_4393676974186888507[116] = 0;
   out_4393676974186888507[117] = 0;
   out_4393676974186888507[118] = 0;
   out_4393676974186888507[119] = 0;
   out_4393676974186888507[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3305816208013416001) {
   out_3305816208013416001[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4780980724345831917) {
   out_4780980724345831917[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4780980724345831917[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4780980724345831917[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4780980724345831917[3] = 0;
   out_4780980724345831917[4] = 0;
   out_4780980724345831917[5] = 0;
   out_4780980724345831917[6] = 1;
   out_4780980724345831917[7] = 0;
   out_4780980724345831917[8] = 0;
   out_4780980724345831917[9] = 0;
   out_4780980724345831917[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2426811405452364869) {
   out_2426811405452364869[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8252454322377627110) {
   out_8252454322377627110[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8252454322377627110[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8252454322377627110[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8252454322377627110[3] = 0;
   out_8252454322377627110[4] = 0;
   out_8252454322377627110[5] = 0;
   out_8252454322377627110[6] = 1;
   out_8252454322377627110[7] = 0;
   out_8252454322377627110[8] = 0;
   out_8252454322377627110[9] = 1;
   out_8252454322377627110[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8737519804286952927) {
   out_8737519804286952927[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3207489715113657734) {
   out_3207489715113657734[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[6] = 0;
   out_3207489715113657734[7] = 1;
   out_3207489715113657734[8] = 0;
   out_3207489715113657734[9] = 0;
   out_3207489715113657734[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8737519804286952927) {
   out_8737519804286952927[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3207489715113657734) {
   out_3207489715113657734[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3207489715113657734[6] = 0;
   out_3207489715113657734[7] = 1;
   out_3207489715113657734[8] = 0;
   out_3207489715113657734[9] = 0;
   out_3207489715113657734[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7494899235073949606) {
  err_fun(nom_x, delta_x, out_7494899235073949606);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_608002279011974407) {
  inv_err_fun(nom_x, true_x, out_608002279011974407);
}
void gnss_H_mod_fun(double *state, double *out_7002084543314776290) {
  H_mod_fun(state, out_7002084543314776290);
}
void gnss_f_fun(double *state, double dt, double *out_7089057631738940520) {
  f_fun(state,  dt, out_7089057631738940520);
}
void gnss_F_fun(double *state, double dt, double *out_4393676974186888507) {
  F_fun(state,  dt, out_4393676974186888507);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3305816208013416001) {
  h_6(state, sat_pos, out_3305816208013416001);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4780980724345831917) {
  H_6(state, sat_pos, out_4780980724345831917);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2426811405452364869) {
  h_20(state, sat_pos, out_2426811405452364869);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8252454322377627110) {
  H_20(state, sat_pos, out_8252454322377627110);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8737519804286952927) {
  h_7(state, sat_pos_vel, out_8737519804286952927);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3207489715113657734) {
  H_7(state, sat_pos_vel, out_3207489715113657734);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8737519804286952927) {
  h_21(state, sat_pos_vel, out_8737519804286952927);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3207489715113657734) {
  H_21(state, sat_pos_vel, out_3207489715113657734);
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
