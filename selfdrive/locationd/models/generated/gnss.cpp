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
void err_fun(double *nom_x, double *delta_x, double *out_3463381613600226321) {
   out_3463381613600226321[0] = delta_x[0] + nom_x[0];
   out_3463381613600226321[1] = delta_x[1] + nom_x[1];
   out_3463381613600226321[2] = delta_x[2] + nom_x[2];
   out_3463381613600226321[3] = delta_x[3] + nom_x[3];
   out_3463381613600226321[4] = delta_x[4] + nom_x[4];
   out_3463381613600226321[5] = delta_x[5] + nom_x[5];
   out_3463381613600226321[6] = delta_x[6] + nom_x[6];
   out_3463381613600226321[7] = delta_x[7] + nom_x[7];
   out_3463381613600226321[8] = delta_x[8] + nom_x[8];
   out_3463381613600226321[9] = delta_x[9] + nom_x[9];
   out_3463381613600226321[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4676674826669643480) {
   out_4676674826669643480[0] = -nom_x[0] + true_x[0];
   out_4676674826669643480[1] = -nom_x[1] + true_x[1];
   out_4676674826669643480[2] = -nom_x[2] + true_x[2];
   out_4676674826669643480[3] = -nom_x[3] + true_x[3];
   out_4676674826669643480[4] = -nom_x[4] + true_x[4];
   out_4676674826669643480[5] = -nom_x[5] + true_x[5];
   out_4676674826669643480[6] = -nom_x[6] + true_x[6];
   out_4676674826669643480[7] = -nom_x[7] + true_x[7];
   out_4676674826669643480[8] = -nom_x[8] + true_x[8];
   out_4676674826669643480[9] = -nom_x[9] + true_x[9];
   out_4676674826669643480[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4538830618860333370) {
   out_4538830618860333370[0] = 1.0;
   out_4538830618860333370[1] = 0;
   out_4538830618860333370[2] = 0;
   out_4538830618860333370[3] = 0;
   out_4538830618860333370[4] = 0;
   out_4538830618860333370[5] = 0;
   out_4538830618860333370[6] = 0;
   out_4538830618860333370[7] = 0;
   out_4538830618860333370[8] = 0;
   out_4538830618860333370[9] = 0;
   out_4538830618860333370[10] = 0;
   out_4538830618860333370[11] = 0;
   out_4538830618860333370[12] = 1.0;
   out_4538830618860333370[13] = 0;
   out_4538830618860333370[14] = 0;
   out_4538830618860333370[15] = 0;
   out_4538830618860333370[16] = 0;
   out_4538830618860333370[17] = 0;
   out_4538830618860333370[18] = 0;
   out_4538830618860333370[19] = 0;
   out_4538830618860333370[20] = 0;
   out_4538830618860333370[21] = 0;
   out_4538830618860333370[22] = 0;
   out_4538830618860333370[23] = 0;
   out_4538830618860333370[24] = 1.0;
   out_4538830618860333370[25] = 0;
   out_4538830618860333370[26] = 0;
   out_4538830618860333370[27] = 0;
   out_4538830618860333370[28] = 0;
   out_4538830618860333370[29] = 0;
   out_4538830618860333370[30] = 0;
   out_4538830618860333370[31] = 0;
   out_4538830618860333370[32] = 0;
   out_4538830618860333370[33] = 0;
   out_4538830618860333370[34] = 0;
   out_4538830618860333370[35] = 0;
   out_4538830618860333370[36] = 1.0;
   out_4538830618860333370[37] = 0;
   out_4538830618860333370[38] = 0;
   out_4538830618860333370[39] = 0;
   out_4538830618860333370[40] = 0;
   out_4538830618860333370[41] = 0;
   out_4538830618860333370[42] = 0;
   out_4538830618860333370[43] = 0;
   out_4538830618860333370[44] = 0;
   out_4538830618860333370[45] = 0;
   out_4538830618860333370[46] = 0;
   out_4538830618860333370[47] = 0;
   out_4538830618860333370[48] = 1.0;
   out_4538830618860333370[49] = 0;
   out_4538830618860333370[50] = 0;
   out_4538830618860333370[51] = 0;
   out_4538830618860333370[52] = 0;
   out_4538830618860333370[53] = 0;
   out_4538830618860333370[54] = 0;
   out_4538830618860333370[55] = 0;
   out_4538830618860333370[56] = 0;
   out_4538830618860333370[57] = 0;
   out_4538830618860333370[58] = 0;
   out_4538830618860333370[59] = 0;
   out_4538830618860333370[60] = 1.0;
   out_4538830618860333370[61] = 0;
   out_4538830618860333370[62] = 0;
   out_4538830618860333370[63] = 0;
   out_4538830618860333370[64] = 0;
   out_4538830618860333370[65] = 0;
   out_4538830618860333370[66] = 0;
   out_4538830618860333370[67] = 0;
   out_4538830618860333370[68] = 0;
   out_4538830618860333370[69] = 0;
   out_4538830618860333370[70] = 0;
   out_4538830618860333370[71] = 0;
   out_4538830618860333370[72] = 1.0;
   out_4538830618860333370[73] = 0;
   out_4538830618860333370[74] = 0;
   out_4538830618860333370[75] = 0;
   out_4538830618860333370[76] = 0;
   out_4538830618860333370[77] = 0;
   out_4538830618860333370[78] = 0;
   out_4538830618860333370[79] = 0;
   out_4538830618860333370[80] = 0;
   out_4538830618860333370[81] = 0;
   out_4538830618860333370[82] = 0;
   out_4538830618860333370[83] = 0;
   out_4538830618860333370[84] = 1.0;
   out_4538830618860333370[85] = 0;
   out_4538830618860333370[86] = 0;
   out_4538830618860333370[87] = 0;
   out_4538830618860333370[88] = 0;
   out_4538830618860333370[89] = 0;
   out_4538830618860333370[90] = 0;
   out_4538830618860333370[91] = 0;
   out_4538830618860333370[92] = 0;
   out_4538830618860333370[93] = 0;
   out_4538830618860333370[94] = 0;
   out_4538830618860333370[95] = 0;
   out_4538830618860333370[96] = 1.0;
   out_4538830618860333370[97] = 0;
   out_4538830618860333370[98] = 0;
   out_4538830618860333370[99] = 0;
   out_4538830618860333370[100] = 0;
   out_4538830618860333370[101] = 0;
   out_4538830618860333370[102] = 0;
   out_4538830618860333370[103] = 0;
   out_4538830618860333370[104] = 0;
   out_4538830618860333370[105] = 0;
   out_4538830618860333370[106] = 0;
   out_4538830618860333370[107] = 0;
   out_4538830618860333370[108] = 1.0;
   out_4538830618860333370[109] = 0;
   out_4538830618860333370[110] = 0;
   out_4538830618860333370[111] = 0;
   out_4538830618860333370[112] = 0;
   out_4538830618860333370[113] = 0;
   out_4538830618860333370[114] = 0;
   out_4538830618860333370[115] = 0;
   out_4538830618860333370[116] = 0;
   out_4538830618860333370[117] = 0;
   out_4538830618860333370[118] = 0;
   out_4538830618860333370[119] = 0;
   out_4538830618860333370[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1811539429430141585) {
   out_1811539429430141585[0] = dt*state[3] + state[0];
   out_1811539429430141585[1] = dt*state[4] + state[1];
   out_1811539429430141585[2] = dt*state[5] + state[2];
   out_1811539429430141585[3] = state[3];
   out_1811539429430141585[4] = state[4];
   out_1811539429430141585[5] = state[5];
   out_1811539429430141585[6] = dt*state[7] + state[6];
   out_1811539429430141585[7] = dt*state[8] + state[7];
   out_1811539429430141585[8] = state[8];
   out_1811539429430141585[9] = state[9];
   out_1811539429430141585[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6885361808005668560) {
   out_6885361808005668560[0] = 1;
   out_6885361808005668560[1] = 0;
   out_6885361808005668560[2] = 0;
   out_6885361808005668560[3] = dt;
   out_6885361808005668560[4] = 0;
   out_6885361808005668560[5] = 0;
   out_6885361808005668560[6] = 0;
   out_6885361808005668560[7] = 0;
   out_6885361808005668560[8] = 0;
   out_6885361808005668560[9] = 0;
   out_6885361808005668560[10] = 0;
   out_6885361808005668560[11] = 0;
   out_6885361808005668560[12] = 1;
   out_6885361808005668560[13] = 0;
   out_6885361808005668560[14] = 0;
   out_6885361808005668560[15] = dt;
   out_6885361808005668560[16] = 0;
   out_6885361808005668560[17] = 0;
   out_6885361808005668560[18] = 0;
   out_6885361808005668560[19] = 0;
   out_6885361808005668560[20] = 0;
   out_6885361808005668560[21] = 0;
   out_6885361808005668560[22] = 0;
   out_6885361808005668560[23] = 0;
   out_6885361808005668560[24] = 1;
   out_6885361808005668560[25] = 0;
   out_6885361808005668560[26] = 0;
   out_6885361808005668560[27] = dt;
   out_6885361808005668560[28] = 0;
   out_6885361808005668560[29] = 0;
   out_6885361808005668560[30] = 0;
   out_6885361808005668560[31] = 0;
   out_6885361808005668560[32] = 0;
   out_6885361808005668560[33] = 0;
   out_6885361808005668560[34] = 0;
   out_6885361808005668560[35] = 0;
   out_6885361808005668560[36] = 1;
   out_6885361808005668560[37] = 0;
   out_6885361808005668560[38] = 0;
   out_6885361808005668560[39] = 0;
   out_6885361808005668560[40] = 0;
   out_6885361808005668560[41] = 0;
   out_6885361808005668560[42] = 0;
   out_6885361808005668560[43] = 0;
   out_6885361808005668560[44] = 0;
   out_6885361808005668560[45] = 0;
   out_6885361808005668560[46] = 0;
   out_6885361808005668560[47] = 0;
   out_6885361808005668560[48] = 1;
   out_6885361808005668560[49] = 0;
   out_6885361808005668560[50] = 0;
   out_6885361808005668560[51] = 0;
   out_6885361808005668560[52] = 0;
   out_6885361808005668560[53] = 0;
   out_6885361808005668560[54] = 0;
   out_6885361808005668560[55] = 0;
   out_6885361808005668560[56] = 0;
   out_6885361808005668560[57] = 0;
   out_6885361808005668560[58] = 0;
   out_6885361808005668560[59] = 0;
   out_6885361808005668560[60] = 1;
   out_6885361808005668560[61] = 0;
   out_6885361808005668560[62] = 0;
   out_6885361808005668560[63] = 0;
   out_6885361808005668560[64] = 0;
   out_6885361808005668560[65] = 0;
   out_6885361808005668560[66] = 0;
   out_6885361808005668560[67] = 0;
   out_6885361808005668560[68] = 0;
   out_6885361808005668560[69] = 0;
   out_6885361808005668560[70] = 0;
   out_6885361808005668560[71] = 0;
   out_6885361808005668560[72] = 1;
   out_6885361808005668560[73] = dt;
   out_6885361808005668560[74] = 0;
   out_6885361808005668560[75] = 0;
   out_6885361808005668560[76] = 0;
   out_6885361808005668560[77] = 0;
   out_6885361808005668560[78] = 0;
   out_6885361808005668560[79] = 0;
   out_6885361808005668560[80] = 0;
   out_6885361808005668560[81] = 0;
   out_6885361808005668560[82] = 0;
   out_6885361808005668560[83] = 0;
   out_6885361808005668560[84] = 1;
   out_6885361808005668560[85] = dt;
   out_6885361808005668560[86] = 0;
   out_6885361808005668560[87] = 0;
   out_6885361808005668560[88] = 0;
   out_6885361808005668560[89] = 0;
   out_6885361808005668560[90] = 0;
   out_6885361808005668560[91] = 0;
   out_6885361808005668560[92] = 0;
   out_6885361808005668560[93] = 0;
   out_6885361808005668560[94] = 0;
   out_6885361808005668560[95] = 0;
   out_6885361808005668560[96] = 1;
   out_6885361808005668560[97] = 0;
   out_6885361808005668560[98] = 0;
   out_6885361808005668560[99] = 0;
   out_6885361808005668560[100] = 0;
   out_6885361808005668560[101] = 0;
   out_6885361808005668560[102] = 0;
   out_6885361808005668560[103] = 0;
   out_6885361808005668560[104] = 0;
   out_6885361808005668560[105] = 0;
   out_6885361808005668560[106] = 0;
   out_6885361808005668560[107] = 0;
   out_6885361808005668560[108] = 1;
   out_6885361808005668560[109] = 0;
   out_6885361808005668560[110] = 0;
   out_6885361808005668560[111] = 0;
   out_6885361808005668560[112] = 0;
   out_6885361808005668560[113] = 0;
   out_6885361808005668560[114] = 0;
   out_6885361808005668560[115] = 0;
   out_6885361808005668560[116] = 0;
   out_6885361808005668560[117] = 0;
   out_6885361808005668560[118] = 0;
   out_6885361808005668560[119] = 0;
   out_6885361808005668560[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4720690199573780707) {
   out_4720690199573780707[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_3501032272112231556) {
   out_3501032272112231556[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3501032272112231556[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3501032272112231556[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3501032272112231556[3] = 0;
   out_3501032272112231556[4] = 0;
   out_3501032272112231556[5] = 0;
   out_3501032272112231556[6] = 1;
   out_3501032272112231556[7] = 0;
   out_3501032272112231556[8] = 0;
   out_3501032272112231556[9] = 0;
   out_3501032272112231556[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7764732769973401328) {
   out_7764732769973401328[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2224619370774720672) {
   out_2224619370774720672[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2224619370774720672[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2224619370774720672[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2224619370774720672[3] = 0;
   out_2224619370774720672[4] = 0;
   out_2224619370774720672[5] = 0;
   out_2224619370774720672[6] = 1;
   out_2224619370774720672[7] = 0;
   out_2224619370774720672[8] = 0;
   out_2224619370774720672[9] = 1;
   out_2224619370774720672[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_367888971529516748) {
   out_367888971529516748[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8328773051766743003) {
   out_8328773051766743003[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[6] = 0;
   out_8328773051766743003[7] = 1;
   out_8328773051766743003[8] = 0;
   out_8328773051766743003[9] = 0;
   out_8328773051766743003[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_367888971529516748) {
   out_367888971529516748[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8328773051766743003) {
   out_8328773051766743003[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8328773051766743003[6] = 0;
   out_8328773051766743003[7] = 1;
   out_8328773051766743003[8] = 0;
   out_8328773051766743003[9] = 0;
   out_8328773051766743003[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3463381613600226321) {
  err_fun(nom_x, delta_x, out_3463381613600226321);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4676674826669643480) {
  inv_err_fun(nom_x, true_x, out_4676674826669643480);
}
void gnss_H_mod_fun(double *state, double *out_4538830618860333370) {
  H_mod_fun(state, out_4538830618860333370);
}
void gnss_f_fun(double *state, double dt, double *out_1811539429430141585) {
  f_fun(state,  dt, out_1811539429430141585);
}
void gnss_F_fun(double *state, double dt, double *out_6885361808005668560) {
  F_fun(state,  dt, out_6885361808005668560);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4720690199573780707) {
  h_6(state, sat_pos, out_4720690199573780707);
}
void gnss_H_6(double *state, double *sat_pos, double *out_3501032272112231556) {
  H_6(state, sat_pos, out_3501032272112231556);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7764732769973401328) {
  h_20(state, sat_pos, out_7764732769973401328);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2224619370774720672) {
  H_20(state, sat_pos, out_2224619370774720672);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_367888971529516748) {
  h_7(state, sat_pos_vel, out_367888971529516748);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8328773051766743003) {
  H_7(state, sat_pos_vel, out_8328773051766743003);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_367888971529516748) {
  h_21(state, sat_pos_vel, out_367888971529516748);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8328773051766743003) {
  H_21(state, sat_pos_vel, out_8328773051766743003);
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
