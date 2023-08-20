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
void err_fun(double *nom_x, double *delta_x, double *out_4321674817797598520) {
   out_4321674817797598520[0] = delta_x[0] + nom_x[0];
   out_4321674817797598520[1] = delta_x[1] + nom_x[1];
   out_4321674817797598520[2] = delta_x[2] + nom_x[2];
   out_4321674817797598520[3] = delta_x[3] + nom_x[3];
   out_4321674817797598520[4] = delta_x[4] + nom_x[4];
   out_4321674817797598520[5] = delta_x[5] + nom_x[5];
   out_4321674817797598520[6] = delta_x[6] + nom_x[6];
   out_4321674817797598520[7] = delta_x[7] + nom_x[7];
   out_4321674817797598520[8] = delta_x[8] + nom_x[8];
   out_4321674817797598520[9] = delta_x[9] + nom_x[9];
   out_4321674817797598520[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3485155282649920022) {
   out_3485155282649920022[0] = -nom_x[0] + true_x[0];
   out_3485155282649920022[1] = -nom_x[1] + true_x[1];
   out_3485155282649920022[2] = -nom_x[2] + true_x[2];
   out_3485155282649920022[3] = -nom_x[3] + true_x[3];
   out_3485155282649920022[4] = -nom_x[4] + true_x[4];
   out_3485155282649920022[5] = -nom_x[5] + true_x[5];
   out_3485155282649920022[6] = -nom_x[6] + true_x[6];
   out_3485155282649920022[7] = -nom_x[7] + true_x[7];
   out_3485155282649920022[8] = -nom_x[8] + true_x[8];
   out_3485155282649920022[9] = -nom_x[9] + true_x[9];
   out_3485155282649920022[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5535577955172455216) {
   out_5535577955172455216[0] = 1.0;
   out_5535577955172455216[1] = 0;
   out_5535577955172455216[2] = 0;
   out_5535577955172455216[3] = 0;
   out_5535577955172455216[4] = 0;
   out_5535577955172455216[5] = 0;
   out_5535577955172455216[6] = 0;
   out_5535577955172455216[7] = 0;
   out_5535577955172455216[8] = 0;
   out_5535577955172455216[9] = 0;
   out_5535577955172455216[10] = 0;
   out_5535577955172455216[11] = 0;
   out_5535577955172455216[12] = 1.0;
   out_5535577955172455216[13] = 0;
   out_5535577955172455216[14] = 0;
   out_5535577955172455216[15] = 0;
   out_5535577955172455216[16] = 0;
   out_5535577955172455216[17] = 0;
   out_5535577955172455216[18] = 0;
   out_5535577955172455216[19] = 0;
   out_5535577955172455216[20] = 0;
   out_5535577955172455216[21] = 0;
   out_5535577955172455216[22] = 0;
   out_5535577955172455216[23] = 0;
   out_5535577955172455216[24] = 1.0;
   out_5535577955172455216[25] = 0;
   out_5535577955172455216[26] = 0;
   out_5535577955172455216[27] = 0;
   out_5535577955172455216[28] = 0;
   out_5535577955172455216[29] = 0;
   out_5535577955172455216[30] = 0;
   out_5535577955172455216[31] = 0;
   out_5535577955172455216[32] = 0;
   out_5535577955172455216[33] = 0;
   out_5535577955172455216[34] = 0;
   out_5535577955172455216[35] = 0;
   out_5535577955172455216[36] = 1.0;
   out_5535577955172455216[37] = 0;
   out_5535577955172455216[38] = 0;
   out_5535577955172455216[39] = 0;
   out_5535577955172455216[40] = 0;
   out_5535577955172455216[41] = 0;
   out_5535577955172455216[42] = 0;
   out_5535577955172455216[43] = 0;
   out_5535577955172455216[44] = 0;
   out_5535577955172455216[45] = 0;
   out_5535577955172455216[46] = 0;
   out_5535577955172455216[47] = 0;
   out_5535577955172455216[48] = 1.0;
   out_5535577955172455216[49] = 0;
   out_5535577955172455216[50] = 0;
   out_5535577955172455216[51] = 0;
   out_5535577955172455216[52] = 0;
   out_5535577955172455216[53] = 0;
   out_5535577955172455216[54] = 0;
   out_5535577955172455216[55] = 0;
   out_5535577955172455216[56] = 0;
   out_5535577955172455216[57] = 0;
   out_5535577955172455216[58] = 0;
   out_5535577955172455216[59] = 0;
   out_5535577955172455216[60] = 1.0;
   out_5535577955172455216[61] = 0;
   out_5535577955172455216[62] = 0;
   out_5535577955172455216[63] = 0;
   out_5535577955172455216[64] = 0;
   out_5535577955172455216[65] = 0;
   out_5535577955172455216[66] = 0;
   out_5535577955172455216[67] = 0;
   out_5535577955172455216[68] = 0;
   out_5535577955172455216[69] = 0;
   out_5535577955172455216[70] = 0;
   out_5535577955172455216[71] = 0;
   out_5535577955172455216[72] = 1.0;
   out_5535577955172455216[73] = 0;
   out_5535577955172455216[74] = 0;
   out_5535577955172455216[75] = 0;
   out_5535577955172455216[76] = 0;
   out_5535577955172455216[77] = 0;
   out_5535577955172455216[78] = 0;
   out_5535577955172455216[79] = 0;
   out_5535577955172455216[80] = 0;
   out_5535577955172455216[81] = 0;
   out_5535577955172455216[82] = 0;
   out_5535577955172455216[83] = 0;
   out_5535577955172455216[84] = 1.0;
   out_5535577955172455216[85] = 0;
   out_5535577955172455216[86] = 0;
   out_5535577955172455216[87] = 0;
   out_5535577955172455216[88] = 0;
   out_5535577955172455216[89] = 0;
   out_5535577955172455216[90] = 0;
   out_5535577955172455216[91] = 0;
   out_5535577955172455216[92] = 0;
   out_5535577955172455216[93] = 0;
   out_5535577955172455216[94] = 0;
   out_5535577955172455216[95] = 0;
   out_5535577955172455216[96] = 1.0;
   out_5535577955172455216[97] = 0;
   out_5535577955172455216[98] = 0;
   out_5535577955172455216[99] = 0;
   out_5535577955172455216[100] = 0;
   out_5535577955172455216[101] = 0;
   out_5535577955172455216[102] = 0;
   out_5535577955172455216[103] = 0;
   out_5535577955172455216[104] = 0;
   out_5535577955172455216[105] = 0;
   out_5535577955172455216[106] = 0;
   out_5535577955172455216[107] = 0;
   out_5535577955172455216[108] = 1.0;
   out_5535577955172455216[109] = 0;
   out_5535577955172455216[110] = 0;
   out_5535577955172455216[111] = 0;
   out_5535577955172455216[112] = 0;
   out_5535577955172455216[113] = 0;
   out_5535577955172455216[114] = 0;
   out_5535577955172455216[115] = 0;
   out_5535577955172455216[116] = 0;
   out_5535577955172455216[117] = 0;
   out_5535577955172455216[118] = 0;
   out_5535577955172455216[119] = 0;
   out_5535577955172455216[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1238601709221895068) {
   out_1238601709221895068[0] = dt*state[3] + state[0];
   out_1238601709221895068[1] = dt*state[4] + state[1];
   out_1238601709221895068[2] = dt*state[5] + state[2];
   out_1238601709221895068[3] = state[3];
   out_1238601709221895068[4] = state[4];
   out_1238601709221895068[5] = state[5];
   out_1238601709221895068[6] = dt*state[7] + state[6];
   out_1238601709221895068[7] = dt*state[8] + state[7];
   out_1238601709221895068[8] = state[8];
   out_1238601709221895068[9] = state[9];
   out_1238601709221895068[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5092949869051383406) {
   out_5092949869051383406[0] = 1;
   out_5092949869051383406[1] = 0;
   out_5092949869051383406[2] = 0;
   out_5092949869051383406[3] = dt;
   out_5092949869051383406[4] = 0;
   out_5092949869051383406[5] = 0;
   out_5092949869051383406[6] = 0;
   out_5092949869051383406[7] = 0;
   out_5092949869051383406[8] = 0;
   out_5092949869051383406[9] = 0;
   out_5092949869051383406[10] = 0;
   out_5092949869051383406[11] = 0;
   out_5092949869051383406[12] = 1;
   out_5092949869051383406[13] = 0;
   out_5092949869051383406[14] = 0;
   out_5092949869051383406[15] = dt;
   out_5092949869051383406[16] = 0;
   out_5092949869051383406[17] = 0;
   out_5092949869051383406[18] = 0;
   out_5092949869051383406[19] = 0;
   out_5092949869051383406[20] = 0;
   out_5092949869051383406[21] = 0;
   out_5092949869051383406[22] = 0;
   out_5092949869051383406[23] = 0;
   out_5092949869051383406[24] = 1;
   out_5092949869051383406[25] = 0;
   out_5092949869051383406[26] = 0;
   out_5092949869051383406[27] = dt;
   out_5092949869051383406[28] = 0;
   out_5092949869051383406[29] = 0;
   out_5092949869051383406[30] = 0;
   out_5092949869051383406[31] = 0;
   out_5092949869051383406[32] = 0;
   out_5092949869051383406[33] = 0;
   out_5092949869051383406[34] = 0;
   out_5092949869051383406[35] = 0;
   out_5092949869051383406[36] = 1;
   out_5092949869051383406[37] = 0;
   out_5092949869051383406[38] = 0;
   out_5092949869051383406[39] = 0;
   out_5092949869051383406[40] = 0;
   out_5092949869051383406[41] = 0;
   out_5092949869051383406[42] = 0;
   out_5092949869051383406[43] = 0;
   out_5092949869051383406[44] = 0;
   out_5092949869051383406[45] = 0;
   out_5092949869051383406[46] = 0;
   out_5092949869051383406[47] = 0;
   out_5092949869051383406[48] = 1;
   out_5092949869051383406[49] = 0;
   out_5092949869051383406[50] = 0;
   out_5092949869051383406[51] = 0;
   out_5092949869051383406[52] = 0;
   out_5092949869051383406[53] = 0;
   out_5092949869051383406[54] = 0;
   out_5092949869051383406[55] = 0;
   out_5092949869051383406[56] = 0;
   out_5092949869051383406[57] = 0;
   out_5092949869051383406[58] = 0;
   out_5092949869051383406[59] = 0;
   out_5092949869051383406[60] = 1;
   out_5092949869051383406[61] = 0;
   out_5092949869051383406[62] = 0;
   out_5092949869051383406[63] = 0;
   out_5092949869051383406[64] = 0;
   out_5092949869051383406[65] = 0;
   out_5092949869051383406[66] = 0;
   out_5092949869051383406[67] = 0;
   out_5092949869051383406[68] = 0;
   out_5092949869051383406[69] = 0;
   out_5092949869051383406[70] = 0;
   out_5092949869051383406[71] = 0;
   out_5092949869051383406[72] = 1;
   out_5092949869051383406[73] = dt;
   out_5092949869051383406[74] = 0;
   out_5092949869051383406[75] = 0;
   out_5092949869051383406[76] = 0;
   out_5092949869051383406[77] = 0;
   out_5092949869051383406[78] = 0;
   out_5092949869051383406[79] = 0;
   out_5092949869051383406[80] = 0;
   out_5092949869051383406[81] = 0;
   out_5092949869051383406[82] = 0;
   out_5092949869051383406[83] = 0;
   out_5092949869051383406[84] = 1;
   out_5092949869051383406[85] = dt;
   out_5092949869051383406[86] = 0;
   out_5092949869051383406[87] = 0;
   out_5092949869051383406[88] = 0;
   out_5092949869051383406[89] = 0;
   out_5092949869051383406[90] = 0;
   out_5092949869051383406[91] = 0;
   out_5092949869051383406[92] = 0;
   out_5092949869051383406[93] = 0;
   out_5092949869051383406[94] = 0;
   out_5092949869051383406[95] = 0;
   out_5092949869051383406[96] = 1;
   out_5092949869051383406[97] = 0;
   out_5092949869051383406[98] = 0;
   out_5092949869051383406[99] = 0;
   out_5092949869051383406[100] = 0;
   out_5092949869051383406[101] = 0;
   out_5092949869051383406[102] = 0;
   out_5092949869051383406[103] = 0;
   out_5092949869051383406[104] = 0;
   out_5092949869051383406[105] = 0;
   out_5092949869051383406[106] = 0;
   out_5092949869051383406[107] = 0;
   out_5092949869051383406[108] = 1;
   out_5092949869051383406[109] = 0;
   out_5092949869051383406[110] = 0;
   out_5092949869051383406[111] = 0;
   out_5092949869051383406[112] = 0;
   out_5092949869051383406[113] = 0;
   out_5092949869051383406[114] = 0;
   out_5092949869051383406[115] = 0;
   out_5092949869051383406[116] = 0;
   out_5092949869051383406[117] = 0;
   out_5092949869051383406[118] = 0;
   out_5092949869051383406[119] = 0;
   out_5092949869051383406[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5638122544984721887) {
   out_5638122544984721887[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7628797803822472301) {
   out_7628797803822472301[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7628797803822472301[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7628797803822472301[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7628797803822472301[3] = 0;
   out_7628797803822472301[4] = 0;
   out_7628797803822472301[5] = 0;
   out_7628797803822472301[6] = 1;
   out_7628797803822472301[7] = 0;
   out_7628797803822472301[8] = 0;
   out_7628797803822472301[9] = 0;
   out_7628797803822472301[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8758860236983935183) {
   out_8758860236983935183[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8519583756205282960) {
   out_8519583756205282960[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8519583756205282960[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8519583756205282960[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8519583756205282960[3] = 0;
   out_8519583756205282960[4] = 0;
   out_8519583756205282960[5] = 0;
   out_8519583756205282960[6] = 1;
   out_8519583756205282960[7] = 0;
   out_8519583756205282960[8] = 0;
   out_8519583756205282960[9] = 1;
   out_8519583756205282960[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_780098669313691505) {
   out_780098669313691505[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3736749610851893537) {
   out_3736749610851893537[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[6] = 0;
   out_3736749610851893537[7] = 1;
   out_3736749610851893537[8] = 0;
   out_3736749610851893537[9] = 0;
   out_3736749610851893537[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_780098669313691505) {
   out_780098669313691505[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3736749610851893537) {
   out_3736749610851893537[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3736749610851893537[6] = 0;
   out_3736749610851893537[7] = 1;
   out_3736749610851893537[8] = 0;
   out_3736749610851893537[9] = 0;
   out_3736749610851893537[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4321674817797598520) {
  err_fun(nom_x, delta_x, out_4321674817797598520);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3485155282649920022) {
  inv_err_fun(nom_x, true_x, out_3485155282649920022);
}
void gnss_H_mod_fun(double *state, double *out_5535577955172455216) {
  H_mod_fun(state, out_5535577955172455216);
}
void gnss_f_fun(double *state, double dt, double *out_1238601709221895068) {
  f_fun(state,  dt, out_1238601709221895068);
}
void gnss_F_fun(double *state, double dt, double *out_5092949869051383406) {
  F_fun(state,  dt, out_5092949869051383406);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5638122544984721887) {
  h_6(state, sat_pos, out_5638122544984721887);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7628797803822472301) {
  H_6(state, sat_pos, out_7628797803822472301);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8758860236983935183) {
  h_20(state, sat_pos, out_8758860236983935183);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8519583756205282960) {
  H_20(state, sat_pos, out_8519583756205282960);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_780098669313691505) {
  h_7(state, sat_pos_vel, out_780098669313691505);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3736749610851893537) {
  H_7(state, sat_pos_vel, out_3736749610851893537);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_780098669313691505) {
  h_21(state, sat_pos_vel, out_780098669313691505);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3736749610851893537) {
  H_21(state, sat_pos_vel, out_3736749610851893537);
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
