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
void err_fun(double *nom_x, double *delta_x, double *out_5002540976583435154) {
   out_5002540976583435154[0] = delta_x[0] + nom_x[0];
   out_5002540976583435154[1] = delta_x[1] + nom_x[1];
   out_5002540976583435154[2] = delta_x[2] + nom_x[2];
   out_5002540976583435154[3] = delta_x[3] + nom_x[3];
   out_5002540976583435154[4] = delta_x[4] + nom_x[4];
   out_5002540976583435154[5] = delta_x[5] + nom_x[5];
   out_5002540976583435154[6] = delta_x[6] + nom_x[6];
   out_5002540976583435154[7] = delta_x[7] + nom_x[7];
   out_5002540976583435154[8] = delta_x[8] + nom_x[8];
   out_5002540976583435154[9] = delta_x[9] + nom_x[9];
   out_5002540976583435154[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8923382853611226714) {
   out_8923382853611226714[0] = -nom_x[0] + true_x[0];
   out_8923382853611226714[1] = -nom_x[1] + true_x[1];
   out_8923382853611226714[2] = -nom_x[2] + true_x[2];
   out_8923382853611226714[3] = -nom_x[3] + true_x[3];
   out_8923382853611226714[4] = -nom_x[4] + true_x[4];
   out_8923382853611226714[5] = -nom_x[5] + true_x[5];
   out_8923382853611226714[6] = -nom_x[6] + true_x[6];
   out_8923382853611226714[7] = -nom_x[7] + true_x[7];
   out_8923382853611226714[8] = -nom_x[8] + true_x[8];
   out_8923382853611226714[9] = -nom_x[9] + true_x[9];
   out_8923382853611226714[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7483018014028836974) {
   out_7483018014028836974[0] = 1.0;
   out_7483018014028836974[1] = 0;
   out_7483018014028836974[2] = 0;
   out_7483018014028836974[3] = 0;
   out_7483018014028836974[4] = 0;
   out_7483018014028836974[5] = 0;
   out_7483018014028836974[6] = 0;
   out_7483018014028836974[7] = 0;
   out_7483018014028836974[8] = 0;
   out_7483018014028836974[9] = 0;
   out_7483018014028836974[10] = 0;
   out_7483018014028836974[11] = 0;
   out_7483018014028836974[12] = 1.0;
   out_7483018014028836974[13] = 0;
   out_7483018014028836974[14] = 0;
   out_7483018014028836974[15] = 0;
   out_7483018014028836974[16] = 0;
   out_7483018014028836974[17] = 0;
   out_7483018014028836974[18] = 0;
   out_7483018014028836974[19] = 0;
   out_7483018014028836974[20] = 0;
   out_7483018014028836974[21] = 0;
   out_7483018014028836974[22] = 0;
   out_7483018014028836974[23] = 0;
   out_7483018014028836974[24] = 1.0;
   out_7483018014028836974[25] = 0;
   out_7483018014028836974[26] = 0;
   out_7483018014028836974[27] = 0;
   out_7483018014028836974[28] = 0;
   out_7483018014028836974[29] = 0;
   out_7483018014028836974[30] = 0;
   out_7483018014028836974[31] = 0;
   out_7483018014028836974[32] = 0;
   out_7483018014028836974[33] = 0;
   out_7483018014028836974[34] = 0;
   out_7483018014028836974[35] = 0;
   out_7483018014028836974[36] = 1.0;
   out_7483018014028836974[37] = 0;
   out_7483018014028836974[38] = 0;
   out_7483018014028836974[39] = 0;
   out_7483018014028836974[40] = 0;
   out_7483018014028836974[41] = 0;
   out_7483018014028836974[42] = 0;
   out_7483018014028836974[43] = 0;
   out_7483018014028836974[44] = 0;
   out_7483018014028836974[45] = 0;
   out_7483018014028836974[46] = 0;
   out_7483018014028836974[47] = 0;
   out_7483018014028836974[48] = 1.0;
   out_7483018014028836974[49] = 0;
   out_7483018014028836974[50] = 0;
   out_7483018014028836974[51] = 0;
   out_7483018014028836974[52] = 0;
   out_7483018014028836974[53] = 0;
   out_7483018014028836974[54] = 0;
   out_7483018014028836974[55] = 0;
   out_7483018014028836974[56] = 0;
   out_7483018014028836974[57] = 0;
   out_7483018014028836974[58] = 0;
   out_7483018014028836974[59] = 0;
   out_7483018014028836974[60] = 1.0;
   out_7483018014028836974[61] = 0;
   out_7483018014028836974[62] = 0;
   out_7483018014028836974[63] = 0;
   out_7483018014028836974[64] = 0;
   out_7483018014028836974[65] = 0;
   out_7483018014028836974[66] = 0;
   out_7483018014028836974[67] = 0;
   out_7483018014028836974[68] = 0;
   out_7483018014028836974[69] = 0;
   out_7483018014028836974[70] = 0;
   out_7483018014028836974[71] = 0;
   out_7483018014028836974[72] = 1.0;
   out_7483018014028836974[73] = 0;
   out_7483018014028836974[74] = 0;
   out_7483018014028836974[75] = 0;
   out_7483018014028836974[76] = 0;
   out_7483018014028836974[77] = 0;
   out_7483018014028836974[78] = 0;
   out_7483018014028836974[79] = 0;
   out_7483018014028836974[80] = 0;
   out_7483018014028836974[81] = 0;
   out_7483018014028836974[82] = 0;
   out_7483018014028836974[83] = 0;
   out_7483018014028836974[84] = 1.0;
   out_7483018014028836974[85] = 0;
   out_7483018014028836974[86] = 0;
   out_7483018014028836974[87] = 0;
   out_7483018014028836974[88] = 0;
   out_7483018014028836974[89] = 0;
   out_7483018014028836974[90] = 0;
   out_7483018014028836974[91] = 0;
   out_7483018014028836974[92] = 0;
   out_7483018014028836974[93] = 0;
   out_7483018014028836974[94] = 0;
   out_7483018014028836974[95] = 0;
   out_7483018014028836974[96] = 1.0;
   out_7483018014028836974[97] = 0;
   out_7483018014028836974[98] = 0;
   out_7483018014028836974[99] = 0;
   out_7483018014028836974[100] = 0;
   out_7483018014028836974[101] = 0;
   out_7483018014028836974[102] = 0;
   out_7483018014028836974[103] = 0;
   out_7483018014028836974[104] = 0;
   out_7483018014028836974[105] = 0;
   out_7483018014028836974[106] = 0;
   out_7483018014028836974[107] = 0;
   out_7483018014028836974[108] = 1.0;
   out_7483018014028836974[109] = 0;
   out_7483018014028836974[110] = 0;
   out_7483018014028836974[111] = 0;
   out_7483018014028836974[112] = 0;
   out_7483018014028836974[113] = 0;
   out_7483018014028836974[114] = 0;
   out_7483018014028836974[115] = 0;
   out_7483018014028836974[116] = 0;
   out_7483018014028836974[117] = 0;
   out_7483018014028836974[118] = 0;
   out_7483018014028836974[119] = 0;
   out_7483018014028836974[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3233547939003755521) {
   out_3233547939003755521[0] = dt*state[3] + state[0];
   out_3233547939003755521[1] = dt*state[4] + state[1];
   out_3233547939003755521[2] = dt*state[5] + state[2];
   out_3233547939003755521[3] = state[3];
   out_3233547939003755521[4] = state[4];
   out_3233547939003755521[5] = state[5];
   out_3233547939003755521[6] = dt*state[7] + state[6];
   out_3233547939003755521[7] = dt*state[8] + state[7];
   out_3233547939003755521[8] = state[8];
   out_3233547939003755521[9] = state[9];
   out_3233547939003755521[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6091345688566986461) {
   out_6091345688566986461[0] = 1;
   out_6091345688566986461[1] = 0;
   out_6091345688566986461[2] = 0;
   out_6091345688566986461[3] = dt;
   out_6091345688566986461[4] = 0;
   out_6091345688566986461[5] = 0;
   out_6091345688566986461[6] = 0;
   out_6091345688566986461[7] = 0;
   out_6091345688566986461[8] = 0;
   out_6091345688566986461[9] = 0;
   out_6091345688566986461[10] = 0;
   out_6091345688566986461[11] = 0;
   out_6091345688566986461[12] = 1;
   out_6091345688566986461[13] = 0;
   out_6091345688566986461[14] = 0;
   out_6091345688566986461[15] = dt;
   out_6091345688566986461[16] = 0;
   out_6091345688566986461[17] = 0;
   out_6091345688566986461[18] = 0;
   out_6091345688566986461[19] = 0;
   out_6091345688566986461[20] = 0;
   out_6091345688566986461[21] = 0;
   out_6091345688566986461[22] = 0;
   out_6091345688566986461[23] = 0;
   out_6091345688566986461[24] = 1;
   out_6091345688566986461[25] = 0;
   out_6091345688566986461[26] = 0;
   out_6091345688566986461[27] = dt;
   out_6091345688566986461[28] = 0;
   out_6091345688566986461[29] = 0;
   out_6091345688566986461[30] = 0;
   out_6091345688566986461[31] = 0;
   out_6091345688566986461[32] = 0;
   out_6091345688566986461[33] = 0;
   out_6091345688566986461[34] = 0;
   out_6091345688566986461[35] = 0;
   out_6091345688566986461[36] = 1;
   out_6091345688566986461[37] = 0;
   out_6091345688566986461[38] = 0;
   out_6091345688566986461[39] = 0;
   out_6091345688566986461[40] = 0;
   out_6091345688566986461[41] = 0;
   out_6091345688566986461[42] = 0;
   out_6091345688566986461[43] = 0;
   out_6091345688566986461[44] = 0;
   out_6091345688566986461[45] = 0;
   out_6091345688566986461[46] = 0;
   out_6091345688566986461[47] = 0;
   out_6091345688566986461[48] = 1;
   out_6091345688566986461[49] = 0;
   out_6091345688566986461[50] = 0;
   out_6091345688566986461[51] = 0;
   out_6091345688566986461[52] = 0;
   out_6091345688566986461[53] = 0;
   out_6091345688566986461[54] = 0;
   out_6091345688566986461[55] = 0;
   out_6091345688566986461[56] = 0;
   out_6091345688566986461[57] = 0;
   out_6091345688566986461[58] = 0;
   out_6091345688566986461[59] = 0;
   out_6091345688566986461[60] = 1;
   out_6091345688566986461[61] = 0;
   out_6091345688566986461[62] = 0;
   out_6091345688566986461[63] = 0;
   out_6091345688566986461[64] = 0;
   out_6091345688566986461[65] = 0;
   out_6091345688566986461[66] = 0;
   out_6091345688566986461[67] = 0;
   out_6091345688566986461[68] = 0;
   out_6091345688566986461[69] = 0;
   out_6091345688566986461[70] = 0;
   out_6091345688566986461[71] = 0;
   out_6091345688566986461[72] = 1;
   out_6091345688566986461[73] = dt;
   out_6091345688566986461[74] = 0;
   out_6091345688566986461[75] = 0;
   out_6091345688566986461[76] = 0;
   out_6091345688566986461[77] = 0;
   out_6091345688566986461[78] = 0;
   out_6091345688566986461[79] = 0;
   out_6091345688566986461[80] = 0;
   out_6091345688566986461[81] = 0;
   out_6091345688566986461[82] = 0;
   out_6091345688566986461[83] = 0;
   out_6091345688566986461[84] = 1;
   out_6091345688566986461[85] = dt;
   out_6091345688566986461[86] = 0;
   out_6091345688566986461[87] = 0;
   out_6091345688566986461[88] = 0;
   out_6091345688566986461[89] = 0;
   out_6091345688566986461[90] = 0;
   out_6091345688566986461[91] = 0;
   out_6091345688566986461[92] = 0;
   out_6091345688566986461[93] = 0;
   out_6091345688566986461[94] = 0;
   out_6091345688566986461[95] = 0;
   out_6091345688566986461[96] = 1;
   out_6091345688566986461[97] = 0;
   out_6091345688566986461[98] = 0;
   out_6091345688566986461[99] = 0;
   out_6091345688566986461[100] = 0;
   out_6091345688566986461[101] = 0;
   out_6091345688566986461[102] = 0;
   out_6091345688566986461[103] = 0;
   out_6091345688566986461[104] = 0;
   out_6091345688566986461[105] = 0;
   out_6091345688566986461[106] = 0;
   out_6091345688566986461[107] = 0;
   out_6091345688566986461[108] = 1;
   out_6091345688566986461[109] = 0;
   out_6091345688566986461[110] = 0;
   out_6091345688566986461[111] = 0;
   out_6091345688566986461[112] = 0;
   out_6091345688566986461[113] = 0;
   out_6091345688566986461[114] = 0;
   out_6091345688566986461[115] = 0;
   out_6091345688566986461[116] = 0;
   out_6091345688566986461[117] = 0;
   out_6091345688566986461[118] = 0;
   out_6091345688566986461[119] = 0;
   out_6091345688566986461[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_1049109194514504534) {
   out_1049109194514504534[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_824724143852124941) {
   out_824724143852124941[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824724143852124941[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824724143852124941[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824724143852124941[3] = 0;
   out_824724143852124941[4] = 0;
   out_824724143852124941[5] = 0;
   out_824724143852124941[6] = 1;
   out_824724143852124941[7] = 0;
   out_824724143852124941[8] = 0;
   out_824724143852124941[9] = 0;
   out_824724143852124941[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_874640839125696915) {
   out_874640839125696915[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4929145430862346338) {
   out_4929145430862346338[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4929145430862346338[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4929145430862346338[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4929145430862346338[3] = 0;
   out_4929145430862346338[4] = 0;
   out_4929145430862346338[5] = 0;
   out_4929145430862346338[6] = 1;
   out_4929145430862346338[7] = 0;
   out_4929145430862346338[8] = 0;
   out_4929145430862346338[9] = 1;
   out_4929145430862346338[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3022852619830856734) {
   out_3022852619830856734[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6808293484879381181) {
   out_6808293484879381181[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[6] = 0;
   out_6808293484879381181[7] = 1;
   out_6808293484879381181[8] = 0;
   out_6808293484879381181[9] = 0;
   out_6808293484879381181[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3022852619830856734) {
   out_3022852619830856734[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6808293484879381181) {
   out_6808293484879381181[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6808293484879381181[6] = 0;
   out_6808293484879381181[7] = 1;
   out_6808293484879381181[8] = 0;
   out_6808293484879381181[9] = 0;
   out_6808293484879381181[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5002540976583435154) {
  err_fun(nom_x, delta_x, out_5002540976583435154);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8923382853611226714) {
  inv_err_fun(nom_x, true_x, out_8923382853611226714);
}
void gnss_H_mod_fun(double *state, double *out_7483018014028836974) {
  H_mod_fun(state, out_7483018014028836974);
}
void gnss_f_fun(double *state, double dt, double *out_3233547939003755521) {
  f_fun(state,  dt, out_3233547939003755521);
}
void gnss_F_fun(double *state, double dt, double *out_6091345688566986461) {
  F_fun(state,  dt, out_6091345688566986461);
}
void gnss_h_6(double *state, double *sat_pos, double *out_1049109194514504534) {
  h_6(state, sat_pos, out_1049109194514504534);
}
void gnss_H_6(double *state, double *sat_pos, double *out_824724143852124941) {
  H_6(state, sat_pos, out_824724143852124941);
}
void gnss_h_20(double *state, double *sat_pos, double *out_874640839125696915) {
  h_20(state, sat_pos, out_874640839125696915);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4929145430862346338) {
  H_20(state, sat_pos, out_4929145430862346338);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3022852619830856734) {
  h_7(state, sat_pos_vel, out_3022852619830856734);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6808293484879381181) {
  H_7(state, sat_pos_vel, out_6808293484879381181);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3022852619830856734) {
  h_21(state, sat_pos_vel, out_3022852619830856734);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6808293484879381181) {
  H_21(state, sat_pos_vel, out_6808293484879381181);
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
