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
void err_fun(double *nom_x, double *delta_x, double *out_444352741485845956) {
   out_444352741485845956[0] = delta_x[0] + nom_x[0];
   out_444352741485845956[1] = delta_x[1] + nom_x[1];
   out_444352741485845956[2] = delta_x[2] + nom_x[2];
   out_444352741485845956[3] = delta_x[3] + nom_x[3];
   out_444352741485845956[4] = delta_x[4] + nom_x[4];
   out_444352741485845956[5] = delta_x[5] + nom_x[5];
   out_444352741485845956[6] = delta_x[6] + nom_x[6];
   out_444352741485845956[7] = delta_x[7] + nom_x[7];
   out_444352741485845956[8] = delta_x[8] + nom_x[8];
   out_444352741485845956[9] = delta_x[9] + nom_x[9];
   out_444352741485845956[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1833548362137252330) {
   out_1833548362137252330[0] = -nom_x[0] + true_x[0];
   out_1833548362137252330[1] = -nom_x[1] + true_x[1];
   out_1833548362137252330[2] = -nom_x[2] + true_x[2];
   out_1833548362137252330[3] = -nom_x[3] + true_x[3];
   out_1833548362137252330[4] = -nom_x[4] + true_x[4];
   out_1833548362137252330[5] = -nom_x[5] + true_x[5];
   out_1833548362137252330[6] = -nom_x[6] + true_x[6];
   out_1833548362137252330[7] = -nom_x[7] + true_x[7];
   out_1833548362137252330[8] = -nom_x[8] + true_x[8];
   out_1833548362137252330[9] = -nom_x[9] + true_x[9];
   out_1833548362137252330[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3884835978187609184) {
   out_3884835978187609184[0] = 1.0;
   out_3884835978187609184[1] = 0;
   out_3884835978187609184[2] = 0;
   out_3884835978187609184[3] = 0;
   out_3884835978187609184[4] = 0;
   out_3884835978187609184[5] = 0;
   out_3884835978187609184[6] = 0;
   out_3884835978187609184[7] = 0;
   out_3884835978187609184[8] = 0;
   out_3884835978187609184[9] = 0;
   out_3884835978187609184[10] = 0;
   out_3884835978187609184[11] = 0;
   out_3884835978187609184[12] = 1.0;
   out_3884835978187609184[13] = 0;
   out_3884835978187609184[14] = 0;
   out_3884835978187609184[15] = 0;
   out_3884835978187609184[16] = 0;
   out_3884835978187609184[17] = 0;
   out_3884835978187609184[18] = 0;
   out_3884835978187609184[19] = 0;
   out_3884835978187609184[20] = 0;
   out_3884835978187609184[21] = 0;
   out_3884835978187609184[22] = 0;
   out_3884835978187609184[23] = 0;
   out_3884835978187609184[24] = 1.0;
   out_3884835978187609184[25] = 0;
   out_3884835978187609184[26] = 0;
   out_3884835978187609184[27] = 0;
   out_3884835978187609184[28] = 0;
   out_3884835978187609184[29] = 0;
   out_3884835978187609184[30] = 0;
   out_3884835978187609184[31] = 0;
   out_3884835978187609184[32] = 0;
   out_3884835978187609184[33] = 0;
   out_3884835978187609184[34] = 0;
   out_3884835978187609184[35] = 0;
   out_3884835978187609184[36] = 1.0;
   out_3884835978187609184[37] = 0;
   out_3884835978187609184[38] = 0;
   out_3884835978187609184[39] = 0;
   out_3884835978187609184[40] = 0;
   out_3884835978187609184[41] = 0;
   out_3884835978187609184[42] = 0;
   out_3884835978187609184[43] = 0;
   out_3884835978187609184[44] = 0;
   out_3884835978187609184[45] = 0;
   out_3884835978187609184[46] = 0;
   out_3884835978187609184[47] = 0;
   out_3884835978187609184[48] = 1.0;
   out_3884835978187609184[49] = 0;
   out_3884835978187609184[50] = 0;
   out_3884835978187609184[51] = 0;
   out_3884835978187609184[52] = 0;
   out_3884835978187609184[53] = 0;
   out_3884835978187609184[54] = 0;
   out_3884835978187609184[55] = 0;
   out_3884835978187609184[56] = 0;
   out_3884835978187609184[57] = 0;
   out_3884835978187609184[58] = 0;
   out_3884835978187609184[59] = 0;
   out_3884835978187609184[60] = 1.0;
   out_3884835978187609184[61] = 0;
   out_3884835978187609184[62] = 0;
   out_3884835978187609184[63] = 0;
   out_3884835978187609184[64] = 0;
   out_3884835978187609184[65] = 0;
   out_3884835978187609184[66] = 0;
   out_3884835978187609184[67] = 0;
   out_3884835978187609184[68] = 0;
   out_3884835978187609184[69] = 0;
   out_3884835978187609184[70] = 0;
   out_3884835978187609184[71] = 0;
   out_3884835978187609184[72] = 1.0;
   out_3884835978187609184[73] = 0;
   out_3884835978187609184[74] = 0;
   out_3884835978187609184[75] = 0;
   out_3884835978187609184[76] = 0;
   out_3884835978187609184[77] = 0;
   out_3884835978187609184[78] = 0;
   out_3884835978187609184[79] = 0;
   out_3884835978187609184[80] = 0;
   out_3884835978187609184[81] = 0;
   out_3884835978187609184[82] = 0;
   out_3884835978187609184[83] = 0;
   out_3884835978187609184[84] = 1.0;
   out_3884835978187609184[85] = 0;
   out_3884835978187609184[86] = 0;
   out_3884835978187609184[87] = 0;
   out_3884835978187609184[88] = 0;
   out_3884835978187609184[89] = 0;
   out_3884835978187609184[90] = 0;
   out_3884835978187609184[91] = 0;
   out_3884835978187609184[92] = 0;
   out_3884835978187609184[93] = 0;
   out_3884835978187609184[94] = 0;
   out_3884835978187609184[95] = 0;
   out_3884835978187609184[96] = 1.0;
   out_3884835978187609184[97] = 0;
   out_3884835978187609184[98] = 0;
   out_3884835978187609184[99] = 0;
   out_3884835978187609184[100] = 0;
   out_3884835978187609184[101] = 0;
   out_3884835978187609184[102] = 0;
   out_3884835978187609184[103] = 0;
   out_3884835978187609184[104] = 0;
   out_3884835978187609184[105] = 0;
   out_3884835978187609184[106] = 0;
   out_3884835978187609184[107] = 0;
   out_3884835978187609184[108] = 1.0;
   out_3884835978187609184[109] = 0;
   out_3884835978187609184[110] = 0;
   out_3884835978187609184[111] = 0;
   out_3884835978187609184[112] = 0;
   out_3884835978187609184[113] = 0;
   out_3884835978187609184[114] = 0;
   out_3884835978187609184[115] = 0;
   out_3884835978187609184[116] = 0;
   out_3884835978187609184[117] = 0;
   out_3884835978187609184[118] = 0;
   out_3884835978187609184[119] = 0;
   out_3884835978187609184[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2799479213853634378) {
   out_2799479213853634378[0] = dt*state[3] + state[0];
   out_2799479213853634378[1] = dt*state[4] + state[1];
   out_2799479213853634378[2] = dt*state[5] + state[2];
   out_2799479213853634378[3] = state[3];
   out_2799479213853634378[4] = state[4];
   out_2799479213853634378[5] = state[5];
   out_2799479213853634378[6] = dt*state[7] + state[6];
   out_2799479213853634378[7] = dt*state[8] + state[7];
   out_2799479213853634378[8] = state[8];
   out_2799479213853634378[9] = state[9];
   out_2799479213853634378[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3097430740682718197) {
   out_3097430740682718197[0] = 1;
   out_3097430740682718197[1] = 0;
   out_3097430740682718197[2] = 0;
   out_3097430740682718197[3] = dt;
   out_3097430740682718197[4] = 0;
   out_3097430740682718197[5] = 0;
   out_3097430740682718197[6] = 0;
   out_3097430740682718197[7] = 0;
   out_3097430740682718197[8] = 0;
   out_3097430740682718197[9] = 0;
   out_3097430740682718197[10] = 0;
   out_3097430740682718197[11] = 0;
   out_3097430740682718197[12] = 1;
   out_3097430740682718197[13] = 0;
   out_3097430740682718197[14] = 0;
   out_3097430740682718197[15] = dt;
   out_3097430740682718197[16] = 0;
   out_3097430740682718197[17] = 0;
   out_3097430740682718197[18] = 0;
   out_3097430740682718197[19] = 0;
   out_3097430740682718197[20] = 0;
   out_3097430740682718197[21] = 0;
   out_3097430740682718197[22] = 0;
   out_3097430740682718197[23] = 0;
   out_3097430740682718197[24] = 1;
   out_3097430740682718197[25] = 0;
   out_3097430740682718197[26] = 0;
   out_3097430740682718197[27] = dt;
   out_3097430740682718197[28] = 0;
   out_3097430740682718197[29] = 0;
   out_3097430740682718197[30] = 0;
   out_3097430740682718197[31] = 0;
   out_3097430740682718197[32] = 0;
   out_3097430740682718197[33] = 0;
   out_3097430740682718197[34] = 0;
   out_3097430740682718197[35] = 0;
   out_3097430740682718197[36] = 1;
   out_3097430740682718197[37] = 0;
   out_3097430740682718197[38] = 0;
   out_3097430740682718197[39] = 0;
   out_3097430740682718197[40] = 0;
   out_3097430740682718197[41] = 0;
   out_3097430740682718197[42] = 0;
   out_3097430740682718197[43] = 0;
   out_3097430740682718197[44] = 0;
   out_3097430740682718197[45] = 0;
   out_3097430740682718197[46] = 0;
   out_3097430740682718197[47] = 0;
   out_3097430740682718197[48] = 1;
   out_3097430740682718197[49] = 0;
   out_3097430740682718197[50] = 0;
   out_3097430740682718197[51] = 0;
   out_3097430740682718197[52] = 0;
   out_3097430740682718197[53] = 0;
   out_3097430740682718197[54] = 0;
   out_3097430740682718197[55] = 0;
   out_3097430740682718197[56] = 0;
   out_3097430740682718197[57] = 0;
   out_3097430740682718197[58] = 0;
   out_3097430740682718197[59] = 0;
   out_3097430740682718197[60] = 1;
   out_3097430740682718197[61] = 0;
   out_3097430740682718197[62] = 0;
   out_3097430740682718197[63] = 0;
   out_3097430740682718197[64] = 0;
   out_3097430740682718197[65] = 0;
   out_3097430740682718197[66] = 0;
   out_3097430740682718197[67] = 0;
   out_3097430740682718197[68] = 0;
   out_3097430740682718197[69] = 0;
   out_3097430740682718197[70] = 0;
   out_3097430740682718197[71] = 0;
   out_3097430740682718197[72] = 1;
   out_3097430740682718197[73] = dt;
   out_3097430740682718197[74] = 0;
   out_3097430740682718197[75] = 0;
   out_3097430740682718197[76] = 0;
   out_3097430740682718197[77] = 0;
   out_3097430740682718197[78] = 0;
   out_3097430740682718197[79] = 0;
   out_3097430740682718197[80] = 0;
   out_3097430740682718197[81] = 0;
   out_3097430740682718197[82] = 0;
   out_3097430740682718197[83] = 0;
   out_3097430740682718197[84] = 1;
   out_3097430740682718197[85] = dt;
   out_3097430740682718197[86] = 0;
   out_3097430740682718197[87] = 0;
   out_3097430740682718197[88] = 0;
   out_3097430740682718197[89] = 0;
   out_3097430740682718197[90] = 0;
   out_3097430740682718197[91] = 0;
   out_3097430740682718197[92] = 0;
   out_3097430740682718197[93] = 0;
   out_3097430740682718197[94] = 0;
   out_3097430740682718197[95] = 0;
   out_3097430740682718197[96] = 1;
   out_3097430740682718197[97] = 0;
   out_3097430740682718197[98] = 0;
   out_3097430740682718197[99] = 0;
   out_3097430740682718197[100] = 0;
   out_3097430740682718197[101] = 0;
   out_3097430740682718197[102] = 0;
   out_3097430740682718197[103] = 0;
   out_3097430740682718197[104] = 0;
   out_3097430740682718197[105] = 0;
   out_3097430740682718197[106] = 0;
   out_3097430740682718197[107] = 0;
   out_3097430740682718197[108] = 1;
   out_3097430740682718197[109] = 0;
   out_3097430740682718197[110] = 0;
   out_3097430740682718197[111] = 0;
   out_3097430740682718197[112] = 0;
   out_3097430740682718197[113] = 0;
   out_3097430740682718197[114] = 0;
   out_3097430740682718197[115] = 0;
   out_3097430740682718197[116] = 0;
   out_3097430740682718197[117] = 0;
   out_3097430740682718197[118] = 0;
   out_3097430740682718197[119] = 0;
   out_3097430740682718197[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7192572793318069684) {
   out_7192572793318069684[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_5970708195009703631) {
   out_5970708195009703631[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5970708195009703631[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5970708195009703631[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5970708195009703631[3] = 0;
   out_5970708195009703631[4] = 0;
   out_5970708195009703631[5] = 0;
   out_5970708195009703631[6] = 1;
   out_5970708195009703631[7] = 0;
   out_5970708195009703631[8] = 0;
   out_5970708195009703631[9] = 0;
   out_5970708195009703631[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8398852110558080067) {
   out_8398852110558080067[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8813653629957763591) {
   out_8813653629957763591[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8813653629957763591[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8813653629957763591[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8813653629957763591[3] = 0;
   out_8813653629957763591[4] = 0;
   out_8813653629957763591[5] = 0;
   out_8813653629957763591[6] = 1;
   out_8813653629957763591[7] = 0;
   out_8813653629957763591[8] = 0;
   out_8813653629957763591[9] = 1;
   out_8813653629957763591[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2767556765111865509) {
   out_2767556765111865509[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4773051478760733724) {
   out_4773051478760733724[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[6] = 0;
   out_4773051478760733724[7] = 1;
   out_4773051478760733724[8] = 0;
   out_4773051478760733724[9] = 0;
   out_4773051478760733724[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2767556765111865509) {
   out_2767556765111865509[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4773051478760733724) {
   out_4773051478760733724[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4773051478760733724[6] = 0;
   out_4773051478760733724[7] = 1;
   out_4773051478760733724[8] = 0;
   out_4773051478760733724[9] = 0;
   out_4773051478760733724[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_444352741485845956) {
  err_fun(nom_x, delta_x, out_444352741485845956);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1833548362137252330) {
  inv_err_fun(nom_x, true_x, out_1833548362137252330);
}
void gnss_H_mod_fun(double *state, double *out_3884835978187609184) {
  H_mod_fun(state, out_3884835978187609184);
}
void gnss_f_fun(double *state, double dt, double *out_2799479213853634378) {
  f_fun(state,  dt, out_2799479213853634378);
}
void gnss_F_fun(double *state, double dt, double *out_3097430740682718197) {
  F_fun(state,  dt, out_3097430740682718197);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7192572793318069684) {
  h_6(state, sat_pos, out_7192572793318069684);
}
void gnss_H_6(double *state, double *sat_pos, double *out_5970708195009703631) {
  H_6(state, sat_pos, out_5970708195009703631);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8398852110558080067) {
  h_20(state, sat_pos, out_8398852110558080067);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8813653629957763591) {
  H_20(state, sat_pos, out_8813653629957763591);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2767556765111865509) {
  h_7(state, sat_pos_vel, out_2767556765111865509);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4773051478760733724) {
  H_7(state, sat_pos_vel, out_4773051478760733724);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2767556765111865509) {
  h_21(state, sat_pos_vel, out_2767556765111865509);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4773051478760733724) {
  H_21(state, sat_pos_vel, out_4773051478760733724);
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
