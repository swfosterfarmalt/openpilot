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
void err_fun(double *nom_x, double *delta_x, double *out_6447414538914227673) {
   out_6447414538914227673[0] = delta_x[0] + nom_x[0];
   out_6447414538914227673[1] = delta_x[1] + nom_x[1];
   out_6447414538914227673[2] = delta_x[2] + nom_x[2];
   out_6447414538914227673[3] = delta_x[3] + nom_x[3];
   out_6447414538914227673[4] = delta_x[4] + nom_x[4];
   out_6447414538914227673[5] = delta_x[5] + nom_x[5];
   out_6447414538914227673[6] = delta_x[6] + nom_x[6];
   out_6447414538914227673[7] = delta_x[7] + nom_x[7];
   out_6447414538914227673[8] = delta_x[8] + nom_x[8];
   out_6447414538914227673[9] = delta_x[9] + nom_x[9];
   out_6447414538914227673[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5000797098598118139) {
   out_5000797098598118139[0] = -nom_x[0] + true_x[0];
   out_5000797098598118139[1] = -nom_x[1] + true_x[1];
   out_5000797098598118139[2] = -nom_x[2] + true_x[2];
   out_5000797098598118139[3] = -nom_x[3] + true_x[3];
   out_5000797098598118139[4] = -nom_x[4] + true_x[4];
   out_5000797098598118139[5] = -nom_x[5] + true_x[5];
   out_5000797098598118139[6] = -nom_x[6] + true_x[6];
   out_5000797098598118139[7] = -nom_x[7] + true_x[7];
   out_5000797098598118139[8] = -nom_x[8] + true_x[8];
   out_5000797098598118139[9] = -nom_x[9] + true_x[9];
   out_5000797098598118139[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4309541226740549276) {
   out_4309541226740549276[0] = 1.0;
   out_4309541226740549276[1] = 0;
   out_4309541226740549276[2] = 0;
   out_4309541226740549276[3] = 0;
   out_4309541226740549276[4] = 0;
   out_4309541226740549276[5] = 0;
   out_4309541226740549276[6] = 0;
   out_4309541226740549276[7] = 0;
   out_4309541226740549276[8] = 0;
   out_4309541226740549276[9] = 0;
   out_4309541226740549276[10] = 0;
   out_4309541226740549276[11] = 0;
   out_4309541226740549276[12] = 1.0;
   out_4309541226740549276[13] = 0;
   out_4309541226740549276[14] = 0;
   out_4309541226740549276[15] = 0;
   out_4309541226740549276[16] = 0;
   out_4309541226740549276[17] = 0;
   out_4309541226740549276[18] = 0;
   out_4309541226740549276[19] = 0;
   out_4309541226740549276[20] = 0;
   out_4309541226740549276[21] = 0;
   out_4309541226740549276[22] = 0;
   out_4309541226740549276[23] = 0;
   out_4309541226740549276[24] = 1.0;
   out_4309541226740549276[25] = 0;
   out_4309541226740549276[26] = 0;
   out_4309541226740549276[27] = 0;
   out_4309541226740549276[28] = 0;
   out_4309541226740549276[29] = 0;
   out_4309541226740549276[30] = 0;
   out_4309541226740549276[31] = 0;
   out_4309541226740549276[32] = 0;
   out_4309541226740549276[33] = 0;
   out_4309541226740549276[34] = 0;
   out_4309541226740549276[35] = 0;
   out_4309541226740549276[36] = 1.0;
   out_4309541226740549276[37] = 0;
   out_4309541226740549276[38] = 0;
   out_4309541226740549276[39] = 0;
   out_4309541226740549276[40] = 0;
   out_4309541226740549276[41] = 0;
   out_4309541226740549276[42] = 0;
   out_4309541226740549276[43] = 0;
   out_4309541226740549276[44] = 0;
   out_4309541226740549276[45] = 0;
   out_4309541226740549276[46] = 0;
   out_4309541226740549276[47] = 0;
   out_4309541226740549276[48] = 1.0;
   out_4309541226740549276[49] = 0;
   out_4309541226740549276[50] = 0;
   out_4309541226740549276[51] = 0;
   out_4309541226740549276[52] = 0;
   out_4309541226740549276[53] = 0;
   out_4309541226740549276[54] = 0;
   out_4309541226740549276[55] = 0;
   out_4309541226740549276[56] = 0;
   out_4309541226740549276[57] = 0;
   out_4309541226740549276[58] = 0;
   out_4309541226740549276[59] = 0;
   out_4309541226740549276[60] = 1.0;
   out_4309541226740549276[61] = 0;
   out_4309541226740549276[62] = 0;
   out_4309541226740549276[63] = 0;
   out_4309541226740549276[64] = 0;
   out_4309541226740549276[65] = 0;
   out_4309541226740549276[66] = 0;
   out_4309541226740549276[67] = 0;
   out_4309541226740549276[68] = 0;
   out_4309541226740549276[69] = 0;
   out_4309541226740549276[70] = 0;
   out_4309541226740549276[71] = 0;
   out_4309541226740549276[72] = 1.0;
   out_4309541226740549276[73] = 0;
   out_4309541226740549276[74] = 0;
   out_4309541226740549276[75] = 0;
   out_4309541226740549276[76] = 0;
   out_4309541226740549276[77] = 0;
   out_4309541226740549276[78] = 0;
   out_4309541226740549276[79] = 0;
   out_4309541226740549276[80] = 0;
   out_4309541226740549276[81] = 0;
   out_4309541226740549276[82] = 0;
   out_4309541226740549276[83] = 0;
   out_4309541226740549276[84] = 1.0;
   out_4309541226740549276[85] = 0;
   out_4309541226740549276[86] = 0;
   out_4309541226740549276[87] = 0;
   out_4309541226740549276[88] = 0;
   out_4309541226740549276[89] = 0;
   out_4309541226740549276[90] = 0;
   out_4309541226740549276[91] = 0;
   out_4309541226740549276[92] = 0;
   out_4309541226740549276[93] = 0;
   out_4309541226740549276[94] = 0;
   out_4309541226740549276[95] = 0;
   out_4309541226740549276[96] = 1.0;
   out_4309541226740549276[97] = 0;
   out_4309541226740549276[98] = 0;
   out_4309541226740549276[99] = 0;
   out_4309541226740549276[100] = 0;
   out_4309541226740549276[101] = 0;
   out_4309541226740549276[102] = 0;
   out_4309541226740549276[103] = 0;
   out_4309541226740549276[104] = 0;
   out_4309541226740549276[105] = 0;
   out_4309541226740549276[106] = 0;
   out_4309541226740549276[107] = 0;
   out_4309541226740549276[108] = 1.0;
   out_4309541226740549276[109] = 0;
   out_4309541226740549276[110] = 0;
   out_4309541226740549276[111] = 0;
   out_4309541226740549276[112] = 0;
   out_4309541226740549276[113] = 0;
   out_4309541226740549276[114] = 0;
   out_4309541226740549276[115] = 0;
   out_4309541226740549276[116] = 0;
   out_4309541226740549276[117] = 0;
   out_4309541226740549276[118] = 0;
   out_4309541226740549276[119] = 0;
   out_4309541226740549276[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_8646939531375188806) {
   out_8646939531375188806[0] = dt*state[3] + state[0];
   out_8646939531375188806[1] = dt*state[4] + state[1];
   out_8646939531375188806[2] = dt*state[5] + state[2];
   out_8646939531375188806[3] = state[3];
   out_8646939531375188806[4] = state[4];
   out_8646939531375188806[5] = state[5];
   out_8646939531375188806[6] = dt*state[7] + state[6];
   out_8646939531375188806[7] = dt*state[8] + state[7];
   out_8646939531375188806[8] = state[8];
   out_8646939531375188806[9] = state[9];
   out_8646939531375188806[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4181600022368642599) {
   out_4181600022368642599[0] = 1;
   out_4181600022368642599[1] = 0;
   out_4181600022368642599[2] = 0;
   out_4181600022368642599[3] = dt;
   out_4181600022368642599[4] = 0;
   out_4181600022368642599[5] = 0;
   out_4181600022368642599[6] = 0;
   out_4181600022368642599[7] = 0;
   out_4181600022368642599[8] = 0;
   out_4181600022368642599[9] = 0;
   out_4181600022368642599[10] = 0;
   out_4181600022368642599[11] = 0;
   out_4181600022368642599[12] = 1;
   out_4181600022368642599[13] = 0;
   out_4181600022368642599[14] = 0;
   out_4181600022368642599[15] = dt;
   out_4181600022368642599[16] = 0;
   out_4181600022368642599[17] = 0;
   out_4181600022368642599[18] = 0;
   out_4181600022368642599[19] = 0;
   out_4181600022368642599[20] = 0;
   out_4181600022368642599[21] = 0;
   out_4181600022368642599[22] = 0;
   out_4181600022368642599[23] = 0;
   out_4181600022368642599[24] = 1;
   out_4181600022368642599[25] = 0;
   out_4181600022368642599[26] = 0;
   out_4181600022368642599[27] = dt;
   out_4181600022368642599[28] = 0;
   out_4181600022368642599[29] = 0;
   out_4181600022368642599[30] = 0;
   out_4181600022368642599[31] = 0;
   out_4181600022368642599[32] = 0;
   out_4181600022368642599[33] = 0;
   out_4181600022368642599[34] = 0;
   out_4181600022368642599[35] = 0;
   out_4181600022368642599[36] = 1;
   out_4181600022368642599[37] = 0;
   out_4181600022368642599[38] = 0;
   out_4181600022368642599[39] = 0;
   out_4181600022368642599[40] = 0;
   out_4181600022368642599[41] = 0;
   out_4181600022368642599[42] = 0;
   out_4181600022368642599[43] = 0;
   out_4181600022368642599[44] = 0;
   out_4181600022368642599[45] = 0;
   out_4181600022368642599[46] = 0;
   out_4181600022368642599[47] = 0;
   out_4181600022368642599[48] = 1;
   out_4181600022368642599[49] = 0;
   out_4181600022368642599[50] = 0;
   out_4181600022368642599[51] = 0;
   out_4181600022368642599[52] = 0;
   out_4181600022368642599[53] = 0;
   out_4181600022368642599[54] = 0;
   out_4181600022368642599[55] = 0;
   out_4181600022368642599[56] = 0;
   out_4181600022368642599[57] = 0;
   out_4181600022368642599[58] = 0;
   out_4181600022368642599[59] = 0;
   out_4181600022368642599[60] = 1;
   out_4181600022368642599[61] = 0;
   out_4181600022368642599[62] = 0;
   out_4181600022368642599[63] = 0;
   out_4181600022368642599[64] = 0;
   out_4181600022368642599[65] = 0;
   out_4181600022368642599[66] = 0;
   out_4181600022368642599[67] = 0;
   out_4181600022368642599[68] = 0;
   out_4181600022368642599[69] = 0;
   out_4181600022368642599[70] = 0;
   out_4181600022368642599[71] = 0;
   out_4181600022368642599[72] = 1;
   out_4181600022368642599[73] = dt;
   out_4181600022368642599[74] = 0;
   out_4181600022368642599[75] = 0;
   out_4181600022368642599[76] = 0;
   out_4181600022368642599[77] = 0;
   out_4181600022368642599[78] = 0;
   out_4181600022368642599[79] = 0;
   out_4181600022368642599[80] = 0;
   out_4181600022368642599[81] = 0;
   out_4181600022368642599[82] = 0;
   out_4181600022368642599[83] = 0;
   out_4181600022368642599[84] = 1;
   out_4181600022368642599[85] = dt;
   out_4181600022368642599[86] = 0;
   out_4181600022368642599[87] = 0;
   out_4181600022368642599[88] = 0;
   out_4181600022368642599[89] = 0;
   out_4181600022368642599[90] = 0;
   out_4181600022368642599[91] = 0;
   out_4181600022368642599[92] = 0;
   out_4181600022368642599[93] = 0;
   out_4181600022368642599[94] = 0;
   out_4181600022368642599[95] = 0;
   out_4181600022368642599[96] = 1;
   out_4181600022368642599[97] = 0;
   out_4181600022368642599[98] = 0;
   out_4181600022368642599[99] = 0;
   out_4181600022368642599[100] = 0;
   out_4181600022368642599[101] = 0;
   out_4181600022368642599[102] = 0;
   out_4181600022368642599[103] = 0;
   out_4181600022368642599[104] = 0;
   out_4181600022368642599[105] = 0;
   out_4181600022368642599[106] = 0;
   out_4181600022368642599[107] = 0;
   out_4181600022368642599[108] = 1;
   out_4181600022368642599[109] = 0;
   out_4181600022368642599[110] = 0;
   out_4181600022368642599[111] = 0;
   out_4181600022368642599[112] = 0;
   out_4181600022368642599[113] = 0;
   out_4181600022368642599[114] = 0;
   out_4181600022368642599[115] = 0;
   out_4181600022368642599[116] = 0;
   out_4181600022368642599[117] = 0;
   out_4181600022368642599[118] = 0;
   out_4181600022368642599[119] = 0;
   out_4181600022368642599[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2257566452131893418) {
   out_2257566452131893418[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_997360191990300889) {
   out_997360191990300889[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_997360191990300889[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_997360191990300889[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_997360191990300889[3] = 0;
   out_997360191990300889[4] = 0;
   out_997360191990300889[5] = 0;
   out_997360191990300889[6] = 1;
   out_997360191990300889[7] = 0;
   out_997360191990300889[8] = 0;
   out_997360191990300889[9] = 0;
   out_997360191990300889[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_788991058837861864) {
   out_788991058837861864[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5102792766153159856) {
   out_5102792766153159856[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5102792766153159856[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5102792766153159856[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5102792766153159856[3] = 0;
   out_5102792766153159856[4] = 0;
   out_5102792766153159856[5] = 0;
   out_5102792766153159856[6] = 1;
   out_5102792766153159856[7] = 0;
   out_5102792766153159856[8] = 0;
   out_5102792766153159856[9] = 1;
   out_5102792766153159856[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2714770949607692313) {
   out_2714770949607692313[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4952928235801302959) {
   out_4952928235801302959[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[6] = 0;
   out_4952928235801302959[7] = 1;
   out_4952928235801302959[8] = 0;
   out_4952928235801302959[9] = 0;
   out_4952928235801302959[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2714770949607692313) {
   out_2714770949607692313[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4952928235801302959) {
   out_4952928235801302959[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4952928235801302959[6] = 0;
   out_4952928235801302959[7] = 1;
   out_4952928235801302959[8] = 0;
   out_4952928235801302959[9] = 0;
   out_4952928235801302959[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6447414538914227673) {
  err_fun(nom_x, delta_x, out_6447414538914227673);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5000797098598118139) {
  inv_err_fun(nom_x, true_x, out_5000797098598118139);
}
void gnss_H_mod_fun(double *state, double *out_4309541226740549276) {
  H_mod_fun(state, out_4309541226740549276);
}
void gnss_f_fun(double *state, double dt, double *out_8646939531375188806) {
  f_fun(state,  dt, out_8646939531375188806);
}
void gnss_F_fun(double *state, double dt, double *out_4181600022368642599) {
  F_fun(state,  dt, out_4181600022368642599);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2257566452131893418) {
  h_6(state, sat_pos, out_2257566452131893418);
}
void gnss_H_6(double *state, double *sat_pos, double *out_997360191990300889) {
  H_6(state, sat_pos, out_997360191990300889);
}
void gnss_h_20(double *state, double *sat_pos, double *out_788991058837861864) {
  h_20(state, sat_pos, out_788991058837861864);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5102792766153159856) {
  H_20(state, sat_pos, out_5102792766153159856);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2714770949607692313) {
  h_7(state, sat_pos_vel, out_2714770949607692313);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4952928235801302959) {
  H_7(state, sat_pos_vel, out_4952928235801302959);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2714770949607692313) {
  h_21(state, sat_pos_vel, out_2714770949607692313);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4952928235801302959) {
  H_21(state, sat_pos_vel, out_4952928235801302959);
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
