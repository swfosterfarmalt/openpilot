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
void err_fun(double *nom_x, double *delta_x, double *out_770261618441231026) {
   out_770261618441231026[0] = delta_x[0] + nom_x[0];
   out_770261618441231026[1] = delta_x[1] + nom_x[1];
   out_770261618441231026[2] = delta_x[2] + nom_x[2];
   out_770261618441231026[3] = delta_x[3] + nom_x[3];
   out_770261618441231026[4] = delta_x[4] + nom_x[4];
   out_770261618441231026[5] = delta_x[5] + nom_x[5];
   out_770261618441231026[6] = delta_x[6] + nom_x[6];
   out_770261618441231026[7] = delta_x[7] + nom_x[7];
   out_770261618441231026[8] = delta_x[8] + nom_x[8];
   out_770261618441231026[9] = delta_x[9] + nom_x[9];
   out_770261618441231026[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7717952509466708825) {
   out_7717952509466708825[0] = -nom_x[0] + true_x[0];
   out_7717952509466708825[1] = -nom_x[1] + true_x[1];
   out_7717952509466708825[2] = -nom_x[2] + true_x[2];
   out_7717952509466708825[3] = -nom_x[3] + true_x[3];
   out_7717952509466708825[4] = -nom_x[4] + true_x[4];
   out_7717952509466708825[5] = -nom_x[5] + true_x[5];
   out_7717952509466708825[6] = -nom_x[6] + true_x[6];
   out_7717952509466708825[7] = -nom_x[7] + true_x[7];
   out_7717952509466708825[8] = -nom_x[8] + true_x[8];
   out_7717952509466708825[9] = -nom_x[9] + true_x[9];
   out_7717952509466708825[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5630546443851562905) {
   out_5630546443851562905[0] = 1.0;
   out_5630546443851562905[1] = 0;
   out_5630546443851562905[2] = 0;
   out_5630546443851562905[3] = 0;
   out_5630546443851562905[4] = 0;
   out_5630546443851562905[5] = 0;
   out_5630546443851562905[6] = 0;
   out_5630546443851562905[7] = 0;
   out_5630546443851562905[8] = 0;
   out_5630546443851562905[9] = 0;
   out_5630546443851562905[10] = 0;
   out_5630546443851562905[11] = 0;
   out_5630546443851562905[12] = 1.0;
   out_5630546443851562905[13] = 0;
   out_5630546443851562905[14] = 0;
   out_5630546443851562905[15] = 0;
   out_5630546443851562905[16] = 0;
   out_5630546443851562905[17] = 0;
   out_5630546443851562905[18] = 0;
   out_5630546443851562905[19] = 0;
   out_5630546443851562905[20] = 0;
   out_5630546443851562905[21] = 0;
   out_5630546443851562905[22] = 0;
   out_5630546443851562905[23] = 0;
   out_5630546443851562905[24] = 1.0;
   out_5630546443851562905[25] = 0;
   out_5630546443851562905[26] = 0;
   out_5630546443851562905[27] = 0;
   out_5630546443851562905[28] = 0;
   out_5630546443851562905[29] = 0;
   out_5630546443851562905[30] = 0;
   out_5630546443851562905[31] = 0;
   out_5630546443851562905[32] = 0;
   out_5630546443851562905[33] = 0;
   out_5630546443851562905[34] = 0;
   out_5630546443851562905[35] = 0;
   out_5630546443851562905[36] = 1.0;
   out_5630546443851562905[37] = 0;
   out_5630546443851562905[38] = 0;
   out_5630546443851562905[39] = 0;
   out_5630546443851562905[40] = 0;
   out_5630546443851562905[41] = 0;
   out_5630546443851562905[42] = 0;
   out_5630546443851562905[43] = 0;
   out_5630546443851562905[44] = 0;
   out_5630546443851562905[45] = 0;
   out_5630546443851562905[46] = 0;
   out_5630546443851562905[47] = 0;
   out_5630546443851562905[48] = 1.0;
   out_5630546443851562905[49] = 0;
   out_5630546443851562905[50] = 0;
   out_5630546443851562905[51] = 0;
   out_5630546443851562905[52] = 0;
   out_5630546443851562905[53] = 0;
   out_5630546443851562905[54] = 0;
   out_5630546443851562905[55] = 0;
   out_5630546443851562905[56] = 0;
   out_5630546443851562905[57] = 0;
   out_5630546443851562905[58] = 0;
   out_5630546443851562905[59] = 0;
   out_5630546443851562905[60] = 1.0;
   out_5630546443851562905[61] = 0;
   out_5630546443851562905[62] = 0;
   out_5630546443851562905[63] = 0;
   out_5630546443851562905[64] = 0;
   out_5630546443851562905[65] = 0;
   out_5630546443851562905[66] = 0;
   out_5630546443851562905[67] = 0;
   out_5630546443851562905[68] = 0;
   out_5630546443851562905[69] = 0;
   out_5630546443851562905[70] = 0;
   out_5630546443851562905[71] = 0;
   out_5630546443851562905[72] = 1.0;
   out_5630546443851562905[73] = 0;
   out_5630546443851562905[74] = 0;
   out_5630546443851562905[75] = 0;
   out_5630546443851562905[76] = 0;
   out_5630546443851562905[77] = 0;
   out_5630546443851562905[78] = 0;
   out_5630546443851562905[79] = 0;
   out_5630546443851562905[80] = 0;
   out_5630546443851562905[81] = 0;
   out_5630546443851562905[82] = 0;
   out_5630546443851562905[83] = 0;
   out_5630546443851562905[84] = 1.0;
   out_5630546443851562905[85] = 0;
   out_5630546443851562905[86] = 0;
   out_5630546443851562905[87] = 0;
   out_5630546443851562905[88] = 0;
   out_5630546443851562905[89] = 0;
   out_5630546443851562905[90] = 0;
   out_5630546443851562905[91] = 0;
   out_5630546443851562905[92] = 0;
   out_5630546443851562905[93] = 0;
   out_5630546443851562905[94] = 0;
   out_5630546443851562905[95] = 0;
   out_5630546443851562905[96] = 1.0;
   out_5630546443851562905[97] = 0;
   out_5630546443851562905[98] = 0;
   out_5630546443851562905[99] = 0;
   out_5630546443851562905[100] = 0;
   out_5630546443851562905[101] = 0;
   out_5630546443851562905[102] = 0;
   out_5630546443851562905[103] = 0;
   out_5630546443851562905[104] = 0;
   out_5630546443851562905[105] = 0;
   out_5630546443851562905[106] = 0;
   out_5630546443851562905[107] = 0;
   out_5630546443851562905[108] = 1.0;
   out_5630546443851562905[109] = 0;
   out_5630546443851562905[110] = 0;
   out_5630546443851562905[111] = 0;
   out_5630546443851562905[112] = 0;
   out_5630546443851562905[113] = 0;
   out_5630546443851562905[114] = 0;
   out_5630546443851562905[115] = 0;
   out_5630546443851562905[116] = 0;
   out_5630546443851562905[117] = 0;
   out_5630546443851562905[118] = 0;
   out_5630546443851562905[119] = 0;
   out_5630546443851562905[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3499546418434173997) {
   out_3499546418434173997[0] = dt*state[3] + state[0];
   out_3499546418434173997[1] = dt*state[4] + state[1];
   out_3499546418434173997[2] = dt*state[5] + state[2];
   out_3499546418434173997[3] = state[3];
   out_3499546418434173997[4] = state[4];
   out_3499546418434173997[5] = state[5];
   out_3499546418434173997[6] = dt*state[7] + state[6];
   out_3499546418434173997[7] = dt*state[8] + state[7];
   out_3499546418434173997[8] = state[8];
   out_3499546418434173997[9] = state[9];
   out_3499546418434173997[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6432318896358695090) {
   out_6432318896358695090[0] = 1;
   out_6432318896358695090[1] = 0;
   out_6432318896358695090[2] = 0;
   out_6432318896358695090[3] = dt;
   out_6432318896358695090[4] = 0;
   out_6432318896358695090[5] = 0;
   out_6432318896358695090[6] = 0;
   out_6432318896358695090[7] = 0;
   out_6432318896358695090[8] = 0;
   out_6432318896358695090[9] = 0;
   out_6432318896358695090[10] = 0;
   out_6432318896358695090[11] = 0;
   out_6432318896358695090[12] = 1;
   out_6432318896358695090[13] = 0;
   out_6432318896358695090[14] = 0;
   out_6432318896358695090[15] = dt;
   out_6432318896358695090[16] = 0;
   out_6432318896358695090[17] = 0;
   out_6432318896358695090[18] = 0;
   out_6432318896358695090[19] = 0;
   out_6432318896358695090[20] = 0;
   out_6432318896358695090[21] = 0;
   out_6432318896358695090[22] = 0;
   out_6432318896358695090[23] = 0;
   out_6432318896358695090[24] = 1;
   out_6432318896358695090[25] = 0;
   out_6432318896358695090[26] = 0;
   out_6432318896358695090[27] = dt;
   out_6432318896358695090[28] = 0;
   out_6432318896358695090[29] = 0;
   out_6432318896358695090[30] = 0;
   out_6432318896358695090[31] = 0;
   out_6432318896358695090[32] = 0;
   out_6432318896358695090[33] = 0;
   out_6432318896358695090[34] = 0;
   out_6432318896358695090[35] = 0;
   out_6432318896358695090[36] = 1;
   out_6432318896358695090[37] = 0;
   out_6432318896358695090[38] = 0;
   out_6432318896358695090[39] = 0;
   out_6432318896358695090[40] = 0;
   out_6432318896358695090[41] = 0;
   out_6432318896358695090[42] = 0;
   out_6432318896358695090[43] = 0;
   out_6432318896358695090[44] = 0;
   out_6432318896358695090[45] = 0;
   out_6432318896358695090[46] = 0;
   out_6432318896358695090[47] = 0;
   out_6432318896358695090[48] = 1;
   out_6432318896358695090[49] = 0;
   out_6432318896358695090[50] = 0;
   out_6432318896358695090[51] = 0;
   out_6432318896358695090[52] = 0;
   out_6432318896358695090[53] = 0;
   out_6432318896358695090[54] = 0;
   out_6432318896358695090[55] = 0;
   out_6432318896358695090[56] = 0;
   out_6432318896358695090[57] = 0;
   out_6432318896358695090[58] = 0;
   out_6432318896358695090[59] = 0;
   out_6432318896358695090[60] = 1;
   out_6432318896358695090[61] = 0;
   out_6432318896358695090[62] = 0;
   out_6432318896358695090[63] = 0;
   out_6432318896358695090[64] = 0;
   out_6432318896358695090[65] = 0;
   out_6432318896358695090[66] = 0;
   out_6432318896358695090[67] = 0;
   out_6432318896358695090[68] = 0;
   out_6432318896358695090[69] = 0;
   out_6432318896358695090[70] = 0;
   out_6432318896358695090[71] = 0;
   out_6432318896358695090[72] = 1;
   out_6432318896358695090[73] = dt;
   out_6432318896358695090[74] = 0;
   out_6432318896358695090[75] = 0;
   out_6432318896358695090[76] = 0;
   out_6432318896358695090[77] = 0;
   out_6432318896358695090[78] = 0;
   out_6432318896358695090[79] = 0;
   out_6432318896358695090[80] = 0;
   out_6432318896358695090[81] = 0;
   out_6432318896358695090[82] = 0;
   out_6432318896358695090[83] = 0;
   out_6432318896358695090[84] = 1;
   out_6432318896358695090[85] = dt;
   out_6432318896358695090[86] = 0;
   out_6432318896358695090[87] = 0;
   out_6432318896358695090[88] = 0;
   out_6432318896358695090[89] = 0;
   out_6432318896358695090[90] = 0;
   out_6432318896358695090[91] = 0;
   out_6432318896358695090[92] = 0;
   out_6432318896358695090[93] = 0;
   out_6432318896358695090[94] = 0;
   out_6432318896358695090[95] = 0;
   out_6432318896358695090[96] = 1;
   out_6432318896358695090[97] = 0;
   out_6432318896358695090[98] = 0;
   out_6432318896358695090[99] = 0;
   out_6432318896358695090[100] = 0;
   out_6432318896358695090[101] = 0;
   out_6432318896358695090[102] = 0;
   out_6432318896358695090[103] = 0;
   out_6432318896358695090[104] = 0;
   out_6432318896358695090[105] = 0;
   out_6432318896358695090[106] = 0;
   out_6432318896358695090[107] = 0;
   out_6432318896358695090[108] = 1;
   out_6432318896358695090[109] = 0;
   out_6432318896358695090[110] = 0;
   out_6432318896358695090[111] = 0;
   out_6432318896358695090[112] = 0;
   out_6432318896358695090[113] = 0;
   out_6432318896358695090[114] = 0;
   out_6432318896358695090[115] = 0;
   out_6432318896358695090[116] = 0;
   out_6432318896358695090[117] = 0;
   out_6432318896358695090[118] = 0;
   out_6432318896358695090[119] = 0;
   out_6432318896358695090[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8382688357812697317) {
   out_8382688357812697317[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2433536653280837101) {
   out_2433536653280837101[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2433536653280837101[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2433536653280837101[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2433536653280837101[3] = 0;
   out_2433536653280837101[4] = 0;
   out_2433536653280837101[5] = 0;
   out_2433536653280837101[6] = 1;
   out_2433536653280837101[7] = 0;
   out_2433536653280837101[8] = 0;
   out_2433536653280837101[9] = 0;
   out_2433536653280837101[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7216821719138343950) {
   out_7216821719138343950[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7082167679812894118) {
   out_7082167679812894118[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7082167679812894118[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7082167679812894118[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7082167679812894118[3] = 0;
   out_7082167679812894118[4] = 0;
   out_7082167679812894118[5] = 0;
   out_7082167679812894118[6] = 1;
   out_7082167679812894118[7] = 0;
   out_7082167679812894118[8] = 0;
   out_7082167679812894118[9] = 1;
   out_7082167679812894118[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6504776456211049501) {
   out_6504776456211049501[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_891179350654830313) {
   out_891179350654830313[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[6] = 0;
   out_891179350654830313[7] = 1;
   out_891179350654830313[8] = 0;
   out_891179350654830313[9] = 0;
   out_891179350654830313[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6504776456211049501) {
   out_6504776456211049501[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_891179350654830313) {
   out_891179350654830313[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_891179350654830313[6] = 0;
   out_891179350654830313[7] = 1;
   out_891179350654830313[8] = 0;
   out_891179350654830313[9] = 0;
   out_891179350654830313[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_770261618441231026) {
  err_fun(nom_x, delta_x, out_770261618441231026);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7717952509466708825) {
  inv_err_fun(nom_x, true_x, out_7717952509466708825);
}
void gnss_H_mod_fun(double *state, double *out_5630546443851562905) {
  H_mod_fun(state, out_5630546443851562905);
}
void gnss_f_fun(double *state, double dt, double *out_3499546418434173997) {
  f_fun(state,  dt, out_3499546418434173997);
}
void gnss_F_fun(double *state, double dt, double *out_6432318896358695090) {
  F_fun(state,  dt, out_6432318896358695090);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8382688357812697317) {
  h_6(state, sat_pos, out_8382688357812697317);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2433536653280837101) {
  H_6(state, sat_pos, out_2433536653280837101);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7216821719138343950) {
  h_20(state, sat_pos, out_7216821719138343950);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7082167679812894118) {
  H_20(state, sat_pos, out_7082167679812894118);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6504776456211049501) {
  h_7(state, sat_pos_vel, out_6504776456211049501);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_891179350654830313) {
  H_7(state, sat_pos_vel, out_891179350654830313);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6504776456211049501) {
  h_21(state, sat_pos_vel, out_6504776456211049501);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_891179350654830313) {
  H_21(state, sat_pos_vel, out_891179350654830313);
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
