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
void err_fun(double *nom_x, double *delta_x, double *out_5909540454426075247) {
   out_5909540454426075247[0] = delta_x[0] + nom_x[0];
   out_5909540454426075247[1] = delta_x[1] + nom_x[1];
   out_5909540454426075247[2] = delta_x[2] + nom_x[2];
   out_5909540454426075247[3] = delta_x[3] + nom_x[3];
   out_5909540454426075247[4] = delta_x[4] + nom_x[4];
   out_5909540454426075247[5] = delta_x[5] + nom_x[5];
   out_5909540454426075247[6] = delta_x[6] + nom_x[6];
   out_5909540454426075247[7] = delta_x[7] + nom_x[7];
   out_5909540454426075247[8] = delta_x[8] + nom_x[8];
   out_5909540454426075247[9] = delta_x[9] + nom_x[9];
   out_5909540454426075247[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8457296220736712046) {
   out_8457296220736712046[0] = -nom_x[0] + true_x[0];
   out_8457296220736712046[1] = -nom_x[1] + true_x[1];
   out_8457296220736712046[2] = -nom_x[2] + true_x[2];
   out_8457296220736712046[3] = -nom_x[3] + true_x[3];
   out_8457296220736712046[4] = -nom_x[4] + true_x[4];
   out_8457296220736712046[5] = -nom_x[5] + true_x[5];
   out_8457296220736712046[6] = -nom_x[6] + true_x[6];
   out_8457296220736712046[7] = -nom_x[7] + true_x[7];
   out_8457296220736712046[8] = -nom_x[8] + true_x[8];
   out_8457296220736712046[9] = -nom_x[9] + true_x[9];
   out_8457296220736712046[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_8531965417689122995) {
   out_8531965417689122995[0] = 1.0;
   out_8531965417689122995[1] = 0;
   out_8531965417689122995[2] = 0;
   out_8531965417689122995[3] = 0;
   out_8531965417689122995[4] = 0;
   out_8531965417689122995[5] = 0;
   out_8531965417689122995[6] = 0;
   out_8531965417689122995[7] = 0;
   out_8531965417689122995[8] = 0;
   out_8531965417689122995[9] = 0;
   out_8531965417689122995[10] = 0;
   out_8531965417689122995[11] = 0;
   out_8531965417689122995[12] = 1.0;
   out_8531965417689122995[13] = 0;
   out_8531965417689122995[14] = 0;
   out_8531965417689122995[15] = 0;
   out_8531965417689122995[16] = 0;
   out_8531965417689122995[17] = 0;
   out_8531965417689122995[18] = 0;
   out_8531965417689122995[19] = 0;
   out_8531965417689122995[20] = 0;
   out_8531965417689122995[21] = 0;
   out_8531965417689122995[22] = 0;
   out_8531965417689122995[23] = 0;
   out_8531965417689122995[24] = 1.0;
   out_8531965417689122995[25] = 0;
   out_8531965417689122995[26] = 0;
   out_8531965417689122995[27] = 0;
   out_8531965417689122995[28] = 0;
   out_8531965417689122995[29] = 0;
   out_8531965417689122995[30] = 0;
   out_8531965417689122995[31] = 0;
   out_8531965417689122995[32] = 0;
   out_8531965417689122995[33] = 0;
   out_8531965417689122995[34] = 0;
   out_8531965417689122995[35] = 0;
   out_8531965417689122995[36] = 1.0;
   out_8531965417689122995[37] = 0;
   out_8531965417689122995[38] = 0;
   out_8531965417689122995[39] = 0;
   out_8531965417689122995[40] = 0;
   out_8531965417689122995[41] = 0;
   out_8531965417689122995[42] = 0;
   out_8531965417689122995[43] = 0;
   out_8531965417689122995[44] = 0;
   out_8531965417689122995[45] = 0;
   out_8531965417689122995[46] = 0;
   out_8531965417689122995[47] = 0;
   out_8531965417689122995[48] = 1.0;
   out_8531965417689122995[49] = 0;
   out_8531965417689122995[50] = 0;
   out_8531965417689122995[51] = 0;
   out_8531965417689122995[52] = 0;
   out_8531965417689122995[53] = 0;
   out_8531965417689122995[54] = 0;
   out_8531965417689122995[55] = 0;
   out_8531965417689122995[56] = 0;
   out_8531965417689122995[57] = 0;
   out_8531965417689122995[58] = 0;
   out_8531965417689122995[59] = 0;
   out_8531965417689122995[60] = 1.0;
   out_8531965417689122995[61] = 0;
   out_8531965417689122995[62] = 0;
   out_8531965417689122995[63] = 0;
   out_8531965417689122995[64] = 0;
   out_8531965417689122995[65] = 0;
   out_8531965417689122995[66] = 0;
   out_8531965417689122995[67] = 0;
   out_8531965417689122995[68] = 0;
   out_8531965417689122995[69] = 0;
   out_8531965417689122995[70] = 0;
   out_8531965417689122995[71] = 0;
   out_8531965417689122995[72] = 1.0;
   out_8531965417689122995[73] = 0;
   out_8531965417689122995[74] = 0;
   out_8531965417689122995[75] = 0;
   out_8531965417689122995[76] = 0;
   out_8531965417689122995[77] = 0;
   out_8531965417689122995[78] = 0;
   out_8531965417689122995[79] = 0;
   out_8531965417689122995[80] = 0;
   out_8531965417689122995[81] = 0;
   out_8531965417689122995[82] = 0;
   out_8531965417689122995[83] = 0;
   out_8531965417689122995[84] = 1.0;
   out_8531965417689122995[85] = 0;
   out_8531965417689122995[86] = 0;
   out_8531965417689122995[87] = 0;
   out_8531965417689122995[88] = 0;
   out_8531965417689122995[89] = 0;
   out_8531965417689122995[90] = 0;
   out_8531965417689122995[91] = 0;
   out_8531965417689122995[92] = 0;
   out_8531965417689122995[93] = 0;
   out_8531965417689122995[94] = 0;
   out_8531965417689122995[95] = 0;
   out_8531965417689122995[96] = 1.0;
   out_8531965417689122995[97] = 0;
   out_8531965417689122995[98] = 0;
   out_8531965417689122995[99] = 0;
   out_8531965417689122995[100] = 0;
   out_8531965417689122995[101] = 0;
   out_8531965417689122995[102] = 0;
   out_8531965417689122995[103] = 0;
   out_8531965417689122995[104] = 0;
   out_8531965417689122995[105] = 0;
   out_8531965417689122995[106] = 0;
   out_8531965417689122995[107] = 0;
   out_8531965417689122995[108] = 1.0;
   out_8531965417689122995[109] = 0;
   out_8531965417689122995[110] = 0;
   out_8531965417689122995[111] = 0;
   out_8531965417689122995[112] = 0;
   out_8531965417689122995[113] = 0;
   out_8531965417689122995[114] = 0;
   out_8531965417689122995[115] = 0;
   out_8531965417689122995[116] = 0;
   out_8531965417689122995[117] = 0;
   out_8531965417689122995[118] = 0;
   out_8531965417689122995[119] = 0;
   out_8531965417689122995[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6195388292700041643) {
   out_6195388292700041643[0] = dt*state[3] + state[0];
   out_6195388292700041643[1] = dt*state[4] + state[1];
   out_6195388292700041643[2] = dt*state[5] + state[2];
   out_6195388292700041643[3] = state[3];
   out_6195388292700041643[4] = state[4];
   out_6195388292700041643[5] = state[5];
   out_6195388292700041643[6] = dt*state[7] + state[6];
   out_6195388292700041643[7] = dt*state[8] + state[7];
   out_6195388292700041643[8] = state[8];
   out_6195388292700041643[9] = state[9];
   out_6195388292700041643[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7909854684745487152) {
   out_7909854684745487152[0] = 1;
   out_7909854684745487152[1] = 0;
   out_7909854684745487152[2] = 0;
   out_7909854684745487152[3] = dt;
   out_7909854684745487152[4] = 0;
   out_7909854684745487152[5] = 0;
   out_7909854684745487152[6] = 0;
   out_7909854684745487152[7] = 0;
   out_7909854684745487152[8] = 0;
   out_7909854684745487152[9] = 0;
   out_7909854684745487152[10] = 0;
   out_7909854684745487152[11] = 0;
   out_7909854684745487152[12] = 1;
   out_7909854684745487152[13] = 0;
   out_7909854684745487152[14] = 0;
   out_7909854684745487152[15] = dt;
   out_7909854684745487152[16] = 0;
   out_7909854684745487152[17] = 0;
   out_7909854684745487152[18] = 0;
   out_7909854684745487152[19] = 0;
   out_7909854684745487152[20] = 0;
   out_7909854684745487152[21] = 0;
   out_7909854684745487152[22] = 0;
   out_7909854684745487152[23] = 0;
   out_7909854684745487152[24] = 1;
   out_7909854684745487152[25] = 0;
   out_7909854684745487152[26] = 0;
   out_7909854684745487152[27] = dt;
   out_7909854684745487152[28] = 0;
   out_7909854684745487152[29] = 0;
   out_7909854684745487152[30] = 0;
   out_7909854684745487152[31] = 0;
   out_7909854684745487152[32] = 0;
   out_7909854684745487152[33] = 0;
   out_7909854684745487152[34] = 0;
   out_7909854684745487152[35] = 0;
   out_7909854684745487152[36] = 1;
   out_7909854684745487152[37] = 0;
   out_7909854684745487152[38] = 0;
   out_7909854684745487152[39] = 0;
   out_7909854684745487152[40] = 0;
   out_7909854684745487152[41] = 0;
   out_7909854684745487152[42] = 0;
   out_7909854684745487152[43] = 0;
   out_7909854684745487152[44] = 0;
   out_7909854684745487152[45] = 0;
   out_7909854684745487152[46] = 0;
   out_7909854684745487152[47] = 0;
   out_7909854684745487152[48] = 1;
   out_7909854684745487152[49] = 0;
   out_7909854684745487152[50] = 0;
   out_7909854684745487152[51] = 0;
   out_7909854684745487152[52] = 0;
   out_7909854684745487152[53] = 0;
   out_7909854684745487152[54] = 0;
   out_7909854684745487152[55] = 0;
   out_7909854684745487152[56] = 0;
   out_7909854684745487152[57] = 0;
   out_7909854684745487152[58] = 0;
   out_7909854684745487152[59] = 0;
   out_7909854684745487152[60] = 1;
   out_7909854684745487152[61] = 0;
   out_7909854684745487152[62] = 0;
   out_7909854684745487152[63] = 0;
   out_7909854684745487152[64] = 0;
   out_7909854684745487152[65] = 0;
   out_7909854684745487152[66] = 0;
   out_7909854684745487152[67] = 0;
   out_7909854684745487152[68] = 0;
   out_7909854684745487152[69] = 0;
   out_7909854684745487152[70] = 0;
   out_7909854684745487152[71] = 0;
   out_7909854684745487152[72] = 1;
   out_7909854684745487152[73] = dt;
   out_7909854684745487152[74] = 0;
   out_7909854684745487152[75] = 0;
   out_7909854684745487152[76] = 0;
   out_7909854684745487152[77] = 0;
   out_7909854684745487152[78] = 0;
   out_7909854684745487152[79] = 0;
   out_7909854684745487152[80] = 0;
   out_7909854684745487152[81] = 0;
   out_7909854684745487152[82] = 0;
   out_7909854684745487152[83] = 0;
   out_7909854684745487152[84] = 1;
   out_7909854684745487152[85] = dt;
   out_7909854684745487152[86] = 0;
   out_7909854684745487152[87] = 0;
   out_7909854684745487152[88] = 0;
   out_7909854684745487152[89] = 0;
   out_7909854684745487152[90] = 0;
   out_7909854684745487152[91] = 0;
   out_7909854684745487152[92] = 0;
   out_7909854684745487152[93] = 0;
   out_7909854684745487152[94] = 0;
   out_7909854684745487152[95] = 0;
   out_7909854684745487152[96] = 1;
   out_7909854684745487152[97] = 0;
   out_7909854684745487152[98] = 0;
   out_7909854684745487152[99] = 0;
   out_7909854684745487152[100] = 0;
   out_7909854684745487152[101] = 0;
   out_7909854684745487152[102] = 0;
   out_7909854684745487152[103] = 0;
   out_7909854684745487152[104] = 0;
   out_7909854684745487152[105] = 0;
   out_7909854684745487152[106] = 0;
   out_7909854684745487152[107] = 0;
   out_7909854684745487152[108] = 1;
   out_7909854684745487152[109] = 0;
   out_7909854684745487152[110] = 0;
   out_7909854684745487152[111] = 0;
   out_7909854684745487152[112] = 0;
   out_7909854684745487152[113] = 0;
   out_7909854684745487152[114] = 0;
   out_7909854684745487152[115] = 0;
   out_7909854684745487152[116] = 0;
   out_7909854684745487152[117] = 0;
   out_7909854684745487152[118] = 0;
   out_7909854684745487152[119] = 0;
   out_7909854684745487152[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3920509931570091493) {
   out_3920509931570091493[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8872851097972471612) {
   out_8872851097972471612[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8872851097972471612[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8872851097972471612[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8872851097972471612[3] = 0;
   out_8872851097972471612[4] = 0;
   out_8872851097972471612[5] = 0;
   out_8872851097972471612[6] = 1;
   out_8872851097972471612[7] = 0;
   out_8872851097972471612[8] = 0;
   out_8872851097972471612[9] = 0;
   out_8872851097972471612[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7175736005951966427) {
   out_7175736005951966427[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3709328815680046837) {
   out_3709328815680046837[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3709328815680046837[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3709328815680046837[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3709328815680046837[3] = 0;
   out_3709328815680046837[4] = 0;
   out_3709328815680046837[5] = 0;
   out_3709328815680046837[6] = 1;
   out_3709328815680046837[7] = 0;
   out_3709328815680046837[8] = 0;
   out_3709328815680046837[9] = 1;
   out_3709328815680046837[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2137916395300013369) {
   out_2137916395300013369[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7240879494761677841) {
   out_7240879494761677841[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[6] = 0;
   out_7240879494761677841[7] = 1;
   out_7240879494761677841[8] = 0;
   out_7240879494761677841[9] = 0;
   out_7240879494761677841[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2137916395300013369) {
   out_2137916395300013369[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7240879494761677841) {
   out_7240879494761677841[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7240879494761677841[6] = 0;
   out_7240879494761677841[7] = 1;
   out_7240879494761677841[8] = 0;
   out_7240879494761677841[9] = 0;
   out_7240879494761677841[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5909540454426075247) {
  err_fun(nom_x, delta_x, out_5909540454426075247);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8457296220736712046) {
  inv_err_fun(nom_x, true_x, out_8457296220736712046);
}
void gnss_H_mod_fun(double *state, double *out_8531965417689122995) {
  H_mod_fun(state, out_8531965417689122995);
}
void gnss_f_fun(double *state, double dt, double *out_6195388292700041643) {
  f_fun(state,  dt, out_6195388292700041643);
}
void gnss_F_fun(double *state, double dt, double *out_7909854684745487152) {
  F_fun(state,  dt, out_7909854684745487152);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3920509931570091493) {
  h_6(state, sat_pos, out_3920509931570091493);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8872851097972471612) {
  H_6(state, sat_pos, out_8872851097972471612);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7175736005951966427) {
  h_20(state, sat_pos, out_7175736005951966427);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3709328815680046837) {
  H_20(state, sat_pos, out_3709328815680046837);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2137916395300013369) {
  h_7(state, sat_pos_vel, out_2137916395300013369);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7240879494761677841) {
  H_7(state, sat_pos_vel, out_7240879494761677841);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2137916395300013369) {
  h_21(state, sat_pos_vel, out_2137916395300013369);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7240879494761677841) {
  H_21(state, sat_pos_vel, out_7240879494761677841);
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
