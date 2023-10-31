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
void err_fun(double *nom_x, double *delta_x, double *out_3580552561584141668) {
   out_3580552561584141668[0] = delta_x[0] + nom_x[0];
   out_3580552561584141668[1] = delta_x[1] + nom_x[1];
   out_3580552561584141668[2] = delta_x[2] + nom_x[2];
   out_3580552561584141668[3] = delta_x[3] + nom_x[3];
   out_3580552561584141668[4] = delta_x[4] + nom_x[4];
   out_3580552561584141668[5] = delta_x[5] + nom_x[5];
   out_3580552561584141668[6] = delta_x[6] + nom_x[6];
   out_3580552561584141668[7] = delta_x[7] + nom_x[7];
   out_3580552561584141668[8] = delta_x[8] + nom_x[8];
   out_3580552561584141668[9] = delta_x[9] + nom_x[9];
   out_3580552561584141668[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3985669647031736288) {
   out_3985669647031736288[0] = -nom_x[0] + true_x[0];
   out_3985669647031736288[1] = -nom_x[1] + true_x[1];
   out_3985669647031736288[2] = -nom_x[2] + true_x[2];
   out_3985669647031736288[3] = -nom_x[3] + true_x[3];
   out_3985669647031736288[4] = -nom_x[4] + true_x[4];
   out_3985669647031736288[5] = -nom_x[5] + true_x[5];
   out_3985669647031736288[6] = -nom_x[6] + true_x[6];
   out_3985669647031736288[7] = -nom_x[7] + true_x[7];
   out_3985669647031736288[8] = -nom_x[8] + true_x[8];
   out_3985669647031736288[9] = -nom_x[9] + true_x[9];
   out_3985669647031736288[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3674751707753416237) {
   out_3674751707753416237[0] = 1.0;
   out_3674751707753416237[1] = 0;
   out_3674751707753416237[2] = 0;
   out_3674751707753416237[3] = 0;
   out_3674751707753416237[4] = 0;
   out_3674751707753416237[5] = 0;
   out_3674751707753416237[6] = 0;
   out_3674751707753416237[7] = 0;
   out_3674751707753416237[8] = 0;
   out_3674751707753416237[9] = 0;
   out_3674751707753416237[10] = 0;
   out_3674751707753416237[11] = 0;
   out_3674751707753416237[12] = 1.0;
   out_3674751707753416237[13] = 0;
   out_3674751707753416237[14] = 0;
   out_3674751707753416237[15] = 0;
   out_3674751707753416237[16] = 0;
   out_3674751707753416237[17] = 0;
   out_3674751707753416237[18] = 0;
   out_3674751707753416237[19] = 0;
   out_3674751707753416237[20] = 0;
   out_3674751707753416237[21] = 0;
   out_3674751707753416237[22] = 0;
   out_3674751707753416237[23] = 0;
   out_3674751707753416237[24] = 1.0;
   out_3674751707753416237[25] = 0;
   out_3674751707753416237[26] = 0;
   out_3674751707753416237[27] = 0;
   out_3674751707753416237[28] = 0;
   out_3674751707753416237[29] = 0;
   out_3674751707753416237[30] = 0;
   out_3674751707753416237[31] = 0;
   out_3674751707753416237[32] = 0;
   out_3674751707753416237[33] = 0;
   out_3674751707753416237[34] = 0;
   out_3674751707753416237[35] = 0;
   out_3674751707753416237[36] = 1.0;
   out_3674751707753416237[37] = 0;
   out_3674751707753416237[38] = 0;
   out_3674751707753416237[39] = 0;
   out_3674751707753416237[40] = 0;
   out_3674751707753416237[41] = 0;
   out_3674751707753416237[42] = 0;
   out_3674751707753416237[43] = 0;
   out_3674751707753416237[44] = 0;
   out_3674751707753416237[45] = 0;
   out_3674751707753416237[46] = 0;
   out_3674751707753416237[47] = 0;
   out_3674751707753416237[48] = 1.0;
   out_3674751707753416237[49] = 0;
   out_3674751707753416237[50] = 0;
   out_3674751707753416237[51] = 0;
   out_3674751707753416237[52] = 0;
   out_3674751707753416237[53] = 0;
   out_3674751707753416237[54] = 0;
   out_3674751707753416237[55] = 0;
   out_3674751707753416237[56] = 0;
   out_3674751707753416237[57] = 0;
   out_3674751707753416237[58] = 0;
   out_3674751707753416237[59] = 0;
   out_3674751707753416237[60] = 1.0;
   out_3674751707753416237[61] = 0;
   out_3674751707753416237[62] = 0;
   out_3674751707753416237[63] = 0;
   out_3674751707753416237[64] = 0;
   out_3674751707753416237[65] = 0;
   out_3674751707753416237[66] = 0;
   out_3674751707753416237[67] = 0;
   out_3674751707753416237[68] = 0;
   out_3674751707753416237[69] = 0;
   out_3674751707753416237[70] = 0;
   out_3674751707753416237[71] = 0;
   out_3674751707753416237[72] = 1.0;
   out_3674751707753416237[73] = 0;
   out_3674751707753416237[74] = 0;
   out_3674751707753416237[75] = 0;
   out_3674751707753416237[76] = 0;
   out_3674751707753416237[77] = 0;
   out_3674751707753416237[78] = 0;
   out_3674751707753416237[79] = 0;
   out_3674751707753416237[80] = 0;
   out_3674751707753416237[81] = 0;
   out_3674751707753416237[82] = 0;
   out_3674751707753416237[83] = 0;
   out_3674751707753416237[84] = 1.0;
   out_3674751707753416237[85] = 0;
   out_3674751707753416237[86] = 0;
   out_3674751707753416237[87] = 0;
   out_3674751707753416237[88] = 0;
   out_3674751707753416237[89] = 0;
   out_3674751707753416237[90] = 0;
   out_3674751707753416237[91] = 0;
   out_3674751707753416237[92] = 0;
   out_3674751707753416237[93] = 0;
   out_3674751707753416237[94] = 0;
   out_3674751707753416237[95] = 0;
   out_3674751707753416237[96] = 1.0;
   out_3674751707753416237[97] = 0;
   out_3674751707753416237[98] = 0;
   out_3674751707753416237[99] = 0;
   out_3674751707753416237[100] = 0;
   out_3674751707753416237[101] = 0;
   out_3674751707753416237[102] = 0;
   out_3674751707753416237[103] = 0;
   out_3674751707753416237[104] = 0;
   out_3674751707753416237[105] = 0;
   out_3674751707753416237[106] = 0;
   out_3674751707753416237[107] = 0;
   out_3674751707753416237[108] = 1.0;
   out_3674751707753416237[109] = 0;
   out_3674751707753416237[110] = 0;
   out_3674751707753416237[111] = 0;
   out_3674751707753416237[112] = 0;
   out_3674751707753416237[113] = 0;
   out_3674751707753416237[114] = 0;
   out_3674751707753416237[115] = 0;
   out_3674751707753416237[116] = 0;
   out_3674751707753416237[117] = 0;
   out_3674751707753416237[118] = 0;
   out_3674751707753416237[119] = 0;
   out_3674751707753416237[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5581775936242371520) {
   out_5581775936242371520[0] = dt*state[3] + state[0];
   out_5581775936242371520[1] = dt*state[4] + state[1];
   out_5581775936242371520[2] = dt*state[5] + state[2];
   out_5581775936242371520[3] = state[3];
   out_5581775936242371520[4] = state[4];
   out_5581775936242371520[5] = state[5];
   out_5581775936242371520[6] = dt*state[7] + state[6];
   out_5581775936242371520[7] = dt*state[8] + state[7];
   out_5581775936242371520[8] = state[8];
   out_5581775936242371520[9] = state[9];
   out_5581775936242371520[10] = state[10];
}
void F_fun(double *state, double dt, double *out_9136268394026262564) {
   out_9136268394026262564[0] = 1;
   out_9136268394026262564[1] = 0;
   out_9136268394026262564[2] = 0;
   out_9136268394026262564[3] = dt;
   out_9136268394026262564[4] = 0;
   out_9136268394026262564[5] = 0;
   out_9136268394026262564[6] = 0;
   out_9136268394026262564[7] = 0;
   out_9136268394026262564[8] = 0;
   out_9136268394026262564[9] = 0;
   out_9136268394026262564[10] = 0;
   out_9136268394026262564[11] = 0;
   out_9136268394026262564[12] = 1;
   out_9136268394026262564[13] = 0;
   out_9136268394026262564[14] = 0;
   out_9136268394026262564[15] = dt;
   out_9136268394026262564[16] = 0;
   out_9136268394026262564[17] = 0;
   out_9136268394026262564[18] = 0;
   out_9136268394026262564[19] = 0;
   out_9136268394026262564[20] = 0;
   out_9136268394026262564[21] = 0;
   out_9136268394026262564[22] = 0;
   out_9136268394026262564[23] = 0;
   out_9136268394026262564[24] = 1;
   out_9136268394026262564[25] = 0;
   out_9136268394026262564[26] = 0;
   out_9136268394026262564[27] = dt;
   out_9136268394026262564[28] = 0;
   out_9136268394026262564[29] = 0;
   out_9136268394026262564[30] = 0;
   out_9136268394026262564[31] = 0;
   out_9136268394026262564[32] = 0;
   out_9136268394026262564[33] = 0;
   out_9136268394026262564[34] = 0;
   out_9136268394026262564[35] = 0;
   out_9136268394026262564[36] = 1;
   out_9136268394026262564[37] = 0;
   out_9136268394026262564[38] = 0;
   out_9136268394026262564[39] = 0;
   out_9136268394026262564[40] = 0;
   out_9136268394026262564[41] = 0;
   out_9136268394026262564[42] = 0;
   out_9136268394026262564[43] = 0;
   out_9136268394026262564[44] = 0;
   out_9136268394026262564[45] = 0;
   out_9136268394026262564[46] = 0;
   out_9136268394026262564[47] = 0;
   out_9136268394026262564[48] = 1;
   out_9136268394026262564[49] = 0;
   out_9136268394026262564[50] = 0;
   out_9136268394026262564[51] = 0;
   out_9136268394026262564[52] = 0;
   out_9136268394026262564[53] = 0;
   out_9136268394026262564[54] = 0;
   out_9136268394026262564[55] = 0;
   out_9136268394026262564[56] = 0;
   out_9136268394026262564[57] = 0;
   out_9136268394026262564[58] = 0;
   out_9136268394026262564[59] = 0;
   out_9136268394026262564[60] = 1;
   out_9136268394026262564[61] = 0;
   out_9136268394026262564[62] = 0;
   out_9136268394026262564[63] = 0;
   out_9136268394026262564[64] = 0;
   out_9136268394026262564[65] = 0;
   out_9136268394026262564[66] = 0;
   out_9136268394026262564[67] = 0;
   out_9136268394026262564[68] = 0;
   out_9136268394026262564[69] = 0;
   out_9136268394026262564[70] = 0;
   out_9136268394026262564[71] = 0;
   out_9136268394026262564[72] = 1;
   out_9136268394026262564[73] = dt;
   out_9136268394026262564[74] = 0;
   out_9136268394026262564[75] = 0;
   out_9136268394026262564[76] = 0;
   out_9136268394026262564[77] = 0;
   out_9136268394026262564[78] = 0;
   out_9136268394026262564[79] = 0;
   out_9136268394026262564[80] = 0;
   out_9136268394026262564[81] = 0;
   out_9136268394026262564[82] = 0;
   out_9136268394026262564[83] = 0;
   out_9136268394026262564[84] = 1;
   out_9136268394026262564[85] = dt;
   out_9136268394026262564[86] = 0;
   out_9136268394026262564[87] = 0;
   out_9136268394026262564[88] = 0;
   out_9136268394026262564[89] = 0;
   out_9136268394026262564[90] = 0;
   out_9136268394026262564[91] = 0;
   out_9136268394026262564[92] = 0;
   out_9136268394026262564[93] = 0;
   out_9136268394026262564[94] = 0;
   out_9136268394026262564[95] = 0;
   out_9136268394026262564[96] = 1;
   out_9136268394026262564[97] = 0;
   out_9136268394026262564[98] = 0;
   out_9136268394026262564[99] = 0;
   out_9136268394026262564[100] = 0;
   out_9136268394026262564[101] = 0;
   out_9136268394026262564[102] = 0;
   out_9136268394026262564[103] = 0;
   out_9136268394026262564[104] = 0;
   out_9136268394026262564[105] = 0;
   out_9136268394026262564[106] = 0;
   out_9136268394026262564[107] = 0;
   out_9136268394026262564[108] = 1;
   out_9136268394026262564[109] = 0;
   out_9136268394026262564[110] = 0;
   out_9136268394026262564[111] = 0;
   out_9136268394026262564[112] = 0;
   out_9136268394026262564[113] = 0;
   out_9136268394026262564[114] = 0;
   out_9136268394026262564[115] = 0;
   out_9136268394026262564[116] = 0;
   out_9136268394026262564[117] = 0;
   out_9136268394026262564[118] = 0;
   out_9136268394026262564[119] = 0;
   out_9136268394026262564[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8396154811134387028) {
   out_8396154811134387028[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8585197235663590162) {
   out_8585197235663590162[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8585197235663590162[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8585197235663590162[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8585197235663590162[3] = 0;
   out_8585197235663590162[4] = 0;
   out_8585197235663590162[5] = 0;
   out_8585197235663590162[6] = 1;
   out_8585197235663590162[7] = 0;
   out_8585197235663590162[8] = 0;
   out_8585197235663590162[9] = 0;
   out_8585197235663590162[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3794563826398603374) {
   out_3794563826398603374[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2931996085169558241) {
   out_2931996085169558241[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2931996085169558241[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2931996085169558241[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2931996085169558241[3] = 0;
   out_2931996085169558241[4] = 0;
   out_2931996085169558241[5] = 0;
   out_2931996085169558241[6] = 1;
   out_2931996085169558241[7] = 0;
   out_2931996085169558241[8] = 0;
   out_2931996085169558241[9] = 1;
   out_2931996085169558241[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7026932990701218247) {
   out_7026932990701218247[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3463380596821841936) {
   out_3463380596821841936[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[6] = 0;
   out_3463380596821841936[7] = 1;
   out_3463380596821841936[8] = 0;
   out_3463380596821841936[9] = 0;
   out_3463380596821841936[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7026932990701218247) {
   out_7026932990701218247[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3463380596821841936) {
   out_3463380596821841936[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3463380596821841936[6] = 0;
   out_3463380596821841936[7] = 1;
   out_3463380596821841936[8] = 0;
   out_3463380596821841936[9] = 0;
   out_3463380596821841936[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3580552561584141668) {
  err_fun(nom_x, delta_x, out_3580552561584141668);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3985669647031736288) {
  inv_err_fun(nom_x, true_x, out_3985669647031736288);
}
void gnss_H_mod_fun(double *state, double *out_3674751707753416237) {
  H_mod_fun(state, out_3674751707753416237);
}
void gnss_f_fun(double *state, double dt, double *out_5581775936242371520) {
  f_fun(state,  dt, out_5581775936242371520);
}
void gnss_F_fun(double *state, double dt, double *out_9136268394026262564) {
  F_fun(state,  dt, out_9136268394026262564);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8396154811134387028) {
  h_6(state, sat_pos, out_8396154811134387028);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8585197235663590162) {
  H_6(state, sat_pos, out_8585197235663590162);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3794563826398603374) {
  h_20(state, sat_pos, out_3794563826398603374);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2931996085169558241) {
  H_20(state, sat_pos, out_2931996085169558241);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7026932990701218247) {
  h_7(state, sat_pos_vel, out_7026932990701218247);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3463380596821841936) {
  H_7(state, sat_pos_vel, out_3463380596821841936);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7026932990701218247) {
  h_21(state, sat_pos_vel, out_7026932990701218247);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3463380596821841936) {
  H_21(state, sat_pos_vel, out_3463380596821841936);
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
