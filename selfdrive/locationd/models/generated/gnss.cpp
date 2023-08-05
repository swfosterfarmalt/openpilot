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
void err_fun(double *nom_x, double *delta_x, double *out_5797809671637013283) {
   out_5797809671637013283[0] = delta_x[0] + nom_x[0];
   out_5797809671637013283[1] = delta_x[1] + nom_x[1];
   out_5797809671637013283[2] = delta_x[2] + nom_x[2];
   out_5797809671637013283[3] = delta_x[3] + nom_x[3];
   out_5797809671637013283[4] = delta_x[4] + nom_x[4];
   out_5797809671637013283[5] = delta_x[5] + nom_x[5];
   out_5797809671637013283[6] = delta_x[6] + nom_x[6];
   out_5797809671637013283[7] = delta_x[7] + nom_x[7];
   out_5797809671637013283[8] = delta_x[8] + nom_x[8];
   out_5797809671637013283[9] = delta_x[9] + nom_x[9];
   out_5797809671637013283[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2749815054075637968) {
   out_2749815054075637968[0] = -nom_x[0] + true_x[0];
   out_2749815054075637968[1] = -nom_x[1] + true_x[1];
   out_2749815054075637968[2] = -nom_x[2] + true_x[2];
   out_2749815054075637968[3] = -nom_x[3] + true_x[3];
   out_2749815054075637968[4] = -nom_x[4] + true_x[4];
   out_2749815054075637968[5] = -nom_x[5] + true_x[5];
   out_2749815054075637968[6] = -nom_x[6] + true_x[6];
   out_2749815054075637968[7] = -nom_x[7] + true_x[7];
   out_2749815054075637968[8] = -nom_x[8] + true_x[8];
   out_2749815054075637968[9] = -nom_x[9] + true_x[9];
   out_2749815054075637968[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2172530486751409334) {
   out_2172530486751409334[0] = 1.0;
   out_2172530486751409334[1] = 0;
   out_2172530486751409334[2] = 0;
   out_2172530486751409334[3] = 0;
   out_2172530486751409334[4] = 0;
   out_2172530486751409334[5] = 0;
   out_2172530486751409334[6] = 0;
   out_2172530486751409334[7] = 0;
   out_2172530486751409334[8] = 0;
   out_2172530486751409334[9] = 0;
   out_2172530486751409334[10] = 0;
   out_2172530486751409334[11] = 0;
   out_2172530486751409334[12] = 1.0;
   out_2172530486751409334[13] = 0;
   out_2172530486751409334[14] = 0;
   out_2172530486751409334[15] = 0;
   out_2172530486751409334[16] = 0;
   out_2172530486751409334[17] = 0;
   out_2172530486751409334[18] = 0;
   out_2172530486751409334[19] = 0;
   out_2172530486751409334[20] = 0;
   out_2172530486751409334[21] = 0;
   out_2172530486751409334[22] = 0;
   out_2172530486751409334[23] = 0;
   out_2172530486751409334[24] = 1.0;
   out_2172530486751409334[25] = 0;
   out_2172530486751409334[26] = 0;
   out_2172530486751409334[27] = 0;
   out_2172530486751409334[28] = 0;
   out_2172530486751409334[29] = 0;
   out_2172530486751409334[30] = 0;
   out_2172530486751409334[31] = 0;
   out_2172530486751409334[32] = 0;
   out_2172530486751409334[33] = 0;
   out_2172530486751409334[34] = 0;
   out_2172530486751409334[35] = 0;
   out_2172530486751409334[36] = 1.0;
   out_2172530486751409334[37] = 0;
   out_2172530486751409334[38] = 0;
   out_2172530486751409334[39] = 0;
   out_2172530486751409334[40] = 0;
   out_2172530486751409334[41] = 0;
   out_2172530486751409334[42] = 0;
   out_2172530486751409334[43] = 0;
   out_2172530486751409334[44] = 0;
   out_2172530486751409334[45] = 0;
   out_2172530486751409334[46] = 0;
   out_2172530486751409334[47] = 0;
   out_2172530486751409334[48] = 1.0;
   out_2172530486751409334[49] = 0;
   out_2172530486751409334[50] = 0;
   out_2172530486751409334[51] = 0;
   out_2172530486751409334[52] = 0;
   out_2172530486751409334[53] = 0;
   out_2172530486751409334[54] = 0;
   out_2172530486751409334[55] = 0;
   out_2172530486751409334[56] = 0;
   out_2172530486751409334[57] = 0;
   out_2172530486751409334[58] = 0;
   out_2172530486751409334[59] = 0;
   out_2172530486751409334[60] = 1.0;
   out_2172530486751409334[61] = 0;
   out_2172530486751409334[62] = 0;
   out_2172530486751409334[63] = 0;
   out_2172530486751409334[64] = 0;
   out_2172530486751409334[65] = 0;
   out_2172530486751409334[66] = 0;
   out_2172530486751409334[67] = 0;
   out_2172530486751409334[68] = 0;
   out_2172530486751409334[69] = 0;
   out_2172530486751409334[70] = 0;
   out_2172530486751409334[71] = 0;
   out_2172530486751409334[72] = 1.0;
   out_2172530486751409334[73] = 0;
   out_2172530486751409334[74] = 0;
   out_2172530486751409334[75] = 0;
   out_2172530486751409334[76] = 0;
   out_2172530486751409334[77] = 0;
   out_2172530486751409334[78] = 0;
   out_2172530486751409334[79] = 0;
   out_2172530486751409334[80] = 0;
   out_2172530486751409334[81] = 0;
   out_2172530486751409334[82] = 0;
   out_2172530486751409334[83] = 0;
   out_2172530486751409334[84] = 1.0;
   out_2172530486751409334[85] = 0;
   out_2172530486751409334[86] = 0;
   out_2172530486751409334[87] = 0;
   out_2172530486751409334[88] = 0;
   out_2172530486751409334[89] = 0;
   out_2172530486751409334[90] = 0;
   out_2172530486751409334[91] = 0;
   out_2172530486751409334[92] = 0;
   out_2172530486751409334[93] = 0;
   out_2172530486751409334[94] = 0;
   out_2172530486751409334[95] = 0;
   out_2172530486751409334[96] = 1.0;
   out_2172530486751409334[97] = 0;
   out_2172530486751409334[98] = 0;
   out_2172530486751409334[99] = 0;
   out_2172530486751409334[100] = 0;
   out_2172530486751409334[101] = 0;
   out_2172530486751409334[102] = 0;
   out_2172530486751409334[103] = 0;
   out_2172530486751409334[104] = 0;
   out_2172530486751409334[105] = 0;
   out_2172530486751409334[106] = 0;
   out_2172530486751409334[107] = 0;
   out_2172530486751409334[108] = 1.0;
   out_2172530486751409334[109] = 0;
   out_2172530486751409334[110] = 0;
   out_2172530486751409334[111] = 0;
   out_2172530486751409334[112] = 0;
   out_2172530486751409334[113] = 0;
   out_2172530486751409334[114] = 0;
   out_2172530486751409334[115] = 0;
   out_2172530486751409334[116] = 0;
   out_2172530486751409334[117] = 0;
   out_2172530486751409334[118] = 0;
   out_2172530486751409334[119] = 0;
   out_2172530486751409334[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3803902687444144824) {
   out_3803902687444144824[0] = dt*state[3] + state[0];
   out_3803902687444144824[1] = dt*state[4] + state[1];
   out_3803902687444144824[2] = dt*state[5] + state[2];
   out_3803902687444144824[3] = state[3];
   out_3803902687444144824[4] = state[4];
   out_3803902687444144824[5] = state[5];
   out_3803902687444144824[6] = dt*state[7] + state[6];
   out_3803902687444144824[7] = dt*state[8] + state[7];
   out_3803902687444144824[8] = state[8];
   out_3803902687444144824[9] = state[9];
   out_3803902687444144824[10] = state[10];
}
void F_fun(double *state, double dt, double *out_448765062510618999) {
   out_448765062510618999[0] = 1;
   out_448765062510618999[1] = 0;
   out_448765062510618999[2] = 0;
   out_448765062510618999[3] = dt;
   out_448765062510618999[4] = 0;
   out_448765062510618999[5] = 0;
   out_448765062510618999[6] = 0;
   out_448765062510618999[7] = 0;
   out_448765062510618999[8] = 0;
   out_448765062510618999[9] = 0;
   out_448765062510618999[10] = 0;
   out_448765062510618999[11] = 0;
   out_448765062510618999[12] = 1;
   out_448765062510618999[13] = 0;
   out_448765062510618999[14] = 0;
   out_448765062510618999[15] = dt;
   out_448765062510618999[16] = 0;
   out_448765062510618999[17] = 0;
   out_448765062510618999[18] = 0;
   out_448765062510618999[19] = 0;
   out_448765062510618999[20] = 0;
   out_448765062510618999[21] = 0;
   out_448765062510618999[22] = 0;
   out_448765062510618999[23] = 0;
   out_448765062510618999[24] = 1;
   out_448765062510618999[25] = 0;
   out_448765062510618999[26] = 0;
   out_448765062510618999[27] = dt;
   out_448765062510618999[28] = 0;
   out_448765062510618999[29] = 0;
   out_448765062510618999[30] = 0;
   out_448765062510618999[31] = 0;
   out_448765062510618999[32] = 0;
   out_448765062510618999[33] = 0;
   out_448765062510618999[34] = 0;
   out_448765062510618999[35] = 0;
   out_448765062510618999[36] = 1;
   out_448765062510618999[37] = 0;
   out_448765062510618999[38] = 0;
   out_448765062510618999[39] = 0;
   out_448765062510618999[40] = 0;
   out_448765062510618999[41] = 0;
   out_448765062510618999[42] = 0;
   out_448765062510618999[43] = 0;
   out_448765062510618999[44] = 0;
   out_448765062510618999[45] = 0;
   out_448765062510618999[46] = 0;
   out_448765062510618999[47] = 0;
   out_448765062510618999[48] = 1;
   out_448765062510618999[49] = 0;
   out_448765062510618999[50] = 0;
   out_448765062510618999[51] = 0;
   out_448765062510618999[52] = 0;
   out_448765062510618999[53] = 0;
   out_448765062510618999[54] = 0;
   out_448765062510618999[55] = 0;
   out_448765062510618999[56] = 0;
   out_448765062510618999[57] = 0;
   out_448765062510618999[58] = 0;
   out_448765062510618999[59] = 0;
   out_448765062510618999[60] = 1;
   out_448765062510618999[61] = 0;
   out_448765062510618999[62] = 0;
   out_448765062510618999[63] = 0;
   out_448765062510618999[64] = 0;
   out_448765062510618999[65] = 0;
   out_448765062510618999[66] = 0;
   out_448765062510618999[67] = 0;
   out_448765062510618999[68] = 0;
   out_448765062510618999[69] = 0;
   out_448765062510618999[70] = 0;
   out_448765062510618999[71] = 0;
   out_448765062510618999[72] = 1;
   out_448765062510618999[73] = dt;
   out_448765062510618999[74] = 0;
   out_448765062510618999[75] = 0;
   out_448765062510618999[76] = 0;
   out_448765062510618999[77] = 0;
   out_448765062510618999[78] = 0;
   out_448765062510618999[79] = 0;
   out_448765062510618999[80] = 0;
   out_448765062510618999[81] = 0;
   out_448765062510618999[82] = 0;
   out_448765062510618999[83] = 0;
   out_448765062510618999[84] = 1;
   out_448765062510618999[85] = dt;
   out_448765062510618999[86] = 0;
   out_448765062510618999[87] = 0;
   out_448765062510618999[88] = 0;
   out_448765062510618999[89] = 0;
   out_448765062510618999[90] = 0;
   out_448765062510618999[91] = 0;
   out_448765062510618999[92] = 0;
   out_448765062510618999[93] = 0;
   out_448765062510618999[94] = 0;
   out_448765062510618999[95] = 0;
   out_448765062510618999[96] = 1;
   out_448765062510618999[97] = 0;
   out_448765062510618999[98] = 0;
   out_448765062510618999[99] = 0;
   out_448765062510618999[100] = 0;
   out_448765062510618999[101] = 0;
   out_448765062510618999[102] = 0;
   out_448765062510618999[103] = 0;
   out_448765062510618999[104] = 0;
   out_448765062510618999[105] = 0;
   out_448765062510618999[106] = 0;
   out_448765062510618999[107] = 0;
   out_448765062510618999[108] = 1;
   out_448765062510618999[109] = 0;
   out_448765062510618999[110] = 0;
   out_448765062510618999[111] = 0;
   out_448765062510618999[112] = 0;
   out_448765062510618999[113] = 0;
   out_448765062510618999[114] = 0;
   out_448765062510618999[115] = 0;
   out_448765062510618999[116] = 0;
   out_448765062510618999[117] = 0;
   out_448765062510618999[118] = 0;
   out_448765062510618999[119] = 0;
   out_448765062510618999[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_7666487914334335596) {
   out_7666487914334335596[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2843628536195130770) {
   out_2843628536195130770[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2843628536195130770[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2843628536195130770[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2843628536195130770[3] = 0;
   out_2843628536195130770[4] = 0;
   out_2843628536195130770[5] = 0;
   out_2843628536195130770[6] = 1;
   out_2843628536195130770[7] = 0;
   out_2843628536195130770[8] = 0;
   out_2843628536195130770[9] = 0;
   out_2843628536195130770[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3527336028405721629) {
   out_3527336028405721629[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1506007362187013094) {
   out_1506007362187013094[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1506007362187013094[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1506007362187013094[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1506007362187013094[3] = 0;
   out_1506007362187013094[4] = 0;
   out_1506007362187013094[5] = 0;
   out_1506007362187013094[6] = 1;
   out_1506007362187013094[7] = 0;
   out_1506007362187013094[8] = 0;
   out_1506007362187013094[9] = 1;
   out_1506007362187013094[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1146256510176586591) {
   out_1146256510176586591[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2837864402044772102) {
   out_2837864402044772102[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[6] = 0;
   out_2837864402044772102[7] = 1;
   out_2837864402044772102[8] = 0;
   out_2837864402044772102[9] = 0;
   out_2837864402044772102[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1146256510176586591) {
   out_1146256510176586591[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2837864402044772102) {
   out_2837864402044772102[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2837864402044772102[6] = 0;
   out_2837864402044772102[7] = 1;
   out_2837864402044772102[8] = 0;
   out_2837864402044772102[9] = 0;
   out_2837864402044772102[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5797809671637013283) {
  err_fun(nom_x, delta_x, out_5797809671637013283);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2749815054075637968) {
  inv_err_fun(nom_x, true_x, out_2749815054075637968);
}
void gnss_H_mod_fun(double *state, double *out_2172530486751409334) {
  H_mod_fun(state, out_2172530486751409334);
}
void gnss_f_fun(double *state, double dt, double *out_3803902687444144824) {
  f_fun(state,  dt, out_3803902687444144824);
}
void gnss_F_fun(double *state, double dt, double *out_448765062510618999) {
  F_fun(state,  dt, out_448765062510618999);
}
void gnss_h_6(double *state, double *sat_pos, double *out_7666487914334335596) {
  h_6(state, sat_pos, out_7666487914334335596);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2843628536195130770) {
  H_6(state, sat_pos, out_2843628536195130770);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3527336028405721629) {
  h_20(state, sat_pos, out_3527336028405721629);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1506007362187013094) {
  H_20(state, sat_pos, out_1506007362187013094);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1146256510176586591) {
  h_7(state, sat_pos_vel, out_1146256510176586591);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2837864402044772102) {
  H_7(state, sat_pos_vel, out_2837864402044772102);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1146256510176586591) {
  h_21(state, sat_pos_vel, out_1146256510176586591);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2837864402044772102) {
  H_21(state, sat_pos_vel, out_2837864402044772102);
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
