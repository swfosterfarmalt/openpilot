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
void err_fun(double *nom_x, double *delta_x, double *out_9006316484952766497) {
   out_9006316484952766497[0] = delta_x[0] + nom_x[0];
   out_9006316484952766497[1] = delta_x[1] + nom_x[1];
   out_9006316484952766497[2] = delta_x[2] + nom_x[2];
   out_9006316484952766497[3] = delta_x[3] + nom_x[3];
   out_9006316484952766497[4] = delta_x[4] + nom_x[4];
   out_9006316484952766497[5] = delta_x[5] + nom_x[5];
   out_9006316484952766497[6] = delta_x[6] + nom_x[6];
   out_9006316484952766497[7] = delta_x[7] + nom_x[7];
   out_9006316484952766497[8] = delta_x[8] + nom_x[8];
   out_9006316484952766497[9] = delta_x[9] + nom_x[9];
   out_9006316484952766497[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7014221750480788203) {
   out_7014221750480788203[0] = -nom_x[0] + true_x[0];
   out_7014221750480788203[1] = -nom_x[1] + true_x[1];
   out_7014221750480788203[2] = -nom_x[2] + true_x[2];
   out_7014221750480788203[3] = -nom_x[3] + true_x[3];
   out_7014221750480788203[4] = -nom_x[4] + true_x[4];
   out_7014221750480788203[5] = -nom_x[5] + true_x[5];
   out_7014221750480788203[6] = -nom_x[6] + true_x[6];
   out_7014221750480788203[7] = -nom_x[7] + true_x[7];
   out_7014221750480788203[8] = -nom_x[8] + true_x[8];
   out_7014221750480788203[9] = -nom_x[9] + true_x[9];
   out_7014221750480788203[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1743468875505645939) {
   out_1743468875505645939[0] = 1.0;
   out_1743468875505645939[1] = 0;
   out_1743468875505645939[2] = 0;
   out_1743468875505645939[3] = 0;
   out_1743468875505645939[4] = 0;
   out_1743468875505645939[5] = 0;
   out_1743468875505645939[6] = 0;
   out_1743468875505645939[7] = 0;
   out_1743468875505645939[8] = 0;
   out_1743468875505645939[9] = 0;
   out_1743468875505645939[10] = 0;
   out_1743468875505645939[11] = 0;
   out_1743468875505645939[12] = 1.0;
   out_1743468875505645939[13] = 0;
   out_1743468875505645939[14] = 0;
   out_1743468875505645939[15] = 0;
   out_1743468875505645939[16] = 0;
   out_1743468875505645939[17] = 0;
   out_1743468875505645939[18] = 0;
   out_1743468875505645939[19] = 0;
   out_1743468875505645939[20] = 0;
   out_1743468875505645939[21] = 0;
   out_1743468875505645939[22] = 0;
   out_1743468875505645939[23] = 0;
   out_1743468875505645939[24] = 1.0;
   out_1743468875505645939[25] = 0;
   out_1743468875505645939[26] = 0;
   out_1743468875505645939[27] = 0;
   out_1743468875505645939[28] = 0;
   out_1743468875505645939[29] = 0;
   out_1743468875505645939[30] = 0;
   out_1743468875505645939[31] = 0;
   out_1743468875505645939[32] = 0;
   out_1743468875505645939[33] = 0;
   out_1743468875505645939[34] = 0;
   out_1743468875505645939[35] = 0;
   out_1743468875505645939[36] = 1.0;
   out_1743468875505645939[37] = 0;
   out_1743468875505645939[38] = 0;
   out_1743468875505645939[39] = 0;
   out_1743468875505645939[40] = 0;
   out_1743468875505645939[41] = 0;
   out_1743468875505645939[42] = 0;
   out_1743468875505645939[43] = 0;
   out_1743468875505645939[44] = 0;
   out_1743468875505645939[45] = 0;
   out_1743468875505645939[46] = 0;
   out_1743468875505645939[47] = 0;
   out_1743468875505645939[48] = 1.0;
   out_1743468875505645939[49] = 0;
   out_1743468875505645939[50] = 0;
   out_1743468875505645939[51] = 0;
   out_1743468875505645939[52] = 0;
   out_1743468875505645939[53] = 0;
   out_1743468875505645939[54] = 0;
   out_1743468875505645939[55] = 0;
   out_1743468875505645939[56] = 0;
   out_1743468875505645939[57] = 0;
   out_1743468875505645939[58] = 0;
   out_1743468875505645939[59] = 0;
   out_1743468875505645939[60] = 1.0;
   out_1743468875505645939[61] = 0;
   out_1743468875505645939[62] = 0;
   out_1743468875505645939[63] = 0;
   out_1743468875505645939[64] = 0;
   out_1743468875505645939[65] = 0;
   out_1743468875505645939[66] = 0;
   out_1743468875505645939[67] = 0;
   out_1743468875505645939[68] = 0;
   out_1743468875505645939[69] = 0;
   out_1743468875505645939[70] = 0;
   out_1743468875505645939[71] = 0;
   out_1743468875505645939[72] = 1.0;
   out_1743468875505645939[73] = 0;
   out_1743468875505645939[74] = 0;
   out_1743468875505645939[75] = 0;
   out_1743468875505645939[76] = 0;
   out_1743468875505645939[77] = 0;
   out_1743468875505645939[78] = 0;
   out_1743468875505645939[79] = 0;
   out_1743468875505645939[80] = 0;
   out_1743468875505645939[81] = 0;
   out_1743468875505645939[82] = 0;
   out_1743468875505645939[83] = 0;
   out_1743468875505645939[84] = 1.0;
   out_1743468875505645939[85] = 0;
   out_1743468875505645939[86] = 0;
   out_1743468875505645939[87] = 0;
   out_1743468875505645939[88] = 0;
   out_1743468875505645939[89] = 0;
   out_1743468875505645939[90] = 0;
   out_1743468875505645939[91] = 0;
   out_1743468875505645939[92] = 0;
   out_1743468875505645939[93] = 0;
   out_1743468875505645939[94] = 0;
   out_1743468875505645939[95] = 0;
   out_1743468875505645939[96] = 1.0;
   out_1743468875505645939[97] = 0;
   out_1743468875505645939[98] = 0;
   out_1743468875505645939[99] = 0;
   out_1743468875505645939[100] = 0;
   out_1743468875505645939[101] = 0;
   out_1743468875505645939[102] = 0;
   out_1743468875505645939[103] = 0;
   out_1743468875505645939[104] = 0;
   out_1743468875505645939[105] = 0;
   out_1743468875505645939[106] = 0;
   out_1743468875505645939[107] = 0;
   out_1743468875505645939[108] = 1.0;
   out_1743468875505645939[109] = 0;
   out_1743468875505645939[110] = 0;
   out_1743468875505645939[111] = 0;
   out_1743468875505645939[112] = 0;
   out_1743468875505645939[113] = 0;
   out_1743468875505645939[114] = 0;
   out_1743468875505645939[115] = 0;
   out_1743468875505645939[116] = 0;
   out_1743468875505645939[117] = 0;
   out_1743468875505645939[118] = 0;
   out_1743468875505645939[119] = 0;
   out_1743468875505645939[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2447433791296152011) {
   out_2447433791296152011[0] = dt*state[3] + state[0];
   out_2447433791296152011[1] = dt*state[4] + state[1];
   out_2447433791296152011[2] = dt*state[5] + state[2];
   out_2447433791296152011[3] = state[3];
   out_2447433791296152011[4] = state[4];
   out_2447433791296152011[5] = state[5];
   out_2447433791296152011[6] = dt*state[7] + state[6];
   out_2447433791296152011[7] = dt*state[8] + state[7];
   out_2447433791296152011[8] = state[8];
   out_2447433791296152011[9] = state[9];
   out_2447433791296152011[10] = state[10];
}
void F_fun(double *state, double dt, double *out_1083846469902090950) {
   out_1083846469902090950[0] = 1;
   out_1083846469902090950[1] = 0;
   out_1083846469902090950[2] = 0;
   out_1083846469902090950[3] = dt;
   out_1083846469902090950[4] = 0;
   out_1083846469902090950[5] = 0;
   out_1083846469902090950[6] = 0;
   out_1083846469902090950[7] = 0;
   out_1083846469902090950[8] = 0;
   out_1083846469902090950[9] = 0;
   out_1083846469902090950[10] = 0;
   out_1083846469902090950[11] = 0;
   out_1083846469902090950[12] = 1;
   out_1083846469902090950[13] = 0;
   out_1083846469902090950[14] = 0;
   out_1083846469902090950[15] = dt;
   out_1083846469902090950[16] = 0;
   out_1083846469902090950[17] = 0;
   out_1083846469902090950[18] = 0;
   out_1083846469902090950[19] = 0;
   out_1083846469902090950[20] = 0;
   out_1083846469902090950[21] = 0;
   out_1083846469902090950[22] = 0;
   out_1083846469902090950[23] = 0;
   out_1083846469902090950[24] = 1;
   out_1083846469902090950[25] = 0;
   out_1083846469902090950[26] = 0;
   out_1083846469902090950[27] = dt;
   out_1083846469902090950[28] = 0;
   out_1083846469902090950[29] = 0;
   out_1083846469902090950[30] = 0;
   out_1083846469902090950[31] = 0;
   out_1083846469902090950[32] = 0;
   out_1083846469902090950[33] = 0;
   out_1083846469902090950[34] = 0;
   out_1083846469902090950[35] = 0;
   out_1083846469902090950[36] = 1;
   out_1083846469902090950[37] = 0;
   out_1083846469902090950[38] = 0;
   out_1083846469902090950[39] = 0;
   out_1083846469902090950[40] = 0;
   out_1083846469902090950[41] = 0;
   out_1083846469902090950[42] = 0;
   out_1083846469902090950[43] = 0;
   out_1083846469902090950[44] = 0;
   out_1083846469902090950[45] = 0;
   out_1083846469902090950[46] = 0;
   out_1083846469902090950[47] = 0;
   out_1083846469902090950[48] = 1;
   out_1083846469902090950[49] = 0;
   out_1083846469902090950[50] = 0;
   out_1083846469902090950[51] = 0;
   out_1083846469902090950[52] = 0;
   out_1083846469902090950[53] = 0;
   out_1083846469902090950[54] = 0;
   out_1083846469902090950[55] = 0;
   out_1083846469902090950[56] = 0;
   out_1083846469902090950[57] = 0;
   out_1083846469902090950[58] = 0;
   out_1083846469902090950[59] = 0;
   out_1083846469902090950[60] = 1;
   out_1083846469902090950[61] = 0;
   out_1083846469902090950[62] = 0;
   out_1083846469902090950[63] = 0;
   out_1083846469902090950[64] = 0;
   out_1083846469902090950[65] = 0;
   out_1083846469902090950[66] = 0;
   out_1083846469902090950[67] = 0;
   out_1083846469902090950[68] = 0;
   out_1083846469902090950[69] = 0;
   out_1083846469902090950[70] = 0;
   out_1083846469902090950[71] = 0;
   out_1083846469902090950[72] = 1;
   out_1083846469902090950[73] = dt;
   out_1083846469902090950[74] = 0;
   out_1083846469902090950[75] = 0;
   out_1083846469902090950[76] = 0;
   out_1083846469902090950[77] = 0;
   out_1083846469902090950[78] = 0;
   out_1083846469902090950[79] = 0;
   out_1083846469902090950[80] = 0;
   out_1083846469902090950[81] = 0;
   out_1083846469902090950[82] = 0;
   out_1083846469902090950[83] = 0;
   out_1083846469902090950[84] = 1;
   out_1083846469902090950[85] = dt;
   out_1083846469902090950[86] = 0;
   out_1083846469902090950[87] = 0;
   out_1083846469902090950[88] = 0;
   out_1083846469902090950[89] = 0;
   out_1083846469902090950[90] = 0;
   out_1083846469902090950[91] = 0;
   out_1083846469902090950[92] = 0;
   out_1083846469902090950[93] = 0;
   out_1083846469902090950[94] = 0;
   out_1083846469902090950[95] = 0;
   out_1083846469902090950[96] = 1;
   out_1083846469902090950[97] = 0;
   out_1083846469902090950[98] = 0;
   out_1083846469902090950[99] = 0;
   out_1083846469902090950[100] = 0;
   out_1083846469902090950[101] = 0;
   out_1083846469902090950[102] = 0;
   out_1083846469902090950[103] = 0;
   out_1083846469902090950[104] = 0;
   out_1083846469902090950[105] = 0;
   out_1083846469902090950[106] = 0;
   out_1083846469902090950[107] = 0;
   out_1083846469902090950[108] = 1;
   out_1083846469902090950[109] = 0;
   out_1083846469902090950[110] = 0;
   out_1083846469902090950[111] = 0;
   out_1083846469902090950[112] = 0;
   out_1083846469902090950[113] = 0;
   out_1083846469902090950[114] = 0;
   out_1083846469902090950[115] = 0;
   out_1083846469902090950[116] = 0;
   out_1083846469902090950[117] = 0;
   out_1083846469902090950[118] = 0;
   out_1083846469902090950[119] = 0;
   out_1083846469902090950[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6943420607144317399) {
   out_6943420607144317399[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_904843385930103144) {
   out_904843385930103144[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_904843385930103144[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_904843385930103144[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_904843385930103144[3] = 0;
   out_904843385930103144[4] = 0;
   out_904843385930103144[5] = 0;
   out_904843385930103144[6] = 1;
   out_904843385930103144[7] = 0;
   out_904843385930103144[8] = 0;
   out_904843385930103144[9] = 0;
   out_904843385930103144[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5773685015670522167) {
   out_5773685015670522167[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5283321743023419019) {
   out_5283321743023419019[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5283321743023419019[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5283321743023419019[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5283321743023419019[3] = 0;
   out_5283321743023419019[4] = 0;
   out_5283321743023419019[5] = 0;
   out_5283321743023419019[6] = 1;
   out_5283321743023419019[7] = 0;
   out_5283321743023419019[8] = 0;
   out_5283321743023419019[9] = 1;
   out_5283321743023419019[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3961974541600867607) {
   out_3961974541600867607[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2426909385454584489) {
   out_2426909385454584489[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[6] = 0;
   out_2426909385454584489[7] = 1;
   out_2426909385454584489[8] = 0;
   out_2426909385454584489[9] = 0;
   out_2426909385454584489[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3961974541600867607) {
   out_3961974541600867607[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2426909385454584489) {
   out_2426909385454584489[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2426909385454584489[6] = 0;
   out_2426909385454584489[7] = 1;
   out_2426909385454584489[8] = 0;
   out_2426909385454584489[9] = 0;
   out_2426909385454584489[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9006316484952766497) {
  err_fun(nom_x, delta_x, out_9006316484952766497);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7014221750480788203) {
  inv_err_fun(nom_x, true_x, out_7014221750480788203);
}
void gnss_H_mod_fun(double *state, double *out_1743468875505645939) {
  H_mod_fun(state, out_1743468875505645939);
}
void gnss_f_fun(double *state, double dt, double *out_2447433791296152011) {
  f_fun(state,  dt, out_2447433791296152011);
}
void gnss_F_fun(double *state, double dt, double *out_1083846469902090950) {
  F_fun(state,  dt, out_1083846469902090950);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6943420607144317399) {
  h_6(state, sat_pos, out_6943420607144317399);
}
void gnss_H_6(double *state, double *sat_pos, double *out_904843385930103144) {
  H_6(state, sat_pos, out_904843385930103144);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5773685015670522167) {
  h_20(state, sat_pos, out_5773685015670522167);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5283321743023419019) {
  H_20(state, sat_pos, out_5283321743023419019);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3961974541600867607) {
  h_7(state, sat_pos_vel, out_3961974541600867607);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2426909385454584489) {
  H_7(state, sat_pos_vel, out_2426909385454584489);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3961974541600867607) {
  h_21(state, sat_pos_vel, out_3961974541600867607);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2426909385454584489) {
  H_21(state, sat_pos_vel, out_2426909385454584489);
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
