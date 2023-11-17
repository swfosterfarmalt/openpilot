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
void err_fun(double *nom_x, double *delta_x, double *out_7898758279284104251) {
   out_7898758279284104251[0] = delta_x[0] + nom_x[0];
   out_7898758279284104251[1] = delta_x[1] + nom_x[1];
   out_7898758279284104251[2] = delta_x[2] + nom_x[2];
   out_7898758279284104251[3] = delta_x[3] + nom_x[3];
   out_7898758279284104251[4] = delta_x[4] + nom_x[4];
   out_7898758279284104251[5] = delta_x[5] + nom_x[5];
   out_7898758279284104251[6] = delta_x[6] + nom_x[6];
   out_7898758279284104251[7] = delta_x[7] + nom_x[7];
   out_7898758279284104251[8] = delta_x[8] + nom_x[8];
   out_7898758279284104251[9] = delta_x[9] + nom_x[9];
   out_7898758279284104251[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_258779395976866605) {
   out_258779395976866605[0] = -nom_x[0] + true_x[0];
   out_258779395976866605[1] = -nom_x[1] + true_x[1];
   out_258779395976866605[2] = -nom_x[2] + true_x[2];
   out_258779395976866605[3] = -nom_x[3] + true_x[3];
   out_258779395976866605[4] = -nom_x[4] + true_x[4];
   out_258779395976866605[5] = -nom_x[5] + true_x[5];
   out_258779395976866605[6] = -nom_x[6] + true_x[6];
   out_258779395976866605[7] = -nom_x[7] + true_x[7];
   out_258779395976866605[8] = -nom_x[8] + true_x[8];
   out_258779395976866605[9] = -nom_x[9] + true_x[9];
   out_258779395976866605[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1068558286524968) {
   out_1068558286524968[0] = 1.0;
   out_1068558286524968[1] = 0;
   out_1068558286524968[2] = 0;
   out_1068558286524968[3] = 0;
   out_1068558286524968[4] = 0;
   out_1068558286524968[5] = 0;
   out_1068558286524968[6] = 0;
   out_1068558286524968[7] = 0;
   out_1068558286524968[8] = 0;
   out_1068558286524968[9] = 0;
   out_1068558286524968[10] = 0;
   out_1068558286524968[11] = 0;
   out_1068558286524968[12] = 1.0;
   out_1068558286524968[13] = 0;
   out_1068558286524968[14] = 0;
   out_1068558286524968[15] = 0;
   out_1068558286524968[16] = 0;
   out_1068558286524968[17] = 0;
   out_1068558286524968[18] = 0;
   out_1068558286524968[19] = 0;
   out_1068558286524968[20] = 0;
   out_1068558286524968[21] = 0;
   out_1068558286524968[22] = 0;
   out_1068558286524968[23] = 0;
   out_1068558286524968[24] = 1.0;
   out_1068558286524968[25] = 0;
   out_1068558286524968[26] = 0;
   out_1068558286524968[27] = 0;
   out_1068558286524968[28] = 0;
   out_1068558286524968[29] = 0;
   out_1068558286524968[30] = 0;
   out_1068558286524968[31] = 0;
   out_1068558286524968[32] = 0;
   out_1068558286524968[33] = 0;
   out_1068558286524968[34] = 0;
   out_1068558286524968[35] = 0;
   out_1068558286524968[36] = 1.0;
   out_1068558286524968[37] = 0;
   out_1068558286524968[38] = 0;
   out_1068558286524968[39] = 0;
   out_1068558286524968[40] = 0;
   out_1068558286524968[41] = 0;
   out_1068558286524968[42] = 0;
   out_1068558286524968[43] = 0;
   out_1068558286524968[44] = 0;
   out_1068558286524968[45] = 0;
   out_1068558286524968[46] = 0;
   out_1068558286524968[47] = 0;
   out_1068558286524968[48] = 1.0;
   out_1068558286524968[49] = 0;
   out_1068558286524968[50] = 0;
   out_1068558286524968[51] = 0;
   out_1068558286524968[52] = 0;
   out_1068558286524968[53] = 0;
   out_1068558286524968[54] = 0;
   out_1068558286524968[55] = 0;
   out_1068558286524968[56] = 0;
   out_1068558286524968[57] = 0;
   out_1068558286524968[58] = 0;
   out_1068558286524968[59] = 0;
   out_1068558286524968[60] = 1.0;
   out_1068558286524968[61] = 0;
   out_1068558286524968[62] = 0;
   out_1068558286524968[63] = 0;
   out_1068558286524968[64] = 0;
   out_1068558286524968[65] = 0;
   out_1068558286524968[66] = 0;
   out_1068558286524968[67] = 0;
   out_1068558286524968[68] = 0;
   out_1068558286524968[69] = 0;
   out_1068558286524968[70] = 0;
   out_1068558286524968[71] = 0;
   out_1068558286524968[72] = 1.0;
   out_1068558286524968[73] = 0;
   out_1068558286524968[74] = 0;
   out_1068558286524968[75] = 0;
   out_1068558286524968[76] = 0;
   out_1068558286524968[77] = 0;
   out_1068558286524968[78] = 0;
   out_1068558286524968[79] = 0;
   out_1068558286524968[80] = 0;
   out_1068558286524968[81] = 0;
   out_1068558286524968[82] = 0;
   out_1068558286524968[83] = 0;
   out_1068558286524968[84] = 1.0;
   out_1068558286524968[85] = 0;
   out_1068558286524968[86] = 0;
   out_1068558286524968[87] = 0;
   out_1068558286524968[88] = 0;
   out_1068558286524968[89] = 0;
   out_1068558286524968[90] = 0;
   out_1068558286524968[91] = 0;
   out_1068558286524968[92] = 0;
   out_1068558286524968[93] = 0;
   out_1068558286524968[94] = 0;
   out_1068558286524968[95] = 0;
   out_1068558286524968[96] = 1.0;
   out_1068558286524968[97] = 0;
   out_1068558286524968[98] = 0;
   out_1068558286524968[99] = 0;
   out_1068558286524968[100] = 0;
   out_1068558286524968[101] = 0;
   out_1068558286524968[102] = 0;
   out_1068558286524968[103] = 0;
   out_1068558286524968[104] = 0;
   out_1068558286524968[105] = 0;
   out_1068558286524968[106] = 0;
   out_1068558286524968[107] = 0;
   out_1068558286524968[108] = 1.0;
   out_1068558286524968[109] = 0;
   out_1068558286524968[110] = 0;
   out_1068558286524968[111] = 0;
   out_1068558286524968[112] = 0;
   out_1068558286524968[113] = 0;
   out_1068558286524968[114] = 0;
   out_1068558286524968[115] = 0;
   out_1068558286524968[116] = 0;
   out_1068558286524968[117] = 0;
   out_1068558286524968[118] = 0;
   out_1068558286524968[119] = 0;
   out_1068558286524968[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6635033905056555325) {
   out_6635033905056555325[0] = dt*state[3] + state[0];
   out_6635033905056555325[1] = dt*state[4] + state[1];
   out_6635033905056555325[2] = dt*state[5] + state[2];
   out_6635033905056555325[3] = state[3];
   out_6635033905056555325[4] = state[4];
   out_6635033905056555325[5] = state[5];
   out_6635033905056555325[6] = dt*state[7] + state[6];
   out_6635033905056555325[7] = dt*state[8] + state[7];
   out_6635033905056555325[8] = state[8];
   out_6635033905056555325[9] = state[9];
   out_6635033905056555325[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8502563063737433073) {
   out_8502563063737433073[0] = 1;
   out_8502563063737433073[1] = 0;
   out_8502563063737433073[2] = 0;
   out_8502563063737433073[3] = dt;
   out_8502563063737433073[4] = 0;
   out_8502563063737433073[5] = 0;
   out_8502563063737433073[6] = 0;
   out_8502563063737433073[7] = 0;
   out_8502563063737433073[8] = 0;
   out_8502563063737433073[9] = 0;
   out_8502563063737433073[10] = 0;
   out_8502563063737433073[11] = 0;
   out_8502563063737433073[12] = 1;
   out_8502563063737433073[13] = 0;
   out_8502563063737433073[14] = 0;
   out_8502563063737433073[15] = dt;
   out_8502563063737433073[16] = 0;
   out_8502563063737433073[17] = 0;
   out_8502563063737433073[18] = 0;
   out_8502563063737433073[19] = 0;
   out_8502563063737433073[20] = 0;
   out_8502563063737433073[21] = 0;
   out_8502563063737433073[22] = 0;
   out_8502563063737433073[23] = 0;
   out_8502563063737433073[24] = 1;
   out_8502563063737433073[25] = 0;
   out_8502563063737433073[26] = 0;
   out_8502563063737433073[27] = dt;
   out_8502563063737433073[28] = 0;
   out_8502563063737433073[29] = 0;
   out_8502563063737433073[30] = 0;
   out_8502563063737433073[31] = 0;
   out_8502563063737433073[32] = 0;
   out_8502563063737433073[33] = 0;
   out_8502563063737433073[34] = 0;
   out_8502563063737433073[35] = 0;
   out_8502563063737433073[36] = 1;
   out_8502563063737433073[37] = 0;
   out_8502563063737433073[38] = 0;
   out_8502563063737433073[39] = 0;
   out_8502563063737433073[40] = 0;
   out_8502563063737433073[41] = 0;
   out_8502563063737433073[42] = 0;
   out_8502563063737433073[43] = 0;
   out_8502563063737433073[44] = 0;
   out_8502563063737433073[45] = 0;
   out_8502563063737433073[46] = 0;
   out_8502563063737433073[47] = 0;
   out_8502563063737433073[48] = 1;
   out_8502563063737433073[49] = 0;
   out_8502563063737433073[50] = 0;
   out_8502563063737433073[51] = 0;
   out_8502563063737433073[52] = 0;
   out_8502563063737433073[53] = 0;
   out_8502563063737433073[54] = 0;
   out_8502563063737433073[55] = 0;
   out_8502563063737433073[56] = 0;
   out_8502563063737433073[57] = 0;
   out_8502563063737433073[58] = 0;
   out_8502563063737433073[59] = 0;
   out_8502563063737433073[60] = 1;
   out_8502563063737433073[61] = 0;
   out_8502563063737433073[62] = 0;
   out_8502563063737433073[63] = 0;
   out_8502563063737433073[64] = 0;
   out_8502563063737433073[65] = 0;
   out_8502563063737433073[66] = 0;
   out_8502563063737433073[67] = 0;
   out_8502563063737433073[68] = 0;
   out_8502563063737433073[69] = 0;
   out_8502563063737433073[70] = 0;
   out_8502563063737433073[71] = 0;
   out_8502563063737433073[72] = 1;
   out_8502563063737433073[73] = dt;
   out_8502563063737433073[74] = 0;
   out_8502563063737433073[75] = 0;
   out_8502563063737433073[76] = 0;
   out_8502563063737433073[77] = 0;
   out_8502563063737433073[78] = 0;
   out_8502563063737433073[79] = 0;
   out_8502563063737433073[80] = 0;
   out_8502563063737433073[81] = 0;
   out_8502563063737433073[82] = 0;
   out_8502563063737433073[83] = 0;
   out_8502563063737433073[84] = 1;
   out_8502563063737433073[85] = dt;
   out_8502563063737433073[86] = 0;
   out_8502563063737433073[87] = 0;
   out_8502563063737433073[88] = 0;
   out_8502563063737433073[89] = 0;
   out_8502563063737433073[90] = 0;
   out_8502563063737433073[91] = 0;
   out_8502563063737433073[92] = 0;
   out_8502563063737433073[93] = 0;
   out_8502563063737433073[94] = 0;
   out_8502563063737433073[95] = 0;
   out_8502563063737433073[96] = 1;
   out_8502563063737433073[97] = 0;
   out_8502563063737433073[98] = 0;
   out_8502563063737433073[99] = 0;
   out_8502563063737433073[100] = 0;
   out_8502563063737433073[101] = 0;
   out_8502563063737433073[102] = 0;
   out_8502563063737433073[103] = 0;
   out_8502563063737433073[104] = 0;
   out_8502563063737433073[105] = 0;
   out_8502563063737433073[106] = 0;
   out_8502563063737433073[107] = 0;
   out_8502563063737433073[108] = 1;
   out_8502563063737433073[109] = 0;
   out_8502563063737433073[110] = 0;
   out_8502563063737433073[111] = 0;
   out_8502563063737433073[112] = 0;
   out_8502563063737433073[113] = 0;
   out_8502563063737433073[114] = 0;
   out_8502563063737433073[115] = 0;
   out_8502563063737433073[116] = 0;
   out_8502563063737433073[117] = 0;
   out_8502563063737433073[118] = 0;
   out_8502563063737433073[119] = 0;
   out_8502563063737433073[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4881918134730526271) {
   out_4881918134730526271[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1972889481654701448) {
   out_1972889481654701448[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1972889481654701448[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1972889481654701448[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1972889481654701448[3] = 0;
   out_1972889481654701448[4] = 0;
   out_1972889481654701448[5] = 0;
   out_1972889481654701448[6] = 1;
   out_1972889481654701448[7] = 0;
   out_1972889481654701448[8] = 0;
   out_1972889481654701448[9] = 0;
   out_1972889481654701448[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5245777435176666735) {
   out_5245777435176666735[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_380673705059921505) {
   out_380673705059921505[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_380673705059921505[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_380673705059921505[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_380673705059921505[3] = 0;
   out_380673705059921505[4] = 0;
   out_380673705059921505[5] = 0;
   out_380673705059921505[6] = 1;
   out_380673705059921505[7] = 0;
   out_380673705059921505[8] = 0;
   out_380673705059921505[9] = 1;
   out_380673705059921505[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1916653750505923582) {
   out_1916653750505923582[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_788355260993627668) {
   out_788355260993627668[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[6] = 0;
   out_788355260993627668[7] = 1;
   out_788355260993627668[8] = 0;
   out_788355260993627668[9] = 0;
   out_788355260993627668[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1916653750505923582) {
   out_1916653750505923582[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_788355260993627668) {
   out_788355260993627668[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_788355260993627668[6] = 0;
   out_788355260993627668[7] = 1;
   out_788355260993627668[8] = 0;
   out_788355260993627668[9] = 0;
   out_788355260993627668[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7898758279284104251) {
  err_fun(nom_x, delta_x, out_7898758279284104251);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_258779395976866605) {
  inv_err_fun(nom_x, true_x, out_258779395976866605);
}
void gnss_H_mod_fun(double *state, double *out_1068558286524968) {
  H_mod_fun(state, out_1068558286524968);
}
void gnss_f_fun(double *state, double dt, double *out_6635033905056555325) {
  f_fun(state,  dt, out_6635033905056555325);
}
void gnss_F_fun(double *state, double dt, double *out_8502563063737433073) {
  F_fun(state,  dt, out_8502563063737433073);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4881918134730526271) {
  h_6(state, sat_pos, out_4881918134730526271);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1972889481654701448) {
  H_6(state, sat_pos, out_1972889481654701448);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5245777435176666735) {
  h_20(state, sat_pos, out_5245777435176666735);
}
void gnss_H_20(double *state, double *sat_pos, double *out_380673705059921505) {
  H_20(state, sat_pos, out_380673705059921505);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1916653750505923582) {
  h_7(state, sat_pos_vel, out_1916653750505923582);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_788355260993627668) {
  H_7(state, sat_pos_vel, out_788355260993627668);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1916653750505923582) {
  h_21(state, sat_pos_vel, out_1916653750505923582);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_788355260993627668) {
  H_21(state, sat_pos_vel, out_788355260993627668);
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
