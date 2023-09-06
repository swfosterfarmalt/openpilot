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
void err_fun(double *nom_x, double *delta_x, double *out_4307062985303373349) {
   out_4307062985303373349[0] = delta_x[0] + nom_x[0];
   out_4307062985303373349[1] = delta_x[1] + nom_x[1];
   out_4307062985303373349[2] = delta_x[2] + nom_x[2];
   out_4307062985303373349[3] = delta_x[3] + nom_x[3];
   out_4307062985303373349[4] = delta_x[4] + nom_x[4];
   out_4307062985303373349[5] = delta_x[5] + nom_x[5];
   out_4307062985303373349[6] = delta_x[6] + nom_x[6];
   out_4307062985303373349[7] = delta_x[7] + nom_x[7];
   out_4307062985303373349[8] = delta_x[8] + nom_x[8];
   out_4307062985303373349[9] = delta_x[9] + nom_x[9];
   out_4307062985303373349[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4110902357590046395) {
   out_4110902357590046395[0] = -nom_x[0] + true_x[0];
   out_4110902357590046395[1] = -nom_x[1] + true_x[1];
   out_4110902357590046395[2] = -nom_x[2] + true_x[2];
   out_4110902357590046395[3] = -nom_x[3] + true_x[3];
   out_4110902357590046395[4] = -nom_x[4] + true_x[4];
   out_4110902357590046395[5] = -nom_x[5] + true_x[5];
   out_4110902357590046395[6] = -nom_x[6] + true_x[6];
   out_4110902357590046395[7] = -nom_x[7] + true_x[7];
   out_4110902357590046395[8] = -nom_x[8] + true_x[8];
   out_4110902357590046395[9] = -nom_x[9] + true_x[9];
   out_4110902357590046395[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6392659668905738982) {
   out_6392659668905738982[0] = 1.0;
   out_6392659668905738982[1] = 0;
   out_6392659668905738982[2] = 0;
   out_6392659668905738982[3] = 0;
   out_6392659668905738982[4] = 0;
   out_6392659668905738982[5] = 0;
   out_6392659668905738982[6] = 0;
   out_6392659668905738982[7] = 0;
   out_6392659668905738982[8] = 0;
   out_6392659668905738982[9] = 0;
   out_6392659668905738982[10] = 0;
   out_6392659668905738982[11] = 0;
   out_6392659668905738982[12] = 1.0;
   out_6392659668905738982[13] = 0;
   out_6392659668905738982[14] = 0;
   out_6392659668905738982[15] = 0;
   out_6392659668905738982[16] = 0;
   out_6392659668905738982[17] = 0;
   out_6392659668905738982[18] = 0;
   out_6392659668905738982[19] = 0;
   out_6392659668905738982[20] = 0;
   out_6392659668905738982[21] = 0;
   out_6392659668905738982[22] = 0;
   out_6392659668905738982[23] = 0;
   out_6392659668905738982[24] = 1.0;
   out_6392659668905738982[25] = 0;
   out_6392659668905738982[26] = 0;
   out_6392659668905738982[27] = 0;
   out_6392659668905738982[28] = 0;
   out_6392659668905738982[29] = 0;
   out_6392659668905738982[30] = 0;
   out_6392659668905738982[31] = 0;
   out_6392659668905738982[32] = 0;
   out_6392659668905738982[33] = 0;
   out_6392659668905738982[34] = 0;
   out_6392659668905738982[35] = 0;
   out_6392659668905738982[36] = 1.0;
   out_6392659668905738982[37] = 0;
   out_6392659668905738982[38] = 0;
   out_6392659668905738982[39] = 0;
   out_6392659668905738982[40] = 0;
   out_6392659668905738982[41] = 0;
   out_6392659668905738982[42] = 0;
   out_6392659668905738982[43] = 0;
   out_6392659668905738982[44] = 0;
   out_6392659668905738982[45] = 0;
   out_6392659668905738982[46] = 0;
   out_6392659668905738982[47] = 0;
   out_6392659668905738982[48] = 1.0;
   out_6392659668905738982[49] = 0;
   out_6392659668905738982[50] = 0;
   out_6392659668905738982[51] = 0;
   out_6392659668905738982[52] = 0;
   out_6392659668905738982[53] = 0;
   out_6392659668905738982[54] = 0;
   out_6392659668905738982[55] = 0;
   out_6392659668905738982[56] = 0;
   out_6392659668905738982[57] = 0;
   out_6392659668905738982[58] = 0;
   out_6392659668905738982[59] = 0;
   out_6392659668905738982[60] = 1.0;
   out_6392659668905738982[61] = 0;
   out_6392659668905738982[62] = 0;
   out_6392659668905738982[63] = 0;
   out_6392659668905738982[64] = 0;
   out_6392659668905738982[65] = 0;
   out_6392659668905738982[66] = 0;
   out_6392659668905738982[67] = 0;
   out_6392659668905738982[68] = 0;
   out_6392659668905738982[69] = 0;
   out_6392659668905738982[70] = 0;
   out_6392659668905738982[71] = 0;
   out_6392659668905738982[72] = 1.0;
   out_6392659668905738982[73] = 0;
   out_6392659668905738982[74] = 0;
   out_6392659668905738982[75] = 0;
   out_6392659668905738982[76] = 0;
   out_6392659668905738982[77] = 0;
   out_6392659668905738982[78] = 0;
   out_6392659668905738982[79] = 0;
   out_6392659668905738982[80] = 0;
   out_6392659668905738982[81] = 0;
   out_6392659668905738982[82] = 0;
   out_6392659668905738982[83] = 0;
   out_6392659668905738982[84] = 1.0;
   out_6392659668905738982[85] = 0;
   out_6392659668905738982[86] = 0;
   out_6392659668905738982[87] = 0;
   out_6392659668905738982[88] = 0;
   out_6392659668905738982[89] = 0;
   out_6392659668905738982[90] = 0;
   out_6392659668905738982[91] = 0;
   out_6392659668905738982[92] = 0;
   out_6392659668905738982[93] = 0;
   out_6392659668905738982[94] = 0;
   out_6392659668905738982[95] = 0;
   out_6392659668905738982[96] = 1.0;
   out_6392659668905738982[97] = 0;
   out_6392659668905738982[98] = 0;
   out_6392659668905738982[99] = 0;
   out_6392659668905738982[100] = 0;
   out_6392659668905738982[101] = 0;
   out_6392659668905738982[102] = 0;
   out_6392659668905738982[103] = 0;
   out_6392659668905738982[104] = 0;
   out_6392659668905738982[105] = 0;
   out_6392659668905738982[106] = 0;
   out_6392659668905738982[107] = 0;
   out_6392659668905738982[108] = 1.0;
   out_6392659668905738982[109] = 0;
   out_6392659668905738982[110] = 0;
   out_6392659668905738982[111] = 0;
   out_6392659668905738982[112] = 0;
   out_6392659668905738982[113] = 0;
   out_6392659668905738982[114] = 0;
   out_6392659668905738982[115] = 0;
   out_6392659668905738982[116] = 0;
   out_6392659668905738982[117] = 0;
   out_6392659668905738982[118] = 0;
   out_6392659668905738982[119] = 0;
   out_6392659668905738982[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7119050100122636677) {
   out_7119050100122636677[0] = dt*state[3] + state[0];
   out_7119050100122636677[1] = dt*state[4] + state[1];
   out_7119050100122636677[2] = dt*state[5] + state[2];
   out_7119050100122636677[3] = state[3];
   out_7119050100122636677[4] = state[4];
   out_7119050100122636677[5] = state[5];
   out_7119050100122636677[6] = dt*state[7] + state[6];
   out_7119050100122636677[7] = dt*state[8] + state[7];
   out_7119050100122636677[8] = state[8];
   out_7119050100122636677[9] = state[9];
   out_7119050100122636677[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8526931677833646522) {
   out_8526931677833646522[0] = 1;
   out_8526931677833646522[1] = 0;
   out_8526931677833646522[2] = 0;
   out_8526931677833646522[3] = dt;
   out_8526931677833646522[4] = 0;
   out_8526931677833646522[5] = 0;
   out_8526931677833646522[6] = 0;
   out_8526931677833646522[7] = 0;
   out_8526931677833646522[8] = 0;
   out_8526931677833646522[9] = 0;
   out_8526931677833646522[10] = 0;
   out_8526931677833646522[11] = 0;
   out_8526931677833646522[12] = 1;
   out_8526931677833646522[13] = 0;
   out_8526931677833646522[14] = 0;
   out_8526931677833646522[15] = dt;
   out_8526931677833646522[16] = 0;
   out_8526931677833646522[17] = 0;
   out_8526931677833646522[18] = 0;
   out_8526931677833646522[19] = 0;
   out_8526931677833646522[20] = 0;
   out_8526931677833646522[21] = 0;
   out_8526931677833646522[22] = 0;
   out_8526931677833646522[23] = 0;
   out_8526931677833646522[24] = 1;
   out_8526931677833646522[25] = 0;
   out_8526931677833646522[26] = 0;
   out_8526931677833646522[27] = dt;
   out_8526931677833646522[28] = 0;
   out_8526931677833646522[29] = 0;
   out_8526931677833646522[30] = 0;
   out_8526931677833646522[31] = 0;
   out_8526931677833646522[32] = 0;
   out_8526931677833646522[33] = 0;
   out_8526931677833646522[34] = 0;
   out_8526931677833646522[35] = 0;
   out_8526931677833646522[36] = 1;
   out_8526931677833646522[37] = 0;
   out_8526931677833646522[38] = 0;
   out_8526931677833646522[39] = 0;
   out_8526931677833646522[40] = 0;
   out_8526931677833646522[41] = 0;
   out_8526931677833646522[42] = 0;
   out_8526931677833646522[43] = 0;
   out_8526931677833646522[44] = 0;
   out_8526931677833646522[45] = 0;
   out_8526931677833646522[46] = 0;
   out_8526931677833646522[47] = 0;
   out_8526931677833646522[48] = 1;
   out_8526931677833646522[49] = 0;
   out_8526931677833646522[50] = 0;
   out_8526931677833646522[51] = 0;
   out_8526931677833646522[52] = 0;
   out_8526931677833646522[53] = 0;
   out_8526931677833646522[54] = 0;
   out_8526931677833646522[55] = 0;
   out_8526931677833646522[56] = 0;
   out_8526931677833646522[57] = 0;
   out_8526931677833646522[58] = 0;
   out_8526931677833646522[59] = 0;
   out_8526931677833646522[60] = 1;
   out_8526931677833646522[61] = 0;
   out_8526931677833646522[62] = 0;
   out_8526931677833646522[63] = 0;
   out_8526931677833646522[64] = 0;
   out_8526931677833646522[65] = 0;
   out_8526931677833646522[66] = 0;
   out_8526931677833646522[67] = 0;
   out_8526931677833646522[68] = 0;
   out_8526931677833646522[69] = 0;
   out_8526931677833646522[70] = 0;
   out_8526931677833646522[71] = 0;
   out_8526931677833646522[72] = 1;
   out_8526931677833646522[73] = dt;
   out_8526931677833646522[74] = 0;
   out_8526931677833646522[75] = 0;
   out_8526931677833646522[76] = 0;
   out_8526931677833646522[77] = 0;
   out_8526931677833646522[78] = 0;
   out_8526931677833646522[79] = 0;
   out_8526931677833646522[80] = 0;
   out_8526931677833646522[81] = 0;
   out_8526931677833646522[82] = 0;
   out_8526931677833646522[83] = 0;
   out_8526931677833646522[84] = 1;
   out_8526931677833646522[85] = dt;
   out_8526931677833646522[86] = 0;
   out_8526931677833646522[87] = 0;
   out_8526931677833646522[88] = 0;
   out_8526931677833646522[89] = 0;
   out_8526931677833646522[90] = 0;
   out_8526931677833646522[91] = 0;
   out_8526931677833646522[92] = 0;
   out_8526931677833646522[93] = 0;
   out_8526931677833646522[94] = 0;
   out_8526931677833646522[95] = 0;
   out_8526931677833646522[96] = 1;
   out_8526931677833646522[97] = 0;
   out_8526931677833646522[98] = 0;
   out_8526931677833646522[99] = 0;
   out_8526931677833646522[100] = 0;
   out_8526931677833646522[101] = 0;
   out_8526931677833646522[102] = 0;
   out_8526931677833646522[103] = 0;
   out_8526931677833646522[104] = 0;
   out_8526931677833646522[105] = 0;
   out_8526931677833646522[106] = 0;
   out_8526931677833646522[107] = 0;
   out_8526931677833646522[108] = 1;
   out_8526931677833646522[109] = 0;
   out_8526931677833646522[110] = 0;
   out_8526931677833646522[111] = 0;
   out_8526931677833646522[112] = 0;
   out_8526931677833646522[113] = 0;
   out_8526931677833646522[114] = 0;
   out_8526931677833646522[115] = 0;
   out_8526931677833646522[116] = 0;
   out_8526931677833646522[117] = 0;
   out_8526931677833646522[118] = 0;
   out_8526931677833646522[119] = 0;
   out_8526931677833646522[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5691792470437897435) {
   out_5691792470437897435[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_3368042863135061588) {
   out_3368042863135061588[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3368042863135061588[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3368042863135061588[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3368042863135061588[3] = 0;
   out_3368042863135061588[4] = 0;
   out_3368042863135061588[5] = 0;
   out_3368042863135061588[6] = 1;
   out_3368042863135061588[7] = 0;
   out_3368042863135061588[8] = 0;
   out_3368042863135061588[9] = 0;
   out_3368042863135061588[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3944654474512252825) {
   out_3944654474512252825[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1517584534183337728) {
   out_1517584534183337728[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1517584534183337728[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1517584534183337728[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1517584534183337728[3] = 0;
   out_1517584534183337728[4] = 0;
   out_1517584534183337728[5] = 0;
   out_1517584534183337728[6] = 1;
   out_1517584534183337728[7] = 0;
   out_1517584534183337728[8] = 0;
   out_1517584534183337728[9] = 1;
   out_1517584534183337728[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6702181570619932104) {
   out_6702181570619932104[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6478109308117993176) {
   out_6478109308117993176[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[6] = 0;
   out_6478109308117993176[7] = 1;
   out_6478109308117993176[8] = 0;
   out_6478109308117993176[9] = 0;
   out_6478109308117993176[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6702181570619932104) {
   out_6702181570619932104[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6478109308117993176) {
   out_6478109308117993176[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6478109308117993176[6] = 0;
   out_6478109308117993176[7] = 1;
   out_6478109308117993176[8] = 0;
   out_6478109308117993176[9] = 0;
   out_6478109308117993176[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4307062985303373349) {
  err_fun(nom_x, delta_x, out_4307062985303373349);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4110902357590046395) {
  inv_err_fun(nom_x, true_x, out_4110902357590046395);
}
void gnss_H_mod_fun(double *state, double *out_6392659668905738982) {
  H_mod_fun(state, out_6392659668905738982);
}
void gnss_f_fun(double *state, double dt, double *out_7119050100122636677) {
  f_fun(state,  dt, out_7119050100122636677);
}
void gnss_F_fun(double *state, double dt, double *out_8526931677833646522) {
  F_fun(state,  dt, out_8526931677833646522);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5691792470437897435) {
  h_6(state, sat_pos, out_5691792470437897435);
}
void gnss_H_6(double *state, double *sat_pos, double *out_3368042863135061588) {
  H_6(state, sat_pos, out_3368042863135061588);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3944654474512252825) {
  h_20(state, sat_pos, out_3944654474512252825);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1517584534183337728) {
  H_20(state, sat_pos, out_1517584534183337728);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6702181570619932104) {
  h_7(state, sat_pos_vel, out_6702181570619932104);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6478109308117993176) {
  H_7(state, sat_pos_vel, out_6478109308117993176);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6702181570619932104) {
  h_21(state, sat_pos_vel, out_6702181570619932104);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6478109308117993176) {
  H_21(state, sat_pos_vel, out_6478109308117993176);
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
