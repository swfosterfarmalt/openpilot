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
void err_fun(double *nom_x, double *delta_x, double *out_2759909455403238108) {
   out_2759909455403238108[0] = delta_x[0] + nom_x[0];
   out_2759909455403238108[1] = delta_x[1] + nom_x[1];
   out_2759909455403238108[2] = delta_x[2] + nom_x[2];
   out_2759909455403238108[3] = delta_x[3] + nom_x[3];
   out_2759909455403238108[4] = delta_x[4] + nom_x[4];
   out_2759909455403238108[5] = delta_x[5] + nom_x[5];
   out_2759909455403238108[6] = delta_x[6] + nom_x[6];
   out_2759909455403238108[7] = delta_x[7] + nom_x[7];
   out_2759909455403238108[8] = delta_x[8] + nom_x[8];
   out_2759909455403238108[9] = delta_x[9] + nom_x[9];
   out_2759909455403238108[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8906084683496585849) {
   out_8906084683496585849[0] = -nom_x[0] + true_x[0];
   out_8906084683496585849[1] = -nom_x[1] + true_x[1];
   out_8906084683496585849[2] = -nom_x[2] + true_x[2];
   out_8906084683496585849[3] = -nom_x[3] + true_x[3];
   out_8906084683496585849[4] = -nom_x[4] + true_x[4];
   out_8906084683496585849[5] = -nom_x[5] + true_x[5];
   out_8906084683496585849[6] = -nom_x[6] + true_x[6];
   out_8906084683496585849[7] = -nom_x[7] + true_x[7];
   out_8906084683496585849[8] = -nom_x[8] + true_x[8];
   out_8906084683496585849[9] = -nom_x[9] + true_x[9];
   out_8906084683496585849[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4553317514464520854) {
   out_4553317514464520854[0] = 1.0;
   out_4553317514464520854[1] = 0;
   out_4553317514464520854[2] = 0;
   out_4553317514464520854[3] = 0;
   out_4553317514464520854[4] = 0;
   out_4553317514464520854[5] = 0;
   out_4553317514464520854[6] = 0;
   out_4553317514464520854[7] = 0;
   out_4553317514464520854[8] = 0;
   out_4553317514464520854[9] = 0;
   out_4553317514464520854[10] = 0;
   out_4553317514464520854[11] = 0;
   out_4553317514464520854[12] = 1.0;
   out_4553317514464520854[13] = 0;
   out_4553317514464520854[14] = 0;
   out_4553317514464520854[15] = 0;
   out_4553317514464520854[16] = 0;
   out_4553317514464520854[17] = 0;
   out_4553317514464520854[18] = 0;
   out_4553317514464520854[19] = 0;
   out_4553317514464520854[20] = 0;
   out_4553317514464520854[21] = 0;
   out_4553317514464520854[22] = 0;
   out_4553317514464520854[23] = 0;
   out_4553317514464520854[24] = 1.0;
   out_4553317514464520854[25] = 0;
   out_4553317514464520854[26] = 0;
   out_4553317514464520854[27] = 0;
   out_4553317514464520854[28] = 0;
   out_4553317514464520854[29] = 0;
   out_4553317514464520854[30] = 0;
   out_4553317514464520854[31] = 0;
   out_4553317514464520854[32] = 0;
   out_4553317514464520854[33] = 0;
   out_4553317514464520854[34] = 0;
   out_4553317514464520854[35] = 0;
   out_4553317514464520854[36] = 1.0;
   out_4553317514464520854[37] = 0;
   out_4553317514464520854[38] = 0;
   out_4553317514464520854[39] = 0;
   out_4553317514464520854[40] = 0;
   out_4553317514464520854[41] = 0;
   out_4553317514464520854[42] = 0;
   out_4553317514464520854[43] = 0;
   out_4553317514464520854[44] = 0;
   out_4553317514464520854[45] = 0;
   out_4553317514464520854[46] = 0;
   out_4553317514464520854[47] = 0;
   out_4553317514464520854[48] = 1.0;
   out_4553317514464520854[49] = 0;
   out_4553317514464520854[50] = 0;
   out_4553317514464520854[51] = 0;
   out_4553317514464520854[52] = 0;
   out_4553317514464520854[53] = 0;
   out_4553317514464520854[54] = 0;
   out_4553317514464520854[55] = 0;
   out_4553317514464520854[56] = 0;
   out_4553317514464520854[57] = 0;
   out_4553317514464520854[58] = 0;
   out_4553317514464520854[59] = 0;
   out_4553317514464520854[60] = 1.0;
   out_4553317514464520854[61] = 0;
   out_4553317514464520854[62] = 0;
   out_4553317514464520854[63] = 0;
   out_4553317514464520854[64] = 0;
   out_4553317514464520854[65] = 0;
   out_4553317514464520854[66] = 0;
   out_4553317514464520854[67] = 0;
   out_4553317514464520854[68] = 0;
   out_4553317514464520854[69] = 0;
   out_4553317514464520854[70] = 0;
   out_4553317514464520854[71] = 0;
   out_4553317514464520854[72] = 1.0;
   out_4553317514464520854[73] = 0;
   out_4553317514464520854[74] = 0;
   out_4553317514464520854[75] = 0;
   out_4553317514464520854[76] = 0;
   out_4553317514464520854[77] = 0;
   out_4553317514464520854[78] = 0;
   out_4553317514464520854[79] = 0;
   out_4553317514464520854[80] = 0;
   out_4553317514464520854[81] = 0;
   out_4553317514464520854[82] = 0;
   out_4553317514464520854[83] = 0;
   out_4553317514464520854[84] = 1.0;
   out_4553317514464520854[85] = 0;
   out_4553317514464520854[86] = 0;
   out_4553317514464520854[87] = 0;
   out_4553317514464520854[88] = 0;
   out_4553317514464520854[89] = 0;
   out_4553317514464520854[90] = 0;
   out_4553317514464520854[91] = 0;
   out_4553317514464520854[92] = 0;
   out_4553317514464520854[93] = 0;
   out_4553317514464520854[94] = 0;
   out_4553317514464520854[95] = 0;
   out_4553317514464520854[96] = 1.0;
   out_4553317514464520854[97] = 0;
   out_4553317514464520854[98] = 0;
   out_4553317514464520854[99] = 0;
   out_4553317514464520854[100] = 0;
   out_4553317514464520854[101] = 0;
   out_4553317514464520854[102] = 0;
   out_4553317514464520854[103] = 0;
   out_4553317514464520854[104] = 0;
   out_4553317514464520854[105] = 0;
   out_4553317514464520854[106] = 0;
   out_4553317514464520854[107] = 0;
   out_4553317514464520854[108] = 1.0;
   out_4553317514464520854[109] = 0;
   out_4553317514464520854[110] = 0;
   out_4553317514464520854[111] = 0;
   out_4553317514464520854[112] = 0;
   out_4553317514464520854[113] = 0;
   out_4553317514464520854[114] = 0;
   out_4553317514464520854[115] = 0;
   out_4553317514464520854[116] = 0;
   out_4553317514464520854[117] = 0;
   out_4553317514464520854[118] = 0;
   out_4553317514464520854[119] = 0;
   out_4553317514464520854[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3315877890026810001) {
   out_3315877890026810001[0] = dt*state[3] + state[0];
   out_3315877890026810001[1] = dt*state[4] + state[1];
   out_3315877890026810001[2] = dt*state[5] + state[2];
   out_3315877890026810001[3] = state[3];
   out_3315877890026810001[4] = state[4];
   out_3315877890026810001[5] = state[5];
   out_3315877890026810001[6] = dt*state[7] + state[6];
   out_3315877890026810001[7] = dt*state[8] + state[7];
   out_3315877890026810001[8] = state[8];
   out_3315877890026810001[9] = state[9];
   out_3315877890026810001[10] = state[10];
}
void F_fun(double *state, double dt, double *out_647918121564994917) {
   out_647918121564994917[0] = 1;
   out_647918121564994917[1] = 0;
   out_647918121564994917[2] = 0;
   out_647918121564994917[3] = dt;
   out_647918121564994917[4] = 0;
   out_647918121564994917[5] = 0;
   out_647918121564994917[6] = 0;
   out_647918121564994917[7] = 0;
   out_647918121564994917[8] = 0;
   out_647918121564994917[9] = 0;
   out_647918121564994917[10] = 0;
   out_647918121564994917[11] = 0;
   out_647918121564994917[12] = 1;
   out_647918121564994917[13] = 0;
   out_647918121564994917[14] = 0;
   out_647918121564994917[15] = dt;
   out_647918121564994917[16] = 0;
   out_647918121564994917[17] = 0;
   out_647918121564994917[18] = 0;
   out_647918121564994917[19] = 0;
   out_647918121564994917[20] = 0;
   out_647918121564994917[21] = 0;
   out_647918121564994917[22] = 0;
   out_647918121564994917[23] = 0;
   out_647918121564994917[24] = 1;
   out_647918121564994917[25] = 0;
   out_647918121564994917[26] = 0;
   out_647918121564994917[27] = dt;
   out_647918121564994917[28] = 0;
   out_647918121564994917[29] = 0;
   out_647918121564994917[30] = 0;
   out_647918121564994917[31] = 0;
   out_647918121564994917[32] = 0;
   out_647918121564994917[33] = 0;
   out_647918121564994917[34] = 0;
   out_647918121564994917[35] = 0;
   out_647918121564994917[36] = 1;
   out_647918121564994917[37] = 0;
   out_647918121564994917[38] = 0;
   out_647918121564994917[39] = 0;
   out_647918121564994917[40] = 0;
   out_647918121564994917[41] = 0;
   out_647918121564994917[42] = 0;
   out_647918121564994917[43] = 0;
   out_647918121564994917[44] = 0;
   out_647918121564994917[45] = 0;
   out_647918121564994917[46] = 0;
   out_647918121564994917[47] = 0;
   out_647918121564994917[48] = 1;
   out_647918121564994917[49] = 0;
   out_647918121564994917[50] = 0;
   out_647918121564994917[51] = 0;
   out_647918121564994917[52] = 0;
   out_647918121564994917[53] = 0;
   out_647918121564994917[54] = 0;
   out_647918121564994917[55] = 0;
   out_647918121564994917[56] = 0;
   out_647918121564994917[57] = 0;
   out_647918121564994917[58] = 0;
   out_647918121564994917[59] = 0;
   out_647918121564994917[60] = 1;
   out_647918121564994917[61] = 0;
   out_647918121564994917[62] = 0;
   out_647918121564994917[63] = 0;
   out_647918121564994917[64] = 0;
   out_647918121564994917[65] = 0;
   out_647918121564994917[66] = 0;
   out_647918121564994917[67] = 0;
   out_647918121564994917[68] = 0;
   out_647918121564994917[69] = 0;
   out_647918121564994917[70] = 0;
   out_647918121564994917[71] = 0;
   out_647918121564994917[72] = 1;
   out_647918121564994917[73] = dt;
   out_647918121564994917[74] = 0;
   out_647918121564994917[75] = 0;
   out_647918121564994917[76] = 0;
   out_647918121564994917[77] = 0;
   out_647918121564994917[78] = 0;
   out_647918121564994917[79] = 0;
   out_647918121564994917[80] = 0;
   out_647918121564994917[81] = 0;
   out_647918121564994917[82] = 0;
   out_647918121564994917[83] = 0;
   out_647918121564994917[84] = 1;
   out_647918121564994917[85] = dt;
   out_647918121564994917[86] = 0;
   out_647918121564994917[87] = 0;
   out_647918121564994917[88] = 0;
   out_647918121564994917[89] = 0;
   out_647918121564994917[90] = 0;
   out_647918121564994917[91] = 0;
   out_647918121564994917[92] = 0;
   out_647918121564994917[93] = 0;
   out_647918121564994917[94] = 0;
   out_647918121564994917[95] = 0;
   out_647918121564994917[96] = 1;
   out_647918121564994917[97] = 0;
   out_647918121564994917[98] = 0;
   out_647918121564994917[99] = 0;
   out_647918121564994917[100] = 0;
   out_647918121564994917[101] = 0;
   out_647918121564994917[102] = 0;
   out_647918121564994917[103] = 0;
   out_647918121564994917[104] = 0;
   out_647918121564994917[105] = 0;
   out_647918121564994917[106] = 0;
   out_647918121564994917[107] = 0;
   out_647918121564994917[108] = 1;
   out_647918121564994917[109] = 0;
   out_647918121564994917[110] = 0;
   out_647918121564994917[111] = 0;
   out_647918121564994917[112] = 0;
   out_647918121564994917[113] = 0;
   out_647918121564994917[114] = 0;
   out_647918121564994917[115] = 0;
   out_647918121564994917[116] = 0;
   out_647918121564994917[117] = 0;
   out_647918121564994917[118] = 0;
   out_647918121564994917[119] = 0;
   out_647918121564994917[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3218075092645176804) {
   out_3218075092645176804[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8344807646612916812) {
   out_8344807646612916812[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8344807646612916812[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8344807646612916812[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8344807646612916812[3] = 0;
   out_8344807646612916812[4] = 0;
   out_8344807646612916812[5] = 0;
   out_8344807646612916812[6] = 1;
   out_8344807646612916812[7] = 0;
   out_8344807646612916812[8] = 0;
   out_8344807646612916812[9] = 0;
   out_8344807646612916812[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_324931965682456940) {
   out_324931965682456940[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2633905436684386266) {
   out_2633905436684386266[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2633905436684386266[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2633905436684386266[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2633905436684386266[3] = 0;
   out_2633905436684386266[4] = 0;
   out_2633905436684386266[5] = 0;
   out_2633905436684386266[6] = 1;
   out_2633905436684386266[7] = 0;
   out_2633905436684386266[8] = 0;
   out_2633905436684386266[9] = 1;
   out_2633905436684386266[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7688980450062471422) {
   out_7688980450062471422[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7177043048130116370) {
   out_7177043048130116370[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[6] = 0;
   out_7177043048130116370[7] = 1;
   out_7177043048130116370[8] = 0;
   out_7177043048130116370[9] = 0;
   out_7177043048130116370[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7688980450062471422) {
   out_7688980450062471422[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7177043048130116370) {
   out_7177043048130116370[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177043048130116370[6] = 0;
   out_7177043048130116370[7] = 1;
   out_7177043048130116370[8] = 0;
   out_7177043048130116370[9] = 0;
   out_7177043048130116370[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2759909455403238108) {
  err_fun(nom_x, delta_x, out_2759909455403238108);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8906084683496585849) {
  inv_err_fun(nom_x, true_x, out_8906084683496585849);
}
void gnss_H_mod_fun(double *state, double *out_4553317514464520854) {
  H_mod_fun(state, out_4553317514464520854);
}
void gnss_f_fun(double *state, double dt, double *out_3315877890026810001) {
  f_fun(state,  dt, out_3315877890026810001);
}
void gnss_F_fun(double *state, double dt, double *out_647918121564994917) {
  F_fun(state,  dt, out_647918121564994917);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3218075092645176804) {
  h_6(state, sat_pos, out_3218075092645176804);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8344807646612916812) {
  H_6(state, sat_pos, out_8344807646612916812);
}
void gnss_h_20(double *state, double *sat_pos, double *out_324931965682456940) {
  h_20(state, sat_pos, out_324931965682456940);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2633905436684386266) {
  H_20(state, sat_pos, out_2633905436684386266);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7688980450062471422) {
  h_7(state, sat_pos_vel, out_7688980450062471422);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7177043048130116370) {
  H_7(state, sat_pos_vel, out_7177043048130116370);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7688980450062471422) {
  h_21(state, sat_pos_vel, out_7688980450062471422);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7177043048130116370) {
  H_21(state, sat_pos_vel, out_7177043048130116370);
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
