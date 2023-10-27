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
void err_fun(double *nom_x, double *delta_x, double *out_1199080895228679500) {
   out_1199080895228679500[0] = delta_x[0] + nom_x[0];
   out_1199080895228679500[1] = delta_x[1] + nom_x[1];
   out_1199080895228679500[2] = delta_x[2] + nom_x[2];
   out_1199080895228679500[3] = delta_x[3] + nom_x[3];
   out_1199080895228679500[4] = delta_x[4] + nom_x[4];
   out_1199080895228679500[5] = delta_x[5] + nom_x[5];
   out_1199080895228679500[6] = delta_x[6] + nom_x[6];
   out_1199080895228679500[7] = delta_x[7] + nom_x[7];
   out_1199080895228679500[8] = delta_x[8] + nom_x[8];
   out_1199080895228679500[9] = delta_x[9] + nom_x[9];
   out_1199080895228679500[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3733508033735132626) {
   out_3733508033735132626[0] = -nom_x[0] + true_x[0];
   out_3733508033735132626[1] = -nom_x[1] + true_x[1];
   out_3733508033735132626[2] = -nom_x[2] + true_x[2];
   out_3733508033735132626[3] = -nom_x[3] + true_x[3];
   out_3733508033735132626[4] = -nom_x[4] + true_x[4];
   out_3733508033735132626[5] = -nom_x[5] + true_x[5];
   out_3733508033735132626[6] = -nom_x[6] + true_x[6];
   out_3733508033735132626[7] = -nom_x[7] + true_x[7];
   out_3733508033735132626[8] = -nom_x[8] + true_x[8];
   out_3733508033735132626[9] = -nom_x[9] + true_x[9];
   out_3733508033735132626[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3459453620548230826) {
   out_3459453620548230826[0] = 1.0;
   out_3459453620548230826[1] = 0;
   out_3459453620548230826[2] = 0;
   out_3459453620548230826[3] = 0;
   out_3459453620548230826[4] = 0;
   out_3459453620548230826[5] = 0;
   out_3459453620548230826[6] = 0;
   out_3459453620548230826[7] = 0;
   out_3459453620548230826[8] = 0;
   out_3459453620548230826[9] = 0;
   out_3459453620548230826[10] = 0;
   out_3459453620548230826[11] = 0;
   out_3459453620548230826[12] = 1.0;
   out_3459453620548230826[13] = 0;
   out_3459453620548230826[14] = 0;
   out_3459453620548230826[15] = 0;
   out_3459453620548230826[16] = 0;
   out_3459453620548230826[17] = 0;
   out_3459453620548230826[18] = 0;
   out_3459453620548230826[19] = 0;
   out_3459453620548230826[20] = 0;
   out_3459453620548230826[21] = 0;
   out_3459453620548230826[22] = 0;
   out_3459453620548230826[23] = 0;
   out_3459453620548230826[24] = 1.0;
   out_3459453620548230826[25] = 0;
   out_3459453620548230826[26] = 0;
   out_3459453620548230826[27] = 0;
   out_3459453620548230826[28] = 0;
   out_3459453620548230826[29] = 0;
   out_3459453620548230826[30] = 0;
   out_3459453620548230826[31] = 0;
   out_3459453620548230826[32] = 0;
   out_3459453620548230826[33] = 0;
   out_3459453620548230826[34] = 0;
   out_3459453620548230826[35] = 0;
   out_3459453620548230826[36] = 1.0;
   out_3459453620548230826[37] = 0;
   out_3459453620548230826[38] = 0;
   out_3459453620548230826[39] = 0;
   out_3459453620548230826[40] = 0;
   out_3459453620548230826[41] = 0;
   out_3459453620548230826[42] = 0;
   out_3459453620548230826[43] = 0;
   out_3459453620548230826[44] = 0;
   out_3459453620548230826[45] = 0;
   out_3459453620548230826[46] = 0;
   out_3459453620548230826[47] = 0;
   out_3459453620548230826[48] = 1.0;
   out_3459453620548230826[49] = 0;
   out_3459453620548230826[50] = 0;
   out_3459453620548230826[51] = 0;
   out_3459453620548230826[52] = 0;
   out_3459453620548230826[53] = 0;
   out_3459453620548230826[54] = 0;
   out_3459453620548230826[55] = 0;
   out_3459453620548230826[56] = 0;
   out_3459453620548230826[57] = 0;
   out_3459453620548230826[58] = 0;
   out_3459453620548230826[59] = 0;
   out_3459453620548230826[60] = 1.0;
   out_3459453620548230826[61] = 0;
   out_3459453620548230826[62] = 0;
   out_3459453620548230826[63] = 0;
   out_3459453620548230826[64] = 0;
   out_3459453620548230826[65] = 0;
   out_3459453620548230826[66] = 0;
   out_3459453620548230826[67] = 0;
   out_3459453620548230826[68] = 0;
   out_3459453620548230826[69] = 0;
   out_3459453620548230826[70] = 0;
   out_3459453620548230826[71] = 0;
   out_3459453620548230826[72] = 1.0;
   out_3459453620548230826[73] = 0;
   out_3459453620548230826[74] = 0;
   out_3459453620548230826[75] = 0;
   out_3459453620548230826[76] = 0;
   out_3459453620548230826[77] = 0;
   out_3459453620548230826[78] = 0;
   out_3459453620548230826[79] = 0;
   out_3459453620548230826[80] = 0;
   out_3459453620548230826[81] = 0;
   out_3459453620548230826[82] = 0;
   out_3459453620548230826[83] = 0;
   out_3459453620548230826[84] = 1.0;
   out_3459453620548230826[85] = 0;
   out_3459453620548230826[86] = 0;
   out_3459453620548230826[87] = 0;
   out_3459453620548230826[88] = 0;
   out_3459453620548230826[89] = 0;
   out_3459453620548230826[90] = 0;
   out_3459453620548230826[91] = 0;
   out_3459453620548230826[92] = 0;
   out_3459453620548230826[93] = 0;
   out_3459453620548230826[94] = 0;
   out_3459453620548230826[95] = 0;
   out_3459453620548230826[96] = 1.0;
   out_3459453620548230826[97] = 0;
   out_3459453620548230826[98] = 0;
   out_3459453620548230826[99] = 0;
   out_3459453620548230826[100] = 0;
   out_3459453620548230826[101] = 0;
   out_3459453620548230826[102] = 0;
   out_3459453620548230826[103] = 0;
   out_3459453620548230826[104] = 0;
   out_3459453620548230826[105] = 0;
   out_3459453620548230826[106] = 0;
   out_3459453620548230826[107] = 0;
   out_3459453620548230826[108] = 1.0;
   out_3459453620548230826[109] = 0;
   out_3459453620548230826[110] = 0;
   out_3459453620548230826[111] = 0;
   out_3459453620548230826[112] = 0;
   out_3459453620548230826[113] = 0;
   out_3459453620548230826[114] = 0;
   out_3459453620548230826[115] = 0;
   out_3459453620548230826[116] = 0;
   out_3459453620548230826[117] = 0;
   out_3459453620548230826[118] = 0;
   out_3459453620548230826[119] = 0;
   out_3459453620548230826[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_9104597615753488104) {
   out_9104597615753488104[0] = dt*state[3] + state[0];
   out_9104597615753488104[1] = dt*state[4] + state[1];
   out_9104597615753488104[2] = dt*state[5] + state[2];
   out_9104597615753488104[3] = state[3];
   out_9104597615753488104[4] = state[4];
   out_9104597615753488104[5] = state[5];
   out_9104597615753488104[6] = dt*state[7] + state[6];
   out_9104597615753488104[7] = dt*state[8] + state[7];
   out_9104597615753488104[8] = state[8];
   out_9104597615753488104[9] = state[9];
   out_9104597615753488104[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8533997876613764308) {
   out_8533997876613764308[0] = 1;
   out_8533997876613764308[1] = 0;
   out_8533997876613764308[2] = 0;
   out_8533997876613764308[3] = dt;
   out_8533997876613764308[4] = 0;
   out_8533997876613764308[5] = 0;
   out_8533997876613764308[6] = 0;
   out_8533997876613764308[7] = 0;
   out_8533997876613764308[8] = 0;
   out_8533997876613764308[9] = 0;
   out_8533997876613764308[10] = 0;
   out_8533997876613764308[11] = 0;
   out_8533997876613764308[12] = 1;
   out_8533997876613764308[13] = 0;
   out_8533997876613764308[14] = 0;
   out_8533997876613764308[15] = dt;
   out_8533997876613764308[16] = 0;
   out_8533997876613764308[17] = 0;
   out_8533997876613764308[18] = 0;
   out_8533997876613764308[19] = 0;
   out_8533997876613764308[20] = 0;
   out_8533997876613764308[21] = 0;
   out_8533997876613764308[22] = 0;
   out_8533997876613764308[23] = 0;
   out_8533997876613764308[24] = 1;
   out_8533997876613764308[25] = 0;
   out_8533997876613764308[26] = 0;
   out_8533997876613764308[27] = dt;
   out_8533997876613764308[28] = 0;
   out_8533997876613764308[29] = 0;
   out_8533997876613764308[30] = 0;
   out_8533997876613764308[31] = 0;
   out_8533997876613764308[32] = 0;
   out_8533997876613764308[33] = 0;
   out_8533997876613764308[34] = 0;
   out_8533997876613764308[35] = 0;
   out_8533997876613764308[36] = 1;
   out_8533997876613764308[37] = 0;
   out_8533997876613764308[38] = 0;
   out_8533997876613764308[39] = 0;
   out_8533997876613764308[40] = 0;
   out_8533997876613764308[41] = 0;
   out_8533997876613764308[42] = 0;
   out_8533997876613764308[43] = 0;
   out_8533997876613764308[44] = 0;
   out_8533997876613764308[45] = 0;
   out_8533997876613764308[46] = 0;
   out_8533997876613764308[47] = 0;
   out_8533997876613764308[48] = 1;
   out_8533997876613764308[49] = 0;
   out_8533997876613764308[50] = 0;
   out_8533997876613764308[51] = 0;
   out_8533997876613764308[52] = 0;
   out_8533997876613764308[53] = 0;
   out_8533997876613764308[54] = 0;
   out_8533997876613764308[55] = 0;
   out_8533997876613764308[56] = 0;
   out_8533997876613764308[57] = 0;
   out_8533997876613764308[58] = 0;
   out_8533997876613764308[59] = 0;
   out_8533997876613764308[60] = 1;
   out_8533997876613764308[61] = 0;
   out_8533997876613764308[62] = 0;
   out_8533997876613764308[63] = 0;
   out_8533997876613764308[64] = 0;
   out_8533997876613764308[65] = 0;
   out_8533997876613764308[66] = 0;
   out_8533997876613764308[67] = 0;
   out_8533997876613764308[68] = 0;
   out_8533997876613764308[69] = 0;
   out_8533997876613764308[70] = 0;
   out_8533997876613764308[71] = 0;
   out_8533997876613764308[72] = 1;
   out_8533997876613764308[73] = dt;
   out_8533997876613764308[74] = 0;
   out_8533997876613764308[75] = 0;
   out_8533997876613764308[76] = 0;
   out_8533997876613764308[77] = 0;
   out_8533997876613764308[78] = 0;
   out_8533997876613764308[79] = 0;
   out_8533997876613764308[80] = 0;
   out_8533997876613764308[81] = 0;
   out_8533997876613764308[82] = 0;
   out_8533997876613764308[83] = 0;
   out_8533997876613764308[84] = 1;
   out_8533997876613764308[85] = dt;
   out_8533997876613764308[86] = 0;
   out_8533997876613764308[87] = 0;
   out_8533997876613764308[88] = 0;
   out_8533997876613764308[89] = 0;
   out_8533997876613764308[90] = 0;
   out_8533997876613764308[91] = 0;
   out_8533997876613764308[92] = 0;
   out_8533997876613764308[93] = 0;
   out_8533997876613764308[94] = 0;
   out_8533997876613764308[95] = 0;
   out_8533997876613764308[96] = 1;
   out_8533997876613764308[97] = 0;
   out_8533997876613764308[98] = 0;
   out_8533997876613764308[99] = 0;
   out_8533997876613764308[100] = 0;
   out_8533997876613764308[101] = 0;
   out_8533997876613764308[102] = 0;
   out_8533997876613764308[103] = 0;
   out_8533997876613764308[104] = 0;
   out_8533997876613764308[105] = 0;
   out_8533997876613764308[106] = 0;
   out_8533997876613764308[107] = 0;
   out_8533997876613764308[108] = 1;
   out_8533997876613764308[109] = 0;
   out_8533997876613764308[110] = 0;
   out_8533997876613764308[111] = 0;
   out_8533997876613764308[112] = 0;
   out_8533997876613764308[113] = 0;
   out_8533997876613764308[114] = 0;
   out_8533997876613764308[115] = 0;
   out_8533997876613764308[116] = 0;
   out_8533997876613764308[117] = 0;
   out_8533997876613764308[118] = 0;
   out_8533997876613764308[119] = 0;
   out_8533997876613764308[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3522191147772980852) {
   out_3522191147772980852[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_41023783411261112) {
   out_41023783411261112[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_41023783411261112[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_41023783411261112[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_41023783411261112[3] = 0;
   out_41023783411261112[4] = 0;
   out_41023783411261112[5] = 0;
   out_41023783411261112[6] = 1;
   out_41023783411261112[7] = 0;
   out_41023783411261112[8] = 0;
   out_41023783411261112[9] = 0;
   out_41023783411261112[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2634656617657040111) {
   out_2634656617657040111[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3182068259027006405) {
   out_3182068259027006405[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3182068259027006405[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3182068259027006405[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3182068259027006405[3] = 0;
   out_3182068259027006405[4] = 0;
   out_3182068259027006405[5] = 0;
   out_3182068259027006405[6] = 1;
   out_3182068259027006405[7] = 0;
   out_3182068259027006405[8] = 0;
   out_3182068259027006405[9] = 1;
   out_3182068259027006405[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3983293769857751604) {
   out_3983293769857751604[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1182800211671042582) {
   out_1182800211671042582[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[6] = 0;
   out_1182800211671042582[7] = 1;
   out_1182800211671042582[8] = 0;
   out_1182800211671042582[9] = 0;
   out_1182800211671042582[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3983293769857751604) {
   out_3983293769857751604[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1182800211671042582) {
   out_1182800211671042582[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1182800211671042582[6] = 0;
   out_1182800211671042582[7] = 1;
   out_1182800211671042582[8] = 0;
   out_1182800211671042582[9] = 0;
   out_1182800211671042582[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1199080895228679500) {
  err_fun(nom_x, delta_x, out_1199080895228679500);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3733508033735132626) {
  inv_err_fun(nom_x, true_x, out_3733508033735132626);
}
void gnss_H_mod_fun(double *state, double *out_3459453620548230826) {
  H_mod_fun(state, out_3459453620548230826);
}
void gnss_f_fun(double *state, double dt, double *out_9104597615753488104) {
  f_fun(state,  dt, out_9104597615753488104);
}
void gnss_F_fun(double *state, double dt, double *out_8533997876613764308) {
  F_fun(state,  dt, out_8533997876613764308);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3522191147772980852) {
  h_6(state, sat_pos, out_3522191147772980852);
}
void gnss_H_6(double *state, double *sat_pos, double *out_41023783411261112) {
  H_6(state, sat_pos, out_41023783411261112);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2634656617657040111) {
  h_20(state, sat_pos, out_2634656617657040111);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3182068259027006405) {
  H_20(state, sat_pos, out_3182068259027006405);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3983293769857751604) {
  h_7(state, sat_pos_vel, out_3983293769857751604);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1182800211671042582) {
  H_7(state, sat_pos_vel, out_1182800211671042582);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3983293769857751604) {
  h_21(state, sat_pos_vel, out_3983293769857751604);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1182800211671042582) {
  H_21(state, sat_pos_vel, out_1182800211671042582);
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
