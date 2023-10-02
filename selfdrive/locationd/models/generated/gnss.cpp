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
void err_fun(double *nom_x, double *delta_x, double *out_4044646051050604849) {
   out_4044646051050604849[0] = delta_x[0] + nom_x[0];
   out_4044646051050604849[1] = delta_x[1] + nom_x[1];
   out_4044646051050604849[2] = delta_x[2] + nom_x[2];
   out_4044646051050604849[3] = delta_x[3] + nom_x[3];
   out_4044646051050604849[4] = delta_x[4] + nom_x[4];
   out_4044646051050604849[5] = delta_x[5] + nom_x[5];
   out_4044646051050604849[6] = delta_x[6] + nom_x[6];
   out_4044646051050604849[7] = delta_x[7] + nom_x[7];
   out_4044646051050604849[8] = delta_x[8] + nom_x[8];
   out_4044646051050604849[9] = delta_x[9] + nom_x[9];
   out_4044646051050604849[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5682799120079336425) {
   out_5682799120079336425[0] = -nom_x[0] + true_x[0];
   out_5682799120079336425[1] = -nom_x[1] + true_x[1];
   out_5682799120079336425[2] = -nom_x[2] + true_x[2];
   out_5682799120079336425[3] = -nom_x[3] + true_x[3];
   out_5682799120079336425[4] = -nom_x[4] + true_x[4];
   out_5682799120079336425[5] = -nom_x[5] + true_x[5];
   out_5682799120079336425[6] = -nom_x[6] + true_x[6];
   out_5682799120079336425[7] = -nom_x[7] + true_x[7];
   out_5682799120079336425[8] = -nom_x[8] + true_x[8];
   out_5682799120079336425[9] = -nom_x[9] + true_x[9];
   out_5682799120079336425[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_8388095332595561320) {
   out_8388095332595561320[0] = 1.0;
   out_8388095332595561320[1] = 0;
   out_8388095332595561320[2] = 0;
   out_8388095332595561320[3] = 0;
   out_8388095332595561320[4] = 0;
   out_8388095332595561320[5] = 0;
   out_8388095332595561320[6] = 0;
   out_8388095332595561320[7] = 0;
   out_8388095332595561320[8] = 0;
   out_8388095332595561320[9] = 0;
   out_8388095332595561320[10] = 0;
   out_8388095332595561320[11] = 0;
   out_8388095332595561320[12] = 1.0;
   out_8388095332595561320[13] = 0;
   out_8388095332595561320[14] = 0;
   out_8388095332595561320[15] = 0;
   out_8388095332595561320[16] = 0;
   out_8388095332595561320[17] = 0;
   out_8388095332595561320[18] = 0;
   out_8388095332595561320[19] = 0;
   out_8388095332595561320[20] = 0;
   out_8388095332595561320[21] = 0;
   out_8388095332595561320[22] = 0;
   out_8388095332595561320[23] = 0;
   out_8388095332595561320[24] = 1.0;
   out_8388095332595561320[25] = 0;
   out_8388095332595561320[26] = 0;
   out_8388095332595561320[27] = 0;
   out_8388095332595561320[28] = 0;
   out_8388095332595561320[29] = 0;
   out_8388095332595561320[30] = 0;
   out_8388095332595561320[31] = 0;
   out_8388095332595561320[32] = 0;
   out_8388095332595561320[33] = 0;
   out_8388095332595561320[34] = 0;
   out_8388095332595561320[35] = 0;
   out_8388095332595561320[36] = 1.0;
   out_8388095332595561320[37] = 0;
   out_8388095332595561320[38] = 0;
   out_8388095332595561320[39] = 0;
   out_8388095332595561320[40] = 0;
   out_8388095332595561320[41] = 0;
   out_8388095332595561320[42] = 0;
   out_8388095332595561320[43] = 0;
   out_8388095332595561320[44] = 0;
   out_8388095332595561320[45] = 0;
   out_8388095332595561320[46] = 0;
   out_8388095332595561320[47] = 0;
   out_8388095332595561320[48] = 1.0;
   out_8388095332595561320[49] = 0;
   out_8388095332595561320[50] = 0;
   out_8388095332595561320[51] = 0;
   out_8388095332595561320[52] = 0;
   out_8388095332595561320[53] = 0;
   out_8388095332595561320[54] = 0;
   out_8388095332595561320[55] = 0;
   out_8388095332595561320[56] = 0;
   out_8388095332595561320[57] = 0;
   out_8388095332595561320[58] = 0;
   out_8388095332595561320[59] = 0;
   out_8388095332595561320[60] = 1.0;
   out_8388095332595561320[61] = 0;
   out_8388095332595561320[62] = 0;
   out_8388095332595561320[63] = 0;
   out_8388095332595561320[64] = 0;
   out_8388095332595561320[65] = 0;
   out_8388095332595561320[66] = 0;
   out_8388095332595561320[67] = 0;
   out_8388095332595561320[68] = 0;
   out_8388095332595561320[69] = 0;
   out_8388095332595561320[70] = 0;
   out_8388095332595561320[71] = 0;
   out_8388095332595561320[72] = 1.0;
   out_8388095332595561320[73] = 0;
   out_8388095332595561320[74] = 0;
   out_8388095332595561320[75] = 0;
   out_8388095332595561320[76] = 0;
   out_8388095332595561320[77] = 0;
   out_8388095332595561320[78] = 0;
   out_8388095332595561320[79] = 0;
   out_8388095332595561320[80] = 0;
   out_8388095332595561320[81] = 0;
   out_8388095332595561320[82] = 0;
   out_8388095332595561320[83] = 0;
   out_8388095332595561320[84] = 1.0;
   out_8388095332595561320[85] = 0;
   out_8388095332595561320[86] = 0;
   out_8388095332595561320[87] = 0;
   out_8388095332595561320[88] = 0;
   out_8388095332595561320[89] = 0;
   out_8388095332595561320[90] = 0;
   out_8388095332595561320[91] = 0;
   out_8388095332595561320[92] = 0;
   out_8388095332595561320[93] = 0;
   out_8388095332595561320[94] = 0;
   out_8388095332595561320[95] = 0;
   out_8388095332595561320[96] = 1.0;
   out_8388095332595561320[97] = 0;
   out_8388095332595561320[98] = 0;
   out_8388095332595561320[99] = 0;
   out_8388095332595561320[100] = 0;
   out_8388095332595561320[101] = 0;
   out_8388095332595561320[102] = 0;
   out_8388095332595561320[103] = 0;
   out_8388095332595561320[104] = 0;
   out_8388095332595561320[105] = 0;
   out_8388095332595561320[106] = 0;
   out_8388095332595561320[107] = 0;
   out_8388095332595561320[108] = 1.0;
   out_8388095332595561320[109] = 0;
   out_8388095332595561320[110] = 0;
   out_8388095332595561320[111] = 0;
   out_8388095332595561320[112] = 0;
   out_8388095332595561320[113] = 0;
   out_8388095332595561320[114] = 0;
   out_8388095332595561320[115] = 0;
   out_8388095332595561320[116] = 0;
   out_8388095332595561320[117] = 0;
   out_8388095332595561320[118] = 0;
   out_8388095332595561320[119] = 0;
   out_8388095332595561320[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5292647404086881635) {
   out_5292647404086881635[0] = dt*state[3] + state[0];
   out_5292647404086881635[1] = dt*state[4] + state[1];
   out_5292647404086881635[2] = dt*state[5] + state[2];
   out_5292647404086881635[3] = state[3];
   out_5292647404086881635[4] = state[4];
   out_5292647404086881635[5] = state[5];
   out_5292647404086881635[6] = dt*state[7] + state[6];
   out_5292647404086881635[7] = dt*state[8] + state[7];
   out_5292647404086881635[8] = state[8];
   out_5292647404086881635[9] = state[9];
   out_5292647404086881635[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7434165543291924497) {
   out_7434165543291924497[0] = 1;
   out_7434165543291924497[1] = 0;
   out_7434165543291924497[2] = 0;
   out_7434165543291924497[3] = dt;
   out_7434165543291924497[4] = 0;
   out_7434165543291924497[5] = 0;
   out_7434165543291924497[6] = 0;
   out_7434165543291924497[7] = 0;
   out_7434165543291924497[8] = 0;
   out_7434165543291924497[9] = 0;
   out_7434165543291924497[10] = 0;
   out_7434165543291924497[11] = 0;
   out_7434165543291924497[12] = 1;
   out_7434165543291924497[13] = 0;
   out_7434165543291924497[14] = 0;
   out_7434165543291924497[15] = dt;
   out_7434165543291924497[16] = 0;
   out_7434165543291924497[17] = 0;
   out_7434165543291924497[18] = 0;
   out_7434165543291924497[19] = 0;
   out_7434165543291924497[20] = 0;
   out_7434165543291924497[21] = 0;
   out_7434165543291924497[22] = 0;
   out_7434165543291924497[23] = 0;
   out_7434165543291924497[24] = 1;
   out_7434165543291924497[25] = 0;
   out_7434165543291924497[26] = 0;
   out_7434165543291924497[27] = dt;
   out_7434165543291924497[28] = 0;
   out_7434165543291924497[29] = 0;
   out_7434165543291924497[30] = 0;
   out_7434165543291924497[31] = 0;
   out_7434165543291924497[32] = 0;
   out_7434165543291924497[33] = 0;
   out_7434165543291924497[34] = 0;
   out_7434165543291924497[35] = 0;
   out_7434165543291924497[36] = 1;
   out_7434165543291924497[37] = 0;
   out_7434165543291924497[38] = 0;
   out_7434165543291924497[39] = 0;
   out_7434165543291924497[40] = 0;
   out_7434165543291924497[41] = 0;
   out_7434165543291924497[42] = 0;
   out_7434165543291924497[43] = 0;
   out_7434165543291924497[44] = 0;
   out_7434165543291924497[45] = 0;
   out_7434165543291924497[46] = 0;
   out_7434165543291924497[47] = 0;
   out_7434165543291924497[48] = 1;
   out_7434165543291924497[49] = 0;
   out_7434165543291924497[50] = 0;
   out_7434165543291924497[51] = 0;
   out_7434165543291924497[52] = 0;
   out_7434165543291924497[53] = 0;
   out_7434165543291924497[54] = 0;
   out_7434165543291924497[55] = 0;
   out_7434165543291924497[56] = 0;
   out_7434165543291924497[57] = 0;
   out_7434165543291924497[58] = 0;
   out_7434165543291924497[59] = 0;
   out_7434165543291924497[60] = 1;
   out_7434165543291924497[61] = 0;
   out_7434165543291924497[62] = 0;
   out_7434165543291924497[63] = 0;
   out_7434165543291924497[64] = 0;
   out_7434165543291924497[65] = 0;
   out_7434165543291924497[66] = 0;
   out_7434165543291924497[67] = 0;
   out_7434165543291924497[68] = 0;
   out_7434165543291924497[69] = 0;
   out_7434165543291924497[70] = 0;
   out_7434165543291924497[71] = 0;
   out_7434165543291924497[72] = 1;
   out_7434165543291924497[73] = dt;
   out_7434165543291924497[74] = 0;
   out_7434165543291924497[75] = 0;
   out_7434165543291924497[76] = 0;
   out_7434165543291924497[77] = 0;
   out_7434165543291924497[78] = 0;
   out_7434165543291924497[79] = 0;
   out_7434165543291924497[80] = 0;
   out_7434165543291924497[81] = 0;
   out_7434165543291924497[82] = 0;
   out_7434165543291924497[83] = 0;
   out_7434165543291924497[84] = 1;
   out_7434165543291924497[85] = dt;
   out_7434165543291924497[86] = 0;
   out_7434165543291924497[87] = 0;
   out_7434165543291924497[88] = 0;
   out_7434165543291924497[89] = 0;
   out_7434165543291924497[90] = 0;
   out_7434165543291924497[91] = 0;
   out_7434165543291924497[92] = 0;
   out_7434165543291924497[93] = 0;
   out_7434165543291924497[94] = 0;
   out_7434165543291924497[95] = 0;
   out_7434165543291924497[96] = 1;
   out_7434165543291924497[97] = 0;
   out_7434165543291924497[98] = 0;
   out_7434165543291924497[99] = 0;
   out_7434165543291924497[100] = 0;
   out_7434165543291924497[101] = 0;
   out_7434165543291924497[102] = 0;
   out_7434165543291924497[103] = 0;
   out_7434165543291924497[104] = 0;
   out_7434165543291924497[105] = 0;
   out_7434165543291924497[106] = 0;
   out_7434165543291924497[107] = 0;
   out_7434165543291924497[108] = 1;
   out_7434165543291924497[109] = 0;
   out_7434165543291924497[110] = 0;
   out_7434165543291924497[111] = 0;
   out_7434165543291924497[112] = 0;
   out_7434165543291924497[113] = 0;
   out_7434165543291924497[114] = 0;
   out_7434165543291924497[115] = 0;
   out_7434165543291924497[116] = 0;
   out_7434165543291924497[117] = 0;
   out_7434165543291924497[118] = 0;
   out_7434165543291924497[119] = 0;
   out_7434165543291924497[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_9172353468857903526) {
   out_9172353468857903526[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_824859287253399737) {
   out_824859287253399737[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824859287253399737[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824859287253399737[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_824859287253399737[3] = 0;
   out_824859287253399737[4] = 0;
   out_824859287253399737[5] = 0;
   out_824859287253399737[6] = 1;
   out_824859287253399737[7] = 0;
   out_824859287253399737[8] = 0;
   out_824859287253399737[9] = 0;
   out_824859287253399737[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2435912712014351082) {
   out_2435912712014351082[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_652244719691705438) {
   out_652244719691705438[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_652244719691705438[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_652244719691705438[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_652244719691705438[3] = 0;
   out_652244719691705438[4] = 0;
   out_652244719691705438[5] = 0;
   out_652244719691705438[6] = 1;
   out_652244719691705438[7] = 0;
   out_652244719691705438[8] = 0;
   out_652244719691705438[9] = 1;
   out_652244719691705438[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2377929683236467987) {
   out_2377929683236467987[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8047101356961090957) {
   out_8047101356961090957[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[6] = 0;
   out_8047101356961090957[7] = 1;
   out_8047101356961090957[8] = 0;
   out_8047101356961090957[9] = 0;
   out_8047101356961090957[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2377929683236467987) {
   out_2377929683236467987[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8047101356961090957) {
   out_8047101356961090957[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047101356961090957[6] = 0;
   out_8047101356961090957[7] = 1;
   out_8047101356961090957[8] = 0;
   out_8047101356961090957[9] = 0;
   out_8047101356961090957[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4044646051050604849) {
  err_fun(nom_x, delta_x, out_4044646051050604849);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5682799120079336425) {
  inv_err_fun(nom_x, true_x, out_5682799120079336425);
}
void gnss_H_mod_fun(double *state, double *out_8388095332595561320) {
  H_mod_fun(state, out_8388095332595561320);
}
void gnss_f_fun(double *state, double dt, double *out_5292647404086881635) {
  f_fun(state,  dt, out_5292647404086881635);
}
void gnss_F_fun(double *state, double dt, double *out_7434165543291924497) {
  F_fun(state,  dt, out_7434165543291924497);
}
void gnss_h_6(double *state, double *sat_pos, double *out_9172353468857903526) {
  h_6(state, sat_pos, out_9172353468857903526);
}
void gnss_H_6(double *state, double *sat_pos, double *out_824859287253399737) {
  H_6(state, sat_pos, out_824859287253399737);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2435912712014351082) {
  h_20(state, sat_pos, out_2435912712014351082);
}
void gnss_H_20(double *state, double *sat_pos, double *out_652244719691705438) {
  H_20(state, sat_pos, out_652244719691705438);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2377929683236467987) {
  h_7(state, sat_pos_vel, out_2377929683236467987);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8047101356961090957) {
  H_7(state, sat_pos_vel, out_8047101356961090957);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2377929683236467987) {
  h_21(state, sat_pos_vel, out_2377929683236467987);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8047101356961090957) {
  H_21(state, sat_pos_vel, out_8047101356961090957);
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
