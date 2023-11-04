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
void err_fun(double *nom_x, double *delta_x, double *out_9018383677196817484) {
   out_9018383677196817484[0] = delta_x[0] + nom_x[0];
   out_9018383677196817484[1] = delta_x[1] + nom_x[1];
   out_9018383677196817484[2] = delta_x[2] + nom_x[2];
   out_9018383677196817484[3] = delta_x[3] + nom_x[3];
   out_9018383677196817484[4] = delta_x[4] + nom_x[4];
   out_9018383677196817484[5] = delta_x[5] + nom_x[5];
   out_9018383677196817484[6] = delta_x[6] + nom_x[6];
   out_9018383677196817484[7] = delta_x[7] + nom_x[7];
   out_9018383677196817484[8] = delta_x[8] + nom_x[8];
   out_9018383677196817484[9] = delta_x[9] + nom_x[9];
   out_9018383677196817484[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7459293352492598460) {
   out_7459293352492598460[0] = -nom_x[0] + true_x[0];
   out_7459293352492598460[1] = -nom_x[1] + true_x[1];
   out_7459293352492598460[2] = -nom_x[2] + true_x[2];
   out_7459293352492598460[3] = -nom_x[3] + true_x[3];
   out_7459293352492598460[4] = -nom_x[4] + true_x[4];
   out_7459293352492598460[5] = -nom_x[5] + true_x[5];
   out_7459293352492598460[6] = -nom_x[6] + true_x[6];
   out_7459293352492598460[7] = -nom_x[7] + true_x[7];
   out_7459293352492598460[8] = -nom_x[8] + true_x[8];
   out_7459293352492598460[9] = -nom_x[9] + true_x[9];
   out_7459293352492598460[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2463199541683517663) {
   out_2463199541683517663[0] = 1.0;
   out_2463199541683517663[1] = 0;
   out_2463199541683517663[2] = 0;
   out_2463199541683517663[3] = 0;
   out_2463199541683517663[4] = 0;
   out_2463199541683517663[5] = 0;
   out_2463199541683517663[6] = 0;
   out_2463199541683517663[7] = 0;
   out_2463199541683517663[8] = 0;
   out_2463199541683517663[9] = 0;
   out_2463199541683517663[10] = 0;
   out_2463199541683517663[11] = 0;
   out_2463199541683517663[12] = 1.0;
   out_2463199541683517663[13] = 0;
   out_2463199541683517663[14] = 0;
   out_2463199541683517663[15] = 0;
   out_2463199541683517663[16] = 0;
   out_2463199541683517663[17] = 0;
   out_2463199541683517663[18] = 0;
   out_2463199541683517663[19] = 0;
   out_2463199541683517663[20] = 0;
   out_2463199541683517663[21] = 0;
   out_2463199541683517663[22] = 0;
   out_2463199541683517663[23] = 0;
   out_2463199541683517663[24] = 1.0;
   out_2463199541683517663[25] = 0;
   out_2463199541683517663[26] = 0;
   out_2463199541683517663[27] = 0;
   out_2463199541683517663[28] = 0;
   out_2463199541683517663[29] = 0;
   out_2463199541683517663[30] = 0;
   out_2463199541683517663[31] = 0;
   out_2463199541683517663[32] = 0;
   out_2463199541683517663[33] = 0;
   out_2463199541683517663[34] = 0;
   out_2463199541683517663[35] = 0;
   out_2463199541683517663[36] = 1.0;
   out_2463199541683517663[37] = 0;
   out_2463199541683517663[38] = 0;
   out_2463199541683517663[39] = 0;
   out_2463199541683517663[40] = 0;
   out_2463199541683517663[41] = 0;
   out_2463199541683517663[42] = 0;
   out_2463199541683517663[43] = 0;
   out_2463199541683517663[44] = 0;
   out_2463199541683517663[45] = 0;
   out_2463199541683517663[46] = 0;
   out_2463199541683517663[47] = 0;
   out_2463199541683517663[48] = 1.0;
   out_2463199541683517663[49] = 0;
   out_2463199541683517663[50] = 0;
   out_2463199541683517663[51] = 0;
   out_2463199541683517663[52] = 0;
   out_2463199541683517663[53] = 0;
   out_2463199541683517663[54] = 0;
   out_2463199541683517663[55] = 0;
   out_2463199541683517663[56] = 0;
   out_2463199541683517663[57] = 0;
   out_2463199541683517663[58] = 0;
   out_2463199541683517663[59] = 0;
   out_2463199541683517663[60] = 1.0;
   out_2463199541683517663[61] = 0;
   out_2463199541683517663[62] = 0;
   out_2463199541683517663[63] = 0;
   out_2463199541683517663[64] = 0;
   out_2463199541683517663[65] = 0;
   out_2463199541683517663[66] = 0;
   out_2463199541683517663[67] = 0;
   out_2463199541683517663[68] = 0;
   out_2463199541683517663[69] = 0;
   out_2463199541683517663[70] = 0;
   out_2463199541683517663[71] = 0;
   out_2463199541683517663[72] = 1.0;
   out_2463199541683517663[73] = 0;
   out_2463199541683517663[74] = 0;
   out_2463199541683517663[75] = 0;
   out_2463199541683517663[76] = 0;
   out_2463199541683517663[77] = 0;
   out_2463199541683517663[78] = 0;
   out_2463199541683517663[79] = 0;
   out_2463199541683517663[80] = 0;
   out_2463199541683517663[81] = 0;
   out_2463199541683517663[82] = 0;
   out_2463199541683517663[83] = 0;
   out_2463199541683517663[84] = 1.0;
   out_2463199541683517663[85] = 0;
   out_2463199541683517663[86] = 0;
   out_2463199541683517663[87] = 0;
   out_2463199541683517663[88] = 0;
   out_2463199541683517663[89] = 0;
   out_2463199541683517663[90] = 0;
   out_2463199541683517663[91] = 0;
   out_2463199541683517663[92] = 0;
   out_2463199541683517663[93] = 0;
   out_2463199541683517663[94] = 0;
   out_2463199541683517663[95] = 0;
   out_2463199541683517663[96] = 1.0;
   out_2463199541683517663[97] = 0;
   out_2463199541683517663[98] = 0;
   out_2463199541683517663[99] = 0;
   out_2463199541683517663[100] = 0;
   out_2463199541683517663[101] = 0;
   out_2463199541683517663[102] = 0;
   out_2463199541683517663[103] = 0;
   out_2463199541683517663[104] = 0;
   out_2463199541683517663[105] = 0;
   out_2463199541683517663[106] = 0;
   out_2463199541683517663[107] = 0;
   out_2463199541683517663[108] = 1.0;
   out_2463199541683517663[109] = 0;
   out_2463199541683517663[110] = 0;
   out_2463199541683517663[111] = 0;
   out_2463199541683517663[112] = 0;
   out_2463199541683517663[113] = 0;
   out_2463199541683517663[114] = 0;
   out_2463199541683517663[115] = 0;
   out_2463199541683517663[116] = 0;
   out_2463199541683517663[117] = 0;
   out_2463199541683517663[118] = 0;
   out_2463199541683517663[119] = 0;
   out_2463199541683517663[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_8399668097283148026) {
   out_8399668097283148026[0] = dt*state[3] + state[0];
   out_8399668097283148026[1] = dt*state[4] + state[1];
   out_8399668097283148026[2] = dt*state[5] + state[2];
   out_8399668097283148026[3] = state[3];
   out_8399668097283148026[4] = state[4];
   out_8399668097283148026[5] = state[5];
   out_8399668097283148026[6] = dt*state[7] + state[6];
   out_8399668097283148026[7] = dt*state[8] + state[7];
   out_8399668097283148026[8] = state[8];
   out_8399668097283148026[9] = state[9];
   out_8399668097283148026[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3552796253967856461) {
   out_3552796253967856461[0] = 1;
   out_3552796253967856461[1] = 0;
   out_3552796253967856461[2] = 0;
   out_3552796253967856461[3] = dt;
   out_3552796253967856461[4] = 0;
   out_3552796253967856461[5] = 0;
   out_3552796253967856461[6] = 0;
   out_3552796253967856461[7] = 0;
   out_3552796253967856461[8] = 0;
   out_3552796253967856461[9] = 0;
   out_3552796253967856461[10] = 0;
   out_3552796253967856461[11] = 0;
   out_3552796253967856461[12] = 1;
   out_3552796253967856461[13] = 0;
   out_3552796253967856461[14] = 0;
   out_3552796253967856461[15] = dt;
   out_3552796253967856461[16] = 0;
   out_3552796253967856461[17] = 0;
   out_3552796253967856461[18] = 0;
   out_3552796253967856461[19] = 0;
   out_3552796253967856461[20] = 0;
   out_3552796253967856461[21] = 0;
   out_3552796253967856461[22] = 0;
   out_3552796253967856461[23] = 0;
   out_3552796253967856461[24] = 1;
   out_3552796253967856461[25] = 0;
   out_3552796253967856461[26] = 0;
   out_3552796253967856461[27] = dt;
   out_3552796253967856461[28] = 0;
   out_3552796253967856461[29] = 0;
   out_3552796253967856461[30] = 0;
   out_3552796253967856461[31] = 0;
   out_3552796253967856461[32] = 0;
   out_3552796253967856461[33] = 0;
   out_3552796253967856461[34] = 0;
   out_3552796253967856461[35] = 0;
   out_3552796253967856461[36] = 1;
   out_3552796253967856461[37] = 0;
   out_3552796253967856461[38] = 0;
   out_3552796253967856461[39] = 0;
   out_3552796253967856461[40] = 0;
   out_3552796253967856461[41] = 0;
   out_3552796253967856461[42] = 0;
   out_3552796253967856461[43] = 0;
   out_3552796253967856461[44] = 0;
   out_3552796253967856461[45] = 0;
   out_3552796253967856461[46] = 0;
   out_3552796253967856461[47] = 0;
   out_3552796253967856461[48] = 1;
   out_3552796253967856461[49] = 0;
   out_3552796253967856461[50] = 0;
   out_3552796253967856461[51] = 0;
   out_3552796253967856461[52] = 0;
   out_3552796253967856461[53] = 0;
   out_3552796253967856461[54] = 0;
   out_3552796253967856461[55] = 0;
   out_3552796253967856461[56] = 0;
   out_3552796253967856461[57] = 0;
   out_3552796253967856461[58] = 0;
   out_3552796253967856461[59] = 0;
   out_3552796253967856461[60] = 1;
   out_3552796253967856461[61] = 0;
   out_3552796253967856461[62] = 0;
   out_3552796253967856461[63] = 0;
   out_3552796253967856461[64] = 0;
   out_3552796253967856461[65] = 0;
   out_3552796253967856461[66] = 0;
   out_3552796253967856461[67] = 0;
   out_3552796253967856461[68] = 0;
   out_3552796253967856461[69] = 0;
   out_3552796253967856461[70] = 0;
   out_3552796253967856461[71] = 0;
   out_3552796253967856461[72] = 1;
   out_3552796253967856461[73] = dt;
   out_3552796253967856461[74] = 0;
   out_3552796253967856461[75] = 0;
   out_3552796253967856461[76] = 0;
   out_3552796253967856461[77] = 0;
   out_3552796253967856461[78] = 0;
   out_3552796253967856461[79] = 0;
   out_3552796253967856461[80] = 0;
   out_3552796253967856461[81] = 0;
   out_3552796253967856461[82] = 0;
   out_3552796253967856461[83] = 0;
   out_3552796253967856461[84] = 1;
   out_3552796253967856461[85] = dt;
   out_3552796253967856461[86] = 0;
   out_3552796253967856461[87] = 0;
   out_3552796253967856461[88] = 0;
   out_3552796253967856461[89] = 0;
   out_3552796253967856461[90] = 0;
   out_3552796253967856461[91] = 0;
   out_3552796253967856461[92] = 0;
   out_3552796253967856461[93] = 0;
   out_3552796253967856461[94] = 0;
   out_3552796253967856461[95] = 0;
   out_3552796253967856461[96] = 1;
   out_3552796253967856461[97] = 0;
   out_3552796253967856461[98] = 0;
   out_3552796253967856461[99] = 0;
   out_3552796253967856461[100] = 0;
   out_3552796253967856461[101] = 0;
   out_3552796253967856461[102] = 0;
   out_3552796253967856461[103] = 0;
   out_3552796253967856461[104] = 0;
   out_3552796253967856461[105] = 0;
   out_3552796253967856461[106] = 0;
   out_3552796253967856461[107] = 0;
   out_3552796253967856461[108] = 1;
   out_3552796253967856461[109] = 0;
   out_3552796253967856461[110] = 0;
   out_3552796253967856461[111] = 0;
   out_3552796253967856461[112] = 0;
   out_3552796253967856461[113] = 0;
   out_3552796253967856461[114] = 0;
   out_3552796253967856461[115] = 0;
   out_3552796253967856461[116] = 0;
   out_3552796253967856461[117] = 0;
   out_3552796253967856461[118] = 0;
   out_3552796253967856461[119] = 0;
   out_3552796253967856461[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6298886154741125804) {
   out_6298886154741125804[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2109310268915088104) {
   out_2109310268915088104[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2109310268915088104[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2109310268915088104[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2109310268915088104[3] = 0;
   out_2109310268915088104[4] = 0;
   out_2109310268915088104[5] = 0;
   out_2109310268915088104[6] = 1;
   out_2109310268915088104[7] = 0;
   out_2109310268915088104[8] = 0;
   out_2109310268915088104[9] = 0;
   out_2109310268915088104[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3273347345638054669) {
   out_3273347345638054669[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8076940145235720636) {
   out_8076940145235720636[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8076940145235720636[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8076940145235720636[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8076940145235720636[3] = 0;
   out_8076940145235720636[4] = 0;
   out_8076940145235720636[5] = 0;
   out_8076940145235720636[6] = 1;
   out_8076940145235720636[7] = 0;
   out_8076940145235720636[8] = 0;
   out_8076940145235720636[9] = 1;
   out_8076940145235720636[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8571555988265576038) {
   out_8571555988265576038[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_697782563777508081) {
   out_697782563777508081[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[6] = 0;
   out_697782563777508081[7] = 1;
   out_697782563777508081[8] = 0;
   out_697782563777508081[9] = 0;
   out_697782563777508081[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8571555988265576038) {
   out_8571555988265576038[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_697782563777508081) {
   out_697782563777508081[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_697782563777508081[6] = 0;
   out_697782563777508081[7] = 1;
   out_697782563777508081[8] = 0;
   out_697782563777508081[9] = 0;
   out_697782563777508081[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9018383677196817484) {
  err_fun(nom_x, delta_x, out_9018383677196817484);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7459293352492598460) {
  inv_err_fun(nom_x, true_x, out_7459293352492598460);
}
void gnss_H_mod_fun(double *state, double *out_2463199541683517663) {
  H_mod_fun(state, out_2463199541683517663);
}
void gnss_f_fun(double *state, double dt, double *out_8399668097283148026) {
  f_fun(state,  dt, out_8399668097283148026);
}
void gnss_F_fun(double *state, double dt, double *out_3552796253967856461) {
  F_fun(state,  dt, out_3552796253967856461);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6298886154741125804) {
  h_6(state, sat_pos, out_6298886154741125804);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2109310268915088104) {
  H_6(state, sat_pos, out_2109310268915088104);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3273347345638054669) {
  h_20(state, sat_pos, out_3273347345638054669);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8076940145235720636) {
  H_20(state, sat_pos, out_8076940145235720636);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8571555988265576038) {
  h_7(state, sat_pos_vel, out_8571555988265576038);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_697782563777508081) {
  H_7(state, sat_pos_vel, out_697782563777508081);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8571555988265576038) {
  h_21(state, sat_pos_vel, out_8571555988265576038);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_697782563777508081) {
  H_21(state, sat_pos_vel, out_697782563777508081);
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
