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
void err_fun(double *nom_x, double *delta_x, double *out_6450837299122467107) {
   out_6450837299122467107[0] = delta_x[0] + nom_x[0];
   out_6450837299122467107[1] = delta_x[1] + nom_x[1];
   out_6450837299122467107[2] = delta_x[2] + nom_x[2];
   out_6450837299122467107[3] = delta_x[3] + nom_x[3];
   out_6450837299122467107[4] = delta_x[4] + nom_x[4];
   out_6450837299122467107[5] = delta_x[5] + nom_x[5];
   out_6450837299122467107[6] = delta_x[6] + nom_x[6];
   out_6450837299122467107[7] = delta_x[7] + nom_x[7];
   out_6450837299122467107[8] = delta_x[8] + nom_x[8];
   out_6450837299122467107[9] = delta_x[9] + nom_x[9];
   out_6450837299122467107[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1281683845617115365) {
   out_1281683845617115365[0] = -nom_x[0] + true_x[0];
   out_1281683845617115365[1] = -nom_x[1] + true_x[1];
   out_1281683845617115365[2] = -nom_x[2] + true_x[2];
   out_1281683845617115365[3] = -nom_x[3] + true_x[3];
   out_1281683845617115365[4] = -nom_x[4] + true_x[4];
   out_1281683845617115365[5] = -nom_x[5] + true_x[5];
   out_1281683845617115365[6] = -nom_x[6] + true_x[6];
   out_1281683845617115365[7] = -nom_x[7] + true_x[7];
   out_1281683845617115365[8] = -nom_x[8] + true_x[8];
   out_1281683845617115365[9] = -nom_x[9] + true_x[9];
   out_1281683845617115365[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1101293044550311056) {
   out_1101293044550311056[0] = 1.0;
   out_1101293044550311056[1] = 0;
   out_1101293044550311056[2] = 0;
   out_1101293044550311056[3] = 0;
   out_1101293044550311056[4] = 0;
   out_1101293044550311056[5] = 0;
   out_1101293044550311056[6] = 0;
   out_1101293044550311056[7] = 0;
   out_1101293044550311056[8] = 0;
   out_1101293044550311056[9] = 0;
   out_1101293044550311056[10] = 0;
   out_1101293044550311056[11] = 0;
   out_1101293044550311056[12] = 1.0;
   out_1101293044550311056[13] = 0;
   out_1101293044550311056[14] = 0;
   out_1101293044550311056[15] = 0;
   out_1101293044550311056[16] = 0;
   out_1101293044550311056[17] = 0;
   out_1101293044550311056[18] = 0;
   out_1101293044550311056[19] = 0;
   out_1101293044550311056[20] = 0;
   out_1101293044550311056[21] = 0;
   out_1101293044550311056[22] = 0;
   out_1101293044550311056[23] = 0;
   out_1101293044550311056[24] = 1.0;
   out_1101293044550311056[25] = 0;
   out_1101293044550311056[26] = 0;
   out_1101293044550311056[27] = 0;
   out_1101293044550311056[28] = 0;
   out_1101293044550311056[29] = 0;
   out_1101293044550311056[30] = 0;
   out_1101293044550311056[31] = 0;
   out_1101293044550311056[32] = 0;
   out_1101293044550311056[33] = 0;
   out_1101293044550311056[34] = 0;
   out_1101293044550311056[35] = 0;
   out_1101293044550311056[36] = 1.0;
   out_1101293044550311056[37] = 0;
   out_1101293044550311056[38] = 0;
   out_1101293044550311056[39] = 0;
   out_1101293044550311056[40] = 0;
   out_1101293044550311056[41] = 0;
   out_1101293044550311056[42] = 0;
   out_1101293044550311056[43] = 0;
   out_1101293044550311056[44] = 0;
   out_1101293044550311056[45] = 0;
   out_1101293044550311056[46] = 0;
   out_1101293044550311056[47] = 0;
   out_1101293044550311056[48] = 1.0;
   out_1101293044550311056[49] = 0;
   out_1101293044550311056[50] = 0;
   out_1101293044550311056[51] = 0;
   out_1101293044550311056[52] = 0;
   out_1101293044550311056[53] = 0;
   out_1101293044550311056[54] = 0;
   out_1101293044550311056[55] = 0;
   out_1101293044550311056[56] = 0;
   out_1101293044550311056[57] = 0;
   out_1101293044550311056[58] = 0;
   out_1101293044550311056[59] = 0;
   out_1101293044550311056[60] = 1.0;
   out_1101293044550311056[61] = 0;
   out_1101293044550311056[62] = 0;
   out_1101293044550311056[63] = 0;
   out_1101293044550311056[64] = 0;
   out_1101293044550311056[65] = 0;
   out_1101293044550311056[66] = 0;
   out_1101293044550311056[67] = 0;
   out_1101293044550311056[68] = 0;
   out_1101293044550311056[69] = 0;
   out_1101293044550311056[70] = 0;
   out_1101293044550311056[71] = 0;
   out_1101293044550311056[72] = 1.0;
   out_1101293044550311056[73] = 0;
   out_1101293044550311056[74] = 0;
   out_1101293044550311056[75] = 0;
   out_1101293044550311056[76] = 0;
   out_1101293044550311056[77] = 0;
   out_1101293044550311056[78] = 0;
   out_1101293044550311056[79] = 0;
   out_1101293044550311056[80] = 0;
   out_1101293044550311056[81] = 0;
   out_1101293044550311056[82] = 0;
   out_1101293044550311056[83] = 0;
   out_1101293044550311056[84] = 1.0;
   out_1101293044550311056[85] = 0;
   out_1101293044550311056[86] = 0;
   out_1101293044550311056[87] = 0;
   out_1101293044550311056[88] = 0;
   out_1101293044550311056[89] = 0;
   out_1101293044550311056[90] = 0;
   out_1101293044550311056[91] = 0;
   out_1101293044550311056[92] = 0;
   out_1101293044550311056[93] = 0;
   out_1101293044550311056[94] = 0;
   out_1101293044550311056[95] = 0;
   out_1101293044550311056[96] = 1.0;
   out_1101293044550311056[97] = 0;
   out_1101293044550311056[98] = 0;
   out_1101293044550311056[99] = 0;
   out_1101293044550311056[100] = 0;
   out_1101293044550311056[101] = 0;
   out_1101293044550311056[102] = 0;
   out_1101293044550311056[103] = 0;
   out_1101293044550311056[104] = 0;
   out_1101293044550311056[105] = 0;
   out_1101293044550311056[106] = 0;
   out_1101293044550311056[107] = 0;
   out_1101293044550311056[108] = 1.0;
   out_1101293044550311056[109] = 0;
   out_1101293044550311056[110] = 0;
   out_1101293044550311056[111] = 0;
   out_1101293044550311056[112] = 0;
   out_1101293044550311056[113] = 0;
   out_1101293044550311056[114] = 0;
   out_1101293044550311056[115] = 0;
   out_1101293044550311056[116] = 0;
   out_1101293044550311056[117] = 0;
   out_1101293044550311056[118] = 0;
   out_1101293044550311056[119] = 0;
   out_1101293044550311056[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_944238033724073937) {
   out_944238033724073937[0] = dt*state[3] + state[0];
   out_944238033724073937[1] = dt*state[4] + state[1];
   out_944238033724073937[2] = dt*state[5] + state[2];
   out_944238033724073937[3] = state[3];
   out_944238033724073937[4] = state[4];
   out_944238033724073937[5] = state[5];
   out_944238033724073937[6] = dt*state[7] + state[6];
   out_944238033724073937[7] = dt*state[8] + state[7];
   out_944238033724073937[8] = state[8];
   out_944238033724073937[9] = state[9];
   out_944238033724073937[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6526830519377011082) {
   out_6526830519377011082[0] = 1;
   out_6526830519377011082[1] = 0;
   out_6526830519377011082[2] = 0;
   out_6526830519377011082[3] = dt;
   out_6526830519377011082[4] = 0;
   out_6526830519377011082[5] = 0;
   out_6526830519377011082[6] = 0;
   out_6526830519377011082[7] = 0;
   out_6526830519377011082[8] = 0;
   out_6526830519377011082[9] = 0;
   out_6526830519377011082[10] = 0;
   out_6526830519377011082[11] = 0;
   out_6526830519377011082[12] = 1;
   out_6526830519377011082[13] = 0;
   out_6526830519377011082[14] = 0;
   out_6526830519377011082[15] = dt;
   out_6526830519377011082[16] = 0;
   out_6526830519377011082[17] = 0;
   out_6526830519377011082[18] = 0;
   out_6526830519377011082[19] = 0;
   out_6526830519377011082[20] = 0;
   out_6526830519377011082[21] = 0;
   out_6526830519377011082[22] = 0;
   out_6526830519377011082[23] = 0;
   out_6526830519377011082[24] = 1;
   out_6526830519377011082[25] = 0;
   out_6526830519377011082[26] = 0;
   out_6526830519377011082[27] = dt;
   out_6526830519377011082[28] = 0;
   out_6526830519377011082[29] = 0;
   out_6526830519377011082[30] = 0;
   out_6526830519377011082[31] = 0;
   out_6526830519377011082[32] = 0;
   out_6526830519377011082[33] = 0;
   out_6526830519377011082[34] = 0;
   out_6526830519377011082[35] = 0;
   out_6526830519377011082[36] = 1;
   out_6526830519377011082[37] = 0;
   out_6526830519377011082[38] = 0;
   out_6526830519377011082[39] = 0;
   out_6526830519377011082[40] = 0;
   out_6526830519377011082[41] = 0;
   out_6526830519377011082[42] = 0;
   out_6526830519377011082[43] = 0;
   out_6526830519377011082[44] = 0;
   out_6526830519377011082[45] = 0;
   out_6526830519377011082[46] = 0;
   out_6526830519377011082[47] = 0;
   out_6526830519377011082[48] = 1;
   out_6526830519377011082[49] = 0;
   out_6526830519377011082[50] = 0;
   out_6526830519377011082[51] = 0;
   out_6526830519377011082[52] = 0;
   out_6526830519377011082[53] = 0;
   out_6526830519377011082[54] = 0;
   out_6526830519377011082[55] = 0;
   out_6526830519377011082[56] = 0;
   out_6526830519377011082[57] = 0;
   out_6526830519377011082[58] = 0;
   out_6526830519377011082[59] = 0;
   out_6526830519377011082[60] = 1;
   out_6526830519377011082[61] = 0;
   out_6526830519377011082[62] = 0;
   out_6526830519377011082[63] = 0;
   out_6526830519377011082[64] = 0;
   out_6526830519377011082[65] = 0;
   out_6526830519377011082[66] = 0;
   out_6526830519377011082[67] = 0;
   out_6526830519377011082[68] = 0;
   out_6526830519377011082[69] = 0;
   out_6526830519377011082[70] = 0;
   out_6526830519377011082[71] = 0;
   out_6526830519377011082[72] = 1;
   out_6526830519377011082[73] = dt;
   out_6526830519377011082[74] = 0;
   out_6526830519377011082[75] = 0;
   out_6526830519377011082[76] = 0;
   out_6526830519377011082[77] = 0;
   out_6526830519377011082[78] = 0;
   out_6526830519377011082[79] = 0;
   out_6526830519377011082[80] = 0;
   out_6526830519377011082[81] = 0;
   out_6526830519377011082[82] = 0;
   out_6526830519377011082[83] = 0;
   out_6526830519377011082[84] = 1;
   out_6526830519377011082[85] = dt;
   out_6526830519377011082[86] = 0;
   out_6526830519377011082[87] = 0;
   out_6526830519377011082[88] = 0;
   out_6526830519377011082[89] = 0;
   out_6526830519377011082[90] = 0;
   out_6526830519377011082[91] = 0;
   out_6526830519377011082[92] = 0;
   out_6526830519377011082[93] = 0;
   out_6526830519377011082[94] = 0;
   out_6526830519377011082[95] = 0;
   out_6526830519377011082[96] = 1;
   out_6526830519377011082[97] = 0;
   out_6526830519377011082[98] = 0;
   out_6526830519377011082[99] = 0;
   out_6526830519377011082[100] = 0;
   out_6526830519377011082[101] = 0;
   out_6526830519377011082[102] = 0;
   out_6526830519377011082[103] = 0;
   out_6526830519377011082[104] = 0;
   out_6526830519377011082[105] = 0;
   out_6526830519377011082[106] = 0;
   out_6526830519377011082[107] = 0;
   out_6526830519377011082[108] = 1;
   out_6526830519377011082[109] = 0;
   out_6526830519377011082[110] = 0;
   out_6526830519377011082[111] = 0;
   out_6526830519377011082[112] = 0;
   out_6526830519377011082[113] = 0;
   out_6526830519377011082[114] = 0;
   out_6526830519377011082[115] = 0;
   out_6526830519377011082[116] = 0;
   out_6526830519377011082[117] = 0;
   out_6526830519377011082[118] = 0;
   out_6526830519377011082[119] = 0;
   out_6526830519377011082[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2527351935937827091) {
   out_2527351935937827091[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7158454718814365747) {
   out_7158454718814365747[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7158454718814365747[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7158454718814365747[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7158454718814365747[3] = 0;
   out_7158454718814365747[4] = 0;
   out_7158454718814365747[5] = 0;
   out_7158454718814365747[6] = 1;
   out_7158454718814365747[7] = 0;
   out_7158454718814365747[8] = 0;
   out_7158454718814365747[9] = 0;
   out_7158454718814365747[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2101410374362704668) {
   out_2101410374362704668[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4847885205478089006) {
   out_4847885205478089006[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4847885205478089006[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4847885205478089006[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4847885205478089006[3] = 0;
   out_4847885205478089006[4] = 0;
   out_4847885205478089006[5] = 0;
   out_4847885205478089006[6] = 1;
   out_4847885205478089006[7] = 0;
   out_4847885205478089006[8] = 0;
   out_4847885205478089006[9] = 1;
   out_4847885205478089006[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_386792629721769053) {
   out_386792629721769053[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4543080048435198643) {
   out_4543080048435198643[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[6] = 0;
   out_4543080048435198643[7] = 1;
   out_4543080048435198643[8] = 0;
   out_4543080048435198643[9] = 0;
   out_4543080048435198643[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_386792629721769053) {
   out_386792629721769053[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4543080048435198643) {
   out_4543080048435198643[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4543080048435198643[6] = 0;
   out_4543080048435198643[7] = 1;
   out_4543080048435198643[8] = 0;
   out_4543080048435198643[9] = 0;
   out_4543080048435198643[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6450837299122467107) {
  err_fun(nom_x, delta_x, out_6450837299122467107);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1281683845617115365) {
  inv_err_fun(nom_x, true_x, out_1281683845617115365);
}
void gnss_H_mod_fun(double *state, double *out_1101293044550311056) {
  H_mod_fun(state, out_1101293044550311056);
}
void gnss_f_fun(double *state, double dt, double *out_944238033724073937) {
  f_fun(state,  dt, out_944238033724073937);
}
void gnss_F_fun(double *state, double dt, double *out_6526830519377011082) {
  F_fun(state,  dt, out_6526830519377011082);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2527351935937827091) {
  h_6(state, sat_pos, out_2527351935937827091);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7158454718814365747) {
  H_6(state, sat_pos, out_7158454718814365747);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2101410374362704668) {
  h_20(state, sat_pos, out_2101410374362704668);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4847885205478089006) {
  H_20(state, sat_pos, out_4847885205478089006);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_386792629721769053) {
  h_7(state, sat_pos_vel, out_386792629721769053);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4543080048435198643) {
  H_7(state, sat_pos_vel, out_4543080048435198643);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_386792629721769053) {
  h_21(state, sat_pos_vel, out_386792629721769053);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4543080048435198643) {
  H_21(state, sat_pos_vel, out_4543080048435198643);
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
