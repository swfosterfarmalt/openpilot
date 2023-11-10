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
void err_fun(double *nom_x, double *delta_x, double *out_4009183954963761830) {
   out_4009183954963761830[0] = delta_x[0] + nom_x[0];
   out_4009183954963761830[1] = delta_x[1] + nom_x[1];
   out_4009183954963761830[2] = delta_x[2] + nom_x[2];
   out_4009183954963761830[3] = delta_x[3] + nom_x[3];
   out_4009183954963761830[4] = delta_x[4] + nom_x[4];
   out_4009183954963761830[5] = delta_x[5] + nom_x[5];
   out_4009183954963761830[6] = delta_x[6] + nom_x[6];
   out_4009183954963761830[7] = delta_x[7] + nom_x[7];
   out_4009183954963761830[8] = delta_x[8] + nom_x[8];
   out_4009183954963761830[9] = delta_x[9] + nom_x[9];
   out_4009183954963761830[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5668670552804156843) {
   out_5668670552804156843[0] = -nom_x[0] + true_x[0];
   out_5668670552804156843[1] = -nom_x[1] + true_x[1];
   out_5668670552804156843[2] = -nom_x[2] + true_x[2];
   out_5668670552804156843[3] = -nom_x[3] + true_x[3];
   out_5668670552804156843[4] = -nom_x[4] + true_x[4];
   out_5668670552804156843[5] = -nom_x[5] + true_x[5];
   out_5668670552804156843[6] = -nom_x[6] + true_x[6];
   out_5668670552804156843[7] = -nom_x[7] + true_x[7];
   out_5668670552804156843[8] = -nom_x[8] + true_x[8];
   out_5668670552804156843[9] = -nom_x[9] + true_x[9];
   out_5668670552804156843[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7793541619644674581) {
   out_7793541619644674581[0] = 1.0;
   out_7793541619644674581[1] = 0;
   out_7793541619644674581[2] = 0;
   out_7793541619644674581[3] = 0;
   out_7793541619644674581[4] = 0;
   out_7793541619644674581[5] = 0;
   out_7793541619644674581[6] = 0;
   out_7793541619644674581[7] = 0;
   out_7793541619644674581[8] = 0;
   out_7793541619644674581[9] = 0;
   out_7793541619644674581[10] = 0;
   out_7793541619644674581[11] = 0;
   out_7793541619644674581[12] = 1.0;
   out_7793541619644674581[13] = 0;
   out_7793541619644674581[14] = 0;
   out_7793541619644674581[15] = 0;
   out_7793541619644674581[16] = 0;
   out_7793541619644674581[17] = 0;
   out_7793541619644674581[18] = 0;
   out_7793541619644674581[19] = 0;
   out_7793541619644674581[20] = 0;
   out_7793541619644674581[21] = 0;
   out_7793541619644674581[22] = 0;
   out_7793541619644674581[23] = 0;
   out_7793541619644674581[24] = 1.0;
   out_7793541619644674581[25] = 0;
   out_7793541619644674581[26] = 0;
   out_7793541619644674581[27] = 0;
   out_7793541619644674581[28] = 0;
   out_7793541619644674581[29] = 0;
   out_7793541619644674581[30] = 0;
   out_7793541619644674581[31] = 0;
   out_7793541619644674581[32] = 0;
   out_7793541619644674581[33] = 0;
   out_7793541619644674581[34] = 0;
   out_7793541619644674581[35] = 0;
   out_7793541619644674581[36] = 1.0;
   out_7793541619644674581[37] = 0;
   out_7793541619644674581[38] = 0;
   out_7793541619644674581[39] = 0;
   out_7793541619644674581[40] = 0;
   out_7793541619644674581[41] = 0;
   out_7793541619644674581[42] = 0;
   out_7793541619644674581[43] = 0;
   out_7793541619644674581[44] = 0;
   out_7793541619644674581[45] = 0;
   out_7793541619644674581[46] = 0;
   out_7793541619644674581[47] = 0;
   out_7793541619644674581[48] = 1.0;
   out_7793541619644674581[49] = 0;
   out_7793541619644674581[50] = 0;
   out_7793541619644674581[51] = 0;
   out_7793541619644674581[52] = 0;
   out_7793541619644674581[53] = 0;
   out_7793541619644674581[54] = 0;
   out_7793541619644674581[55] = 0;
   out_7793541619644674581[56] = 0;
   out_7793541619644674581[57] = 0;
   out_7793541619644674581[58] = 0;
   out_7793541619644674581[59] = 0;
   out_7793541619644674581[60] = 1.0;
   out_7793541619644674581[61] = 0;
   out_7793541619644674581[62] = 0;
   out_7793541619644674581[63] = 0;
   out_7793541619644674581[64] = 0;
   out_7793541619644674581[65] = 0;
   out_7793541619644674581[66] = 0;
   out_7793541619644674581[67] = 0;
   out_7793541619644674581[68] = 0;
   out_7793541619644674581[69] = 0;
   out_7793541619644674581[70] = 0;
   out_7793541619644674581[71] = 0;
   out_7793541619644674581[72] = 1.0;
   out_7793541619644674581[73] = 0;
   out_7793541619644674581[74] = 0;
   out_7793541619644674581[75] = 0;
   out_7793541619644674581[76] = 0;
   out_7793541619644674581[77] = 0;
   out_7793541619644674581[78] = 0;
   out_7793541619644674581[79] = 0;
   out_7793541619644674581[80] = 0;
   out_7793541619644674581[81] = 0;
   out_7793541619644674581[82] = 0;
   out_7793541619644674581[83] = 0;
   out_7793541619644674581[84] = 1.0;
   out_7793541619644674581[85] = 0;
   out_7793541619644674581[86] = 0;
   out_7793541619644674581[87] = 0;
   out_7793541619644674581[88] = 0;
   out_7793541619644674581[89] = 0;
   out_7793541619644674581[90] = 0;
   out_7793541619644674581[91] = 0;
   out_7793541619644674581[92] = 0;
   out_7793541619644674581[93] = 0;
   out_7793541619644674581[94] = 0;
   out_7793541619644674581[95] = 0;
   out_7793541619644674581[96] = 1.0;
   out_7793541619644674581[97] = 0;
   out_7793541619644674581[98] = 0;
   out_7793541619644674581[99] = 0;
   out_7793541619644674581[100] = 0;
   out_7793541619644674581[101] = 0;
   out_7793541619644674581[102] = 0;
   out_7793541619644674581[103] = 0;
   out_7793541619644674581[104] = 0;
   out_7793541619644674581[105] = 0;
   out_7793541619644674581[106] = 0;
   out_7793541619644674581[107] = 0;
   out_7793541619644674581[108] = 1.0;
   out_7793541619644674581[109] = 0;
   out_7793541619644674581[110] = 0;
   out_7793541619644674581[111] = 0;
   out_7793541619644674581[112] = 0;
   out_7793541619644674581[113] = 0;
   out_7793541619644674581[114] = 0;
   out_7793541619644674581[115] = 0;
   out_7793541619644674581[116] = 0;
   out_7793541619644674581[117] = 0;
   out_7793541619644674581[118] = 0;
   out_7793541619644674581[119] = 0;
   out_7793541619644674581[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2953486288822651743) {
   out_2953486288822651743[0] = dt*state[3] + state[0];
   out_2953486288822651743[1] = dt*state[4] + state[1];
   out_2953486288822651743[2] = dt*state[5] + state[2];
   out_2953486288822651743[3] = state[3];
   out_2953486288822651743[4] = state[4];
   out_2953486288822651743[5] = state[5];
   out_2953486288822651743[6] = dt*state[7] + state[6];
   out_2953486288822651743[7] = dt*state[8] + state[7];
   out_2953486288822651743[8] = state[8];
   out_2953486288822651743[9] = state[9];
   out_2953486288822651743[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7502400574187806738) {
   out_7502400574187806738[0] = 1;
   out_7502400574187806738[1] = 0;
   out_7502400574187806738[2] = 0;
   out_7502400574187806738[3] = dt;
   out_7502400574187806738[4] = 0;
   out_7502400574187806738[5] = 0;
   out_7502400574187806738[6] = 0;
   out_7502400574187806738[7] = 0;
   out_7502400574187806738[8] = 0;
   out_7502400574187806738[9] = 0;
   out_7502400574187806738[10] = 0;
   out_7502400574187806738[11] = 0;
   out_7502400574187806738[12] = 1;
   out_7502400574187806738[13] = 0;
   out_7502400574187806738[14] = 0;
   out_7502400574187806738[15] = dt;
   out_7502400574187806738[16] = 0;
   out_7502400574187806738[17] = 0;
   out_7502400574187806738[18] = 0;
   out_7502400574187806738[19] = 0;
   out_7502400574187806738[20] = 0;
   out_7502400574187806738[21] = 0;
   out_7502400574187806738[22] = 0;
   out_7502400574187806738[23] = 0;
   out_7502400574187806738[24] = 1;
   out_7502400574187806738[25] = 0;
   out_7502400574187806738[26] = 0;
   out_7502400574187806738[27] = dt;
   out_7502400574187806738[28] = 0;
   out_7502400574187806738[29] = 0;
   out_7502400574187806738[30] = 0;
   out_7502400574187806738[31] = 0;
   out_7502400574187806738[32] = 0;
   out_7502400574187806738[33] = 0;
   out_7502400574187806738[34] = 0;
   out_7502400574187806738[35] = 0;
   out_7502400574187806738[36] = 1;
   out_7502400574187806738[37] = 0;
   out_7502400574187806738[38] = 0;
   out_7502400574187806738[39] = 0;
   out_7502400574187806738[40] = 0;
   out_7502400574187806738[41] = 0;
   out_7502400574187806738[42] = 0;
   out_7502400574187806738[43] = 0;
   out_7502400574187806738[44] = 0;
   out_7502400574187806738[45] = 0;
   out_7502400574187806738[46] = 0;
   out_7502400574187806738[47] = 0;
   out_7502400574187806738[48] = 1;
   out_7502400574187806738[49] = 0;
   out_7502400574187806738[50] = 0;
   out_7502400574187806738[51] = 0;
   out_7502400574187806738[52] = 0;
   out_7502400574187806738[53] = 0;
   out_7502400574187806738[54] = 0;
   out_7502400574187806738[55] = 0;
   out_7502400574187806738[56] = 0;
   out_7502400574187806738[57] = 0;
   out_7502400574187806738[58] = 0;
   out_7502400574187806738[59] = 0;
   out_7502400574187806738[60] = 1;
   out_7502400574187806738[61] = 0;
   out_7502400574187806738[62] = 0;
   out_7502400574187806738[63] = 0;
   out_7502400574187806738[64] = 0;
   out_7502400574187806738[65] = 0;
   out_7502400574187806738[66] = 0;
   out_7502400574187806738[67] = 0;
   out_7502400574187806738[68] = 0;
   out_7502400574187806738[69] = 0;
   out_7502400574187806738[70] = 0;
   out_7502400574187806738[71] = 0;
   out_7502400574187806738[72] = 1;
   out_7502400574187806738[73] = dt;
   out_7502400574187806738[74] = 0;
   out_7502400574187806738[75] = 0;
   out_7502400574187806738[76] = 0;
   out_7502400574187806738[77] = 0;
   out_7502400574187806738[78] = 0;
   out_7502400574187806738[79] = 0;
   out_7502400574187806738[80] = 0;
   out_7502400574187806738[81] = 0;
   out_7502400574187806738[82] = 0;
   out_7502400574187806738[83] = 0;
   out_7502400574187806738[84] = 1;
   out_7502400574187806738[85] = dt;
   out_7502400574187806738[86] = 0;
   out_7502400574187806738[87] = 0;
   out_7502400574187806738[88] = 0;
   out_7502400574187806738[89] = 0;
   out_7502400574187806738[90] = 0;
   out_7502400574187806738[91] = 0;
   out_7502400574187806738[92] = 0;
   out_7502400574187806738[93] = 0;
   out_7502400574187806738[94] = 0;
   out_7502400574187806738[95] = 0;
   out_7502400574187806738[96] = 1;
   out_7502400574187806738[97] = 0;
   out_7502400574187806738[98] = 0;
   out_7502400574187806738[99] = 0;
   out_7502400574187806738[100] = 0;
   out_7502400574187806738[101] = 0;
   out_7502400574187806738[102] = 0;
   out_7502400574187806738[103] = 0;
   out_7502400574187806738[104] = 0;
   out_7502400574187806738[105] = 0;
   out_7502400574187806738[106] = 0;
   out_7502400574187806738[107] = 0;
   out_7502400574187806738[108] = 1;
   out_7502400574187806738[109] = 0;
   out_7502400574187806738[110] = 0;
   out_7502400574187806738[111] = 0;
   out_7502400574187806738[112] = 0;
   out_7502400574187806738[113] = 0;
   out_7502400574187806738[114] = 0;
   out_7502400574187806738[115] = 0;
   out_7502400574187806738[116] = 0;
   out_7502400574187806738[117] = 0;
   out_7502400574187806738[118] = 0;
   out_7502400574187806738[119] = 0;
   out_7502400574187806738[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5515239915342403667) {
   out_5515239915342403667[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_691111391283485275) {
   out_691111391283485275[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_691111391283485275[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_691111391283485275[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_691111391283485275[3] = 0;
   out_691111391283485275[4] = 0;
   out_691111391283485275[5] = 0;
   out_691111391283485275[6] = 1;
   out_691111391283485275[7] = 0;
   out_691111391283485275[8] = 0;
   out_691111391283485275[9] = 0;
   out_691111391283485275[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6421220721641647458) {
   out_6421220721641647458[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4436024380608800520) {
   out_4436024380608800520[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4436024380608800520[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4436024380608800520[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4436024380608800520[3] = 0;
   out_4436024380608800520[4] = 0;
   out_4436024380608800520[5] = 0;
   out_4436024380608800520[6] = 1;
   out_4436024380608800520[7] = 0;
   out_4436024380608800520[8] = 0;
   out_4436024380608800520[9] = 1;
   out_4436024380608800520[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8868946558143862461) {
   out_8868946558143862461[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8408613646792434496) {
   out_8408613646792434496[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[6] = 0;
   out_8408613646792434496[7] = 1;
   out_8408613646792434496[8] = 0;
   out_8408613646792434496[9] = 0;
   out_8408613646792434496[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8868946558143862461) {
   out_8868946558143862461[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8408613646792434496) {
   out_8408613646792434496[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8408613646792434496[6] = 0;
   out_8408613646792434496[7] = 1;
   out_8408613646792434496[8] = 0;
   out_8408613646792434496[9] = 0;
   out_8408613646792434496[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4009183954963761830) {
  err_fun(nom_x, delta_x, out_4009183954963761830);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5668670552804156843) {
  inv_err_fun(nom_x, true_x, out_5668670552804156843);
}
void gnss_H_mod_fun(double *state, double *out_7793541619644674581) {
  H_mod_fun(state, out_7793541619644674581);
}
void gnss_f_fun(double *state, double dt, double *out_2953486288822651743) {
  f_fun(state,  dt, out_2953486288822651743);
}
void gnss_F_fun(double *state, double dt, double *out_7502400574187806738) {
  F_fun(state,  dt, out_7502400574187806738);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5515239915342403667) {
  h_6(state, sat_pos, out_5515239915342403667);
}
void gnss_H_6(double *state, double *sat_pos, double *out_691111391283485275) {
  H_6(state, sat_pos, out_691111391283485275);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6421220721641647458) {
  h_20(state, sat_pos, out_6421220721641647458);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4436024380608800520) {
  H_20(state, sat_pos, out_4436024380608800520);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8868946558143862461) {
  h_7(state, sat_pos_vel, out_8868946558143862461);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8408613646792434496) {
  H_7(state, sat_pos_vel, out_8408613646792434496);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8868946558143862461) {
  h_21(state, sat_pos_vel, out_8868946558143862461);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8408613646792434496) {
  H_21(state, sat_pos_vel, out_8408613646792434496);
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
