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
void err_fun(double *nom_x, double *delta_x, double *out_4516170446114263232) {
   out_4516170446114263232[0] = delta_x[0] + nom_x[0];
   out_4516170446114263232[1] = delta_x[1] + nom_x[1];
   out_4516170446114263232[2] = delta_x[2] + nom_x[2];
   out_4516170446114263232[3] = delta_x[3] + nom_x[3];
   out_4516170446114263232[4] = delta_x[4] + nom_x[4];
   out_4516170446114263232[5] = delta_x[5] + nom_x[5];
   out_4516170446114263232[6] = delta_x[6] + nom_x[6];
   out_4516170446114263232[7] = delta_x[7] + nom_x[7];
   out_4516170446114263232[8] = delta_x[8] + nom_x[8];
   out_4516170446114263232[9] = delta_x[9] + nom_x[9];
   out_4516170446114263232[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2808882605666685159) {
   out_2808882605666685159[0] = -nom_x[0] + true_x[0];
   out_2808882605666685159[1] = -nom_x[1] + true_x[1];
   out_2808882605666685159[2] = -nom_x[2] + true_x[2];
   out_2808882605666685159[3] = -nom_x[3] + true_x[3];
   out_2808882605666685159[4] = -nom_x[4] + true_x[4];
   out_2808882605666685159[5] = -nom_x[5] + true_x[5];
   out_2808882605666685159[6] = -nom_x[6] + true_x[6];
   out_2808882605666685159[7] = -nom_x[7] + true_x[7];
   out_2808882605666685159[8] = -nom_x[8] + true_x[8];
   out_2808882605666685159[9] = -nom_x[9] + true_x[9];
   out_2808882605666685159[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2215232348373469859) {
   out_2215232348373469859[0] = 1.0;
   out_2215232348373469859[1] = 0;
   out_2215232348373469859[2] = 0;
   out_2215232348373469859[3] = 0;
   out_2215232348373469859[4] = 0;
   out_2215232348373469859[5] = 0;
   out_2215232348373469859[6] = 0;
   out_2215232348373469859[7] = 0;
   out_2215232348373469859[8] = 0;
   out_2215232348373469859[9] = 0;
   out_2215232348373469859[10] = 0;
   out_2215232348373469859[11] = 0;
   out_2215232348373469859[12] = 1.0;
   out_2215232348373469859[13] = 0;
   out_2215232348373469859[14] = 0;
   out_2215232348373469859[15] = 0;
   out_2215232348373469859[16] = 0;
   out_2215232348373469859[17] = 0;
   out_2215232348373469859[18] = 0;
   out_2215232348373469859[19] = 0;
   out_2215232348373469859[20] = 0;
   out_2215232348373469859[21] = 0;
   out_2215232348373469859[22] = 0;
   out_2215232348373469859[23] = 0;
   out_2215232348373469859[24] = 1.0;
   out_2215232348373469859[25] = 0;
   out_2215232348373469859[26] = 0;
   out_2215232348373469859[27] = 0;
   out_2215232348373469859[28] = 0;
   out_2215232348373469859[29] = 0;
   out_2215232348373469859[30] = 0;
   out_2215232348373469859[31] = 0;
   out_2215232348373469859[32] = 0;
   out_2215232348373469859[33] = 0;
   out_2215232348373469859[34] = 0;
   out_2215232348373469859[35] = 0;
   out_2215232348373469859[36] = 1.0;
   out_2215232348373469859[37] = 0;
   out_2215232348373469859[38] = 0;
   out_2215232348373469859[39] = 0;
   out_2215232348373469859[40] = 0;
   out_2215232348373469859[41] = 0;
   out_2215232348373469859[42] = 0;
   out_2215232348373469859[43] = 0;
   out_2215232348373469859[44] = 0;
   out_2215232348373469859[45] = 0;
   out_2215232348373469859[46] = 0;
   out_2215232348373469859[47] = 0;
   out_2215232348373469859[48] = 1.0;
   out_2215232348373469859[49] = 0;
   out_2215232348373469859[50] = 0;
   out_2215232348373469859[51] = 0;
   out_2215232348373469859[52] = 0;
   out_2215232348373469859[53] = 0;
   out_2215232348373469859[54] = 0;
   out_2215232348373469859[55] = 0;
   out_2215232348373469859[56] = 0;
   out_2215232348373469859[57] = 0;
   out_2215232348373469859[58] = 0;
   out_2215232348373469859[59] = 0;
   out_2215232348373469859[60] = 1.0;
   out_2215232348373469859[61] = 0;
   out_2215232348373469859[62] = 0;
   out_2215232348373469859[63] = 0;
   out_2215232348373469859[64] = 0;
   out_2215232348373469859[65] = 0;
   out_2215232348373469859[66] = 0;
   out_2215232348373469859[67] = 0;
   out_2215232348373469859[68] = 0;
   out_2215232348373469859[69] = 0;
   out_2215232348373469859[70] = 0;
   out_2215232348373469859[71] = 0;
   out_2215232348373469859[72] = 1.0;
   out_2215232348373469859[73] = 0;
   out_2215232348373469859[74] = 0;
   out_2215232348373469859[75] = 0;
   out_2215232348373469859[76] = 0;
   out_2215232348373469859[77] = 0;
   out_2215232348373469859[78] = 0;
   out_2215232348373469859[79] = 0;
   out_2215232348373469859[80] = 0;
   out_2215232348373469859[81] = 0;
   out_2215232348373469859[82] = 0;
   out_2215232348373469859[83] = 0;
   out_2215232348373469859[84] = 1.0;
   out_2215232348373469859[85] = 0;
   out_2215232348373469859[86] = 0;
   out_2215232348373469859[87] = 0;
   out_2215232348373469859[88] = 0;
   out_2215232348373469859[89] = 0;
   out_2215232348373469859[90] = 0;
   out_2215232348373469859[91] = 0;
   out_2215232348373469859[92] = 0;
   out_2215232348373469859[93] = 0;
   out_2215232348373469859[94] = 0;
   out_2215232348373469859[95] = 0;
   out_2215232348373469859[96] = 1.0;
   out_2215232348373469859[97] = 0;
   out_2215232348373469859[98] = 0;
   out_2215232348373469859[99] = 0;
   out_2215232348373469859[100] = 0;
   out_2215232348373469859[101] = 0;
   out_2215232348373469859[102] = 0;
   out_2215232348373469859[103] = 0;
   out_2215232348373469859[104] = 0;
   out_2215232348373469859[105] = 0;
   out_2215232348373469859[106] = 0;
   out_2215232348373469859[107] = 0;
   out_2215232348373469859[108] = 1.0;
   out_2215232348373469859[109] = 0;
   out_2215232348373469859[110] = 0;
   out_2215232348373469859[111] = 0;
   out_2215232348373469859[112] = 0;
   out_2215232348373469859[113] = 0;
   out_2215232348373469859[114] = 0;
   out_2215232348373469859[115] = 0;
   out_2215232348373469859[116] = 0;
   out_2215232348373469859[117] = 0;
   out_2215232348373469859[118] = 0;
   out_2215232348373469859[119] = 0;
   out_2215232348373469859[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1776385195625842248) {
   out_1776385195625842248[0] = dt*state[3] + state[0];
   out_1776385195625842248[1] = dt*state[4] + state[1];
   out_1776385195625842248[2] = dt*state[5] + state[2];
   out_1776385195625842248[3] = state[3];
   out_1776385195625842248[4] = state[4];
   out_1776385195625842248[5] = state[5];
   out_1776385195625842248[6] = dt*state[7] + state[6];
   out_1776385195625842248[7] = dt*state[8] + state[7];
   out_1776385195625842248[8] = state[8];
   out_1776385195625842248[9] = state[9];
   out_1776385195625842248[10] = state[10];
}
void F_fun(double *state, double dt, double *out_900808918083314584) {
   out_900808918083314584[0] = 1;
   out_900808918083314584[1] = 0;
   out_900808918083314584[2] = 0;
   out_900808918083314584[3] = dt;
   out_900808918083314584[4] = 0;
   out_900808918083314584[5] = 0;
   out_900808918083314584[6] = 0;
   out_900808918083314584[7] = 0;
   out_900808918083314584[8] = 0;
   out_900808918083314584[9] = 0;
   out_900808918083314584[10] = 0;
   out_900808918083314584[11] = 0;
   out_900808918083314584[12] = 1;
   out_900808918083314584[13] = 0;
   out_900808918083314584[14] = 0;
   out_900808918083314584[15] = dt;
   out_900808918083314584[16] = 0;
   out_900808918083314584[17] = 0;
   out_900808918083314584[18] = 0;
   out_900808918083314584[19] = 0;
   out_900808918083314584[20] = 0;
   out_900808918083314584[21] = 0;
   out_900808918083314584[22] = 0;
   out_900808918083314584[23] = 0;
   out_900808918083314584[24] = 1;
   out_900808918083314584[25] = 0;
   out_900808918083314584[26] = 0;
   out_900808918083314584[27] = dt;
   out_900808918083314584[28] = 0;
   out_900808918083314584[29] = 0;
   out_900808918083314584[30] = 0;
   out_900808918083314584[31] = 0;
   out_900808918083314584[32] = 0;
   out_900808918083314584[33] = 0;
   out_900808918083314584[34] = 0;
   out_900808918083314584[35] = 0;
   out_900808918083314584[36] = 1;
   out_900808918083314584[37] = 0;
   out_900808918083314584[38] = 0;
   out_900808918083314584[39] = 0;
   out_900808918083314584[40] = 0;
   out_900808918083314584[41] = 0;
   out_900808918083314584[42] = 0;
   out_900808918083314584[43] = 0;
   out_900808918083314584[44] = 0;
   out_900808918083314584[45] = 0;
   out_900808918083314584[46] = 0;
   out_900808918083314584[47] = 0;
   out_900808918083314584[48] = 1;
   out_900808918083314584[49] = 0;
   out_900808918083314584[50] = 0;
   out_900808918083314584[51] = 0;
   out_900808918083314584[52] = 0;
   out_900808918083314584[53] = 0;
   out_900808918083314584[54] = 0;
   out_900808918083314584[55] = 0;
   out_900808918083314584[56] = 0;
   out_900808918083314584[57] = 0;
   out_900808918083314584[58] = 0;
   out_900808918083314584[59] = 0;
   out_900808918083314584[60] = 1;
   out_900808918083314584[61] = 0;
   out_900808918083314584[62] = 0;
   out_900808918083314584[63] = 0;
   out_900808918083314584[64] = 0;
   out_900808918083314584[65] = 0;
   out_900808918083314584[66] = 0;
   out_900808918083314584[67] = 0;
   out_900808918083314584[68] = 0;
   out_900808918083314584[69] = 0;
   out_900808918083314584[70] = 0;
   out_900808918083314584[71] = 0;
   out_900808918083314584[72] = 1;
   out_900808918083314584[73] = dt;
   out_900808918083314584[74] = 0;
   out_900808918083314584[75] = 0;
   out_900808918083314584[76] = 0;
   out_900808918083314584[77] = 0;
   out_900808918083314584[78] = 0;
   out_900808918083314584[79] = 0;
   out_900808918083314584[80] = 0;
   out_900808918083314584[81] = 0;
   out_900808918083314584[82] = 0;
   out_900808918083314584[83] = 0;
   out_900808918083314584[84] = 1;
   out_900808918083314584[85] = dt;
   out_900808918083314584[86] = 0;
   out_900808918083314584[87] = 0;
   out_900808918083314584[88] = 0;
   out_900808918083314584[89] = 0;
   out_900808918083314584[90] = 0;
   out_900808918083314584[91] = 0;
   out_900808918083314584[92] = 0;
   out_900808918083314584[93] = 0;
   out_900808918083314584[94] = 0;
   out_900808918083314584[95] = 0;
   out_900808918083314584[96] = 1;
   out_900808918083314584[97] = 0;
   out_900808918083314584[98] = 0;
   out_900808918083314584[99] = 0;
   out_900808918083314584[100] = 0;
   out_900808918083314584[101] = 0;
   out_900808918083314584[102] = 0;
   out_900808918083314584[103] = 0;
   out_900808918083314584[104] = 0;
   out_900808918083314584[105] = 0;
   out_900808918083314584[106] = 0;
   out_900808918083314584[107] = 0;
   out_900808918083314584[108] = 1;
   out_900808918083314584[109] = 0;
   out_900808918083314584[110] = 0;
   out_900808918083314584[111] = 0;
   out_900808918083314584[112] = 0;
   out_900808918083314584[113] = 0;
   out_900808918083314584[114] = 0;
   out_900808918083314584[115] = 0;
   out_900808918083314584[116] = 0;
   out_900808918083314584[117] = 0;
   out_900808918083314584[118] = 0;
   out_900808918083314584[119] = 0;
   out_900808918083314584[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8025372775447727249) {
   out_8025372775447727249[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4754726028637191613) {
   out_4754726028637191613[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4754726028637191613[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4754726028637191613[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4754726028637191613[3] = 0;
   out_4754726028637191613[4] = 0;
   out_4754726028637191613[5] = 0;
   out_4754726028637191613[6] = 1;
   out_4754726028637191613[7] = 0;
   out_4754726028637191613[8] = 0;
   out_4754726028637191613[9] = 0;
   out_4754726028637191613[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7770384175390878934) {
   out_7770384175390878934[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3064333999394004580) {
   out_3064333999394004580[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3064333999394004580[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3064333999394004580[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3064333999394004580[3] = 0;
   out_3064333999394004580[4] = 0;
   out_3064333999394004580[5] = 0;
   out_3064333999394004580[6] = 1;
   out_3064333999394004580[7] = 0;
   out_3064333999394004580[8] = 0;
   out_3064333999394004580[9] = 1;
   out_3064333999394004580[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6740989689516723293) {
   out_6740989689516723293[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3412074168452321369) {
   out_3412074168452321369[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[6] = 0;
   out_3412074168452321369[7] = 1;
   out_3412074168452321369[8] = 0;
   out_3412074168452321369[9] = 0;
   out_3412074168452321369[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6740989689516723293) {
   out_6740989689516723293[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3412074168452321369) {
   out_3412074168452321369[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3412074168452321369[6] = 0;
   out_3412074168452321369[7] = 1;
   out_3412074168452321369[8] = 0;
   out_3412074168452321369[9] = 0;
   out_3412074168452321369[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4516170446114263232) {
  err_fun(nom_x, delta_x, out_4516170446114263232);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2808882605666685159) {
  inv_err_fun(nom_x, true_x, out_2808882605666685159);
}
void gnss_H_mod_fun(double *state, double *out_2215232348373469859) {
  H_mod_fun(state, out_2215232348373469859);
}
void gnss_f_fun(double *state, double dt, double *out_1776385195625842248) {
  f_fun(state,  dt, out_1776385195625842248);
}
void gnss_F_fun(double *state, double dt, double *out_900808918083314584) {
  F_fun(state,  dt, out_900808918083314584);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8025372775447727249) {
  h_6(state, sat_pos, out_8025372775447727249);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4754726028637191613) {
  H_6(state, sat_pos, out_4754726028637191613);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7770384175390878934) {
  h_20(state, sat_pos, out_7770384175390878934);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3064333999394004580) {
  H_20(state, sat_pos, out_3064333999394004580);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6740989689516723293) {
  h_7(state, sat_pos_vel, out_6740989689516723293);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3412074168452321369) {
  H_7(state, sat_pos_vel, out_3412074168452321369);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6740989689516723293) {
  h_21(state, sat_pos_vel, out_6740989689516723293);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3412074168452321369) {
  H_21(state, sat_pos_vel, out_3412074168452321369);
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
