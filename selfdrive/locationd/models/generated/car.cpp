#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3502673967241664248) {
   out_3502673967241664248[0] = delta_x[0] + nom_x[0];
   out_3502673967241664248[1] = delta_x[1] + nom_x[1];
   out_3502673967241664248[2] = delta_x[2] + nom_x[2];
   out_3502673967241664248[3] = delta_x[3] + nom_x[3];
   out_3502673967241664248[4] = delta_x[4] + nom_x[4];
   out_3502673967241664248[5] = delta_x[5] + nom_x[5];
   out_3502673967241664248[6] = delta_x[6] + nom_x[6];
   out_3502673967241664248[7] = delta_x[7] + nom_x[7];
   out_3502673967241664248[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5131477847977226664) {
   out_5131477847977226664[0] = -nom_x[0] + true_x[0];
   out_5131477847977226664[1] = -nom_x[1] + true_x[1];
   out_5131477847977226664[2] = -nom_x[2] + true_x[2];
   out_5131477847977226664[3] = -nom_x[3] + true_x[3];
   out_5131477847977226664[4] = -nom_x[4] + true_x[4];
   out_5131477847977226664[5] = -nom_x[5] + true_x[5];
   out_5131477847977226664[6] = -nom_x[6] + true_x[6];
   out_5131477847977226664[7] = -nom_x[7] + true_x[7];
   out_5131477847977226664[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2530919945440300104) {
   out_2530919945440300104[0] = 1.0;
   out_2530919945440300104[1] = 0;
   out_2530919945440300104[2] = 0;
   out_2530919945440300104[3] = 0;
   out_2530919945440300104[4] = 0;
   out_2530919945440300104[5] = 0;
   out_2530919945440300104[6] = 0;
   out_2530919945440300104[7] = 0;
   out_2530919945440300104[8] = 0;
   out_2530919945440300104[9] = 0;
   out_2530919945440300104[10] = 1.0;
   out_2530919945440300104[11] = 0;
   out_2530919945440300104[12] = 0;
   out_2530919945440300104[13] = 0;
   out_2530919945440300104[14] = 0;
   out_2530919945440300104[15] = 0;
   out_2530919945440300104[16] = 0;
   out_2530919945440300104[17] = 0;
   out_2530919945440300104[18] = 0;
   out_2530919945440300104[19] = 0;
   out_2530919945440300104[20] = 1.0;
   out_2530919945440300104[21] = 0;
   out_2530919945440300104[22] = 0;
   out_2530919945440300104[23] = 0;
   out_2530919945440300104[24] = 0;
   out_2530919945440300104[25] = 0;
   out_2530919945440300104[26] = 0;
   out_2530919945440300104[27] = 0;
   out_2530919945440300104[28] = 0;
   out_2530919945440300104[29] = 0;
   out_2530919945440300104[30] = 1.0;
   out_2530919945440300104[31] = 0;
   out_2530919945440300104[32] = 0;
   out_2530919945440300104[33] = 0;
   out_2530919945440300104[34] = 0;
   out_2530919945440300104[35] = 0;
   out_2530919945440300104[36] = 0;
   out_2530919945440300104[37] = 0;
   out_2530919945440300104[38] = 0;
   out_2530919945440300104[39] = 0;
   out_2530919945440300104[40] = 1.0;
   out_2530919945440300104[41] = 0;
   out_2530919945440300104[42] = 0;
   out_2530919945440300104[43] = 0;
   out_2530919945440300104[44] = 0;
   out_2530919945440300104[45] = 0;
   out_2530919945440300104[46] = 0;
   out_2530919945440300104[47] = 0;
   out_2530919945440300104[48] = 0;
   out_2530919945440300104[49] = 0;
   out_2530919945440300104[50] = 1.0;
   out_2530919945440300104[51] = 0;
   out_2530919945440300104[52] = 0;
   out_2530919945440300104[53] = 0;
   out_2530919945440300104[54] = 0;
   out_2530919945440300104[55] = 0;
   out_2530919945440300104[56] = 0;
   out_2530919945440300104[57] = 0;
   out_2530919945440300104[58] = 0;
   out_2530919945440300104[59] = 0;
   out_2530919945440300104[60] = 1.0;
   out_2530919945440300104[61] = 0;
   out_2530919945440300104[62] = 0;
   out_2530919945440300104[63] = 0;
   out_2530919945440300104[64] = 0;
   out_2530919945440300104[65] = 0;
   out_2530919945440300104[66] = 0;
   out_2530919945440300104[67] = 0;
   out_2530919945440300104[68] = 0;
   out_2530919945440300104[69] = 0;
   out_2530919945440300104[70] = 1.0;
   out_2530919945440300104[71] = 0;
   out_2530919945440300104[72] = 0;
   out_2530919945440300104[73] = 0;
   out_2530919945440300104[74] = 0;
   out_2530919945440300104[75] = 0;
   out_2530919945440300104[76] = 0;
   out_2530919945440300104[77] = 0;
   out_2530919945440300104[78] = 0;
   out_2530919945440300104[79] = 0;
   out_2530919945440300104[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7205150727819091354) {
   out_7205150727819091354[0] = state[0];
   out_7205150727819091354[1] = state[1];
   out_7205150727819091354[2] = state[2];
   out_7205150727819091354[3] = state[3];
   out_7205150727819091354[4] = state[4];
   out_7205150727819091354[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7205150727819091354[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7205150727819091354[7] = state[7];
   out_7205150727819091354[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7417487238667392724) {
   out_7417487238667392724[0] = 1;
   out_7417487238667392724[1] = 0;
   out_7417487238667392724[2] = 0;
   out_7417487238667392724[3] = 0;
   out_7417487238667392724[4] = 0;
   out_7417487238667392724[5] = 0;
   out_7417487238667392724[6] = 0;
   out_7417487238667392724[7] = 0;
   out_7417487238667392724[8] = 0;
   out_7417487238667392724[9] = 0;
   out_7417487238667392724[10] = 1;
   out_7417487238667392724[11] = 0;
   out_7417487238667392724[12] = 0;
   out_7417487238667392724[13] = 0;
   out_7417487238667392724[14] = 0;
   out_7417487238667392724[15] = 0;
   out_7417487238667392724[16] = 0;
   out_7417487238667392724[17] = 0;
   out_7417487238667392724[18] = 0;
   out_7417487238667392724[19] = 0;
   out_7417487238667392724[20] = 1;
   out_7417487238667392724[21] = 0;
   out_7417487238667392724[22] = 0;
   out_7417487238667392724[23] = 0;
   out_7417487238667392724[24] = 0;
   out_7417487238667392724[25] = 0;
   out_7417487238667392724[26] = 0;
   out_7417487238667392724[27] = 0;
   out_7417487238667392724[28] = 0;
   out_7417487238667392724[29] = 0;
   out_7417487238667392724[30] = 1;
   out_7417487238667392724[31] = 0;
   out_7417487238667392724[32] = 0;
   out_7417487238667392724[33] = 0;
   out_7417487238667392724[34] = 0;
   out_7417487238667392724[35] = 0;
   out_7417487238667392724[36] = 0;
   out_7417487238667392724[37] = 0;
   out_7417487238667392724[38] = 0;
   out_7417487238667392724[39] = 0;
   out_7417487238667392724[40] = 1;
   out_7417487238667392724[41] = 0;
   out_7417487238667392724[42] = 0;
   out_7417487238667392724[43] = 0;
   out_7417487238667392724[44] = 0;
   out_7417487238667392724[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7417487238667392724[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7417487238667392724[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7417487238667392724[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7417487238667392724[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7417487238667392724[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7417487238667392724[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7417487238667392724[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7417487238667392724[53] = -9.8000000000000007*dt;
   out_7417487238667392724[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7417487238667392724[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7417487238667392724[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7417487238667392724[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7417487238667392724[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7417487238667392724[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7417487238667392724[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7417487238667392724[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7417487238667392724[62] = 0;
   out_7417487238667392724[63] = 0;
   out_7417487238667392724[64] = 0;
   out_7417487238667392724[65] = 0;
   out_7417487238667392724[66] = 0;
   out_7417487238667392724[67] = 0;
   out_7417487238667392724[68] = 0;
   out_7417487238667392724[69] = 0;
   out_7417487238667392724[70] = 1;
   out_7417487238667392724[71] = 0;
   out_7417487238667392724[72] = 0;
   out_7417487238667392724[73] = 0;
   out_7417487238667392724[74] = 0;
   out_7417487238667392724[75] = 0;
   out_7417487238667392724[76] = 0;
   out_7417487238667392724[77] = 0;
   out_7417487238667392724[78] = 0;
   out_7417487238667392724[79] = 0;
   out_7417487238667392724[80] = 1;
}
void h_25(double *state, double *unused, double *out_2841136473189617083) {
   out_2841136473189617083[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8264885873019158718) {
   out_8264885873019158718[0] = 0;
   out_8264885873019158718[1] = 0;
   out_8264885873019158718[2] = 0;
   out_8264885873019158718[3] = 0;
   out_8264885873019158718[4] = 0;
   out_8264885873019158718[5] = 0;
   out_8264885873019158718[6] = 1;
   out_8264885873019158718[7] = 0;
   out_8264885873019158718[8] = 0;
}
void h_24(double *state, double *unused, double *out_466890035114431795) {
   out_466890035114431795[0] = state[4];
   out_466890035114431795[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3606286394098874797) {
   out_3606286394098874797[0] = 0;
   out_3606286394098874797[1] = 0;
   out_3606286394098874797[2] = 0;
   out_3606286394098874797[3] = 0;
   out_3606286394098874797[4] = 1;
   out_3606286394098874797[5] = 0;
   out_3606286394098874797[6] = 0;
   out_3606286394098874797[7] = 0;
   out_3606286394098874797[8] = 0;
   out_3606286394098874797[9] = 0;
   out_3606286394098874797[10] = 0;
   out_3606286394098874797[11] = 0;
   out_3606286394098874797[12] = 0;
   out_3606286394098874797[13] = 0;
   out_3606286394098874797[14] = 1;
   out_3606286394098874797[15] = 0;
   out_3606286394098874797[16] = 0;
   out_3606286394098874797[17] = 0;
}
void h_30(double *state, double *unused, double *out_5040136177564432102) {
   out_5040136177564432102[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3265167859198776143) {
   out_3265167859198776143[0] = 0;
   out_3265167859198776143[1] = 0;
   out_3265167859198776143[2] = 0;
   out_3265167859198776143[3] = 0;
   out_3265167859198776143[4] = 1;
   out_3265167859198776143[5] = 0;
   out_3265167859198776143[6] = 0;
   out_3265167859198776143[7] = 0;
   out_3265167859198776143[8] = 0;
}
void h_26(double *state, double *unused, double *out_38462909903586457) {
   out_38462909903586457[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4523382554145102494) {
   out_4523382554145102494[0] = 0;
   out_4523382554145102494[1] = 0;
   out_4523382554145102494[2] = 0;
   out_4523382554145102494[3] = 0;
   out_4523382554145102494[4] = 0;
   out_4523382554145102494[5] = 0;
   out_4523382554145102494[6] = 0;
   out_4523382554145102494[7] = 1;
   out_4523382554145102494[8] = 0;
}
void h_27(double *state, double *unused, double *out_3784646326091279518) {
   out_3784646326091279518[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5439931170999201054) {
   out_5439931170999201054[0] = 0;
   out_5439931170999201054[1] = 0;
   out_5439931170999201054[2] = 0;
   out_5439931170999201054[3] = 1;
   out_5439931170999201054[4] = 0;
   out_5439931170999201054[5] = 0;
   out_5439931170999201054[6] = 0;
   out_5439931170999201054[7] = 0;
   out_5439931170999201054[8] = 0;
}
void h_29(double *state, double *unused, double *out_4096626324662769667) {
   out_4096626324662769667[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7153293897868752087) {
   out_7153293897868752087[0] = 0;
   out_7153293897868752087[1] = 1;
   out_7153293897868752087[2] = 0;
   out_7153293897868752087[3] = 0;
   out_7153293897868752087[4] = 0;
   out_7153293897868752087[5] = 0;
   out_7153293897868752087[6] = 0;
   out_7153293897868752087[7] = 0;
   out_7153293897868752087[8] = 0;
}
void h_28(double *state, double *unused, double *out_8263056230204422952) {
   out_8263056230204422952[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6211051158771268955) {
   out_6211051158771268955[0] = 1;
   out_6211051158771268955[1] = 0;
   out_6211051158771268955[2] = 0;
   out_6211051158771268955[3] = 0;
   out_6211051158771268955[4] = 0;
   out_6211051158771268955[5] = 0;
   out_6211051158771268955[6] = 0;
   out_6211051158771268955[7] = 0;
   out_6211051158771268955[8] = 0;
}
void h_31(double *state, double *unused, double *out_2004104806923773306) {
   out_2004104806923773306[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8295531834896119146) {
   out_8295531834896119146[0] = 0;
   out_8295531834896119146[1] = 0;
   out_8295531834896119146[2] = 0;
   out_8295531834896119146[3] = 0;
   out_8295531834896119146[4] = 0;
   out_8295531834896119146[5] = 0;
   out_8295531834896119146[6] = 0;
   out_8295531834896119146[7] = 0;
   out_8295531834896119146[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3502673967241664248) {
  err_fun(nom_x, delta_x, out_3502673967241664248);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5131477847977226664) {
  inv_err_fun(nom_x, true_x, out_5131477847977226664);
}
void car_H_mod_fun(double *state, double *out_2530919945440300104) {
  H_mod_fun(state, out_2530919945440300104);
}
void car_f_fun(double *state, double dt, double *out_7205150727819091354) {
  f_fun(state,  dt, out_7205150727819091354);
}
void car_F_fun(double *state, double dt, double *out_7417487238667392724) {
  F_fun(state,  dt, out_7417487238667392724);
}
void car_h_25(double *state, double *unused, double *out_2841136473189617083) {
  h_25(state, unused, out_2841136473189617083);
}
void car_H_25(double *state, double *unused, double *out_8264885873019158718) {
  H_25(state, unused, out_8264885873019158718);
}
void car_h_24(double *state, double *unused, double *out_466890035114431795) {
  h_24(state, unused, out_466890035114431795);
}
void car_H_24(double *state, double *unused, double *out_3606286394098874797) {
  H_24(state, unused, out_3606286394098874797);
}
void car_h_30(double *state, double *unused, double *out_5040136177564432102) {
  h_30(state, unused, out_5040136177564432102);
}
void car_H_30(double *state, double *unused, double *out_3265167859198776143) {
  H_30(state, unused, out_3265167859198776143);
}
void car_h_26(double *state, double *unused, double *out_38462909903586457) {
  h_26(state, unused, out_38462909903586457);
}
void car_H_26(double *state, double *unused, double *out_4523382554145102494) {
  H_26(state, unused, out_4523382554145102494);
}
void car_h_27(double *state, double *unused, double *out_3784646326091279518) {
  h_27(state, unused, out_3784646326091279518);
}
void car_H_27(double *state, double *unused, double *out_5439931170999201054) {
  H_27(state, unused, out_5439931170999201054);
}
void car_h_29(double *state, double *unused, double *out_4096626324662769667) {
  h_29(state, unused, out_4096626324662769667);
}
void car_H_29(double *state, double *unused, double *out_7153293897868752087) {
  H_29(state, unused, out_7153293897868752087);
}
void car_h_28(double *state, double *unused, double *out_8263056230204422952) {
  h_28(state, unused, out_8263056230204422952);
}
void car_H_28(double *state, double *unused, double *out_6211051158771268955) {
  H_28(state, unused, out_6211051158771268955);
}
void car_h_31(double *state, double *unused, double *out_2004104806923773306) {
  h_31(state, unused, out_2004104806923773306);
}
void car_H_31(double *state, double *unused, double *out_8295531834896119146) {
  H_31(state, unused, out_8295531834896119146);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
