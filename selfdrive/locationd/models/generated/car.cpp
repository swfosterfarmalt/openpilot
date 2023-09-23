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
void err_fun(double *nom_x, double *delta_x, double *out_5369626594687161812) {
   out_5369626594687161812[0] = delta_x[0] + nom_x[0];
   out_5369626594687161812[1] = delta_x[1] + nom_x[1];
   out_5369626594687161812[2] = delta_x[2] + nom_x[2];
   out_5369626594687161812[3] = delta_x[3] + nom_x[3];
   out_5369626594687161812[4] = delta_x[4] + nom_x[4];
   out_5369626594687161812[5] = delta_x[5] + nom_x[5];
   out_5369626594687161812[6] = delta_x[6] + nom_x[6];
   out_5369626594687161812[7] = delta_x[7] + nom_x[7];
   out_5369626594687161812[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1748580837222196880) {
   out_1748580837222196880[0] = -nom_x[0] + true_x[0];
   out_1748580837222196880[1] = -nom_x[1] + true_x[1];
   out_1748580837222196880[2] = -nom_x[2] + true_x[2];
   out_1748580837222196880[3] = -nom_x[3] + true_x[3];
   out_1748580837222196880[4] = -nom_x[4] + true_x[4];
   out_1748580837222196880[5] = -nom_x[5] + true_x[5];
   out_1748580837222196880[6] = -nom_x[6] + true_x[6];
   out_1748580837222196880[7] = -nom_x[7] + true_x[7];
   out_1748580837222196880[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2693258022412199625) {
   out_2693258022412199625[0] = 1.0;
   out_2693258022412199625[1] = 0;
   out_2693258022412199625[2] = 0;
   out_2693258022412199625[3] = 0;
   out_2693258022412199625[4] = 0;
   out_2693258022412199625[5] = 0;
   out_2693258022412199625[6] = 0;
   out_2693258022412199625[7] = 0;
   out_2693258022412199625[8] = 0;
   out_2693258022412199625[9] = 0;
   out_2693258022412199625[10] = 1.0;
   out_2693258022412199625[11] = 0;
   out_2693258022412199625[12] = 0;
   out_2693258022412199625[13] = 0;
   out_2693258022412199625[14] = 0;
   out_2693258022412199625[15] = 0;
   out_2693258022412199625[16] = 0;
   out_2693258022412199625[17] = 0;
   out_2693258022412199625[18] = 0;
   out_2693258022412199625[19] = 0;
   out_2693258022412199625[20] = 1.0;
   out_2693258022412199625[21] = 0;
   out_2693258022412199625[22] = 0;
   out_2693258022412199625[23] = 0;
   out_2693258022412199625[24] = 0;
   out_2693258022412199625[25] = 0;
   out_2693258022412199625[26] = 0;
   out_2693258022412199625[27] = 0;
   out_2693258022412199625[28] = 0;
   out_2693258022412199625[29] = 0;
   out_2693258022412199625[30] = 1.0;
   out_2693258022412199625[31] = 0;
   out_2693258022412199625[32] = 0;
   out_2693258022412199625[33] = 0;
   out_2693258022412199625[34] = 0;
   out_2693258022412199625[35] = 0;
   out_2693258022412199625[36] = 0;
   out_2693258022412199625[37] = 0;
   out_2693258022412199625[38] = 0;
   out_2693258022412199625[39] = 0;
   out_2693258022412199625[40] = 1.0;
   out_2693258022412199625[41] = 0;
   out_2693258022412199625[42] = 0;
   out_2693258022412199625[43] = 0;
   out_2693258022412199625[44] = 0;
   out_2693258022412199625[45] = 0;
   out_2693258022412199625[46] = 0;
   out_2693258022412199625[47] = 0;
   out_2693258022412199625[48] = 0;
   out_2693258022412199625[49] = 0;
   out_2693258022412199625[50] = 1.0;
   out_2693258022412199625[51] = 0;
   out_2693258022412199625[52] = 0;
   out_2693258022412199625[53] = 0;
   out_2693258022412199625[54] = 0;
   out_2693258022412199625[55] = 0;
   out_2693258022412199625[56] = 0;
   out_2693258022412199625[57] = 0;
   out_2693258022412199625[58] = 0;
   out_2693258022412199625[59] = 0;
   out_2693258022412199625[60] = 1.0;
   out_2693258022412199625[61] = 0;
   out_2693258022412199625[62] = 0;
   out_2693258022412199625[63] = 0;
   out_2693258022412199625[64] = 0;
   out_2693258022412199625[65] = 0;
   out_2693258022412199625[66] = 0;
   out_2693258022412199625[67] = 0;
   out_2693258022412199625[68] = 0;
   out_2693258022412199625[69] = 0;
   out_2693258022412199625[70] = 1.0;
   out_2693258022412199625[71] = 0;
   out_2693258022412199625[72] = 0;
   out_2693258022412199625[73] = 0;
   out_2693258022412199625[74] = 0;
   out_2693258022412199625[75] = 0;
   out_2693258022412199625[76] = 0;
   out_2693258022412199625[77] = 0;
   out_2693258022412199625[78] = 0;
   out_2693258022412199625[79] = 0;
   out_2693258022412199625[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7366003018438160507) {
   out_7366003018438160507[0] = state[0];
   out_7366003018438160507[1] = state[1];
   out_7366003018438160507[2] = state[2];
   out_7366003018438160507[3] = state[3];
   out_7366003018438160507[4] = state[4];
   out_7366003018438160507[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7366003018438160507[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7366003018438160507[7] = state[7];
   out_7366003018438160507[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9173119452061934407) {
   out_9173119452061934407[0] = 1;
   out_9173119452061934407[1] = 0;
   out_9173119452061934407[2] = 0;
   out_9173119452061934407[3] = 0;
   out_9173119452061934407[4] = 0;
   out_9173119452061934407[5] = 0;
   out_9173119452061934407[6] = 0;
   out_9173119452061934407[7] = 0;
   out_9173119452061934407[8] = 0;
   out_9173119452061934407[9] = 0;
   out_9173119452061934407[10] = 1;
   out_9173119452061934407[11] = 0;
   out_9173119452061934407[12] = 0;
   out_9173119452061934407[13] = 0;
   out_9173119452061934407[14] = 0;
   out_9173119452061934407[15] = 0;
   out_9173119452061934407[16] = 0;
   out_9173119452061934407[17] = 0;
   out_9173119452061934407[18] = 0;
   out_9173119452061934407[19] = 0;
   out_9173119452061934407[20] = 1;
   out_9173119452061934407[21] = 0;
   out_9173119452061934407[22] = 0;
   out_9173119452061934407[23] = 0;
   out_9173119452061934407[24] = 0;
   out_9173119452061934407[25] = 0;
   out_9173119452061934407[26] = 0;
   out_9173119452061934407[27] = 0;
   out_9173119452061934407[28] = 0;
   out_9173119452061934407[29] = 0;
   out_9173119452061934407[30] = 1;
   out_9173119452061934407[31] = 0;
   out_9173119452061934407[32] = 0;
   out_9173119452061934407[33] = 0;
   out_9173119452061934407[34] = 0;
   out_9173119452061934407[35] = 0;
   out_9173119452061934407[36] = 0;
   out_9173119452061934407[37] = 0;
   out_9173119452061934407[38] = 0;
   out_9173119452061934407[39] = 0;
   out_9173119452061934407[40] = 1;
   out_9173119452061934407[41] = 0;
   out_9173119452061934407[42] = 0;
   out_9173119452061934407[43] = 0;
   out_9173119452061934407[44] = 0;
   out_9173119452061934407[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9173119452061934407[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9173119452061934407[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9173119452061934407[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9173119452061934407[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9173119452061934407[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9173119452061934407[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9173119452061934407[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9173119452061934407[53] = -9.8000000000000007*dt;
   out_9173119452061934407[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9173119452061934407[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9173119452061934407[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9173119452061934407[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9173119452061934407[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9173119452061934407[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9173119452061934407[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9173119452061934407[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9173119452061934407[62] = 0;
   out_9173119452061934407[63] = 0;
   out_9173119452061934407[64] = 0;
   out_9173119452061934407[65] = 0;
   out_9173119452061934407[66] = 0;
   out_9173119452061934407[67] = 0;
   out_9173119452061934407[68] = 0;
   out_9173119452061934407[69] = 0;
   out_9173119452061934407[70] = 1;
   out_9173119452061934407[71] = 0;
   out_9173119452061934407[72] = 0;
   out_9173119452061934407[73] = 0;
   out_9173119452061934407[74] = 0;
   out_9173119452061934407[75] = 0;
   out_9173119452061934407[76] = 0;
   out_9173119452061934407[77] = 0;
   out_9173119452061934407[78] = 0;
   out_9173119452061934407[79] = 0;
   out_9173119452061934407[80] = 1;
}
void h_25(double *state, double *unused, double *out_4312494488748408182) {
   out_4312494488748408182[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1156567547106706327) {
   out_1156567547106706327[0] = 0;
   out_1156567547106706327[1] = 0;
   out_1156567547106706327[2] = 0;
   out_1156567547106706327[3] = 0;
   out_1156567547106706327[4] = 0;
   out_1156567547106706327[5] = 0;
   out_1156567547106706327[6] = 1;
   out_1156567547106706327[7] = 0;
   out_1156567547106706327[8] = 0;
}
void h_24(double *state, double *unused, double *out_4071417845229739232) {
   out_4071417845229739232[0] = state[4];
   out_4071417845229739232[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1016082051898793239) {
   out_1016082051898793239[0] = 0;
   out_1016082051898793239[1] = 0;
   out_1016082051898793239[2] = 0;
   out_1016082051898793239[3] = 0;
   out_1016082051898793239[4] = 1;
   out_1016082051898793239[5] = 0;
   out_1016082051898793239[6] = 0;
   out_1016082051898793239[7] = 0;
   out_1016082051898793239[8] = 0;
   out_1016082051898793239[9] = 0;
   out_1016082051898793239[10] = 0;
   out_1016082051898793239[11] = 0;
   out_1016082051898793239[12] = 0;
   out_1016082051898793239[13] = 0;
   out_1016082051898793239[14] = 1;
   out_1016082051898793239[15] = 0;
   out_1016082051898793239[16] = 0;
   out_1016082051898793239[17] = 0;
}
void h_30(double *state, double *unused, double *out_3568778162005641003) {
   out_3568778162005641003[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3674900505613954954) {
   out_3674900505613954954[0] = 0;
   out_3674900505613954954[1] = 0;
   out_3674900505613954954[2] = 0;
   out_3674900505613954954[3] = 0;
   out_3674900505613954954[4] = 1;
   out_3674900505613954954[5] = 0;
   out_3674900505613954954[6] = 0;
   out_3674900505613954954[7] = 0;
   out_3674900505613954954[8] = 0;
}
void h_26(double *state, double *unused, double *out_8033318351721624937) {
   out_8033318351721624937[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2584935771767349897) {
   out_2584935771767349897[0] = 0;
   out_2584935771767349897[1] = 0;
   out_2584935771767349897[2] = 0;
   out_2584935771767349897[3] = 0;
   out_2584935771767349897[4] = 0;
   out_2584935771767349897[5] = 0;
   out_2584935771767349897[6] = 0;
   out_2584935771767349897[7] = 1;
   out_2584935771767349897[8] = 0;
}
void h_27(double *state, double *unused, double *out_689178778912437463) {
   out_689178778912437463[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1147534711836958654) {
   out_1147534711836958654[0] = 0;
   out_1147534711836958654[1] = 0;
   out_1147534711836958654[2] = 0;
   out_1147534711836958654[3] = 1;
   out_1147534711836958654[4] = 0;
   out_1147534711836958654[5] = 0;
   out_1147534711836958654[6] = 0;
   out_1147534711836958654[7] = 0;
   out_1147534711836958654[8] = 0;
}
void h_29(double *state, double *unused, double *out_6844625974753097592) {
   out_6844625974753097592[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4185131849928347138) {
   out_4185131849928347138[0] = 0;
   out_4185131849928347138[1] = 1;
   out_4185131849928347138[2] = 0;
   out_4185131849928347138[3] = 0;
   out_4185131849928347138[4] = 0;
   out_4185131849928347138[5] = 0;
   out_4185131849928347138[6] = 0;
   out_4185131849928347138[7] = 0;
   out_4185131849928347138[8] = 0;
}
void h_28(double *state, double *unused, double *out_254331073989224972) {
   out_254331073989224972[0] = state[0];
}
void H_28(double *state, double *unused, double *out_897267167141183436) {
   out_897267167141183436[0] = 1;
   out_897267167141183436[1] = 0;
   out_897267167141183436[2] = 0;
   out_897267167141183436[3] = 0;
   out_897267167141183436[4] = 0;
   out_897267167141183436[5] = 0;
   out_897267167141183436[6] = 0;
   out_897267167141183436[7] = 0;
   out_897267167141183436[8] = 0;
}
void h_31(double *state, double *unused, double *out_3450770768072264259) {
   out_3450770768072264259[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1187213508983666755) {
   out_1187213508983666755[0] = 0;
   out_1187213508983666755[1] = 0;
   out_1187213508983666755[2] = 0;
   out_1187213508983666755[3] = 0;
   out_1187213508983666755[4] = 0;
   out_1187213508983666755[5] = 0;
   out_1187213508983666755[6] = 0;
   out_1187213508983666755[7] = 0;
   out_1187213508983666755[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5369626594687161812) {
  err_fun(nom_x, delta_x, out_5369626594687161812);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1748580837222196880) {
  inv_err_fun(nom_x, true_x, out_1748580837222196880);
}
void car_H_mod_fun(double *state, double *out_2693258022412199625) {
  H_mod_fun(state, out_2693258022412199625);
}
void car_f_fun(double *state, double dt, double *out_7366003018438160507) {
  f_fun(state,  dt, out_7366003018438160507);
}
void car_F_fun(double *state, double dt, double *out_9173119452061934407) {
  F_fun(state,  dt, out_9173119452061934407);
}
void car_h_25(double *state, double *unused, double *out_4312494488748408182) {
  h_25(state, unused, out_4312494488748408182);
}
void car_H_25(double *state, double *unused, double *out_1156567547106706327) {
  H_25(state, unused, out_1156567547106706327);
}
void car_h_24(double *state, double *unused, double *out_4071417845229739232) {
  h_24(state, unused, out_4071417845229739232);
}
void car_H_24(double *state, double *unused, double *out_1016082051898793239) {
  H_24(state, unused, out_1016082051898793239);
}
void car_h_30(double *state, double *unused, double *out_3568778162005641003) {
  h_30(state, unused, out_3568778162005641003);
}
void car_H_30(double *state, double *unused, double *out_3674900505613954954) {
  H_30(state, unused, out_3674900505613954954);
}
void car_h_26(double *state, double *unused, double *out_8033318351721624937) {
  h_26(state, unused, out_8033318351721624937);
}
void car_H_26(double *state, double *unused, double *out_2584935771767349897) {
  H_26(state, unused, out_2584935771767349897);
}
void car_h_27(double *state, double *unused, double *out_689178778912437463) {
  h_27(state, unused, out_689178778912437463);
}
void car_H_27(double *state, double *unused, double *out_1147534711836958654) {
  H_27(state, unused, out_1147534711836958654);
}
void car_h_29(double *state, double *unused, double *out_6844625974753097592) {
  h_29(state, unused, out_6844625974753097592);
}
void car_H_29(double *state, double *unused, double *out_4185131849928347138) {
  H_29(state, unused, out_4185131849928347138);
}
void car_h_28(double *state, double *unused, double *out_254331073989224972) {
  h_28(state, unused, out_254331073989224972);
}
void car_H_28(double *state, double *unused, double *out_897267167141183436) {
  H_28(state, unused, out_897267167141183436);
}
void car_h_31(double *state, double *unused, double *out_3450770768072264259) {
  h_31(state, unused, out_3450770768072264259);
}
void car_H_31(double *state, double *unused, double *out_1187213508983666755) {
  H_31(state, unused, out_1187213508983666755);
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
