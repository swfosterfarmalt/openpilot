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
void err_fun(double *nom_x, double *delta_x, double *out_5388900906574639090) {
   out_5388900906574639090[0] = delta_x[0] + nom_x[0];
   out_5388900906574639090[1] = delta_x[1] + nom_x[1];
   out_5388900906574639090[2] = delta_x[2] + nom_x[2];
   out_5388900906574639090[3] = delta_x[3] + nom_x[3];
   out_5388900906574639090[4] = delta_x[4] + nom_x[4];
   out_5388900906574639090[5] = delta_x[5] + nom_x[5];
   out_5388900906574639090[6] = delta_x[6] + nom_x[6];
   out_5388900906574639090[7] = delta_x[7] + nom_x[7];
   out_5388900906574639090[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8931120564270050854) {
   out_8931120564270050854[0] = -nom_x[0] + true_x[0];
   out_8931120564270050854[1] = -nom_x[1] + true_x[1];
   out_8931120564270050854[2] = -nom_x[2] + true_x[2];
   out_8931120564270050854[3] = -nom_x[3] + true_x[3];
   out_8931120564270050854[4] = -nom_x[4] + true_x[4];
   out_8931120564270050854[5] = -nom_x[5] + true_x[5];
   out_8931120564270050854[6] = -nom_x[6] + true_x[6];
   out_8931120564270050854[7] = -nom_x[7] + true_x[7];
   out_8931120564270050854[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4693862061305480966) {
   out_4693862061305480966[0] = 1.0;
   out_4693862061305480966[1] = 0;
   out_4693862061305480966[2] = 0;
   out_4693862061305480966[3] = 0;
   out_4693862061305480966[4] = 0;
   out_4693862061305480966[5] = 0;
   out_4693862061305480966[6] = 0;
   out_4693862061305480966[7] = 0;
   out_4693862061305480966[8] = 0;
   out_4693862061305480966[9] = 0;
   out_4693862061305480966[10] = 1.0;
   out_4693862061305480966[11] = 0;
   out_4693862061305480966[12] = 0;
   out_4693862061305480966[13] = 0;
   out_4693862061305480966[14] = 0;
   out_4693862061305480966[15] = 0;
   out_4693862061305480966[16] = 0;
   out_4693862061305480966[17] = 0;
   out_4693862061305480966[18] = 0;
   out_4693862061305480966[19] = 0;
   out_4693862061305480966[20] = 1.0;
   out_4693862061305480966[21] = 0;
   out_4693862061305480966[22] = 0;
   out_4693862061305480966[23] = 0;
   out_4693862061305480966[24] = 0;
   out_4693862061305480966[25] = 0;
   out_4693862061305480966[26] = 0;
   out_4693862061305480966[27] = 0;
   out_4693862061305480966[28] = 0;
   out_4693862061305480966[29] = 0;
   out_4693862061305480966[30] = 1.0;
   out_4693862061305480966[31] = 0;
   out_4693862061305480966[32] = 0;
   out_4693862061305480966[33] = 0;
   out_4693862061305480966[34] = 0;
   out_4693862061305480966[35] = 0;
   out_4693862061305480966[36] = 0;
   out_4693862061305480966[37] = 0;
   out_4693862061305480966[38] = 0;
   out_4693862061305480966[39] = 0;
   out_4693862061305480966[40] = 1.0;
   out_4693862061305480966[41] = 0;
   out_4693862061305480966[42] = 0;
   out_4693862061305480966[43] = 0;
   out_4693862061305480966[44] = 0;
   out_4693862061305480966[45] = 0;
   out_4693862061305480966[46] = 0;
   out_4693862061305480966[47] = 0;
   out_4693862061305480966[48] = 0;
   out_4693862061305480966[49] = 0;
   out_4693862061305480966[50] = 1.0;
   out_4693862061305480966[51] = 0;
   out_4693862061305480966[52] = 0;
   out_4693862061305480966[53] = 0;
   out_4693862061305480966[54] = 0;
   out_4693862061305480966[55] = 0;
   out_4693862061305480966[56] = 0;
   out_4693862061305480966[57] = 0;
   out_4693862061305480966[58] = 0;
   out_4693862061305480966[59] = 0;
   out_4693862061305480966[60] = 1.0;
   out_4693862061305480966[61] = 0;
   out_4693862061305480966[62] = 0;
   out_4693862061305480966[63] = 0;
   out_4693862061305480966[64] = 0;
   out_4693862061305480966[65] = 0;
   out_4693862061305480966[66] = 0;
   out_4693862061305480966[67] = 0;
   out_4693862061305480966[68] = 0;
   out_4693862061305480966[69] = 0;
   out_4693862061305480966[70] = 1.0;
   out_4693862061305480966[71] = 0;
   out_4693862061305480966[72] = 0;
   out_4693862061305480966[73] = 0;
   out_4693862061305480966[74] = 0;
   out_4693862061305480966[75] = 0;
   out_4693862061305480966[76] = 0;
   out_4693862061305480966[77] = 0;
   out_4693862061305480966[78] = 0;
   out_4693862061305480966[79] = 0;
   out_4693862061305480966[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3234639509369413484) {
   out_3234639509369413484[0] = state[0];
   out_3234639509369413484[1] = state[1];
   out_3234639509369413484[2] = state[2];
   out_3234639509369413484[3] = state[3];
   out_3234639509369413484[4] = state[4];
   out_3234639509369413484[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3234639509369413484[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3234639509369413484[7] = state[7];
   out_3234639509369413484[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8767551863233107118) {
   out_8767551863233107118[0] = 1;
   out_8767551863233107118[1] = 0;
   out_8767551863233107118[2] = 0;
   out_8767551863233107118[3] = 0;
   out_8767551863233107118[4] = 0;
   out_8767551863233107118[5] = 0;
   out_8767551863233107118[6] = 0;
   out_8767551863233107118[7] = 0;
   out_8767551863233107118[8] = 0;
   out_8767551863233107118[9] = 0;
   out_8767551863233107118[10] = 1;
   out_8767551863233107118[11] = 0;
   out_8767551863233107118[12] = 0;
   out_8767551863233107118[13] = 0;
   out_8767551863233107118[14] = 0;
   out_8767551863233107118[15] = 0;
   out_8767551863233107118[16] = 0;
   out_8767551863233107118[17] = 0;
   out_8767551863233107118[18] = 0;
   out_8767551863233107118[19] = 0;
   out_8767551863233107118[20] = 1;
   out_8767551863233107118[21] = 0;
   out_8767551863233107118[22] = 0;
   out_8767551863233107118[23] = 0;
   out_8767551863233107118[24] = 0;
   out_8767551863233107118[25] = 0;
   out_8767551863233107118[26] = 0;
   out_8767551863233107118[27] = 0;
   out_8767551863233107118[28] = 0;
   out_8767551863233107118[29] = 0;
   out_8767551863233107118[30] = 1;
   out_8767551863233107118[31] = 0;
   out_8767551863233107118[32] = 0;
   out_8767551863233107118[33] = 0;
   out_8767551863233107118[34] = 0;
   out_8767551863233107118[35] = 0;
   out_8767551863233107118[36] = 0;
   out_8767551863233107118[37] = 0;
   out_8767551863233107118[38] = 0;
   out_8767551863233107118[39] = 0;
   out_8767551863233107118[40] = 1;
   out_8767551863233107118[41] = 0;
   out_8767551863233107118[42] = 0;
   out_8767551863233107118[43] = 0;
   out_8767551863233107118[44] = 0;
   out_8767551863233107118[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8767551863233107118[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8767551863233107118[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8767551863233107118[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8767551863233107118[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8767551863233107118[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8767551863233107118[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8767551863233107118[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8767551863233107118[53] = -9.8000000000000007*dt;
   out_8767551863233107118[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8767551863233107118[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8767551863233107118[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8767551863233107118[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8767551863233107118[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8767551863233107118[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8767551863233107118[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8767551863233107118[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8767551863233107118[62] = 0;
   out_8767551863233107118[63] = 0;
   out_8767551863233107118[64] = 0;
   out_8767551863233107118[65] = 0;
   out_8767551863233107118[66] = 0;
   out_8767551863233107118[67] = 0;
   out_8767551863233107118[68] = 0;
   out_8767551863233107118[69] = 0;
   out_8767551863233107118[70] = 1;
   out_8767551863233107118[71] = 0;
   out_8767551863233107118[72] = 0;
   out_8767551863233107118[73] = 0;
   out_8767551863233107118[74] = 0;
   out_8767551863233107118[75] = 0;
   out_8767551863233107118[76] = 0;
   out_8767551863233107118[77] = 0;
   out_8767551863233107118[78] = 0;
   out_8767551863233107118[79] = 0;
   out_8767551863233107118[80] = 1;
}
void h_25(double *state, double *unused, double *out_1318005858210708024) {
   out_1318005858210708024[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3383746956418488080) {
   out_3383746956418488080[0] = 0;
   out_3383746956418488080[1] = 0;
   out_3383746956418488080[2] = 0;
   out_3383746956418488080[3] = 0;
   out_3383746956418488080[4] = 0;
   out_3383746956418488080[5] = 0;
   out_3383746956418488080[6] = 1;
   out_3383746956418488080[7] = 0;
   out_3383746956418488080[8] = 0;
}
void h_24(double *state, double *unused, double *out_3565217635435425234) {
   out_3565217635435425234[0] = state[4];
   out_3565217635435425234[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4034644677258438813) {
   out_4034644677258438813[0] = 0;
   out_4034644677258438813[1] = 0;
   out_4034644677258438813[2] = 0;
   out_4034644677258438813[3] = 0;
   out_4034644677258438813[4] = 1;
   out_4034644677258438813[5] = 0;
   out_4034644677258438813[6] = 0;
   out_4034644677258438813[7] = 0;
   out_4034644677258438813[8] = 0;
   out_4034644677258438813[9] = 0;
   out_4034644677258438813[10] = 0;
   out_4034644677258438813[11] = 0;
   out_4034644677258438813[12] = 0;
   out_4034644677258438813[13] = 0;
   out_4034644677258438813[14] = 1;
   out_4034644677258438813[15] = 0;
   out_4034644677258438813[16] = 0;
   out_4034644677258438813[17] = 0;
}
void h_30(double *state, double *unused, double *out_7178654823053031968) {
   out_7178654823053031968[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7911443286546096278) {
   out_7911443286546096278[0] = 0;
   out_7911443286546096278[1] = 0;
   out_7911443286546096278[2] = 0;
   out_7911443286546096278[3] = 0;
   out_7911443286546096278[4] = 1;
   out_7911443286546096278[5] = 0;
   out_7911443286546096278[6] = 0;
   out_7911443286546096278[7] = 0;
   out_7911443286546096278[8] = 0;
}
void h_26(double *state, double *unused, double *out_4557986449716406676) {
   out_4557986449716406676[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7125250275292544304) {
   out_7125250275292544304[0] = 0;
   out_7125250275292544304[1] = 0;
   out_7125250275292544304[2] = 0;
   out_7125250275292544304[3] = 0;
   out_7125250275292544304[4] = 0;
   out_7125250275292544304[5] = 0;
   out_7125250275292544304[6] = 0;
   out_7125250275292544304[7] = 1;
   out_7125250275292544304[8] = 0;
}
void h_27(double *state, double *unused, double *out_6319679125871553669) {
   out_6319679125871553669[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5687849215362153061) {
   out_5687849215362153061[0] = 0;
   out_5687849215362153061[1] = 0;
   out_5687849215362153061[2] = 0;
   out_5687849215362153061[3] = 1;
   out_5687849215362153061[4] = 0;
   out_5687849215362153061[5] = 0;
   out_5687849215362153061[6] = 0;
   out_5687849215362153061[7] = 0;
   out_5687849215362153061[8] = 0;
}
void h_29(double *state, double *unused, double *out_2176981555392186323) {
   out_2176981555392186323[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7401211942231704094) {
   out_7401211942231704094[0] = 0;
   out_7401211942231704094[1] = 1;
   out_7401211942231704094[2] = 0;
   out_7401211942231704094[3] = 0;
   out_7401211942231704094[4] = 0;
   out_7401211942231704094[5] = 0;
   out_7401211942231704094[6] = 0;
   out_7401211942231704094[7] = 0;
   out_7401211942231704094[8] = 0;
}
void h_28(double *state, double *unused, double *out_2903405836222483653) {
   out_2903405836222483653[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5437581670666377843) {
   out_5437581670666377843[0] = 1;
   out_5437581670666377843[1] = 0;
   out_5437581670666377843[2] = 0;
   out_5437581670666377843[3] = 0;
   out_5437581670666377843[4] = 0;
   out_5437581670666377843[5] = 0;
   out_5437581670666377843[6] = 0;
   out_5437581670666377843[7] = 0;
   out_5437581670666377843[8] = 0;
}
void h_31(double *state, double *unused, double *out_1042811795926202135) {
   out_1042811795926202135[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3353100994541527652) {
   out_3353100994541527652[0] = 0;
   out_3353100994541527652[1] = 0;
   out_3353100994541527652[2] = 0;
   out_3353100994541527652[3] = 0;
   out_3353100994541527652[4] = 0;
   out_3353100994541527652[5] = 0;
   out_3353100994541527652[6] = 0;
   out_3353100994541527652[7] = 0;
   out_3353100994541527652[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5388900906574639090) {
  err_fun(nom_x, delta_x, out_5388900906574639090);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8931120564270050854) {
  inv_err_fun(nom_x, true_x, out_8931120564270050854);
}
void car_H_mod_fun(double *state, double *out_4693862061305480966) {
  H_mod_fun(state, out_4693862061305480966);
}
void car_f_fun(double *state, double dt, double *out_3234639509369413484) {
  f_fun(state,  dt, out_3234639509369413484);
}
void car_F_fun(double *state, double dt, double *out_8767551863233107118) {
  F_fun(state,  dt, out_8767551863233107118);
}
void car_h_25(double *state, double *unused, double *out_1318005858210708024) {
  h_25(state, unused, out_1318005858210708024);
}
void car_H_25(double *state, double *unused, double *out_3383746956418488080) {
  H_25(state, unused, out_3383746956418488080);
}
void car_h_24(double *state, double *unused, double *out_3565217635435425234) {
  h_24(state, unused, out_3565217635435425234);
}
void car_H_24(double *state, double *unused, double *out_4034644677258438813) {
  H_24(state, unused, out_4034644677258438813);
}
void car_h_30(double *state, double *unused, double *out_7178654823053031968) {
  h_30(state, unused, out_7178654823053031968);
}
void car_H_30(double *state, double *unused, double *out_7911443286546096278) {
  H_30(state, unused, out_7911443286546096278);
}
void car_h_26(double *state, double *unused, double *out_4557986449716406676) {
  h_26(state, unused, out_4557986449716406676);
}
void car_H_26(double *state, double *unused, double *out_7125250275292544304) {
  H_26(state, unused, out_7125250275292544304);
}
void car_h_27(double *state, double *unused, double *out_6319679125871553669) {
  h_27(state, unused, out_6319679125871553669);
}
void car_H_27(double *state, double *unused, double *out_5687849215362153061) {
  H_27(state, unused, out_5687849215362153061);
}
void car_h_29(double *state, double *unused, double *out_2176981555392186323) {
  h_29(state, unused, out_2176981555392186323);
}
void car_H_29(double *state, double *unused, double *out_7401211942231704094) {
  H_29(state, unused, out_7401211942231704094);
}
void car_h_28(double *state, double *unused, double *out_2903405836222483653) {
  h_28(state, unused, out_2903405836222483653);
}
void car_H_28(double *state, double *unused, double *out_5437581670666377843) {
  H_28(state, unused, out_5437581670666377843);
}
void car_h_31(double *state, double *unused, double *out_1042811795926202135) {
  h_31(state, unused, out_1042811795926202135);
}
void car_H_31(double *state, double *unused, double *out_3353100994541527652) {
  H_31(state, unused, out_3353100994541527652);
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
