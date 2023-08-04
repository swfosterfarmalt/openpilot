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
void err_fun(double *nom_x, double *delta_x, double *out_3229655548484910964) {
   out_3229655548484910964[0] = delta_x[0] + nom_x[0];
   out_3229655548484910964[1] = delta_x[1] + nom_x[1];
   out_3229655548484910964[2] = delta_x[2] + nom_x[2];
   out_3229655548484910964[3] = delta_x[3] + nom_x[3];
   out_3229655548484910964[4] = delta_x[4] + nom_x[4];
   out_3229655548484910964[5] = delta_x[5] + nom_x[5];
   out_3229655548484910964[6] = delta_x[6] + nom_x[6];
   out_3229655548484910964[7] = delta_x[7] + nom_x[7];
   out_3229655548484910964[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_154977718050279836) {
   out_154977718050279836[0] = -nom_x[0] + true_x[0];
   out_154977718050279836[1] = -nom_x[1] + true_x[1];
   out_154977718050279836[2] = -nom_x[2] + true_x[2];
   out_154977718050279836[3] = -nom_x[3] + true_x[3];
   out_154977718050279836[4] = -nom_x[4] + true_x[4];
   out_154977718050279836[5] = -nom_x[5] + true_x[5];
   out_154977718050279836[6] = -nom_x[6] + true_x[6];
   out_154977718050279836[7] = -nom_x[7] + true_x[7];
   out_154977718050279836[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1011263767238227610) {
   out_1011263767238227610[0] = 1.0;
   out_1011263767238227610[1] = 0;
   out_1011263767238227610[2] = 0;
   out_1011263767238227610[3] = 0;
   out_1011263767238227610[4] = 0;
   out_1011263767238227610[5] = 0;
   out_1011263767238227610[6] = 0;
   out_1011263767238227610[7] = 0;
   out_1011263767238227610[8] = 0;
   out_1011263767238227610[9] = 0;
   out_1011263767238227610[10] = 1.0;
   out_1011263767238227610[11] = 0;
   out_1011263767238227610[12] = 0;
   out_1011263767238227610[13] = 0;
   out_1011263767238227610[14] = 0;
   out_1011263767238227610[15] = 0;
   out_1011263767238227610[16] = 0;
   out_1011263767238227610[17] = 0;
   out_1011263767238227610[18] = 0;
   out_1011263767238227610[19] = 0;
   out_1011263767238227610[20] = 1.0;
   out_1011263767238227610[21] = 0;
   out_1011263767238227610[22] = 0;
   out_1011263767238227610[23] = 0;
   out_1011263767238227610[24] = 0;
   out_1011263767238227610[25] = 0;
   out_1011263767238227610[26] = 0;
   out_1011263767238227610[27] = 0;
   out_1011263767238227610[28] = 0;
   out_1011263767238227610[29] = 0;
   out_1011263767238227610[30] = 1.0;
   out_1011263767238227610[31] = 0;
   out_1011263767238227610[32] = 0;
   out_1011263767238227610[33] = 0;
   out_1011263767238227610[34] = 0;
   out_1011263767238227610[35] = 0;
   out_1011263767238227610[36] = 0;
   out_1011263767238227610[37] = 0;
   out_1011263767238227610[38] = 0;
   out_1011263767238227610[39] = 0;
   out_1011263767238227610[40] = 1.0;
   out_1011263767238227610[41] = 0;
   out_1011263767238227610[42] = 0;
   out_1011263767238227610[43] = 0;
   out_1011263767238227610[44] = 0;
   out_1011263767238227610[45] = 0;
   out_1011263767238227610[46] = 0;
   out_1011263767238227610[47] = 0;
   out_1011263767238227610[48] = 0;
   out_1011263767238227610[49] = 0;
   out_1011263767238227610[50] = 1.0;
   out_1011263767238227610[51] = 0;
   out_1011263767238227610[52] = 0;
   out_1011263767238227610[53] = 0;
   out_1011263767238227610[54] = 0;
   out_1011263767238227610[55] = 0;
   out_1011263767238227610[56] = 0;
   out_1011263767238227610[57] = 0;
   out_1011263767238227610[58] = 0;
   out_1011263767238227610[59] = 0;
   out_1011263767238227610[60] = 1.0;
   out_1011263767238227610[61] = 0;
   out_1011263767238227610[62] = 0;
   out_1011263767238227610[63] = 0;
   out_1011263767238227610[64] = 0;
   out_1011263767238227610[65] = 0;
   out_1011263767238227610[66] = 0;
   out_1011263767238227610[67] = 0;
   out_1011263767238227610[68] = 0;
   out_1011263767238227610[69] = 0;
   out_1011263767238227610[70] = 1.0;
   out_1011263767238227610[71] = 0;
   out_1011263767238227610[72] = 0;
   out_1011263767238227610[73] = 0;
   out_1011263767238227610[74] = 0;
   out_1011263767238227610[75] = 0;
   out_1011263767238227610[76] = 0;
   out_1011263767238227610[77] = 0;
   out_1011263767238227610[78] = 0;
   out_1011263767238227610[79] = 0;
   out_1011263767238227610[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5721512869151697320) {
   out_5721512869151697320[0] = state[0];
   out_5721512869151697320[1] = state[1];
   out_5721512869151697320[2] = state[2];
   out_5721512869151697320[3] = state[3];
   out_5721512869151697320[4] = state[4];
   out_5721512869151697320[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5721512869151697320[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5721512869151697320[7] = state[7];
   out_5721512869151697320[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1692340814292537798) {
   out_1692340814292537798[0] = 1;
   out_1692340814292537798[1] = 0;
   out_1692340814292537798[2] = 0;
   out_1692340814292537798[3] = 0;
   out_1692340814292537798[4] = 0;
   out_1692340814292537798[5] = 0;
   out_1692340814292537798[6] = 0;
   out_1692340814292537798[7] = 0;
   out_1692340814292537798[8] = 0;
   out_1692340814292537798[9] = 0;
   out_1692340814292537798[10] = 1;
   out_1692340814292537798[11] = 0;
   out_1692340814292537798[12] = 0;
   out_1692340814292537798[13] = 0;
   out_1692340814292537798[14] = 0;
   out_1692340814292537798[15] = 0;
   out_1692340814292537798[16] = 0;
   out_1692340814292537798[17] = 0;
   out_1692340814292537798[18] = 0;
   out_1692340814292537798[19] = 0;
   out_1692340814292537798[20] = 1;
   out_1692340814292537798[21] = 0;
   out_1692340814292537798[22] = 0;
   out_1692340814292537798[23] = 0;
   out_1692340814292537798[24] = 0;
   out_1692340814292537798[25] = 0;
   out_1692340814292537798[26] = 0;
   out_1692340814292537798[27] = 0;
   out_1692340814292537798[28] = 0;
   out_1692340814292537798[29] = 0;
   out_1692340814292537798[30] = 1;
   out_1692340814292537798[31] = 0;
   out_1692340814292537798[32] = 0;
   out_1692340814292537798[33] = 0;
   out_1692340814292537798[34] = 0;
   out_1692340814292537798[35] = 0;
   out_1692340814292537798[36] = 0;
   out_1692340814292537798[37] = 0;
   out_1692340814292537798[38] = 0;
   out_1692340814292537798[39] = 0;
   out_1692340814292537798[40] = 1;
   out_1692340814292537798[41] = 0;
   out_1692340814292537798[42] = 0;
   out_1692340814292537798[43] = 0;
   out_1692340814292537798[44] = 0;
   out_1692340814292537798[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1692340814292537798[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1692340814292537798[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1692340814292537798[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1692340814292537798[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1692340814292537798[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1692340814292537798[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1692340814292537798[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1692340814292537798[53] = -9.8000000000000007*dt;
   out_1692340814292537798[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1692340814292537798[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1692340814292537798[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1692340814292537798[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1692340814292537798[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1692340814292537798[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1692340814292537798[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1692340814292537798[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1692340814292537798[62] = 0;
   out_1692340814292537798[63] = 0;
   out_1692340814292537798[64] = 0;
   out_1692340814292537798[65] = 0;
   out_1692340814292537798[66] = 0;
   out_1692340814292537798[67] = 0;
   out_1692340814292537798[68] = 0;
   out_1692340814292537798[69] = 0;
   out_1692340814292537798[70] = 1;
   out_1692340814292537798[71] = 0;
   out_1692340814292537798[72] = 0;
   out_1692340814292537798[73] = 0;
   out_1692340814292537798[74] = 0;
   out_1692340814292537798[75] = 0;
   out_1692340814292537798[76] = 0;
   out_1692340814292537798[77] = 0;
   out_1692340814292537798[78] = 0;
   out_1692340814292537798[79] = 0;
   out_1692340814292537798[80] = 1;
}
void h_25(double *state, double *unused, double *out_2597451784216131708) {
   out_2597451784216131708[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4643745684543313011) {
   out_4643745684543313011[0] = 0;
   out_4643745684543313011[1] = 0;
   out_4643745684543313011[2] = 0;
   out_4643745684543313011[3] = 0;
   out_4643745684543313011[4] = 0;
   out_4643745684543313011[5] = 0;
   out_4643745684543313011[6] = 1;
   out_4643745684543313011[7] = 0;
   out_4643745684543313011[8] = 0;
}
void h_24(double *state, double *unused, double *out_5427474373712947275) {
   out_5427474373712947275[0] = state[4];
   out_5427474373712947275[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7148792931058052848) {
   out_7148792931058052848[0] = 0;
   out_7148792931058052848[1] = 0;
   out_7148792931058052848[2] = 0;
   out_7148792931058052848[3] = 0;
   out_7148792931058052848[4] = 1;
   out_7148792931058052848[5] = 0;
   out_7148792931058052848[6] = 0;
   out_7148792931058052848[7] = 0;
   out_7148792931058052848[8] = 0;
   out_7148792931058052848[9] = 0;
   out_7148792931058052848[10] = 0;
   out_7148792931058052848[11] = 0;
   out_7148792931058052848[12] = 0;
   out_7148792931058052848[13] = 0;
   out_7148792931058052848[14] = 1;
   out_7148792931058052848[15] = 0;
   out_7148792931058052848[16] = 0;
   out_7148792931058052848[17] = 0;
}
void h_30(double *state, double *unused, double *out_2322257721931625819) {
   out_2322257721931625819[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4514406737400072941) {
   out_4514406737400072941[0] = 0;
   out_4514406737400072941[1] = 0;
   out_4514406737400072941[2] = 0;
   out_4514406737400072941[3] = 0;
   out_4514406737400072941[4] = 1;
   out_4514406737400072941[5] = 0;
   out_4514406737400072941[6] = 0;
   out_4514406737400072941[7] = 0;
   out_4514406737400072941[8] = 0;
}
void h_26(double *state, double *unused, double *out_7323930989592471464) {
   out_7323930989592471464[0] = state[7];
}
void H_26(double *state, double *unused, double *out_902242365669256787) {
   out_902242365669256787[0] = 0;
   out_902242365669256787[1] = 0;
   out_902242365669256787[2] = 0;
   out_902242365669256787[3] = 0;
   out_902242365669256787[4] = 0;
   out_902242365669256787[5] = 0;
   out_902242365669256787[6] = 0;
   out_902242365669256787[7] = 1;
   out_902242365669256787[8] = 0;
}
void h_27(double *state, double *unused, double *out_3540961637117794143) {
   out_3540961637117794143[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2339643425599648030) {
   out_2339643425599648030[0] = 0;
   out_2339643425599648030[1] = 0;
   out_2339643425599648030[2] = 0;
   out_2339643425599648030[3] = 1;
   out_2339643425599648030[4] = 0;
   out_2339643425599648030[5] = 0;
   out_2339643425599648030[6] = 0;
   out_2339643425599648030[7] = 0;
   out_2339643425599648030[8] = 0;
}
void h_29(double *state, double *unused, double *out_3265767574833288254) {
   out_3265767574833288254[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5024638081714465125) {
   out_5024638081714465125[0] = 0;
   out_5024638081714465125[1] = 1;
   out_5024638081714465125[2] = 0;
   out_5024638081714465125[3] = 0;
   out_5024638081714465125[4] = 0;
   out_5024638081714465125[5] = 0;
   out_5024638081714465125[6] = 0;
   out_5024638081714465125[7] = 0;
   out_5024638081714465125[8] = 0;
}
void h_28(double *state, double *unused, double *out_4108383536193540199) {
   out_4108383536193540199[0] = state[0];
}
void H_28(double *state, double *unused, double *out_57760935355065449) {
   out_57760935355065449[0] = 1;
   out_57760935355065449[1] = 0;
   out_57760935355065449[2] = 0;
   out_57760935355065449[3] = 0;
   out_57760935355065449[4] = 0;
   out_57760935355065449[5] = 0;
   out_57760935355065449[6] = 0;
   out_57760935355065449[7] = 0;
   out_57760935355065449[8] = 0;
}
void h_31(double *state, double *unused, double *out_4444331606499267924) {
   out_4444331606499267924[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4674391646420273439) {
   out_4674391646420273439[0] = 0;
   out_4674391646420273439[1] = 0;
   out_4674391646420273439[2] = 0;
   out_4674391646420273439[3] = 0;
   out_4674391646420273439[4] = 0;
   out_4674391646420273439[5] = 0;
   out_4674391646420273439[6] = 0;
   out_4674391646420273439[7] = 0;
   out_4674391646420273439[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3229655548484910964) {
  err_fun(nom_x, delta_x, out_3229655548484910964);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_154977718050279836) {
  inv_err_fun(nom_x, true_x, out_154977718050279836);
}
void car_H_mod_fun(double *state, double *out_1011263767238227610) {
  H_mod_fun(state, out_1011263767238227610);
}
void car_f_fun(double *state, double dt, double *out_5721512869151697320) {
  f_fun(state,  dt, out_5721512869151697320);
}
void car_F_fun(double *state, double dt, double *out_1692340814292537798) {
  F_fun(state,  dt, out_1692340814292537798);
}
void car_h_25(double *state, double *unused, double *out_2597451784216131708) {
  h_25(state, unused, out_2597451784216131708);
}
void car_H_25(double *state, double *unused, double *out_4643745684543313011) {
  H_25(state, unused, out_4643745684543313011);
}
void car_h_24(double *state, double *unused, double *out_5427474373712947275) {
  h_24(state, unused, out_5427474373712947275);
}
void car_H_24(double *state, double *unused, double *out_7148792931058052848) {
  H_24(state, unused, out_7148792931058052848);
}
void car_h_30(double *state, double *unused, double *out_2322257721931625819) {
  h_30(state, unused, out_2322257721931625819);
}
void car_H_30(double *state, double *unused, double *out_4514406737400072941) {
  H_30(state, unused, out_4514406737400072941);
}
void car_h_26(double *state, double *unused, double *out_7323930989592471464) {
  h_26(state, unused, out_7323930989592471464);
}
void car_H_26(double *state, double *unused, double *out_902242365669256787) {
  H_26(state, unused, out_902242365669256787);
}
void car_h_27(double *state, double *unused, double *out_3540961637117794143) {
  h_27(state, unused, out_3540961637117794143);
}
void car_H_27(double *state, double *unused, double *out_2339643425599648030) {
  H_27(state, unused, out_2339643425599648030);
}
void car_h_29(double *state, double *unused, double *out_3265767574833288254) {
  h_29(state, unused, out_3265767574833288254);
}
void car_H_29(double *state, double *unused, double *out_5024638081714465125) {
  H_29(state, unused, out_5024638081714465125);
}
void car_h_28(double *state, double *unused, double *out_4108383536193540199) {
  h_28(state, unused, out_4108383536193540199);
}
void car_H_28(double *state, double *unused, double *out_57760935355065449) {
  H_28(state, unused, out_57760935355065449);
}
void car_h_31(double *state, double *unused, double *out_4444331606499267924) {
  h_31(state, unused, out_4444331606499267924);
}
void car_H_31(double *state, double *unused, double *out_4674391646420273439) {
  H_31(state, unused, out_4674391646420273439);
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
