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
void err_fun(double *nom_x, double *delta_x, double *out_7167355006837459689) {
   out_7167355006837459689[0] = delta_x[0] + nom_x[0];
   out_7167355006837459689[1] = delta_x[1] + nom_x[1];
   out_7167355006837459689[2] = delta_x[2] + nom_x[2];
   out_7167355006837459689[3] = delta_x[3] + nom_x[3];
   out_7167355006837459689[4] = delta_x[4] + nom_x[4];
   out_7167355006837459689[5] = delta_x[5] + nom_x[5];
   out_7167355006837459689[6] = delta_x[6] + nom_x[6];
   out_7167355006837459689[7] = delta_x[7] + nom_x[7];
   out_7167355006837459689[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_171878989924026497) {
   out_171878989924026497[0] = -nom_x[0] + true_x[0];
   out_171878989924026497[1] = -nom_x[1] + true_x[1];
   out_171878989924026497[2] = -nom_x[2] + true_x[2];
   out_171878989924026497[3] = -nom_x[3] + true_x[3];
   out_171878989924026497[4] = -nom_x[4] + true_x[4];
   out_171878989924026497[5] = -nom_x[5] + true_x[5];
   out_171878989924026497[6] = -nom_x[6] + true_x[6];
   out_171878989924026497[7] = -nom_x[7] + true_x[7];
   out_171878989924026497[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2596456598414479484) {
   out_2596456598414479484[0] = 1.0;
   out_2596456598414479484[1] = 0;
   out_2596456598414479484[2] = 0;
   out_2596456598414479484[3] = 0;
   out_2596456598414479484[4] = 0;
   out_2596456598414479484[5] = 0;
   out_2596456598414479484[6] = 0;
   out_2596456598414479484[7] = 0;
   out_2596456598414479484[8] = 0;
   out_2596456598414479484[9] = 0;
   out_2596456598414479484[10] = 1.0;
   out_2596456598414479484[11] = 0;
   out_2596456598414479484[12] = 0;
   out_2596456598414479484[13] = 0;
   out_2596456598414479484[14] = 0;
   out_2596456598414479484[15] = 0;
   out_2596456598414479484[16] = 0;
   out_2596456598414479484[17] = 0;
   out_2596456598414479484[18] = 0;
   out_2596456598414479484[19] = 0;
   out_2596456598414479484[20] = 1.0;
   out_2596456598414479484[21] = 0;
   out_2596456598414479484[22] = 0;
   out_2596456598414479484[23] = 0;
   out_2596456598414479484[24] = 0;
   out_2596456598414479484[25] = 0;
   out_2596456598414479484[26] = 0;
   out_2596456598414479484[27] = 0;
   out_2596456598414479484[28] = 0;
   out_2596456598414479484[29] = 0;
   out_2596456598414479484[30] = 1.0;
   out_2596456598414479484[31] = 0;
   out_2596456598414479484[32] = 0;
   out_2596456598414479484[33] = 0;
   out_2596456598414479484[34] = 0;
   out_2596456598414479484[35] = 0;
   out_2596456598414479484[36] = 0;
   out_2596456598414479484[37] = 0;
   out_2596456598414479484[38] = 0;
   out_2596456598414479484[39] = 0;
   out_2596456598414479484[40] = 1.0;
   out_2596456598414479484[41] = 0;
   out_2596456598414479484[42] = 0;
   out_2596456598414479484[43] = 0;
   out_2596456598414479484[44] = 0;
   out_2596456598414479484[45] = 0;
   out_2596456598414479484[46] = 0;
   out_2596456598414479484[47] = 0;
   out_2596456598414479484[48] = 0;
   out_2596456598414479484[49] = 0;
   out_2596456598414479484[50] = 1.0;
   out_2596456598414479484[51] = 0;
   out_2596456598414479484[52] = 0;
   out_2596456598414479484[53] = 0;
   out_2596456598414479484[54] = 0;
   out_2596456598414479484[55] = 0;
   out_2596456598414479484[56] = 0;
   out_2596456598414479484[57] = 0;
   out_2596456598414479484[58] = 0;
   out_2596456598414479484[59] = 0;
   out_2596456598414479484[60] = 1.0;
   out_2596456598414479484[61] = 0;
   out_2596456598414479484[62] = 0;
   out_2596456598414479484[63] = 0;
   out_2596456598414479484[64] = 0;
   out_2596456598414479484[65] = 0;
   out_2596456598414479484[66] = 0;
   out_2596456598414479484[67] = 0;
   out_2596456598414479484[68] = 0;
   out_2596456598414479484[69] = 0;
   out_2596456598414479484[70] = 1.0;
   out_2596456598414479484[71] = 0;
   out_2596456598414479484[72] = 0;
   out_2596456598414479484[73] = 0;
   out_2596456598414479484[74] = 0;
   out_2596456598414479484[75] = 0;
   out_2596456598414479484[76] = 0;
   out_2596456598414479484[77] = 0;
   out_2596456598414479484[78] = 0;
   out_2596456598414479484[79] = 0;
   out_2596456598414479484[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2608799569241697690) {
   out_2608799569241697690[0] = state[0];
   out_2608799569241697690[1] = state[1];
   out_2608799569241697690[2] = state[2];
   out_2608799569241697690[3] = state[3];
   out_2608799569241697690[4] = state[4];
   out_2608799569241697690[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2608799569241697690[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2608799569241697690[7] = state[7];
   out_2608799569241697690[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2281501308483998435) {
   out_2281501308483998435[0] = 1;
   out_2281501308483998435[1] = 0;
   out_2281501308483998435[2] = 0;
   out_2281501308483998435[3] = 0;
   out_2281501308483998435[4] = 0;
   out_2281501308483998435[5] = 0;
   out_2281501308483998435[6] = 0;
   out_2281501308483998435[7] = 0;
   out_2281501308483998435[8] = 0;
   out_2281501308483998435[9] = 0;
   out_2281501308483998435[10] = 1;
   out_2281501308483998435[11] = 0;
   out_2281501308483998435[12] = 0;
   out_2281501308483998435[13] = 0;
   out_2281501308483998435[14] = 0;
   out_2281501308483998435[15] = 0;
   out_2281501308483998435[16] = 0;
   out_2281501308483998435[17] = 0;
   out_2281501308483998435[18] = 0;
   out_2281501308483998435[19] = 0;
   out_2281501308483998435[20] = 1;
   out_2281501308483998435[21] = 0;
   out_2281501308483998435[22] = 0;
   out_2281501308483998435[23] = 0;
   out_2281501308483998435[24] = 0;
   out_2281501308483998435[25] = 0;
   out_2281501308483998435[26] = 0;
   out_2281501308483998435[27] = 0;
   out_2281501308483998435[28] = 0;
   out_2281501308483998435[29] = 0;
   out_2281501308483998435[30] = 1;
   out_2281501308483998435[31] = 0;
   out_2281501308483998435[32] = 0;
   out_2281501308483998435[33] = 0;
   out_2281501308483998435[34] = 0;
   out_2281501308483998435[35] = 0;
   out_2281501308483998435[36] = 0;
   out_2281501308483998435[37] = 0;
   out_2281501308483998435[38] = 0;
   out_2281501308483998435[39] = 0;
   out_2281501308483998435[40] = 1;
   out_2281501308483998435[41] = 0;
   out_2281501308483998435[42] = 0;
   out_2281501308483998435[43] = 0;
   out_2281501308483998435[44] = 0;
   out_2281501308483998435[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2281501308483998435[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2281501308483998435[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2281501308483998435[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2281501308483998435[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2281501308483998435[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2281501308483998435[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2281501308483998435[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2281501308483998435[53] = -9.8000000000000007*dt;
   out_2281501308483998435[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2281501308483998435[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2281501308483998435[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2281501308483998435[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2281501308483998435[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2281501308483998435[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2281501308483998435[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2281501308483998435[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2281501308483998435[62] = 0;
   out_2281501308483998435[63] = 0;
   out_2281501308483998435[64] = 0;
   out_2281501308483998435[65] = 0;
   out_2281501308483998435[66] = 0;
   out_2281501308483998435[67] = 0;
   out_2281501308483998435[68] = 0;
   out_2281501308483998435[69] = 0;
   out_2281501308483998435[70] = 1;
   out_2281501308483998435[71] = 0;
   out_2281501308483998435[72] = 0;
   out_2281501308483998435[73] = 0;
   out_2281501308483998435[74] = 0;
   out_2281501308483998435[75] = 0;
   out_2281501308483998435[76] = 0;
   out_2281501308483998435[77] = 0;
   out_2281501308483998435[78] = 0;
   out_2281501308483998435[79] = 0;
   out_2281501308483998435[80] = 1;
}
void h_25(double *state, double *unused, double *out_320415058552087998) {
   out_320415058552087998[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6663260843557340531) {
   out_6663260843557340531[0] = 0;
   out_6663260843557340531[1] = 0;
   out_6663260843557340531[2] = 0;
   out_6663260843557340531[3] = 0;
   out_6663260843557340531[4] = 0;
   out_6663260843557340531[5] = 0;
   out_6663260843557340531[6] = 1;
   out_6663260843557340531[7] = 0;
   out_6663260843557340531[8] = 0;
}
void h_24(double *state, double *unused, double *out_4906346470436907167) {
   out_4906346470436907167[0] = state[4];
   out_4906346470436907167[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7508257452341035930) {
   out_7508257452341035930[0] = 0;
   out_7508257452341035930[1] = 0;
   out_7508257452341035930[2] = 0;
   out_7508257452341035930[3] = 0;
   out_7508257452341035930[4] = 1;
   out_7508257452341035930[5] = 0;
   out_7508257452341035930[6] = 0;
   out_7508257452341035930[7] = 0;
   out_7508257452341035930[8] = 0;
   out_7508257452341035930[9] = 0;
   out_7508257452341035930[10] = 0;
   out_7508257452341035930[11] = 0;
   out_7508257452341035930[12] = 0;
   out_7508257452341035930[13] = 0;
   out_7508257452341035930[14] = 1;
   out_7508257452341035930[15] = 0;
   out_7508257452341035930[16] = 0;
   out_7508257452341035930[17] = 0;
}
void h_30(double *state, double *unused, double *out_595835607152529081) {
   out_595835607152529081[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4866792888660594330) {
   out_4866792888660594330[0] = 0;
   out_4866792888660594330[1] = 0;
   out_4866792888660594330[2] = 0;
   out_4866792888660594330[3] = 0;
   out_4866792888660594330[4] = 1;
   out_4866792888660594330[5] = 0;
   out_4866792888660594330[6] = 0;
   out_4866792888660594330[7] = 0;
   out_4866792888660594330[8] = 0;
}
void h_26(double *state, double *unused, double *out_6196407366479202698) {
   out_6196407366479202698[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2921757524683284307) {
   out_2921757524683284307[0] = 0;
   out_2921757524683284307[1] = 0;
   out_2921757524683284307[2] = 0;
   out_2921757524683284307[3] = 0;
   out_2921757524683284307[4] = 0;
   out_2921757524683284307[5] = 0;
   out_2921757524683284307[6] = 0;
   out_2921757524683284307[7] = 1;
   out_2921757524683284307[8] = 0;
}
void h_27(double *state, double *unused, double *out_6078399972545825954) {
   out_6078399972545825954[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7041556200461019241) {
   out_7041556200461019241[0] = 0;
   out_7041556200461019241[1] = 0;
   out_7041556200461019241[2] = 0;
   out_7041556200461019241[3] = 1;
   out_7041556200461019241[4] = 0;
   out_7041556200461019241[5] = 0;
   out_7041556200461019241[6] = 0;
   out_7041556200461019241[7] = 0;
   out_7041556200461019241[8] = 0;
}
void h_29(double *state, double *unused, double *out_5597508874813374726) {
   out_5597508874813374726[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8754918927330570274) {
   out_8754918927330570274[0] = 0;
   out_8754918927330570274[1] = 1;
   out_8754918927330570274[2] = 0;
   out_8754918927330570274[3] = 0;
   out_8754918927330570274[4] = 0;
   out_8754918927330570274[5] = 0;
   out_8754918927330570274[6] = 0;
   out_8754918927330570274[7] = 0;
   out_8754918927330570274[8] = 0;
}
void h_28(double *state, double *unused, double *out_1076726704884980309) {
   out_1076726704884980309[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4609426129309450768) {
   out_4609426129309450768[0] = 1;
   out_4609426129309450768[1] = 0;
   out_4609426129309450768[2] = 0;
   out_4609426129309450768[3] = 0;
   out_4609426129309450768[4] = 0;
   out_4609426129309450768[5] = 0;
   out_4609426129309450768[6] = 0;
   out_4609426129309450768[7] = 0;
   out_4609426129309450768[8] = 0;
}
void h_31(double *state, double *unused, double *out_3316807983385999158) {
   out_3316807983385999158[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6693906805434300959) {
   out_6693906805434300959[0] = 0;
   out_6693906805434300959[1] = 0;
   out_6693906805434300959[2] = 0;
   out_6693906805434300959[3] = 0;
   out_6693906805434300959[4] = 0;
   out_6693906805434300959[5] = 0;
   out_6693906805434300959[6] = 0;
   out_6693906805434300959[7] = 0;
   out_6693906805434300959[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7167355006837459689) {
  err_fun(nom_x, delta_x, out_7167355006837459689);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_171878989924026497) {
  inv_err_fun(nom_x, true_x, out_171878989924026497);
}
void car_H_mod_fun(double *state, double *out_2596456598414479484) {
  H_mod_fun(state, out_2596456598414479484);
}
void car_f_fun(double *state, double dt, double *out_2608799569241697690) {
  f_fun(state,  dt, out_2608799569241697690);
}
void car_F_fun(double *state, double dt, double *out_2281501308483998435) {
  F_fun(state,  dt, out_2281501308483998435);
}
void car_h_25(double *state, double *unused, double *out_320415058552087998) {
  h_25(state, unused, out_320415058552087998);
}
void car_H_25(double *state, double *unused, double *out_6663260843557340531) {
  H_25(state, unused, out_6663260843557340531);
}
void car_h_24(double *state, double *unused, double *out_4906346470436907167) {
  h_24(state, unused, out_4906346470436907167);
}
void car_H_24(double *state, double *unused, double *out_7508257452341035930) {
  H_24(state, unused, out_7508257452341035930);
}
void car_h_30(double *state, double *unused, double *out_595835607152529081) {
  h_30(state, unused, out_595835607152529081);
}
void car_H_30(double *state, double *unused, double *out_4866792888660594330) {
  H_30(state, unused, out_4866792888660594330);
}
void car_h_26(double *state, double *unused, double *out_6196407366479202698) {
  h_26(state, unused, out_6196407366479202698);
}
void car_H_26(double *state, double *unused, double *out_2921757524683284307) {
  H_26(state, unused, out_2921757524683284307);
}
void car_h_27(double *state, double *unused, double *out_6078399972545825954) {
  h_27(state, unused, out_6078399972545825954);
}
void car_H_27(double *state, double *unused, double *out_7041556200461019241) {
  H_27(state, unused, out_7041556200461019241);
}
void car_h_29(double *state, double *unused, double *out_5597508874813374726) {
  h_29(state, unused, out_5597508874813374726);
}
void car_H_29(double *state, double *unused, double *out_8754918927330570274) {
  H_29(state, unused, out_8754918927330570274);
}
void car_h_28(double *state, double *unused, double *out_1076726704884980309) {
  h_28(state, unused, out_1076726704884980309);
}
void car_H_28(double *state, double *unused, double *out_4609426129309450768) {
  H_28(state, unused, out_4609426129309450768);
}
void car_h_31(double *state, double *unused, double *out_3316807983385999158) {
  h_31(state, unused, out_3316807983385999158);
}
void car_H_31(double *state, double *unused, double *out_6693906805434300959) {
  H_31(state, unused, out_6693906805434300959);
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
