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
void err_fun(double *nom_x, double *delta_x, double *out_3195962060965127192) {
   out_3195962060965127192[0] = delta_x[0] + nom_x[0];
   out_3195962060965127192[1] = delta_x[1] + nom_x[1];
   out_3195962060965127192[2] = delta_x[2] + nom_x[2];
   out_3195962060965127192[3] = delta_x[3] + nom_x[3];
   out_3195962060965127192[4] = delta_x[4] + nom_x[4];
   out_3195962060965127192[5] = delta_x[5] + nom_x[5];
   out_3195962060965127192[6] = delta_x[6] + nom_x[6];
   out_3195962060965127192[7] = delta_x[7] + nom_x[7];
   out_3195962060965127192[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5174661106618167489) {
   out_5174661106618167489[0] = -nom_x[0] + true_x[0];
   out_5174661106618167489[1] = -nom_x[1] + true_x[1];
   out_5174661106618167489[2] = -nom_x[2] + true_x[2];
   out_5174661106618167489[3] = -nom_x[3] + true_x[3];
   out_5174661106618167489[4] = -nom_x[4] + true_x[4];
   out_5174661106618167489[5] = -nom_x[5] + true_x[5];
   out_5174661106618167489[6] = -nom_x[6] + true_x[6];
   out_5174661106618167489[7] = -nom_x[7] + true_x[7];
   out_5174661106618167489[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6206683640939866959) {
   out_6206683640939866959[0] = 1.0;
   out_6206683640939866959[1] = 0;
   out_6206683640939866959[2] = 0;
   out_6206683640939866959[3] = 0;
   out_6206683640939866959[4] = 0;
   out_6206683640939866959[5] = 0;
   out_6206683640939866959[6] = 0;
   out_6206683640939866959[7] = 0;
   out_6206683640939866959[8] = 0;
   out_6206683640939866959[9] = 0;
   out_6206683640939866959[10] = 1.0;
   out_6206683640939866959[11] = 0;
   out_6206683640939866959[12] = 0;
   out_6206683640939866959[13] = 0;
   out_6206683640939866959[14] = 0;
   out_6206683640939866959[15] = 0;
   out_6206683640939866959[16] = 0;
   out_6206683640939866959[17] = 0;
   out_6206683640939866959[18] = 0;
   out_6206683640939866959[19] = 0;
   out_6206683640939866959[20] = 1.0;
   out_6206683640939866959[21] = 0;
   out_6206683640939866959[22] = 0;
   out_6206683640939866959[23] = 0;
   out_6206683640939866959[24] = 0;
   out_6206683640939866959[25] = 0;
   out_6206683640939866959[26] = 0;
   out_6206683640939866959[27] = 0;
   out_6206683640939866959[28] = 0;
   out_6206683640939866959[29] = 0;
   out_6206683640939866959[30] = 1.0;
   out_6206683640939866959[31] = 0;
   out_6206683640939866959[32] = 0;
   out_6206683640939866959[33] = 0;
   out_6206683640939866959[34] = 0;
   out_6206683640939866959[35] = 0;
   out_6206683640939866959[36] = 0;
   out_6206683640939866959[37] = 0;
   out_6206683640939866959[38] = 0;
   out_6206683640939866959[39] = 0;
   out_6206683640939866959[40] = 1.0;
   out_6206683640939866959[41] = 0;
   out_6206683640939866959[42] = 0;
   out_6206683640939866959[43] = 0;
   out_6206683640939866959[44] = 0;
   out_6206683640939866959[45] = 0;
   out_6206683640939866959[46] = 0;
   out_6206683640939866959[47] = 0;
   out_6206683640939866959[48] = 0;
   out_6206683640939866959[49] = 0;
   out_6206683640939866959[50] = 1.0;
   out_6206683640939866959[51] = 0;
   out_6206683640939866959[52] = 0;
   out_6206683640939866959[53] = 0;
   out_6206683640939866959[54] = 0;
   out_6206683640939866959[55] = 0;
   out_6206683640939866959[56] = 0;
   out_6206683640939866959[57] = 0;
   out_6206683640939866959[58] = 0;
   out_6206683640939866959[59] = 0;
   out_6206683640939866959[60] = 1.0;
   out_6206683640939866959[61] = 0;
   out_6206683640939866959[62] = 0;
   out_6206683640939866959[63] = 0;
   out_6206683640939866959[64] = 0;
   out_6206683640939866959[65] = 0;
   out_6206683640939866959[66] = 0;
   out_6206683640939866959[67] = 0;
   out_6206683640939866959[68] = 0;
   out_6206683640939866959[69] = 0;
   out_6206683640939866959[70] = 1.0;
   out_6206683640939866959[71] = 0;
   out_6206683640939866959[72] = 0;
   out_6206683640939866959[73] = 0;
   out_6206683640939866959[74] = 0;
   out_6206683640939866959[75] = 0;
   out_6206683640939866959[76] = 0;
   out_6206683640939866959[77] = 0;
   out_6206683640939866959[78] = 0;
   out_6206683640939866959[79] = 0;
   out_6206683640939866959[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_256968889868331023) {
   out_256968889868331023[0] = state[0];
   out_256968889868331023[1] = state[1];
   out_256968889868331023[2] = state[2];
   out_256968889868331023[3] = state[3];
   out_256968889868331023[4] = state[4];
   out_256968889868331023[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_256968889868331023[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_256968889868331023[7] = state[7];
   out_256968889868331023[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4251600485297607562) {
   out_4251600485297607562[0] = 1;
   out_4251600485297607562[1] = 0;
   out_4251600485297607562[2] = 0;
   out_4251600485297607562[3] = 0;
   out_4251600485297607562[4] = 0;
   out_4251600485297607562[5] = 0;
   out_4251600485297607562[6] = 0;
   out_4251600485297607562[7] = 0;
   out_4251600485297607562[8] = 0;
   out_4251600485297607562[9] = 0;
   out_4251600485297607562[10] = 1;
   out_4251600485297607562[11] = 0;
   out_4251600485297607562[12] = 0;
   out_4251600485297607562[13] = 0;
   out_4251600485297607562[14] = 0;
   out_4251600485297607562[15] = 0;
   out_4251600485297607562[16] = 0;
   out_4251600485297607562[17] = 0;
   out_4251600485297607562[18] = 0;
   out_4251600485297607562[19] = 0;
   out_4251600485297607562[20] = 1;
   out_4251600485297607562[21] = 0;
   out_4251600485297607562[22] = 0;
   out_4251600485297607562[23] = 0;
   out_4251600485297607562[24] = 0;
   out_4251600485297607562[25] = 0;
   out_4251600485297607562[26] = 0;
   out_4251600485297607562[27] = 0;
   out_4251600485297607562[28] = 0;
   out_4251600485297607562[29] = 0;
   out_4251600485297607562[30] = 1;
   out_4251600485297607562[31] = 0;
   out_4251600485297607562[32] = 0;
   out_4251600485297607562[33] = 0;
   out_4251600485297607562[34] = 0;
   out_4251600485297607562[35] = 0;
   out_4251600485297607562[36] = 0;
   out_4251600485297607562[37] = 0;
   out_4251600485297607562[38] = 0;
   out_4251600485297607562[39] = 0;
   out_4251600485297607562[40] = 1;
   out_4251600485297607562[41] = 0;
   out_4251600485297607562[42] = 0;
   out_4251600485297607562[43] = 0;
   out_4251600485297607562[44] = 0;
   out_4251600485297607562[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4251600485297607562[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4251600485297607562[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4251600485297607562[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4251600485297607562[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4251600485297607562[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4251600485297607562[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4251600485297607562[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4251600485297607562[53] = -9.8000000000000007*dt;
   out_4251600485297607562[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4251600485297607562[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4251600485297607562[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4251600485297607562[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4251600485297607562[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4251600485297607562[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4251600485297607562[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4251600485297607562[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4251600485297607562[62] = 0;
   out_4251600485297607562[63] = 0;
   out_4251600485297607562[64] = 0;
   out_4251600485297607562[65] = 0;
   out_4251600485297607562[66] = 0;
   out_4251600485297607562[67] = 0;
   out_4251600485297607562[68] = 0;
   out_4251600485297607562[69] = 0;
   out_4251600485297607562[70] = 1;
   out_4251600485297607562[71] = 0;
   out_4251600485297607562[72] = 0;
   out_4251600485297607562[73] = 0;
   out_4251600485297607562[74] = 0;
   out_4251600485297607562[75] = 0;
   out_4251600485297607562[76] = 0;
   out_4251600485297607562[77] = 0;
   out_4251600485297607562[78] = 0;
   out_4251600485297607562[79] = 0;
   out_4251600485297607562[80] = 1;
}
void h_25(double *state, double *unused, double *out_1546507495354430623) {
   out_1546507495354430623[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6942253115235371962) {
   out_6942253115235371962[0] = 0;
   out_6942253115235371962[1] = 0;
   out_6942253115235371962[2] = 0;
   out_6942253115235371962[3] = 0;
   out_6942253115235371962[4] = 0;
   out_6942253115235371962[5] = 0;
   out_6942253115235371962[6] = 1;
   out_6942253115235371962[7] = 0;
   out_6942253115235371962[8] = 0;
}
void h_24(double *state, double *unused, double *out_962415991559381739) {
   out_962415991559381739[0] = state[4];
   out_962415991559381739[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4716545331256503400) {
   out_4716545331256503400[0] = 0;
   out_4716545331256503400[1] = 0;
   out_4716545331256503400[2] = 0;
   out_4716545331256503400[3] = 0;
   out_4716545331256503400[4] = 1;
   out_4716545331256503400[5] = 0;
   out_4716545331256503400[6] = 0;
   out_4716545331256503400[7] = 0;
   out_4716545331256503400[8] = 0;
   out_4716545331256503400[9] = 0;
   out_4716545331256503400[10] = 0;
   out_4716545331256503400[11] = 0;
   out_4716545331256503400[12] = 0;
   out_4716545331256503400[13] = 0;
   out_4716545331256503400[14] = 1;
   out_4716545331256503400[15] = 0;
   out_4716545331256503400[16] = 0;
   out_4716545331256503400[17] = 0;
}
void h_30(double *state, double *unused, double *out_3061883139040810868) {
   out_3061883139040810868[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7071592062378612032) {
   out_7071592062378612032[0] = 0;
   out_7071592062378612032[1] = 0;
   out_7071592062378612032[2] = 0;
   out_7071592062378612032[3] = 0;
   out_7071592062378612032[4] = 1;
   out_7071592062378612032[5] = 0;
   out_7071592062378612032[6] = 0;
   out_7071592062378612032[7] = 0;
   out_7071592062378612032[8] = 0;
}
void h_26(double *state, double *unused, double *out_8063556406701656513) {
   out_8063556406701656513[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7762987639600123430) {
   out_7762987639600123430[0] = 0;
   out_7762987639600123430[1] = 0;
   out_7762987639600123430[2] = 0;
   out_7762987639600123430[3] = 0;
   out_7762987639600123430[4] = 0;
   out_7762987639600123430[5] = 0;
   out_7762987639600123430[6] = 0;
   out_7762987639600123430[7] = 1;
   out_7762987639600123430[8] = 0;
}
void h_27(double *state, double *unused, double *out_7945549012768279769) {
   out_7945549012768279769[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9200388699530514673) {
   out_9200388699530514673[0] = 0;
   out_9200388699530514673[1] = 0;
   out_9200388699530514673[2] = 0;
   out_9200388699530514673[3] = 1;
   out_9200388699530514673[4] = 0;
   out_9200388699530514673[5] = 0;
   out_9200388699530514673[6] = 0;
   out_9200388699530514673[7] = 0;
   out_9200388699530514673[8] = 0;
}
void h_29(double *state, double *unused, double *out_8403750374926841431) {
   out_8403750374926841431[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6561360718064219848) {
   out_6561360718064219848[0] = 0;
   out_6561360718064219848[1] = 1;
   out_6561360718064219848[2] = 0;
   out_6561360718064219848[3] = 0;
   out_6561360718064219848[4] = 0;
   out_6561360718064219848[5] = 0;
   out_6561360718064219848[6] = 0;
   out_6561360718064219848[7] = 0;
   out_6561360718064219848[8] = 0;
}
void h_28(double *state, double *unused, double *out_2943875745107434124) {
   out_2943875745107434124[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4597730446498893597) {
   out_4597730446498893597[0] = 1;
   out_4597730446498893597[1] = 0;
   out_4597730446498893597[2] = 0;
   out_4597730446498893597[3] = 0;
   out_4597730446498893597[4] = 0;
   out_4597730446498893597[5] = 0;
   out_4597730446498893597[6] = 0;
   out_4597730446498893597[7] = 0;
   out_4597730446498893597[8] = 0;
}
void h_31(double *state, double *unused, double *out_5183957023608452973) {
   out_5183957023608452973[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6911607153358411534) {
   out_6911607153358411534[0] = 0;
   out_6911607153358411534[1] = 0;
   out_6911607153358411534[2] = 0;
   out_6911607153358411534[3] = 0;
   out_6911607153358411534[4] = 0;
   out_6911607153358411534[5] = 0;
   out_6911607153358411534[6] = 0;
   out_6911607153358411534[7] = 0;
   out_6911607153358411534[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3195962060965127192) {
  err_fun(nom_x, delta_x, out_3195962060965127192);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5174661106618167489) {
  inv_err_fun(nom_x, true_x, out_5174661106618167489);
}
void car_H_mod_fun(double *state, double *out_6206683640939866959) {
  H_mod_fun(state, out_6206683640939866959);
}
void car_f_fun(double *state, double dt, double *out_256968889868331023) {
  f_fun(state,  dt, out_256968889868331023);
}
void car_F_fun(double *state, double dt, double *out_4251600485297607562) {
  F_fun(state,  dt, out_4251600485297607562);
}
void car_h_25(double *state, double *unused, double *out_1546507495354430623) {
  h_25(state, unused, out_1546507495354430623);
}
void car_H_25(double *state, double *unused, double *out_6942253115235371962) {
  H_25(state, unused, out_6942253115235371962);
}
void car_h_24(double *state, double *unused, double *out_962415991559381739) {
  h_24(state, unused, out_962415991559381739);
}
void car_H_24(double *state, double *unused, double *out_4716545331256503400) {
  H_24(state, unused, out_4716545331256503400);
}
void car_h_30(double *state, double *unused, double *out_3061883139040810868) {
  h_30(state, unused, out_3061883139040810868);
}
void car_H_30(double *state, double *unused, double *out_7071592062378612032) {
  H_30(state, unused, out_7071592062378612032);
}
void car_h_26(double *state, double *unused, double *out_8063556406701656513) {
  h_26(state, unused, out_8063556406701656513);
}
void car_H_26(double *state, double *unused, double *out_7762987639600123430) {
  H_26(state, unused, out_7762987639600123430);
}
void car_h_27(double *state, double *unused, double *out_7945549012768279769) {
  h_27(state, unused, out_7945549012768279769);
}
void car_H_27(double *state, double *unused, double *out_9200388699530514673) {
  H_27(state, unused, out_9200388699530514673);
}
void car_h_29(double *state, double *unused, double *out_8403750374926841431) {
  h_29(state, unused, out_8403750374926841431);
}
void car_H_29(double *state, double *unused, double *out_6561360718064219848) {
  H_29(state, unused, out_6561360718064219848);
}
void car_h_28(double *state, double *unused, double *out_2943875745107434124) {
  h_28(state, unused, out_2943875745107434124);
}
void car_H_28(double *state, double *unused, double *out_4597730446498893597) {
  H_28(state, unused, out_4597730446498893597);
}
void car_h_31(double *state, double *unused, double *out_5183957023608452973) {
  h_31(state, unused, out_5183957023608452973);
}
void car_H_31(double *state, double *unused, double *out_6911607153358411534) {
  H_31(state, unused, out_6911607153358411534);
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
