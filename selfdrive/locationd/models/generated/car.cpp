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
void err_fun(double *nom_x, double *delta_x, double *out_2574937435638925744) {
   out_2574937435638925744[0] = delta_x[0] + nom_x[0];
   out_2574937435638925744[1] = delta_x[1] + nom_x[1];
   out_2574937435638925744[2] = delta_x[2] + nom_x[2];
   out_2574937435638925744[3] = delta_x[3] + nom_x[3];
   out_2574937435638925744[4] = delta_x[4] + nom_x[4];
   out_2574937435638925744[5] = delta_x[5] + nom_x[5];
   out_2574937435638925744[6] = delta_x[6] + nom_x[6];
   out_2574937435638925744[7] = delta_x[7] + nom_x[7];
   out_2574937435638925744[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4900970792020499484) {
   out_4900970792020499484[0] = -nom_x[0] + true_x[0];
   out_4900970792020499484[1] = -nom_x[1] + true_x[1];
   out_4900970792020499484[2] = -nom_x[2] + true_x[2];
   out_4900970792020499484[3] = -nom_x[3] + true_x[3];
   out_4900970792020499484[4] = -nom_x[4] + true_x[4];
   out_4900970792020499484[5] = -nom_x[5] + true_x[5];
   out_4900970792020499484[6] = -nom_x[6] + true_x[6];
   out_4900970792020499484[7] = -nom_x[7] + true_x[7];
   out_4900970792020499484[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8451084579596259389) {
   out_8451084579596259389[0] = 1.0;
   out_8451084579596259389[1] = 0;
   out_8451084579596259389[2] = 0;
   out_8451084579596259389[3] = 0;
   out_8451084579596259389[4] = 0;
   out_8451084579596259389[5] = 0;
   out_8451084579596259389[6] = 0;
   out_8451084579596259389[7] = 0;
   out_8451084579596259389[8] = 0;
   out_8451084579596259389[9] = 0;
   out_8451084579596259389[10] = 1.0;
   out_8451084579596259389[11] = 0;
   out_8451084579596259389[12] = 0;
   out_8451084579596259389[13] = 0;
   out_8451084579596259389[14] = 0;
   out_8451084579596259389[15] = 0;
   out_8451084579596259389[16] = 0;
   out_8451084579596259389[17] = 0;
   out_8451084579596259389[18] = 0;
   out_8451084579596259389[19] = 0;
   out_8451084579596259389[20] = 1.0;
   out_8451084579596259389[21] = 0;
   out_8451084579596259389[22] = 0;
   out_8451084579596259389[23] = 0;
   out_8451084579596259389[24] = 0;
   out_8451084579596259389[25] = 0;
   out_8451084579596259389[26] = 0;
   out_8451084579596259389[27] = 0;
   out_8451084579596259389[28] = 0;
   out_8451084579596259389[29] = 0;
   out_8451084579596259389[30] = 1.0;
   out_8451084579596259389[31] = 0;
   out_8451084579596259389[32] = 0;
   out_8451084579596259389[33] = 0;
   out_8451084579596259389[34] = 0;
   out_8451084579596259389[35] = 0;
   out_8451084579596259389[36] = 0;
   out_8451084579596259389[37] = 0;
   out_8451084579596259389[38] = 0;
   out_8451084579596259389[39] = 0;
   out_8451084579596259389[40] = 1.0;
   out_8451084579596259389[41] = 0;
   out_8451084579596259389[42] = 0;
   out_8451084579596259389[43] = 0;
   out_8451084579596259389[44] = 0;
   out_8451084579596259389[45] = 0;
   out_8451084579596259389[46] = 0;
   out_8451084579596259389[47] = 0;
   out_8451084579596259389[48] = 0;
   out_8451084579596259389[49] = 0;
   out_8451084579596259389[50] = 1.0;
   out_8451084579596259389[51] = 0;
   out_8451084579596259389[52] = 0;
   out_8451084579596259389[53] = 0;
   out_8451084579596259389[54] = 0;
   out_8451084579596259389[55] = 0;
   out_8451084579596259389[56] = 0;
   out_8451084579596259389[57] = 0;
   out_8451084579596259389[58] = 0;
   out_8451084579596259389[59] = 0;
   out_8451084579596259389[60] = 1.0;
   out_8451084579596259389[61] = 0;
   out_8451084579596259389[62] = 0;
   out_8451084579596259389[63] = 0;
   out_8451084579596259389[64] = 0;
   out_8451084579596259389[65] = 0;
   out_8451084579596259389[66] = 0;
   out_8451084579596259389[67] = 0;
   out_8451084579596259389[68] = 0;
   out_8451084579596259389[69] = 0;
   out_8451084579596259389[70] = 1.0;
   out_8451084579596259389[71] = 0;
   out_8451084579596259389[72] = 0;
   out_8451084579596259389[73] = 0;
   out_8451084579596259389[74] = 0;
   out_8451084579596259389[75] = 0;
   out_8451084579596259389[76] = 0;
   out_8451084579596259389[77] = 0;
   out_8451084579596259389[78] = 0;
   out_8451084579596259389[79] = 0;
   out_8451084579596259389[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7474040492891634637) {
   out_7474040492891634637[0] = state[0];
   out_7474040492891634637[1] = state[1];
   out_7474040492891634637[2] = state[2];
   out_7474040492891634637[3] = state[3];
   out_7474040492891634637[4] = state[4];
   out_7474040492891634637[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7474040492891634637[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7474040492891634637[7] = state[7];
   out_7474040492891634637[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3691743665564862162) {
   out_3691743665564862162[0] = 1;
   out_3691743665564862162[1] = 0;
   out_3691743665564862162[2] = 0;
   out_3691743665564862162[3] = 0;
   out_3691743665564862162[4] = 0;
   out_3691743665564862162[5] = 0;
   out_3691743665564862162[6] = 0;
   out_3691743665564862162[7] = 0;
   out_3691743665564862162[8] = 0;
   out_3691743665564862162[9] = 0;
   out_3691743665564862162[10] = 1;
   out_3691743665564862162[11] = 0;
   out_3691743665564862162[12] = 0;
   out_3691743665564862162[13] = 0;
   out_3691743665564862162[14] = 0;
   out_3691743665564862162[15] = 0;
   out_3691743665564862162[16] = 0;
   out_3691743665564862162[17] = 0;
   out_3691743665564862162[18] = 0;
   out_3691743665564862162[19] = 0;
   out_3691743665564862162[20] = 1;
   out_3691743665564862162[21] = 0;
   out_3691743665564862162[22] = 0;
   out_3691743665564862162[23] = 0;
   out_3691743665564862162[24] = 0;
   out_3691743665564862162[25] = 0;
   out_3691743665564862162[26] = 0;
   out_3691743665564862162[27] = 0;
   out_3691743665564862162[28] = 0;
   out_3691743665564862162[29] = 0;
   out_3691743665564862162[30] = 1;
   out_3691743665564862162[31] = 0;
   out_3691743665564862162[32] = 0;
   out_3691743665564862162[33] = 0;
   out_3691743665564862162[34] = 0;
   out_3691743665564862162[35] = 0;
   out_3691743665564862162[36] = 0;
   out_3691743665564862162[37] = 0;
   out_3691743665564862162[38] = 0;
   out_3691743665564862162[39] = 0;
   out_3691743665564862162[40] = 1;
   out_3691743665564862162[41] = 0;
   out_3691743665564862162[42] = 0;
   out_3691743665564862162[43] = 0;
   out_3691743665564862162[44] = 0;
   out_3691743665564862162[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3691743665564862162[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3691743665564862162[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3691743665564862162[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3691743665564862162[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3691743665564862162[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3691743665564862162[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3691743665564862162[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3691743665564862162[53] = -9.8000000000000007*dt;
   out_3691743665564862162[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3691743665564862162[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3691743665564862162[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3691743665564862162[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3691743665564862162[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3691743665564862162[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3691743665564862162[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3691743665564862162[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3691743665564862162[62] = 0;
   out_3691743665564862162[63] = 0;
   out_3691743665564862162[64] = 0;
   out_3691743665564862162[65] = 0;
   out_3691743665564862162[66] = 0;
   out_3691743665564862162[67] = 0;
   out_3691743665564862162[68] = 0;
   out_3691743665564862162[69] = 0;
   out_3691743665564862162[70] = 1;
   out_3691743665564862162[71] = 0;
   out_3691743665564862162[72] = 0;
   out_3691743665564862162[73] = 0;
   out_3691743665564862162[74] = 0;
   out_3691743665564862162[75] = 0;
   out_3691743665564862162[76] = 0;
   out_3691743665564862162[77] = 0;
   out_3691743665564862162[78] = 0;
   out_3691743665564862162[79] = 0;
   out_3691743665564862162[80] = 1;
}
void h_25(double *state, double *unused, double *out_928977834463644485) {
   out_928977834463644485[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2178384050109672028) {
   out_2178384050109672028[0] = 0;
   out_2178384050109672028[1] = 0;
   out_2178384050109672028[2] = 0;
   out_2178384050109672028[3] = 0;
   out_2178384050109672028[4] = 0;
   out_2178384050109672028[5] = 0;
   out_2178384050109672028[6] = 1;
   out_2178384050109672028[7] = 0;
   out_2178384050109672028[8] = 0;
}
void h_24(double *state, double *unused, double *out_4056782944777882268) {
   out_4056782944777882268[0] = state[4];
   out_4056782944777882268[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5736309648911995164) {
   out_5736309648911995164[0] = 0;
   out_5736309648911995164[1] = 0;
   out_5736309648911995164[2] = 0;
   out_5736309648911995164[3] = 0;
   out_5736309648911995164[4] = 1;
   out_5736309648911995164[5] = 0;
   out_5736309648911995164[6] = 0;
   out_5736309648911995164[7] = 0;
   out_5736309648911995164[8] = 0;
   out_5736309648911995164[9] = 0;
   out_5736309648911995164[10] = 0;
   out_5736309648911995164[11] = 0;
   out_5736309648911995164[12] = 0;
   out_5736309648911995164[13] = 0;
   out_5736309648911995164[14] = 1;
   out_5736309648911995164[15] = 0;
   out_5736309648911995164[16] = 0;
   out_5736309648911995164[17] = 0;
}
void h_30(double *state, double *unused, double *out_4566427362717666835) {
   out_4566427362717666835[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2049045102966431958) {
   out_2049045102966431958[0] = 0;
   out_2049045102966431958[1] = 0;
   out_2049045102966431958[2] = 0;
   out_2049045102966431958[3] = 0;
   out_2049045102966431958[4] = 1;
   out_2049045102966431958[5] = 0;
   out_2049045102966431958[6] = 0;
   out_2049045102966431958[7] = 0;
   out_2049045102966431958[8] = 0;
}
void h_26(double *state, double *unused, double *out_6689255928088982829) {
   out_6689255928088982829[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1563119268764384196) {
   out_1563119268764384196[0] = 0;
   out_1563119268764384196[1] = 0;
   out_1563119268764384196[2] = 0;
   out_1563119268764384196[3] = 0;
   out_1563119268764384196[4] = 0;
   out_1563119268764384196[5] = 0;
   out_1563119268764384196[6] = 0;
   out_1563119268764384196[7] = 1;
   out_1563119268764384196[8] = 0;
}
void h_27(double *state, double *unused, double *out_6807263322022359573) {
   out_6807263322022359573[0] = state[3];
}
void H_27(double *state, double *unused, double *out_125718208833992953) {
   out_125718208833992953[0] = 0;
   out_125718208833992953[1] = 0;
   out_125718208833992953[2] = 0;
   out_125718208833992953[3] = 1;
   out_125718208833992953[4] = 0;
   out_125718208833992953[5] = 0;
   out_125718208833992953[6] = 0;
   out_125718208833992953[7] = 0;
   out_125718208833992953[8] = 0;
}
void h_29(double *state, double *unused, double *out_1832614154696182311) {
   out_1832614154696182311[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2559276447280824142) {
   out_2559276447280824142[0] = 0;
   out_2559276447280824142[1] = 1;
   out_2559276447280824142[2] = 0;
   out_2559276447280824142[3] = 0;
   out_2559276447280824142[4] = 0;
   out_2559276447280824142[5] = 0;
   out_2559276447280824142[6] = 0;
   out_2559276447280824142[7] = 0;
   out_2559276447280824142[8] = 0;
}
void h_28(double *state, double *unused, double *out_3051318069882350635) {
   out_3051318069882350635[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2523122569788706432) {
   out_2523122569788706432[0] = 1;
   out_2523122569788706432[1] = 0;
   out_2523122569788706432[2] = 0;
   out_2523122569788706432[3] = 0;
   out_2523122569788706432[4] = 0;
   out_2523122569788706432[5] = 0;
   out_2523122569788706432[6] = 0;
   out_2523122569788706432[7] = 0;
   out_2523122569788706432[8] = 0;
}
void h_31(double *state, double *unused, double *out_1530395992077008039) {
   out_1530395992077008039[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2209030011986632456) {
   out_2209030011986632456[0] = 0;
   out_2209030011986632456[1] = 0;
   out_2209030011986632456[2] = 0;
   out_2209030011986632456[3] = 0;
   out_2209030011986632456[4] = 0;
   out_2209030011986632456[5] = 0;
   out_2209030011986632456[6] = 0;
   out_2209030011986632456[7] = 0;
   out_2209030011986632456[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2574937435638925744) {
  err_fun(nom_x, delta_x, out_2574937435638925744);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4900970792020499484) {
  inv_err_fun(nom_x, true_x, out_4900970792020499484);
}
void car_H_mod_fun(double *state, double *out_8451084579596259389) {
  H_mod_fun(state, out_8451084579596259389);
}
void car_f_fun(double *state, double dt, double *out_7474040492891634637) {
  f_fun(state,  dt, out_7474040492891634637);
}
void car_F_fun(double *state, double dt, double *out_3691743665564862162) {
  F_fun(state,  dt, out_3691743665564862162);
}
void car_h_25(double *state, double *unused, double *out_928977834463644485) {
  h_25(state, unused, out_928977834463644485);
}
void car_H_25(double *state, double *unused, double *out_2178384050109672028) {
  H_25(state, unused, out_2178384050109672028);
}
void car_h_24(double *state, double *unused, double *out_4056782944777882268) {
  h_24(state, unused, out_4056782944777882268);
}
void car_H_24(double *state, double *unused, double *out_5736309648911995164) {
  H_24(state, unused, out_5736309648911995164);
}
void car_h_30(double *state, double *unused, double *out_4566427362717666835) {
  h_30(state, unused, out_4566427362717666835);
}
void car_H_30(double *state, double *unused, double *out_2049045102966431958) {
  H_30(state, unused, out_2049045102966431958);
}
void car_h_26(double *state, double *unused, double *out_6689255928088982829) {
  h_26(state, unused, out_6689255928088982829);
}
void car_H_26(double *state, double *unused, double *out_1563119268764384196) {
  H_26(state, unused, out_1563119268764384196);
}
void car_h_27(double *state, double *unused, double *out_6807263322022359573) {
  h_27(state, unused, out_6807263322022359573);
}
void car_H_27(double *state, double *unused, double *out_125718208833992953) {
  H_27(state, unused, out_125718208833992953);
}
void car_h_29(double *state, double *unused, double *out_1832614154696182311) {
  h_29(state, unused, out_1832614154696182311);
}
void car_H_29(double *state, double *unused, double *out_2559276447280824142) {
  H_29(state, unused, out_2559276447280824142);
}
void car_h_28(double *state, double *unused, double *out_3051318069882350635) {
  h_28(state, unused, out_3051318069882350635);
}
void car_H_28(double *state, double *unused, double *out_2523122569788706432) {
  H_28(state, unused, out_2523122569788706432);
}
void car_h_31(double *state, double *unused, double *out_1530395992077008039) {
  h_31(state, unused, out_1530395992077008039);
}
void car_H_31(double *state, double *unused, double *out_2209030011986632456) {
  H_31(state, unused, out_2209030011986632456);
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
