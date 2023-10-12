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
void err_fun(double *nom_x, double *delta_x, double *out_7739919700438188629) {
   out_7739919700438188629[0] = delta_x[0] + nom_x[0];
   out_7739919700438188629[1] = delta_x[1] + nom_x[1];
   out_7739919700438188629[2] = delta_x[2] + nom_x[2];
   out_7739919700438188629[3] = delta_x[3] + nom_x[3];
   out_7739919700438188629[4] = delta_x[4] + nom_x[4];
   out_7739919700438188629[5] = delta_x[5] + nom_x[5];
   out_7739919700438188629[6] = delta_x[6] + nom_x[6];
   out_7739919700438188629[7] = delta_x[7] + nom_x[7];
   out_7739919700438188629[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6319375409011776283) {
   out_6319375409011776283[0] = -nom_x[0] + true_x[0];
   out_6319375409011776283[1] = -nom_x[1] + true_x[1];
   out_6319375409011776283[2] = -nom_x[2] + true_x[2];
   out_6319375409011776283[3] = -nom_x[3] + true_x[3];
   out_6319375409011776283[4] = -nom_x[4] + true_x[4];
   out_6319375409011776283[5] = -nom_x[5] + true_x[5];
   out_6319375409011776283[6] = -nom_x[6] + true_x[6];
   out_6319375409011776283[7] = -nom_x[7] + true_x[7];
   out_6319375409011776283[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2844378829659981252) {
   out_2844378829659981252[0] = 1.0;
   out_2844378829659981252[1] = 0;
   out_2844378829659981252[2] = 0;
   out_2844378829659981252[3] = 0;
   out_2844378829659981252[4] = 0;
   out_2844378829659981252[5] = 0;
   out_2844378829659981252[6] = 0;
   out_2844378829659981252[7] = 0;
   out_2844378829659981252[8] = 0;
   out_2844378829659981252[9] = 0;
   out_2844378829659981252[10] = 1.0;
   out_2844378829659981252[11] = 0;
   out_2844378829659981252[12] = 0;
   out_2844378829659981252[13] = 0;
   out_2844378829659981252[14] = 0;
   out_2844378829659981252[15] = 0;
   out_2844378829659981252[16] = 0;
   out_2844378829659981252[17] = 0;
   out_2844378829659981252[18] = 0;
   out_2844378829659981252[19] = 0;
   out_2844378829659981252[20] = 1.0;
   out_2844378829659981252[21] = 0;
   out_2844378829659981252[22] = 0;
   out_2844378829659981252[23] = 0;
   out_2844378829659981252[24] = 0;
   out_2844378829659981252[25] = 0;
   out_2844378829659981252[26] = 0;
   out_2844378829659981252[27] = 0;
   out_2844378829659981252[28] = 0;
   out_2844378829659981252[29] = 0;
   out_2844378829659981252[30] = 1.0;
   out_2844378829659981252[31] = 0;
   out_2844378829659981252[32] = 0;
   out_2844378829659981252[33] = 0;
   out_2844378829659981252[34] = 0;
   out_2844378829659981252[35] = 0;
   out_2844378829659981252[36] = 0;
   out_2844378829659981252[37] = 0;
   out_2844378829659981252[38] = 0;
   out_2844378829659981252[39] = 0;
   out_2844378829659981252[40] = 1.0;
   out_2844378829659981252[41] = 0;
   out_2844378829659981252[42] = 0;
   out_2844378829659981252[43] = 0;
   out_2844378829659981252[44] = 0;
   out_2844378829659981252[45] = 0;
   out_2844378829659981252[46] = 0;
   out_2844378829659981252[47] = 0;
   out_2844378829659981252[48] = 0;
   out_2844378829659981252[49] = 0;
   out_2844378829659981252[50] = 1.0;
   out_2844378829659981252[51] = 0;
   out_2844378829659981252[52] = 0;
   out_2844378829659981252[53] = 0;
   out_2844378829659981252[54] = 0;
   out_2844378829659981252[55] = 0;
   out_2844378829659981252[56] = 0;
   out_2844378829659981252[57] = 0;
   out_2844378829659981252[58] = 0;
   out_2844378829659981252[59] = 0;
   out_2844378829659981252[60] = 1.0;
   out_2844378829659981252[61] = 0;
   out_2844378829659981252[62] = 0;
   out_2844378829659981252[63] = 0;
   out_2844378829659981252[64] = 0;
   out_2844378829659981252[65] = 0;
   out_2844378829659981252[66] = 0;
   out_2844378829659981252[67] = 0;
   out_2844378829659981252[68] = 0;
   out_2844378829659981252[69] = 0;
   out_2844378829659981252[70] = 1.0;
   out_2844378829659981252[71] = 0;
   out_2844378829659981252[72] = 0;
   out_2844378829659981252[73] = 0;
   out_2844378829659981252[74] = 0;
   out_2844378829659981252[75] = 0;
   out_2844378829659981252[76] = 0;
   out_2844378829659981252[77] = 0;
   out_2844378829659981252[78] = 0;
   out_2844378829659981252[79] = 0;
   out_2844378829659981252[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4884965143351282392) {
   out_4884965143351282392[0] = state[0];
   out_4884965143351282392[1] = state[1];
   out_4884965143351282392[2] = state[2];
   out_4884965143351282392[3] = state[3];
   out_4884965143351282392[4] = state[4];
   out_4884965143351282392[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4884965143351282392[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4884965143351282392[7] = state[7];
   out_4884965143351282392[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6963307994085528976) {
   out_6963307994085528976[0] = 1;
   out_6963307994085528976[1] = 0;
   out_6963307994085528976[2] = 0;
   out_6963307994085528976[3] = 0;
   out_6963307994085528976[4] = 0;
   out_6963307994085528976[5] = 0;
   out_6963307994085528976[6] = 0;
   out_6963307994085528976[7] = 0;
   out_6963307994085528976[8] = 0;
   out_6963307994085528976[9] = 0;
   out_6963307994085528976[10] = 1;
   out_6963307994085528976[11] = 0;
   out_6963307994085528976[12] = 0;
   out_6963307994085528976[13] = 0;
   out_6963307994085528976[14] = 0;
   out_6963307994085528976[15] = 0;
   out_6963307994085528976[16] = 0;
   out_6963307994085528976[17] = 0;
   out_6963307994085528976[18] = 0;
   out_6963307994085528976[19] = 0;
   out_6963307994085528976[20] = 1;
   out_6963307994085528976[21] = 0;
   out_6963307994085528976[22] = 0;
   out_6963307994085528976[23] = 0;
   out_6963307994085528976[24] = 0;
   out_6963307994085528976[25] = 0;
   out_6963307994085528976[26] = 0;
   out_6963307994085528976[27] = 0;
   out_6963307994085528976[28] = 0;
   out_6963307994085528976[29] = 0;
   out_6963307994085528976[30] = 1;
   out_6963307994085528976[31] = 0;
   out_6963307994085528976[32] = 0;
   out_6963307994085528976[33] = 0;
   out_6963307994085528976[34] = 0;
   out_6963307994085528976[35] = 0;
   out_6963307994085528976[36] = 0;
   out_6963307994085528976[37] = 0;
   out_6963307994085528976[38] = 0;
   out_6963307994085528976[39] = 0;
   out_6963307994085528976[40] = 1;
   out_6963307994085528976[41] = 0;
   out_6963307994085528976[42] = 0;
   out_6963307994085528976[43] = 0;
   out_6963307994085528976[44] = 0;
   out_6963307994085528976[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6963307994085528976[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6963307994085528976[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6963307994085528976[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6963307994085528976[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6963307994085528976[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6963307994085528976[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6963307994085528976[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6963307994085528976[53] = -9.8000000000000007*dt;
   out_6963307994085528976[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6963307994085528976[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6963307994085528976[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6963307994085528976[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6963307994085528976[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6963307994085528976[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6963307994085528976[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6963307994085528976[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6963307994085528976[62] = 0;
   out_6963307994085528976[63] = 0;
   out_6963307994085528976[64] = 0;
   out_6963307994085528976[65] = 0;
   out_6963307994085528976[66] = 0;
   out_6963307994085528976[67] = 0;
   out_6963307994085528976[68] = 0;
   out_6963307994085528976[69] = 0;
   out_6963307994085528976[70] = 1;
   out_6963307994085528976[71] = 0;
   out_6963307994085528976[72] = 0;
   out_6963307994085528976[73] = 0;
   out_6963307994085528976[74] = 0;
   out_6963307994085528976[75] = 0;
   out_6963307994085528976[76] = 0;
   out_6963307994085528976[77] = 0;
   out_6963307994085528976[78] = 0;
   out_6963307994085528976[79] = 0;
   out_6963307994085528976[80] = 1;
}
void h_25(double *state, double *unused, double *out_4344813781663258756) {
   out_4344813781663258756[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7241202889850392582) {
   out_7241202889850392582[0] = 0;
   out_7241202889850392582[1] = 0;
   out_7241202889850392582[2] = 0;
   out_7241202889850392582[3] = 0;
   out_7241202889850392582[4] = 0;
   out_7241202889850392582[5] = 0;
   out_7241202889850392582[6] = 1;
   out_7241202889850392582[7] = 0;
   out_7241202889850392582[8] = 0;
}
void h_24(double *state, double *unused, double *out_3932708101001244534) {
   out_3932708101001244534[0] = state[4];
   out_3932708101001244534[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1044291530223732530) {
   out_1044291530223732530[0] = 0;
   out_1044291530223732530[1] = 0;
   out_1044291530223732530[2] = 0;
   out_1044291530223732530[3] = 0;
   out_1044291530223732530[4] = 1;
   out_1044291530223732530[5] = 0;
   out_1044291530223732530[6] = 0;
   out_1044291530223732530[7] = 0;
   out_1044291530223732530[8] = 0;
   out_1044291530223732530[9] = 0;
   out_1044291530223732530[10] = 0;
   out_1044291530223732530[11] = 0;
   out_1044291530223732530[12] = 0;
   out_1044291530223732530[13] = 0;
   out_1044291530223732530[14] = 1;
   out_1044291530223732530[15] = 0;
   out_1044291530223732530[16] = 0;
   out_1044291530223732530[17] = 0;
}
void h_30(double *state, double *unused, double *out_5170465887451101612) {
   out_5170465887451101612[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8687208225351910407) {
   out_8687208225351910407[0] = 0;
   out_8687208225351910407[1] = 0;
   out_8687208225351910407[2] = 0;
   out_8687208225351910407[3] = 0;
   out_8687208225351910407[4] = 1;
   out_8687208225351910407[5] = 0;
   out_8687208225351910407[6] = 0;
   out_8687208225351910407[7] = 0;
   out_8687208225351910407[8] = 0;
}
void h_26(double *state, double *unused, double *out_8274604918597604359) {
   out_8274604918597604359[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7901015214098358433) {
   out_7901015214098358433[0] = 0;
   out_7901015214098358433[1] = 0;
   out_7901015214098358433[2] = 0;
   out_7901015214098358433[3] = 0;
   out_7901015214098358433[4] = 0;
   out_7901015214098358433[5] = 0;
   out_7901015214098358433[6] = 0;
   out_7901015214098358433[7] = 1;
   out_7901015214098358433[8] = 0;
}
void h_27(double *state, double *unused, double *out_53138299618370256) {
   out_53138299618370256[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7584772536557216298) {
   out_7584772536557216298[0] = 0;
   out_7584772536557216298[1] = 0;
   out_7584772536557216298[2] = 0;
   out_7584772536557216298[3] = 1;
   out_7584772536557216298[4] = 0;
   out_7584772536557216298[5] = 0;
   out_7584772536557216298[6] = 0;
   out_7584772536557216298[7] = 0;
   out_7584772536557216298[8] = 0;
}
void h_29(double *state, double *unused, double *out_104048368732758889) {
   out_104048368732758889[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8176976881037518223) {
   out_8176976881037518223[0] = 0;
   out_8176976881037518223[1] = 1;
   out_8176976881037518223[2] = 0;
   out_8176976881037518223[3] = 0;
   out_8176976881037518223[4] = 0;
   out_8176976881037518223[5] = 0;
   out_8176976881037518223[6] = 0;
   out_8176976881037518223[7] = 0;
   out_8176976881037518223[8] = 0;
}
void h_28(double *state, double *unused, double *out_5054811567279215901) {
   out_5054811567279215901[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5187368175602502819) {
   out_5187368175602502819[0] = 1;
   out_5187368175602502819[1] = 0;
   out_5187368175602502819[2] = 0;
   out_5187368175602502819[3] = 0;
   out_5187368175602502819[4] = 0;
   out_5187368175602502819[5] = 0;
   out_5187368175602502819[6] = 0;
   out_5187368175602502819[7] = 0;
   out_5187368175602502819[8] = 0;
}
void h_31(double *state, double *unused, double *out_8585984496296497739) {
   out_8585984496296497739[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2873491468742984882) {
   out_2873491468742984882[0] = 0;
   out_2873491468742984882[1] = 0;
   out_2873491468742984882[2] = 0;
   out_2873491468742984882[3] = 0;
   out_2873491468742984882[4] = 0;
   out_2873491468742984882[5] = 0;
   out_2873491468742984882[6] = 0;
   out_2873491468742984882[7] = 0;
   out_2873491468742984882[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7739919700438188629) {
  err_fun(nom_x, delta_x, out_7739919700438188629);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6319375409011776283) {
  inv_err_fun(nom_x, true_x, out_6319375409011776283);
}
void car_H_mod_fun(double *state, double *out_2844378829659981252) {
  H_mod_fun(state, out_2844378829659981252);
}
void car_f_fun(double *state, double dt, double *out_4884965143351282392) {
  f_fun(state,  dt, out_4884965143351282392);
}
void car_F_fun(double *state, double dt, double *out_6963307994085528976) {
  F_fun(state,  dt, out_6963307994085528976);
}
void car_h_25(double *state, double *unused, double *out_4344813781663258756) {
  h_25(state, unused, out_4344813781663258756);
}
void car_H_25(double *state, double *unused, double *out_7241202889850392582) {
  H_25(state, unused, out_7241202889850392582);
}
void car_h_24(double *state, double *unused, double *out_3932708101001244534) {
  h_24(state, unused, out_3932708101001244534);
}
void car_H_24(double *state, double *unused, double *out_1044291530223732530) {
  H_24(state, unused, out_1044291530223732530);
}
void car_h_30(double *state, double *unused, double *out_5170465887451101612) {
  h_30(state, unused, out_5170465887451101612);
}
void car_H_30(double *state, double *unused, double *out_8687208225351910407) {
  H_30(state, unused, out_8687208225351910407);
}
void car_h_26(double *state, double *unused, double *out_8274604918597604359) {
  h_26(state, unused, out_8274604918597604359);
}
void car_H_26(double *state, double *unused, double *out_7901015214098358433) {
  H_26(state, unused, out_7901015214098358433);
}
void car_h_27(double *state, double *unused, double *out_53138299618370256) {
  h_27(state, unused, out_53138299618370256);
}
void car_H_27(double *state, double *unused, double *out_7584772536557216298) {
  H_27(state, unused, out_7584772536557216298);
}
void car_h_29(double *state, double *unused, double *out_104048368732758889) {
  h_29(state, unused, out_104048368732758889);
}
void car_H_29(double *state, double *unused, double *out_8176976881037518223) {
  H_29(state, unused, out_8176976881037518223);
}
void car_h_28(double *state, double *unused, double *out_5054811567279215901) {
  h_28(state, unused, out_5054811567279215901);
}
void car_H_28(double *state, double *unused, double *out_5187368175602502819) {
  H_28(state, unused, out_5187368175602502819);
}
void car_h_31(double *state, double *unused, double *out_8585984496296497739) {
  h_31(state, unused, out_8585984496296497739);
}
void car_H_31(double *state, double *unused, double *out_2873491468742984882) {
  H_31(state, unused, out_2873491468742984882);
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
