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
void err_fun(double *nom_x, double *delta_x, double *out_9197230539031214470) {
   out_9197230539031214470[0] = delta_x[0] + nom_x[0];
   out_9197230539031214470[1] = delta_x[1] + nom_x[1];
   out_9197230539031214470[2] = delta_x[2] + nom_x[2];
   out_9197230539031214470[3] = delta_x[3] + nom_x[3];
   out_9197230539031214470[4] = delta_x[4] + nom_x[4];
   out_9197230539031214470[5] = delta_x[5] + nom_x[5];
   out_9197230539031214470[6] = delta_x[6] + nom_x[6];
   out_9197230539031214470[7] = delta_x[7] + nom_x[7];
   out_9197230539031214470[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5205798968448591509) {
   out_5205798968448591509[0] = -nom_x[0] + true_x[0];
   out_5205798968448591509[1] = -nom_x[1] + true_x[1];
   out_5205798968448591509[2] = -nom_x[2] + true_x[2];
   out_5205798968448591509[3] = -nom_x[3] + true_x[3];
   out_5205798968448591509[4] = -nom_x[4] + true_x[4];
   out_5205798968448591509[5] = -nom_x[5] + true_x[5];
   out_5205798968448591509[6] = -nom_x[6] + true_x[6];
   out_5205798968448591509[7] = -nom_x[7] + true_x[7];
   out_5205798968448591509[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6671470295335338631) {
   out_6671470295335338631[0] = 1.0;
   out_6671470295335338631[1] = 0;
   out_6671470295335338631[2] = 0;
   out_6671470295335338631[3] = 0;
   out_6671470295335338631[4] = 0;
   out_6671470295335338631[5] = 0;
   out_6671470295335338631[6] = 0;
   out_6671470295335338631[7] = 0;
   out_6671470295335338631[8] = 0;
   out_6671470295335338631[9] = 0;
   out_6671470295335338631[10] = 1.0;
   out_6671470295335338631[11] = 0;
   out_6671470295335338631[12] = 0;
   out_6671470295335338631[13] = 0;
   out_6671470295335338631[14] = 0;
   out_6671470295335338631[15] = 0;
   out_6671470295335338631[16] = 0;
   out_6671470295335338631[17] = 0;
   out_6671470295335338631[18] = 0;
   out_6671470295335338631[19] = 0;
   out_6671470295335338631[20] = 1.0;
   out_6671470295335338631[21] = 0;
   out_6671470295335338631[22] = 0;
   out_6671470295335338631[23] = 0;
   out_6671470295335338631[24] = 0;
   out_6671470295335338631[25] = 0;
   out_6671470295335338631[26] = 0;
   out_6671470295335338631[27] = 0;
   out_6671470295335338631[28] = 0;
   out_6671470295335338631[29] = 0;
   out_6671470295335338631[30] = 1.0;
   out_6671470295335338631[31] = 0;
   out_6671470295335338631[32] = 0;
   out_6671470295335338631[33] = 0;
   out_6671470295335338631[34] = 0;
   out_6671470295335338631[35] = 0;
   out_6671470295335338631[36] = 0;
   out_6671470295335338631[37] = 0;
   out_6671470295335338631[38] = 0;
   out_6671470295335338631[39] = 0;
   out_6671470295335338631[40] = 1.0;
   out_6671470295335338631[41] = 0;
   out_6671470295335338631[42] = 0;
   out_6671470295335338631[43] = 0;
   out_6671470295335338631[44] = 0;
   out_6671470295335338631[45] = 0;
   out_6671470295335338631[46] = 0;
   out_6671470295335338631[47] = 0;
   out_6671470295335338631[48] = 0;
   out_6671470295335338631[49] = 0;
   out_6671470295335338631[50] = 1.0;
   out_6671470295335338631[51] = 0;
   out_6671470295335338631[52] = 0;
   out_6671470295335338631[53] = 0;
   out_6671470295335338631[54] = 0;
   out_6671470295335338631[55] = 0;
   out_6671470295335338631[56] = 0;
   out_6671470295335338631[57] = 0;
   out_6671470295335338631[58] = 0;
   out_6671470295335338631[59] = 0;
   out_6671470295335338631[60] = 1.0;
   out_6671470295335338631[61] = 0;
   out_6671470295335338631[62] = 0;
   out_6671470295335338631[63] = 0;
   out_6671470295335338631[64] = 0;
   out_6671470295335338631[65] = 0;
   out_6671470295335338631[66] = 0;
   out_6671470295335338631[67] = 0;
   out_6671470295335338631[68] = 0;
   out_6671470295335338631[69] = 0;
   out_6671470295335338631[70] = 1.0;
   out_6671470295335338631[71] = 0;
   out_6671470295335338631[72] = 0;
   out_6671470295335338631[73] = 0;
   out_6671470295335338631[74] = 0;
   out_6671470295335338631[75] = 0;
   out_6671470295335338631[76] = 0;
   out_6671470295335338631[77] = 0;
   out_6671470295335338631[78] = 0;
   out_6671470295335338631[79] = 0;
   out_6671470295335338631[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1262138838755003200) {
   out_1262138838755003200[0] = state[0];
   out_1262138838755003200[1] = state[1];
   out_1262138838755003200[2] = state[2];
   out_1262138838755003200[3] = state[3];
   out_1262138838755003200[4] = state[4];
   out_1262138838755003200[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1262138838755003200[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1262138838755003200[7] = state[7];
   out_1262138838755003200[8] = state[8];
}
void F_fun(double *state, double dt, double *out_125455060826810986) {
   out_125455060826810986[0] = 1;
   out_125455060826810986[1] = 0;
   out_125455060826810986[2] = 0;
   out_125455060826810986[3] = 0;
   out_125455060826810986[4] = 0;
   out_125455060826810986[5] = 0;
   out_125455060826810986[6] = 0;
   out_125455060826810986[7] = 0;
   out_125455060826810986[8] = 0;
   out_125455060826810986[9] = 0;
   out_125455060826810986[10] = 1;
   out_125455060826810986[11] = 0;
   out_125455060826810986[12] = 0;
   out_125455060826810986[13] = 0;
   out_125455060826810986[14] = 0;
   out_125455060826810986[15] = 0;
   out_125455060826810986[16] = 0;
   out_125455060826810986[17] = 0;
   out_125455060826810986[18] = 0;
   out_125455060826810986[19] = 0;
   out_125455060826810986[20] = 1;
   out_125455060826810986[21] = 0;
   out_125455060826810986[22] = 0;
   out_125455060826810986[23] = 0;
   out_125455060826810986[24] = 0;
   out_125455060826810986[25] = 0;
   out_125455060826810986[26] = 0;
   out_125455060826810986[27] = 0;
   out_125455060826810986[28] = 0;
   out_125455060826810986[29] = 0;
   out_125455060826810986[30] = 1;
   out_125455060826810986[31] = 0;
   out_125455060826810986[32] = 0;
   out_125455060826810986[33] = 0;
   out_125455060826810986[34] = 0;
   out_125455060826810986[35] = 0;
   out_125455060826810986[36] = 0;
   out_125455060826810986[37] = 0;
   out_125455060826810986[38] = 0;
   out_125455060826810986[39] = 0;
   out_125455060826810986[40] = 1;
   out_125455060826810986[41] = 0;
   out_125455060826810986[42] = 0;
   out_125455060826810986[43] = 0;
   out_125455060826810986[44] = 0;
   out_125455060826810986[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_125455060826810986[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_125455060826810986[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_125455060826810986[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_125455060826810986[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_125455060826810986[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_125455060826810986[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_125455060826810986[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_125455060826810986[53] = -9.8000000000000007*dt;
   out_125455060826810986[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_125455060826810986[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_125455060826810986[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_125455060826810986[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_125455060826810986[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_125455060826810986[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_125455060826810986[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_125455060826810986[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_125455060826810986[62] = 0;
   out_125455060826810986[63] = 0;
   out_125455060826810986[64] = 0;
   out_125455060826810986[65] = 0;
   out_125455060826810986[66] = 0;
   out_125455060826810986[67] = 0;
   out_125455060826810986[68] = 0;
   out_125455060826810986[69] = 0;
   out_125455060826810986[70] = 1;
   out_125455060826810986[71] = 0;
   out_125455060826810986[72] = 0;
   out_125455060826810986[73] = 0;
   out_125455060826810986[74] = 0;
   out_125455060826810986[75] = 0;
   out_125455060826810986[76] = 0;
   out_125455060826810986[77] = 0;
   out_125455060826810986[78] = 0;
   out_125455060826810986[79] = 0;
   out_125455060826810986[80] = 1;
}
void h_25(double *state, double *unused, double *out_15986601659185824) {
   out_15986601659185824[0] = state[6];
}
void H_25(double *state, double *unused, double *out_546342770720130705) {
   out_546342770720130705[0] = 0;
   out_546342770720130705[1] = 0;
   out_546342770720130705[2] = 0;
   out_546342770720130705[3] = 0;
   out_546342770720130705[4] = 0;
   out_546342770720130705[5] = 0;
   out_546342770720130705[6] = 1;
   out_546342770720130705[7] = 0;
   out_546342770720130705[8] = 0;
}
void h_24(double *state, double *unused, double *out_5620042477706618172) {
   out_5620042477706618172[0] = state[4];
   out_5620042477706618172[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8598523939230746457) {
   out_8598523939230746457[0] = 0;
   out_8598523939230746457[1] = 0;
   out_8598523939230746457[2] = 0;
   out_8598523939230746457[3] = 0;
   out_8598523939230746457[4] = 1;
   out_8598523939230746457[5] = 0;
   out_8598523939230746457[6] = 0;
   out_8598523939230746457[7] = 0;
   out_8598523939230746457[8] = 0;
   out_8598523939230746457[9] = 0;
   out_8598523939230746457[10] = 0;
   out_8598523939230746457[11] = 0;
   out_8598523939230746457[12] = 0;
   out_8598523939230746457[13] = 0;
   out_8598523939230746457[14] = 1;
   out_8598523939230746457[15] = 0;
   out_8598523939230746457[16] = 0;
   out_8598523939230746457[17] = 0;
}
void h_30(double *state, double *unused, double *out_9121730683024665358) {
   out_9121730683024665358[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3981353559407477493) {
   out_3981353559407477493[0] = 0;
   out_3981353559407477493[1] = 0;
   out_3981353559407477493[2] = 0;
   out_3981353559407477493[3] = 0;
   out_3981353559407477493[4] = 1;
   out_3981353559407477493[5] = 0;
   out_3981353559407477493[6] = 0;
   out_3981353559407477493[7] = 0;
   out_3981353559407477493[8] = 0;
}
void h_26(double *state, double *unused, double *out_5587104823615023375) {
   out_5587104823615023375[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3195160548153925519) {
   out_3195160548153925519[0] = 0;
   out_3195160548153925519[1] = 0;
   out_3195160548153925519[2] = 0;
   out_3195160548153925519[3] = 0;
   out_3195160548153925519[4] = 0;
   out_3195160548153925519[5] = 0;
   out_3195160548153925519[6] = 0;
   out_3195160548153925519[7] = 1;
   out_3195160548153925519[8] = 0;
}
void h_27(double *state, double *unused, double *out_5017659869320031469) {
   out_5017659869320031469[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6156116871207902404) {
   out_6156116871207902404[0] = 0;
   out_6156116871207902404[1] = 0;
   out_6156116871207902404[2] = 0;
   out_6156116871207902404[3] = 1;
   out_6156116871207902404[4] = 0;
   out_6156116871207902404[5] = 0;
   out_6156116871207902404[6] = 0;
   out_6156116871207902404[7] = 0;
   out_6156116871207902404[8] = 0;
}
void h_29(double *state, double *unused, double *out_8879343345748533121) {
   out_8879343345748533121[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3471122215093085309) {
   out_3471122215093085309[0] = 0;
   out_3471122215093085309[1] = 1;
   out_3471122215093085309[2] = 0;
   out_3471122215093085309[3] = 0;
   out_3471122215093085309[4] = 0;
   out_3471122215093085309[5] = 0;
   out_3471122215093085309[6] = 0;
   out_3471122215093085309[7] = 0;
   out_3471122215093085309[8] = 0;
}
void h_28(double *state, double *unused, double *out_2193249616934190042) {
   out_2193249616934190042[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1507491943527759058) {
   out_1507491943527759058[0] = 1;
   out_1507491943527759058[1] = 0;
   out_1507491943527759058[2] = 0;
   out_1507491943527759058[3] = 0;
   out_1507491943527759058[4] = 0;
   out_1507491943527759058[5] = 0;
   out_1507491943527759058[6] = 0;
   out_1507491943527759058[7] = 0;
   out_1507491943527759058[8] = 0;
}
void h_31(double *state, double *unused, double *out_259207460625320065) {
   out_259207460625320065[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3821368650387276995) {
   out_3821368650387276995[0] = 0;
   out_3821368650387276995[1] = 0;
   out_3821368650387276995[2] = 0;
   out_3821368650387276995[3] = 0;
   out_3821368650387276995[4] = 0;
   out_3821368650387276995[5] = 0;
   out_3821368650387276995[6] = 0;
   out_3821368650387276995[7] = 0;
   out_3821368650387276995[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_9197230539031214470) {
  err_fun(nom_x, delta_x, out_9197230539031214470);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5205798968448591509) {
  inv_err_fun(nom_x, true_x, out_5205798968448591509);
}
void car_H_mod_fun(double *state, double *out_6671470295335338631) {
  H_mod_fun(state, out_6671470295335338631);
}
void car_f_fun(double *state, double dt, double *out_1262138838755003200) {
  f_fun(state,  dt, out_1262138838755003200);
}
void car_F_fun(double *state, double dt, double *out_125455060826810986) {
  F_fun(state,  dt, out_125455060826810986);
}
void car_h_25(double *state, double *unused, double *out_15986601659185824) {
  h_25(state, unused, out_15986601659185824);
}
void car_H_25(double *state, double *unused, double *out_546342770720130705) {
  H_25(state, unused, out_546342770720130705);
}
void car_h_24(double *state, double *unused, double *out_5620042477706618172) {
  h_24(state, unused, out_5620042477706618172);
}
void car_H_24(double *state, double *unused, double *out_8598523939230746457) {
  H_24(state, unused, out_8598523939230746457);
}
void car_h_30(double *state, double *unused, double *out_9121730683024665358) {
  h_30(state, unused, out_9121730683024665358);
}
void car_H_30(double *state, double *unused, double *out_3981353559407477493) {
  H_30(state, unused, out_3981353559407477493);
}
void car_h_26(double *state, double *unused, double *out_5587104823615023375) {
  h_26(state, unused, out_5587104823615023375);
}
void car_H_26(double *state, double *unused, double *out_3195160548153925519) {
  H_26(state, unused, out_3195160548153925519);
}
void car_h_27(double *state, double *unused, double *out_5017659869320031469) {
  h_27(state, unused, out_5017659869320031469);
}
void car_H_27(double *state, double *unused, double *out_6156116871207902404) {
  H_27(state, unused, out_6156116871207902404);
}
void car_h_29(double *state, double *unused, double *out_8879343345748533121) {
  h_29(state, unused, out_8879343345748533121);
}
void car_H_29(double *state, double *unused, double *out_3471122215093085309) {
  H_29(state, unused, out_3471122215093085309);
}
void car_h_28(double *state, double *unused, double *out_2193249616934190042) {
  h_28(state, unused, out_2193249616934190042);
}
void car_H_28(double *state, double *unused, double *out_1507491943527759058) {
  H_28(state, unused, out_1507491943527759058);
}
void car_h_31(double *state, double *unused, double *out_259207460625320065) {
  h_31(state, unused, out_259207460625320065);
}
void car_H_31(double *state, double *unused, double *out_3821368650387276995) {
  H_31(state, unused, out_3821368650387276995);
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
