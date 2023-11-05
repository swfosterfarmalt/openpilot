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
void err_fun(double *nom_x, double *delta_x, double *out_7068129843327768808) {
   out_7068129843327768808[0] = delta_x[0] + nom_x[0];
   out_7068129843327768808[1] = delta_x[1] + nom_x[1];
   out_7068129843327768808[2] = delta_x[2] + nom_x[2];
   out_7068129843327768808[3] = delta_x[3] + nom_x[3];
   out_7068129843327768808[4] = delta_x[4] + nom_x[4];
   out_7068129843327768808[5] = delta_x[5] + nom_x[5];
   out_7068129843327768808[6] = delta_x[6] + nom_x[6];
   out_7068129843327768808[7] = delta_x[7] + nom_x[7];
   out_7068129843327768808[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_733494678229042308) {
   out_733494678229042308[0] = -nom_x[0] + true_x[0];
   out_733494678229042308[1] = -nom_x[1] + true_x[1];
   out_733494678229042308[2] = -nom_x[2] + true_x[2];
   out_733494678229042308[3] = -nom_x[3] + true_x[3];
   out_733494678229042308[4] = -nom_x[4] + true_x[4];
   out_733494678229042308[5] = -nom_x[5] + true_x[5];
   out_733494678229042308[6] = -nom_x[6] + true_x[6];
   out_733494678229042308[7] = -nom_x[7] + true_x[7];
   out_733494678229042308[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7571385414202809552) {
   out_7571385414202809552[0] = 1.0;
   out_7571385414202809552[1] = 0;
   out_7571385414202809552[2] = 0;
   out_7571385414202809552[3] = 0;
   out_7571385414202809552[4] = 0;
   out_7571385414202809552[5] = 0;
   out_7571385414202809552[6] = 0;
   out_7571385414202809552[7] = 0;
   out_7571385414202809552[8] = 0;
   out_7571385414202809552[9] = 0;
   out_7571385414202809552[10] = 1.0;
   out_7571385414202809552[11] = 0;
   out_7571385414202809552[12] = 0;
   out_7571385414202809552[13] = 0;
   out_7571385414202809552[14] = 0;
   out_7571385414202809552[15] = 0;
   out_7571385414202809552[16] = 0;
   out_7571385414202809552[17] = 0;
   out_7571385414202809552[18] = 0;
   out_7571385414202809552[19] = 0;
   out_7571385414202809552[20] = 1.0;
   out_7571385414202809552[21] = 0;
   out_7571385414202809552[22] = 0;
   out_7571385414202809552[23] = 0;
   out_7571385414202809552[24] = 0;
   out_7571385414202809552[25] = 0;
   out_7571385414202809552[26] = 0;
   out_7571385414202809552[27] = 0;
   out_7571385414202809552[28] = 0;
   out_7571385414202809552[29] = 0;
   out_7571385414202809552[30] = 1.0;
   out_7571385414202809552[31] = 0;
   out_7571385414202809552[32] = 0;
   out_7571385414202809552[33] = 0;
   out_7571385414202809552[34] = 0;
   out_7571385414202809552[35] = 0;
   out_7571385414202809552[36] = 0;
   out_7571385414202809552[37] = 0;
   out_7571385414202809552[38] = 0;
   out_7571385414202809552[39] = 0;
   out_7571385414202809552[40] = 1.0;
   out_7571385414202809552[41] = 0;
   out_7571385414202809552[42] = 0;
   out_7571385414202809552[43] = 0;
   out_7571385414202809552[44] = 0;
   out_7571385414202809552[45] = 0;
   out_7571385414202809552[46] = 0;
   out_7571385414202809552[47] = 0;
   out_7571385414202809552[48] = 0;
   out_7571385414202809552[49] = 0;
   out_7571385414202809552[50] = 1.0;
   out_7571385414202809552[51] = 0;
   out_7571385414202809552[52] = 0;
   out_7571385414202809552[53] = 0;
   out_7571385414202809552[54] = 0;
   out_7571385414202809552[55] = 0;
   out_7571385414202809552[56] = 0;
   out_7571385414202809552[57] = 0;
   out_7571385414202809552[58] = 0;
   out_7571385414202809552[59] = 0;
   out_7571385414202809552[60] = 1.0;
   out_7571385414202809552[61] = 0;
   out_7571385414202809552[62] = 0;
   out_7571385414202809552[63] = 0;
   out_7571385414202809552[64] = 0;
   out_7571385414202809552[65] = 0;
   out_7571385414202809552[66] = 0;
   out_7571385414202809552[67] = 0;
   out_7571385414202809552[68] = 0;
   out_7571385414202809552[69] = 0;
   out_7571385414202809552[70] = 1.0;
   out_7571385414202809552[71] = 0;
   out_7571385414202809552[72] = 0;
   out_7571385414202809552[73] = 0;
   out_7571385414202809552[74] = 0;
   out_7571385414202809552[75] = 0;
   out_7571385414202809552[76] = 0;
   out_7571385414202809552[77] = 0;
   out_7571385414202809552[78] = 0;
   out_7571385414202809552[79] = 0;
   out_7571385414202809552[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9021702938268023425) {
   out_9021702938268023425[0] = state[0];
   out_9021702938268023425[1] = state[1];
   out_9021702938268023425[2] = state[2];
   out_9021702938268023425[3] = state[3];
   out_9021702938268023425[4] = state[4];
   out_9021702938268023425[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9021702938268023425[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9021702938268023425[7] = state[7];
   out_9021702938268023425[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3272499767417673680) {
   out_3272499767417673680[0] = 1;
   out_3272499767417673680[1] = 0;
   out_3272499767417673680[2] = 0;
   out_3272499767417673680[3] = 0;
   out_3272499767417673680[4] = 0;
   out_3272499767417673680[5] = 0;
   out_3272499767417673680[6] = 0;
   out_3272499767417673680[7] = 0;
   out_3272499767417673680[8] = 0;
   out_3272499767417673680[9] = 0;
   out_3272499767417673680[10] = 1;
   out_3272499767417673680[11] = 0;
   out_3272499767417673680[12] = 0;
   out_3272499767417673680[13] = 0;
   out_3272499767417673680[14] = 0;
   out_3272499767417673680[15] = 0;
   out_3272499767417673680[16] = 0;
   out_3272499767417673680[17] = 0;
   out_3272499767417673680[18] = 0;
   out_3272499767417673680[19] = 0;
   out_3272499767417673680[20] = 1;
   out_3272499767417673680[21] = 0;
   out_3272499767417673680[22] = 0;
   out_3272499767417673680[23] = 0;
   out_3272499767417673680[24] = 0;
   out_3272499767417673680[25] = 0;
   out_3272499767417673680[26] = 0;
   out_3272499767417673680[27] = 0;
   out_3272499767417673680[28] = 0;
   out_3272499767417673680[29] = 0;
   out_3272499767417673680[30] = 1;
   out_3272499767417673680[31] = 0;
   out_3272499767417673680[32] = 0;
   out_3272499767417673680[33] = 0;
   out_3272499767417673680[34] = 0;
   out_3272499767417673680[35] = 0;
   out_3272499767417673680[36] = 0;
   out_3272499767417673680[37] = 0;
   out_3272499767417673680[38] = 0;
   out_3272499767417673680[39] = 0;
   out_3272499767417673680[40] = 1;
   out_3272499767417673680[41] = 0;
   out_3272499767417673680[42] = 0;
   out_3272499767417673680[43] = 0;
   out_3272499767417673680[44] = 0;
   out_3272499767417673680[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3272499767417673680[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3272499767417673680[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3272499767417673680[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3272499767417673680[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3272499767417673680[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3272499767417673680[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3272499767417673680[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3272499767417673680[53] = -9.8000000000000007*dt;
   out_3272499767417673680[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3272499767417673680[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3272499767417673680[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3272499767417673680[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3272499767417673680[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3272499767417673680[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3272499767417673680[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3272499767417673680[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3272499767417673680[62] = 0;
   out_3272499767417673680[63] = 0;
   out_3272499767417673680[64] = 0;
   out_3272499767417673680[65] = 0;
   out_3272499767417673680[66] = 0;
   out_3272499767417673680[67] = 0;
   out_3272499767417673680[68] = 0;
   out_3272499767417673680[69] = 0;
   out_3272499767417673680[70] = 1;
   out_3272499767417673680[71] = 0;
   out_3272499767417673680[72] = 0;
   out_3272499767417673680[73] = 0;
   out_3272499767417673680[74] = 0;
   out_3272499767417673680[75] = 0;
   out_3272499767417673680[76] = 0;
   out_3272499767417673680[77] = 0;
   out_3272499767417673680[78] = 0;
   out_3272499767417673680[79] = 0;
   out_3272499767417673680[80] = 1;
}
void h_25(double *state, double *unused, double *out_7443089287366681720) {
   out_7443089287366681720[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2778685672947439072) {
   out_2778685672947439072[0] = 0;
   out_2778685672947439072[1] = 0;
   out_2778685672947439072[2] = 0;
   out_2778685672947439072[3] = 0;
   out_2778685672947439072[4] = 0;
   out_2778685672947439072[5] = 0;
   out_2778685672947439072[6] = 1;
   out_2778685672947439072[7] = 0;
   out_2778685672947439072[8] = 0;
}
void h_24(double *state, double *unused, double *out_9187770942234931660) {
   out_9187770942234931660[0] = state[4];
   out_9187770942234931660[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1967823654019069962) {
   out_1967823654019069962[0] = 0;
   out_1967823654019069962[1] = 0;
   out_1967823654019069962[2] = 0;
   out_1967823654019069962[3] = 0;
   out_1967823654019069962[4] = 1;
   out_1967823654019069962[5] = 0;
   out_1967823654019069962[6] = 0;
   out_1967823654019069962[7] = 0;
   out_1967823654019069962[8] = 0;
   out_1967823654019069962[9] = 0;
   out_1967823654019069962[10] = 0;
   out_1967823654019069962[11] = 0;
   out_1967823654019069962[12] = 0;
   out_1967823654019069962[13] = 0;
   out_1967823654019069962[14] = 1;
   out_1967823654019069962[15] = 0;
   out_1967823654019069962[16] = 0;
   out_1967823654019069962[17] = 0;
}
void h_30(double *state, double *unused, double *out_8893720679995564887) {
   out_8893720679995564887[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5297018631454687699) {
   out_5297018631454687699[0] = 0;
   out_5297018631454687699[1] = 0;
   out_5297018631454687699[2] = 0;
   out_5297018631454687699[3] = 0;
   out_5297018631454687699[4] = 1;
   out_5297018631454687699[5] = 0;
   out_5297018631454687699[6] = 0;
   out_5297018631454687699[7] = 0;
   out_5297018631454687699[8] = 0;
}
void h_26(double *state, double *unused, double *out_1682811193741343376) {
   out_1682811193741343376[0] = state[7];
}
void H_26(double *state, double *unused, double *out_962817645926617152) {
   out_962817645926617152[0] = 0;
   out_962817645926617152[1] = 0;
   out_962817645926617152[2] = 0;
   out_962817645926617152[3] = 0;
   out_962817645926617152[4] = 0;
   out_962817645926617152[5] = 0;
   out_962817645926617152[6] = 0;
   out_962817645926617152[7] = 1;
   out_962817645926617152[8] = 0;
}
void h_27(double *state, double *unused, double *out_8399249465534172245) {
   out_8399249465534172245[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3122255319654262788) {
   out_3122255319654262788[0] = 0;
   out_3122255319654262788[1] = 0;
   out_3122255319654262788[2] = 0;
   out_3122255319654262788[3] = 1;
   out_3122255319654262788[4] = 0;
   out_3122255319654262788[5] = 0;
   out_3122255319654262788[6] = 0;
   out_3122255319654262788[7] = 0;
   out_3122255319654262788[8] = 0;
}
void h_29(double *state, double *unused, double *out_3892047412334719242) {
   out_3892047412334719242[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5807249975769079883) {
   out_5807249975769079883[0] = 0;
   out_5807249975769079883[1] = 1;
   out_5807249975769079883[2] = 0;
   out_5807249975769079883[3] = 0;
   out_5807249975769079883[4] = 0;
   out_5807249975769079883[5] = 0;
   out_5807249975769079883[6] = 0;
   out_5807249975769079883[7] = 0;
   out_5807249975769079883[8] = 0;
}
void h_28(double *state, double *unused, double *out_964106031105471432) {
   out_964106031105471432[0] = state[0];
}
void H_28(double *state, double *unused, double *out_724850958699549309) {
   out_724850958699549309[0] = 1;
   out_724850958699549309[1] = 0;
   out_724850958699549309[2] = 0;
   out_724850958699549309[3] = 0;
   out_724850958699549309[4] = 0;
   out_724850958699549309[5] = 0;
   out_724850958699549309[6] = 0;
   out_724850958699549309[7] = 0;
   out_724850958699549309[8] = 0;
}
void h_31(double *state, double *unused, double *out_3122382135588820711) {
   out_3122382135588820711[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1589025748159968628) {
   out_1589025748159968628[0] = 0;
   out_1589025748159968628[1] = 0;
   out_1589025748159968628[2] = 0;
   out_1589025748159968628[3] = 0;
   out_1589025748159968628[4] = 0;
   out_1589025748159968628[5] = 0;
   out_1589025748159968628[6] = 0;
   out_1589025748159968628[7] = 0;
   out_1589025748159968628[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7068129843327768808) {
  err_fun(nom_x, delta_x, out_7068129843327768808);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_733494678229042308) {
  inv_err_fun(nom_x, true_x, out_733494678229042308);
}
void car_H_mod_fun(double *state, double *out_7571385414202809552) {
  H_mod_fun(state, out_7571385414202809552);
}
void car_f_fun(double *state, double dt, double *out_9021702938268023425) {
  f_fun(state,  dt, out_9021702938268023425);
}
void car_F_fun(double *state, double dt, double *out_3272499767417673680) {
  F_fun(state,  dt, out_3272499767417673680);
}
void car_h_25(double *state, double *unused, double *out_7443089287366681720) {
  h_25(state, unused, out_7443089287366681720);
}
void car_H_25(double *state, double *unused, double *out_2778685672947439072) {
  H_25(state, unused, out_2778685672947439072);
}
void car_h_24(double *state, double *unused, double *out_9187770942234931660) {
  h_24(state, unused, out_9187770942234931660);
}
void car_H_24(double *state, double *unused, double *out_1967823654019069962) {
  H_24(state, unused, out_1967823654019069962);
}
void car_h_30(double *state, double *unused, double *out_8893720679995564887) {
  h_30(state, unused, out_8893720679995564887);
}
void car_H_30(double *state, double *unused, double *out_5297018631454687699) {
  H_30(state, unused, out_5297018631454687699);
}
void car_h_26(double *state, double *unused, double *out_1682811193741343376) {
  h_26(state, unused, out_1682811193741343376);
}
void car_H_26(double *state, double *unused, double *out_962817645926617152) {
  H_26(state, unused, out_962817645926617152);
}
void car_h_27(double *state, double *unused, double *out_8399249465534172245) {
  h_27(state, unused, out_8399249465534172245);
}
void car_H_27(double *state, double *unused, double *out_3122255319654262788) {
  H_27(state, unused, out_3122255319654262788);
}
void car_h_29(double *state, double *unused, double *out_3892047412334719242) {
  h_29(state, unused, out_3892047412334719242);
}
void car_H_29(double *state, double *unused, double *out_5807249975769079883) {
  H_29(state, unused, out_5807249975769079883);
}
void car_h_28(double *state, double *unused, double *out_964106031105471432) {
  h_28(state, unused, out_964106031105471432);
}
void car_H_28(double *state, double *unused, double *out_724850958699549309) {
  H_28(state, unused, out_724850958699549309);
}
void car_h_31(double *state, double *unused, double *out_3122382135588820711) {
  h_31(state, unused, out_3122382135588820711);
}
void car_H_31(double *state, double *unused, double *out_1589025748159968628) {
  H_31(state, unused, out_1589025748159968628);
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
