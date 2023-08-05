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
void err_fun(double *nom_x, double *delta_x, double *out_6639263734267575362) {
   out_6639263734267575362[0] = delta_x[0] + nom_x[0];
   out_6639263734267575362[1] = delta_x[1] + nom_x[1];
   out_6639263734267575362[2] = delta_x[2] + nom_x[2];
   out_6639263734267575362[3] = delta_x[3] + nom_x[3];
   out_6639263734267575362[4] = delta_x[4] + nom_x[4];
   out_6639263734267575362[5] = delta_x[5] + nom_x[5];
   out_6639263734267575362[6] = delta_x[6] + nom_x[6];
   out_6639263734267575362[7] = delta_x[7] + nom_x[7];
   out_6639263734267575362[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8779595821562168901) {
   out_8779595821562168901[0] = -nom_x[0] + true_x[0];
   out_8779595821562168901[1] = -nom_x[1] + true_x[1];
   out_8779595821562168901[2] = -nom_x[2] + true_x[2];
   out_8779595821562168901[3] = -nom_x[3] + true_x[3];
   out_8779595821562168901[4] = -nom_x[4] + true_x[4];
   out_8779595821562168901[5] = -nom_x[5] + true_x[5];
   out_8779595821562168901[6] = -nom_x[6] + true_x[6];
   out_8779595821562168901[7] = -nom_x[7] + true_x[7];
   out_8779595821562168901[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1887421269059698674) {
   out_1887421269059698674[0] = 1.0;
   out_1887421269059698674[1] = 0;
   out_1887421269059698674[2] = 0;
   out_1887421269059698674[3] = 0;
   out_1887421269059698674[4] = 0;
   out_1887421269059698674[5] = 0;
   out_1887421269059698674[6] = 0;
   out_1887421269059698674[7] = 0;
   out_1887421269059698674[8] = 0;
   out_1887421269059698674[9] = 0;
   out_1887421269059698674[10] = 1.0;
   out_1887421269059698674[11] = 0;
   out_1887421269059698674[12] = 0;
   out_1887421269059698674[13] = 0;
   out_1887421269059698674[14] = 0;
   out_1887421269059698674[15] = 0;
   out_1887421269059698674[16] = 0;
   out_1887421269059698674[17] = 0;
   out_1887421269059698674[18] = 0;
   out_1887421269059698674[19] = 0;
   out_1887421269059698674[20] = 1.0;
   out_1887421269059698674[21] = 0;
   out_1887421269059698674[22] = 0;
   out_1887421269059698674[23] = 0;
   out_1887421269059698674[24] = 0;
   out_1887421269059698674[25] = 0;
   out_1887421269059698674[26] = 0;
   out_1887421269059698674[27] = 0;
   out_1887421269059698674[28] = 0;
   out_1887421269059698674[29] = 0;
   out_1887421269059698674[30] = 1.0;
   out_1887421269059698674[31] = 0;
   out_1887421269059698674[32] = 0;
   out_1887421269059698674[33] = 0;
   out_1887421269059698674[34] = 0;
   out_1887421269059698674[35] = 0;
   out_1887421269059698674[36] = 0;
   out_1887421269059698674[37] = 0;
   out_1887421269059698674[38] = 0;
   out_1887421269059698674[39] = 0;
   out_1887421269059698674[40] = 1.0;
   out_1887421269059698674[41] = 0;
   out_1887421269059698674[42] = 0;
   out_1887421269059698674[43] = 0;
   out_1887421269059698674[44] = 0;
   out_1887421269059698674[45] = 0;
   out_1887421269059698674[46] = 0;
   out_1887421269059698674[47] = 0;
   out_1887421269059698674[48] = 0;
   out_1887421269059698674[49] = 0;
   out_1887421269059698674[50] = 1.0;
   out_1887421269059698674[51] = 0;
   out_1887421269059698674[52] = 0;
   out_1887421269059698674[53] = 0;
   out_1887421269059698674[54] = 0;
   out_1887421269059698674[55] = 0;
   out_1887421269059698674[56] = 0;
   out_1887421269059698674[57] = 0;
   out_1887421269059698674[58] = 0;
   out_1887421269059698674[59] = 0;
   out_1887421269059698674[60] = 1.0;
   out_1887421269059698674[61] = 0;
   out_1887421269059698674[62] = 0;
   out_1887421269059698674[63] = 0;
   out_1887421269059698674[64] = 0;
   out_1887421269059698674[65] = 0;
   out_1887421269059698674[66] = 0;
   out_1887421269059698674[67] = 0;
   out_1887421269059698674[68] = 0;
   out_1887421269059698674[69] = 0;
   out_1887421269059698674[70] = 1.0;
   out_1887421269059698674[71] = 0;
   out_1887421269059698674[72] = 0;
   out_1887421269059698674[73] = 0;
   out_1887421269059698674[74] = 0;
   out_1887421269059698674[75] = 0;
   out_1887421269059698674[76] = 0;
   out_1887421269059698674[77] = 0;
   out_1887421269059698674[78] = 0;
   out_1887421269059698674[79] = 0;
   out_1887421269059698674[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5758214277974107615) {
   out_5758214277974107615[0] = state[0];
   out_5758214277974107615[1] = state[1];
   out_5758214277974107615[2] = state[2];
   out_5758214277974107615[3] = state[3];
   out_5758214277974107615[4] = state[4];
   out_5758214277974107615[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5758214277974107615[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5758214277974107615[7] = state[7];
   out_5758214277974107615[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4628961427457330570) {
   out_4628961427457330570[0] = 1;
   out_4628961427457330570[1] = 0;
   out_4628961427457330570[2] = 0;
   out_4628961427457330570[3] = 0;
   out_4628961427457330570[4] = 0;
   out_4628961427457330570[5] = 0;
   out_4628961427457330570[6] = 0;
   out_4628961427457330570[7] = 0;
   out_4628961427457330570[8] = 0;
   out_4628961427457330570[9] = 0;
   out_4628961427457330570[10] = 1;
   out_4628961427457330570[11] = 0;
   out_4628961427457330570[12] = 0;
   out_4628961427457330570[13] = 0;
   out_4628961427457330570[14] = 0;
   out_4628961427457330570[15] = 0;
   out_4628961427457330570[16] = 0;
   out_4628961427457330570[17] = 0;
   out_4628961427457330570[18] = 0;
   out_4628961427457330570[19] = 0;
   out_4628961427457330570[20] = 1;
   out_4628961427457330570[21] = 0;
   out_4628961427457330570[22] = 0;
   out_4628961427457330570[23] = 0;
   out_4628961427457330570[24] = 0;
   out_4628961427457330570[25] = 0;
   out_4628961427457330570[26] = 0;
   out_4628961427457330570[27] = 0;
   out_4628961427457330570[28] = 0;
   out_4628961427457330570[29] = 0;
   out_4628961427457330570[30] = 1;
   out_4628961427457330570[31] = 0;
   out_4628961427457330570[32] = 0;
   out_4628961427457330570[33] = 0;
   out_4628961427457330570[34] = 0;
   out_4628961427457330570[35] = 0;
   out_4628961427457330570[36] = 0;
   out_4628961427457330570[37] = 0;
   out_4628961427457330570[38] = 0;
   out_4628961427457330570[39] = 0;
   out_4628961427457330570[40] = 1;
   out_4628961427457330570[41] = 0;
   out_4628961427457330570[42] = 0;
   out_4628961427457330570[43] = 0;
   out_4628961427457330570[44] = 0;
   out_4628961427457330570[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4628961427457330570[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4628961427457330570[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4628961427457330570[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4628961427457330570[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4628961427457330570[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4628961427457330570[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4628961427457330570[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4628961427457330570[53] = -9.8000000000000007*dt;
   out_4628961427457330570[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4628961427457330570[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4628961427457330570[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4628961427457330570[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4628961427457330570[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4628961427457330570[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4628961427457330570[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4628961427457330570[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4628961427457330570[62] = 0;
   out_4628961427457330570[63] = 0;
   out_4628961427457330570[64] = 0;
   out_4628961427457330570[65] = 0;
   out_4628961427457330570[66] = 0;
   out_4628961427457330570[67] = 0;
   out_4628961427457330570[68] = 0;
   out_4628961427457330570[69] = 0;
   out_4628961427457330570[70] = 1;
   out_4628961427457330570[71] = 0;
   out_4628961427457330570[72] = 0;
   out_4628961427457330570[73] = 0;
   out_4628961427457330570[74] = 0;
   out_4628961427457330570[75] = 0;
   out_4628961427457330570[76] = 0;
   out_4628961427457330570[77] = 0;
   out_4628961427457330570[78] = 0;
   out_4628961427457330570[79] = 0;
   out_4628961427457330570[80] = 1;
}
void h_25(double *state, double *unused, double *out_8692943286833080319) {
   out_8692943286833080319[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2901388286093755664) {
   out_2901388286093755664[0] = 0;
   out_2901388286093755664[1] = 0;
   out_2901388286093755664[2] = 0;
   out_2901388286093755664[3] = 0;
   out_2901388286093755664[4] = 0;
   out_2901388286093755664[5] = 0;
   out_2901388286093755664[6] = 1;
   out_2901388286093755664[7] = 0;
   out_2901388286093755664[8] = 0;
}
void h_24(double *state, double *unused, double *out_139111919068464966) {
   out_139111919068464966[0] = state[4];
   out_139111919068464966[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6317290601546600727) {
   out_6317290601546600727[0] = 0;
   out_6317290601546600727[1] = 0;
   out_6317290601546600727[2] = 0;
   out_6317290601546600727[3] = 0;
   out_6317290601546600727[4] = 1;
   out_6317290601546600727[5] = 0;
   out_6317290601546600727[6] = 0;
   out_6317290601546600727[7] = 0;
   out_6317290601546600727[8] = 0;
   out_6317290601546600727[9] = 0;
   out_6317290601546600727[10] = 0;
   out_6317290601546600727[11] = 0;
   out_6317290601546600727[12] = 0;
   out_6317290601546600727[13] = 0;
   out_6317290601546600727[14] = 1;
   out_6317290601546600727[15] = 0;
   out_6317290601546600727[16] = 0;
   out_6317290601546600727[17] = 0;
}
void h_30(double *state, double *unused, double *out_196282605569340327) {
   out_196282605569340327[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1626308044033852534) {
   out_1626308044033852534[0] = 0;
   out_1626308044033852534[1] = 0;
   out_1626308044033852534[2] = 0;
   out_1626308044033852534[3] = 0;
   out_1626308044033852534[4] = 1;
   out_1626308044033852534[5] = 0;
   out_1626308044033852534[6] = 0;
   out_1626308044033852534[7] = 0;
   out_1626308044033852534[8] = 0;
}
void h_26(double *state, double *unused, double *out_785801487765957785) {
   out_785801487765957785[0] = state[7];
}
void H_26(double *state, double *unused, double *out_840115032780300560) {
   out_840115032780300560[0] = 0;
   out_840115032780300560[1] = 0;
   out_840115032780300560[2] = 0;
   out_840115032780300560[3] = 0;
   out_840115032780300560[4] = 0;
   out_840115032780300560[5] = 0;
   out_840115032780300560[6] = 0;
   out_840115032780300560[7] = 1;
   out_840115032780300560[8] = 0;
}
void h_27(double *state, double *unused, double *out_4608910723760823760) {
   out_4608910723760823760[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3801071355834277445) {
   out_3801071355834277445[0] = 0;
   out_3801071355834277445[1] = 0;
   out_3801071355834277445[2] = 0;
   out_3801071355834277445[3] = 1;
   out_3801071355834277445[4] = 0;
   out_3801071355834277445[5] = 0;
   out_3801071355834277445[6] = 0;
   out_3801071355834277445[7] = 0;
   out_3801071355834277445[8] = 0;
}
void h_29(double *state, double *unused, double *out_5197955873230185972) {
   out_5197955873230185972[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1116076699719460350) {
   out_1116076699719460350[0] = 0;
   out_1116076699719460350[1] = 1;
   out_1116076699719460350[2] = 0;
   out_1116076699719460350[3] = 0;
   out_1116076699719460350[4] = 0;
   out_1116076699719460350[5] = 0;
   out_1116076699719460350[6] = 0;
   out_1116076699719460350[7] = 0;
   out_1116076699719460350[8] = 0;
}
void h_28(double *state, double *unused, double *out_7421462827030677143) {
   out_7421462827030677143[0] = state[0];
}
void H_28(double *state, double *unused, double *out_847553571845865901) {
   out_847553571845865901[0] = 1;
   out_847553571845865901[1] = 0;
   out_847553571845865901[2] = 0;
   out_847553571845865901[3] = 0;
   out_847553571845865901[4] = 0;
   out_847553571845865901[5] = 0;
   out_847553571845865901[6] = 0;
   out_847553571845865901[7] = 0;
   out_847553571845865901[8] = 0;
}
void h_31(double *state, double *unused, double *out_8417749224548574430) {
   out_8417749224548574430[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1466323135013652036) {
   out_1466323135013652036[0] = 0;
   out_1466323135013652036[1] = 0;
   out_1466323135013652036[2] = 0;
   out_1466323135013652036[3] = 0;
   out_1466323135013652036[4] = 0;
   out_1466323135013652036[5] = 0;
   out_1466323135013652036[6] = 0;
   out_1466323135013652036[7] = 0;
   out_1466323135013652036[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6639263734267575362) {
  err_fun(nom_x, delta_x, out_6639263734267575362);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8779595821562168901) {
  inv_err_fun(nom_x, true_x, out_8779595821562168901);
}
void car_H_mod_fun(double *state, double *out_1887421269059698674) {
  H_mod_fun(state, out_1887421269059698674);
}
void car_f_fun(double *state, double dt, double *out_5758214277974107615) {
  f_fun(state,  dt, out_5758214277974107615);
}
void car_F_fun(double *state, double dt, double *out_4628961427457330570) {
  F_fun(state,  dt, out_4628961427457330570);
}
void car_h_25(double *state, double *unused, double *out_8692943286833080319) {
  h_25(state, unused, out_8692943286833080319);
}
void car_H_25(double *state, double *unused, double *out_2901388286093755664) {
  H_25(state, unused, out_2901388286093755664);
}
void car_h_24(double *state, double *unused, double *out_139111919068464966) {
  h_24(state, unused, out_139111919068464966);
}
void car_H_24(double *state, double *unused, double *out_6317290601546600727) {
  H_24(state, unused, out_6317290601546600727);
}
void car_h_30(double *state, double *unused, double *out_196282605569340327) {
  h_30(state, unused, out_196282605569340327);
}
void car_H_30(double *state, double *unused, double *out_1626308044033852534) {
  H_30(state, unused, out_1626308044033852534);
}
void car_h_26(double *state, double *unused, double *out_785801487765957785) {
  h_26(state, unused, out_785801487765957785);
}
void car_H_26(double *state, double *unused, double *out_840115032780300560) {
  H_26(state, unused, out_840115032780300560);
}
void car_h_27(double *state, double *unused, double *out_4608910723760823760) {
  h_27(state, unused, out_4608910723760823760);
}
void car_H_27(double *state, double *unused, double *out_3801071355834277445) {
  H_27(state, unused, out_3801071355834277445);
}
void car_h_29(double *state, double *unused, double *out_5197955873230185972) {
  h_29(state, unused, out_5197955873230185972);
}
void car_H_29(double *state, double *unused, double *out_1116076699719460350) {
  H_29(state, unused, out_1116076699719460350);
}
void car_h_28(double *state, double *unused, double *out_7421462827030677143) {
  h_28(state, unused, out_7421462827030677143);
}
void car_H_28(double *state, double *unused, double *out_847553571845865901) {
  H_28(state, unused, out_847553571845865901);
}
void car_h_31(double *state, double *unused, double *out_8417749224548574430) {
  h_31(state, unused, out_8417749224548574430);
}
void car_H_31(double *state, double *unused, double *out_1466323135013652036) {
  H_31(state, unused, out_1466323135013652036);
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
