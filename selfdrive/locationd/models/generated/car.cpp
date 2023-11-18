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
void err_fun(double *nom_x, double *delta_x, double *out_2474516941466880566) {
   out_2474516941466880566[0] = delta_x[0] + nom_x[0];
   out_2474516941466880566[1] = delta_x[1] + nom_x[1];
   out_2474516941466880566[2] = delta_x[2] + nom_x[2];
   out_2474516941466880566[3] = delta_x[3] + nom_x[3];
   out_2474516941466880566[4] = delta_x[4] + nom_x[4];
   out_2474516941466880566[5] = delta_x[5] + nom_x[5];
   out_2474516941466880566[6] = delta_x[6] + nom_x[6];
   out_2474516941466880566[7] = delta_x[7] + nom_x[7];
   out_2474516941466880566[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9123673598000259248) {
   out_9123673598000259248[0] = -nom_x[0] + true_x[0];
   out_9123673598000259248[1] = -nom_x[1] + true_x[1];
   out_9123673598000259248[2] = -nom_x[2] + true_x[2];
   out_9123673598000259248[3] = -nom_x[3] + true_x[3];
   out_9123673598000259248[4] = -nom_x[4] + true_x[4];
   out_9123673598000259248[5] = -nom_x[5] + true_x[5];
   out_9123673598000259248[6] = -nom_x[6] + true_x[6];
   out_9123673598000259248[7] = -nom_x[7] + true_x[7];
   out_9123673598000259248[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2745915890047717913) {
   out_2745915890047717913[0] = 1.0;
   out_2745915890047717913[1] = 0;
   out_2745915890047717913[2] = 0;
   out_2745915890047717913[3] = 0;
   out_2745915890047717913[4] = 0;
   out_2745915890047717913[5] = 0;
   out_2745915890047717913[6] = 0;
   out_2745915890047717913[7] = 0;
   out_2745915890047717913[8] = 0;
   out_2745915890047717913[9] = 0;
   out_2745915890047717913[10] = 1.0;
   out_2745915890047717913[11] = 0;
   out_2745915890047717913[12] = 0;
   out_2745915890047717913[13] = 0;
   out_2745915890047717913[14] = 0;
   out_2745915890047717913[15] = 0;
   out_2745915890047717913[16] = 0;
   out_2745915890047717913[17] = 0;
   out_2745915890047717913[18] = 0;
   out_2745915890047717913[19] = 0;
   out_2745915890047717913[20] = 1.0;
   out_2745915890047717913[21] = 0;
   out_2745915890047717913[22] = 0;
   out_2745915890047717913[23] = 0;
   out_2745915890047717913[24] = 0;
   out_2745915890047717913[25] = 0;
   out_2745915890047717913[26] = 0;
   out_2745915890047717913[27] = 0;
   out_2745915890047717913[28] = 0;
   out_2745915890047717913[29] = 0;
   out_2745915890047717913[30] = 1.0;
   out_2745915890047717913[31] = 0;
   out_2745915890047717913[32] = 0;
   out_2745915890047717913[33] = 0;
   out_2745915890047717913[34] = 0;
   out_2745915890047717913[35] = 0;
   out_2745915890047717913[36] = 0;
   out_2745915890047717913[37] = 0;
   out_2745915890047717913[38] = 0;
   out_2745915890047717913[39] = 0;
   out_2745915890047717913[40] = 1.0;
   out_2745915890047717913[41] = 0;
   out_2745915890047717913[42] = 0;
   out_2745915890047717913[43] = 0;
   out_2745915890047717913[44] = 0;
   out_2745915890047717913[45] = 0;
   out_2745915890047717913[46] = 0;
   out_2745915890047717913[47] = 0;
   out_2745915890047717913[48] = 0;
   out_2745915890047717913[49] = 0;
   out_2745915890047717913[50] = 1.0;
   out_2745915890047717913[51] = 0;
   out_2745915890047717913[52] = 0;
   out_2745915890047717913[53] = 0;
   out_2745915890047717913[54] = 0;
   out_2745915890047717913[55] = 0;
   out_2745915890047717913[56] = 0;
   out_2745915890047717913[57] = 0;
   out_2745915890047717913[58] = 0;
   out_2745915890047717913[59] = 0;
   out_2745915890047717913[60] = 1.0;
   out_2745915890047717913[61] = 0;
   out_2745915890047717913[62] = 0;
   out_2745915890047717913[63] = 0;
   out_2745915890047717913[64] = 0;
   out_2745915890047717913[65] = 0;
   out_2745915890047717913[66] = 0;
   out_2745915890047717913[67] = 0;
   out_2745915890047717913[68] = 0;
   out_2745915890047717913[69] = 0;
   out_2745915890047717913[70] = 1.0;
   out_2745915890047717913[71] = 0;
   out_2745915890047717913[72] = 0;
   out_2745915890047717913[73] = 0;
   out_2745915890047717913[74] = 0;
   out_2745915890047717913[75] = 0;
   out_2745915890047717913[76] = 0;
   out_2745915890047717913[77] = 0;
   out_2745915890047717913[78] = 0;
   out_2745915890047717913[79] = 0;
   out_2745915890047717913[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2687781910986821476) {
   out_2687781910986821476[0] = state[0];
   out_2687781910986821476[1] = state[1];
   out_2687781910986821476[2] = state[2];
   out_2687781910986821476[3] = state[3];
   out_2687781910986821476[4] = state[4];
   out_2687781910986821476[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2687781910986821476[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2687781910986821476[7] = state[7];
   out_2687781910986821476[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1020427047685603609) {
   out_1020427047685603609[0] = 1;
   out_1020427047685603609[1] = 0;
   out_1020427047685603609[2] = 0;
   out_1020427047685603609[3] = 0;
   out_1020427047685603609[4] = 0;
   out_1020427047685603609[5] = 0;
   out_1020427047685603609[6] = 0;
   out_1020427047685603609[7] = 0;
   out_1020427047685603609[8] = 0;
   out_1020427047685603609[9] = 0;
   out_1020427047685603609[10] = 1;
   out_1020427047685603609[11] = 0;
   out_1020427047685603609[12] = 0;
   out_1020427047685603609[13] = 0;
   out_1020427047685603609[14] = 0;
   out_1020427047685603609[15] = 0;
   out_1020427047685603609[16] = 0;
   out_1020427047685603609[17] = 0;
   out_1020427047685603609[18] = 0;
   out_1020427047685603609[19] = 0;
   out_1020427047685603609[20] = 1;
   out_1020427047685603609[21] = 0;
   out_1020427047685603609[22] = 0;
   out_1020427047685603609[23] = 0;
   out_1020427047685603609[24] = 0;
   out_1020427047685603609[25] = 0;
   out_1020427047685603609[26] = 0;
   out_1020427047685603609[27] = 0;
   out_1020427047685603609[28] = 0;
   out_1020427047685603609[29] = 0;
   out_1020427047685603609[30] = 1;
   out_1020427047685603609[31] = 0;
   out_1020427047685603609[32] = 0;
   out_1020427047685603609[33] = 0;
   out_1020427047685603609[34] = 0;
   out_1020427047685603609[35] = 0;
   out_1020427047685603609[36] = 0;
   out_1020427047685603609[37] = 0;
   out_1020427047685603609[38] = 0;
   out_1020427047685603609[39] = 0;
   out_1020427047685603609[40] = 1;
   out_1020427047685603609[41] = 0;
   out_1020427047685603609[42] = 0;
   out_1020427047685603609[43] = 0;
   out_1020427047685603609[44] = 0;
   out_1020427047685603609[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1020427047685603609[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1020427047685603609[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1020427047685603609[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1020427047685603609[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1020427047685603609[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1020427047685603609[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1020427047685603609[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1020427047685603609[53] = -9.8000000000000007*dt;
   out_1020427047685603609[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1020427047685603609[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1020427047685603609[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1020427047685603609[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1020427047685603609[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1020427047685603609[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1020427047685603609[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1020427047685603609[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1020427047685603609[62] = 0;
   out_1020427047685603609[63] = 0;
   out_1020427047685603609[64] = 0;
   out_1020427047685603609[65] = 0;
   out_1020427047685603609[66] = 0;
   out_1020427047685603609[67] = 0;
   out_1020427047685603609[68] = 0;
   out_1020427047685603609[69] = 0;
   out_1020427047685603609[70] = 1;
   out_1020427047685603609[71] = 0;
   out_1020427047685603609[72] = 0;
   out_1020427047685603609[73] = 0;
   out_1020427047685603609[74] = 0;
   out_1020427047685603609[75] = 0;
   out_1020427047685603609[76] = 0;
   out_1020427047685603609[77] = 0;
   out_1020427047685603609[78] = 0;
   out_1020427047685603609[79] = 0;
   out_1020427047685603609[80] = 1;
}
void h_25(double *state, double *unused, double *out_89412062583394035) {
   out_89412062583394035[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1503933484884863987) {
   out_1503933484884863987[0] = 0;
   out_1503933484884863987[1] = 0;
   out_1503933484884863987[2] = 0;
   out_1503933484884863987[3] = 0;
   out_1503933484884863987[4] = 0;
   out_1503933484884863987[5] = 0;
   out_1503933484884863987[6] = 1;
   out_1503933484884863987[7] = 0;
   out_1503933484884863987[8] = 0;
}
void h_24(double *state, double *unused, double *out_7955775995665634223) {
   out_7955775995665634223[0] = state[4];
   out_7955775995665634223[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6777427767797876581) {
   out_6777427767797876581[0] = 0;
   out_6777427767797876581[1] = 0;
   out_6777427767797876581[2] = 0;
   out_6777427767797876581[3] = 0;
   out_6777427767797876581[4] = 1;
   out_6777427767797876581[5] = 0;
   out_6777427767797876581[6] = 0;
   out_6777427767797876581[7] = 0;
   out_6777427767797876581[8] = 0;
   out_6777427767797876581[9] = 0;
   out_6777427767797876581[10] = 0;
   out_6777427767797876581[11] = 0;
   out_6777427767797876581[12] = 0;
   out_6777427767797876581[13] = 0;
   out_6777427767797876581[14] = 1;
   out_6777427767797876581[15] = 0;
   out_6777427767797876581[16] = 0;
   out_6777427767797876581[17] = 0;
}
void h_30(double *state, double *unused, double *out_10697938629569704) {
   out_10697938629569704[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4022266443392112614) {
   out_4022266443392112614[0] = 0;
   out_4022266443392112614[1] = 0;
   out_4022266443392112614[2] = 0;
   out_4022266443392112614[3] = 0;
   out_4022266443392112614[4] = 1;
   out_4022266443392112614[5] = 0;
   out_4022266443392112614[6] = 0;
   out_4022266443392112614[7] = 0;
   out_4022266443392112614[8] = 0;
}
void h_26(double *state, double *unused, double *out_4990975329031275941) {
   out_4990975329031275941[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2237569833989192237) {
   out_2237569833989192237[0] = 0;
   out_2237569833989192237[1] = 0;
   out_2237569833989192237[2] = 0;
   out_2237569833989192237[3] = 0;
   out_2237569833989192237[4] = 0;
   out_2237569833989192237[5] = 0;
   out_2237569833989192237[6] = 0;
   out_2237569833989192237[7] = 1;
   out_2237569833989192237[8] = 0;
}
void h_27(double *state, double *unused, double *out_1439665929093890077) {
   out_1439665929093890077[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6245860514576055831) {
   out_6245860514576055831[0] = 0;
   out_6245860514576055831[1] = 0;
   out_6245860514576055831[2] = 0;
   out_6245860514576055831[3] = 1;
   out_6245860514576055831[4] = 0;
   out_6245860514576055831[5] = 0;
   out_6245860514576055831[6] = 0;
   out_6245860514576055831[7] = 0;
   out_6245860514576055831[8] = 0;
}
void h_29(double *state, double *unused, double *out_6767830816989016773) {
   out_6767830816989016773[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4532497787706504798) {
   out_4532497787706504798[0] = 0;
   out_4532497787706504798[1] = 1;
   out_4532497787706504798[2] = 0;
   out_4532497787706504798[3] = 0;
   out_4532497787706504798[4] = 0;
   out_4532497787706504798[5] = 0;
   out_4532497787706504798[6] = 0;
   out_4532497787706504798[7] = 0;
   out_4532497787706504798[8] = 0;
}
void h_28(double *state, double *unused, double *out_6441339196754735722) {
   out_6441339196754735722[0] = state[0];
}
void H_28(double *state, double *unused, double *out_549901229363025776) {
   out_549901229363025776[0] = 1;
   out_549901229363025776[1] = 0;
   out_549901229363025776[2] = 0;
   out_549901229363025776[3] = 0;
   out_549901229363025776[4] = 0;
   out_549901229363025776[5] = 0;
   out_549901229363025776[6] = 0;
   out_549901229363025776[7] = 0;
   out_549901229363025776[8] = 0;
}
void h_31(double *state, double *unused, double *out_7852677319404066476) {
   out_7852677319404066476[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2863777936222543713) {
   out_2863777936222543713[0] = 0;
   out_2863777936222543713[1] = 0;
   out_2863777936222543713[2] = 0;
   out_2863777936222543713[3] = 0;
   out_2863777936222543713[4] = 0;
   out_2863777936222543713[5] = 0;
   out_2863777936222543713[6] = 0;
   out_2863777936222543713[7] = 0;
   out_2863777936222543713[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2474516941466880566) {
  err_fun(nom_x, delta_x, out_2474516941466880566);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9123673598000259248) {
  inv_err_fun(nom_x, true_x, out_9123673598000259248);
}
void car_H_mod_fun(double *state, double *out_2745915890047717913) {
  H_mod_fun(state, out_2745915890047717913);
}
void car_f_fun(double *state, double dt, double *out_2687781910986821476) {
  f_fun(state,  dt, out_2687781910986821476);
}
void car_F_fun(double *state, double dt, double *out_1020427047685603609) {
  F_fun(state,  dt, out_1020427047685603609);
}
void car_h_25(double *state, double *unused, double *out_89412062583394035) {
  h_25(state, unused, out_89412062583394035);
}
void car_H_25(double *state, double *unused, double *out_1503933484884863987) {
  H_25(state, unused, out_1503933484884863987);
}
void car_h_24(double *state, double *unused, double *out_7955775995665634223) {
  h_24(state, unused, out_7955775995665634223);
}
void car_H_24(double *state, double *unused, double *out_6777427767797876581) {
  H_24(state, unused, out_6777427767797876581);
}
void car_h_30(double *state, double *unused, double *out_10697938629569704) {
  h_30(state, unused, out_10697938629569704);
}
void car_H_30(double *state, double *unused, double *out_4022266443392112614) {
  H_30(state, unused, out_4022266443392112614);
}
void car_h_26(double *state, double *unused, double *out_4990975329031275941) {
  h_26(state, unused, out_4990975329031275941);
}
void car_H_26(double *state, double *unused, double *out_2237569833989192237) {
  H_26(state, unused, out_2237569833989192237);
}
void car_h_27(double *state, double *unused, double *out_1439665929093890077) {
  h_27(state, unused, out_1439665929093890077);
}
void car_H_27(double *state, double *unused, double *out_6245860514576055831) {
  H_27(state, unused, out_6245860514576055831);
}
void car_h_29(double *state, double *unused, double *out_6767830816989016773) {
  h_29(state, unused, out_6767830816989016773);
}
void car_H_29(double *state, double *unused, double *out_4532497787706504798) {
  H_29(state, unused, out_4532497787706504798);
}
void car_h_28(double *state, double *unused, double *out_6441339196754735722) {
  h_28(state, unused, out_6441339196754735722);
}
void car_H_28(double *state, double *unused, double *out_549901229363025776) {
  H_28(state, unused, out_549901229363025776);
}
void car_h_31(double *state, double *unused, double *out_7852677319404066476) {
  h_31(state, unused, out_7852677319404066476);
}
void car_H_31(double *state, double *unused, double *out_2863777936222543713) {
  H_31(state, unused, out_2863777936222543713);
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
