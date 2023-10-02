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
void err_fun(double *nom_x, double *delta_x, double *out_8485107762980471691) {
   out_8485107762980471691[0] = delta_x[0] + nom_x[0];
   out_8485107762980471691[1] = delta_x[1] + nom_x[1];
   out_8485107762980471691[2] = delta_x[2] + nom_x[2];
   out_8485107762980471691[3] = delta_x[3] + nom_x[3];
   out_8485107762980471691[4] = delta_x[4] + nom_x[4];
   out_8485107762980471691[5] = delta_x[5] + nom_x[5];
   out_8485107762980471691[6] = delta_x[6] + nom_x[6];
   out_8485107762980471691[7] = delta_x[7] + nom_x[7];
   out_8485107762980471691[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_752342401365760782) {
   out_752342401365760782[0] = -nom_x[0] + true_x[0];
   out_752342401365760782[1] = -nom_x[1] + true_x[1];
   out_752342401365760782[2] = -nom_x[2] + true_x[2];
   out_752342401365760782[3] = -nom_x[3] + true_x[3];
   out_752342401365760782[4] = -nom_x[4] + true_x[4];
   out_752342401365760782[5] = -nom_x[5] + true_x[5];
   out_752342401365760782[6] = -nom_x[6] + true_x[6];
   out_752342401365760782[7] = -nom_x[7] + true_x[7];
   out_752342401365760782[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4723183051327282475) {
   out_4723183051327282475[0] = 1.0;
   out_4723183051327282475[1] = 0;
   out_4723183051327282475[2] = 0;
   out_4723183051327282475[3] = 0;
   out_4723183051327282475[4] = 0;
   out_4723183051327282475[5] = 0;
   out_4723183051327282475[6] = 0;
   out_4723183051327282475[7] = 0;
   out_4723183051327282475[8] = 0;
   out_4723183051327282475[9] = 0;
   out_4723183051327282475[10] = 1.0;
   out_4723183051327282475[11] = 0;
   out_4723183051327282475[12] = 0;
   out_4723183051327282475[13] = 0;
   out_4723183051327282475[14] = 0;
   out_4723183051327282475[15] = 0;
   out_4723183051327282475[16] = 0;
   out_4723183051327282475[17] = 0;
   out_4723183051327282475[18] = 0;
   out_4723183051327282475[19] = 0;
   out_4723183051327282475[20] = 1.0;
   out_4723183051327282475[21] = 0;
   out_4723183051327282475[22] = 0;
   out_4723183051327282475[23] = 0;
   out_4723183051327282475[24] = 0;
   out_4723183051327282475[25] = 0;
   out_4723183051327282475[26] = 0;
   out_4723183051327282475[27] = 0;
   out_4723183051327282475[28] = 0;
   out_4723183051327282475[29] = 0;
   out_4723183051327282475[30] = 1.0;
   out_4723183051327282475[31] = 0;
   out_4723183051327282475[32] = 0;
   out_4723183051327282475[33] = 0;
   out_4723183051327282475[34] = 0;
   out_4723183051327282475[35] = 0;
   out_4723183051327282475[36] = 0;
   out_4723183051327282475[37] = 0;
   out_4723183051327282475[38] = 0;
   out_4723183051327282475[39] = 0;
   out_4723183051327282475[40] = 1.0;
   out_4723183051327282475[41] = 0;
   out_4723183051327282475[42] = 0;
   out_4723183051327282475[43] = 0;
   out_4723183051327282475[44] = 0;
   out_4723183051327282475[45] = 0;
   out_4723183051327282475[46] = 0;
   out_4723183051327282475[47] = 0;
   out_4723183051327282475[48] = 0;
   out_4723183051327282475[49] = 0;
   out_4723183051327282475[50] = 1.0;
   out_4723183051327282475[51] = 0;
   out_4723183051327282475[52] = 0;
   out_4723183051327282475[53] = 0;
   out_4723183051327282475[54] = 0;
   out_4723183051327282475[55] = 0;
   out_4723183051327282475[56] = 0;
   out_4723183051327282475[57] = 0;
   out_4723183051327282475[58] = 0;
   out_4723183051327282475[59] = 0;
   out_4723183051327282475[60] = 1.0;
   out_4723183051327282475[61] = 0;
   out_4723183051327282475[62] = 0;
   out_4723183051327282475[63] = 0;
   out_4723183051327282475[64] = 0;
   out_4723183051327282475[65] = 0;
   out_4723183051327282475[66] = 0;
   out_4723183051327282475[67] = 0;
   out_4723183051327282475[68] = 0;
   out_4723183051327282475[69] = 0;
   out_4723183051327282475[70] = 1.0;
   out_4723183051327282475[71] = 0;
   out_4723183051327282475[72] = 0;
   out_4723183051327282475[73] = 0;
   out_4723183051327282475[74] = 0;
   out_4723183051327282475[75] = 0;
   out_4723183051327282475[76] = 0;
   out_4723183051327282475[77] = 0;
   out_4723183051327282475[78] = 0;
   out_4723183051327282475[79] = 0;
   out_4723183051327282475[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6080379077548842545) {
   out_6080379077548842545[0] = state[0];
   out_6080379077548842545[1] = state[1];
   out_6080379077548842545[2] = state[2];
   out_6080379077548842545[3] = state[3];
   out_6080379077548842545[4] = state[4];
   out_6080379077548842545[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6080379077548842545[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6080379077548842545[7] = state[7];
   out_6080379077548842545[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5795928051360757777) {
   out_5795928051360757777[0] = 1;
   out_5795928051360757777[1] = 0;
   out_5795928051360757777[2] = 0;
   out_5795928051360757777[3] = 0;
   out_5795928051360757777[4] = 0;
   out_5795928051360757777[5] = 0;
   out_5795928051360757777[6] = 0;
   out_5795928051360757777[7] = 0;
   out_5795928051360757777[8] = 0;
   out_5795928051360757777[9] = 0;
   out_5795928051360757777[10] = 1;
   out_5795928051360757777[11] = 0;
   out_5795928051360757777[12] = 0;
   out_5795928051360757777[13] = 0;
   out_5795928051360757777[14] = 0;
   out_5795928051360757777[15] = 0;
   out_5795928051360757777[16] = 0;
   out_5795928051360757777[17] = 0;
   out_5795928051360757777[18] = 0;
   out_5795928051360757777[19] = 0;
   out_5795928051360757777[20] = 1;
   out_5795928051360757777[21] = 0;
   out_5795928051360757777[22] = 0;
   out_5795928051360757777[23] = 0;
   out_5795928051360757777[24] = 0;
   out_5795928051360757777[25] = 0;
   out_5795928051360757777[26] = 0;
   out_5795928051360757777[27] = 0;
   out_5795928051360757777[28] = 0;
   out_5795928051360757777[29] = 0;
   out_5795928051360757777[30] = 1;
   out_5795928051360757777[31] = 0;
   out_5795928051360757777[32] = 0;
   out_5795928051360757777[33] = 0;
   out_5795928051360757777[34] = 0;
   out_5795928051360757777[35] = 0;
   out_5795928051360757777[36] = 0;
   out_5795928051360757777[37] = 0;
   out_5795928051360757777[38] = 0;
   out_5795928051360757777[39] = 0;
   out_5795928051360757777[40] = 1;
   out_5795928051360757777[41] = 0;
   out_5795928051360757777[42] = 0;
   out_5795928051360757777[43] = 0;
   out_5795928051360757777[44] = 0;
   out_5795928051360757777[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5795928051360757777[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5795928051360757777[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5795928051360757777[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5795928051360757777[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5795928051360757777[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5795928051360757777[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5795928051360757777[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5795928051360757777[53] = -9.8000000000000007*dt;
   out_5795928051360757777[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5795928051360757777[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5795928051360757777[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5795928051360757777[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5795928051360757777[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5795928051360757777[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5795928051360757777[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5795928051360757777[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5795928051360757777[62] = 0;
   out_5795928051360757777[63] = 0;
   out_5795928051360757777[64] = 0;
   out_5795928051360757777[65] = 0;
   out_5795928051360757777[66] = 0;
   out_5795928051360757777[67] = 0;
   out_5795928051360757777[68] = 0;
   out_5795928051360757777[69] = 0;
   out_5795928051360757777[70] = 1;
   out_5795928051360757777[71] = 0;
   out_5795928051360757777[72] = 0;
   out_5795928051360757777[73] = 0;
   out_5795928051360757777[74] = 0;
   out_5795928051360757777[75] = 0;
   out_5795928051360757777[76] = 0;
   out_5795928051360757777[77] = 0;
   out_5795928051360757777[78] = 0;
   out_5795928051360757777[79] = 0;
   out_5795928051360757777[80] = 1;
}
void h_25(double *state, double *unused, double *out_7016055871807953819) {
   out_7016055871807953819[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5102981612663619538) {
   out_5102981612663619538[0] = 0;
   out_5102981612663619538[1] = 0;
   out_5102981612663619538[2] = 0;
   out_5102981612663619538[3] = 0;
   out_5102981612663619538[4] = 0;
   out_5102981612663619538[5] = 0;
   out_5102981612663619538[6] = 1;
   out_5102981612663619538[7] = 0;
   out_5102981612663619538[8] = 0;
}
void h_24(double *state, double *unused, double *out_242298661145333250) {
   out_242298661145333250[0] = state[4];
   out_242298661145333250[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2930332013658119972) {
   out_2930332013658119972[0] = 0;
   out_2930332013658119972[1] = 0;
   out_2930332013658119972[2] = 0;
   out_2930332013658119972[3] = 0;
   out_2930332013658119972[4] = 1;
   out_2930332013658119972[5] = 0;
   out_2930332013658119972[6] = 0;
   out_2930332013658119972[7] = 0;
   out_2930332013658119972[8] = 0;
   out_2930332013658119972[9] = 0;
   out_2930332013658119972[10] = 0;
   out_2930332013658119972[11] = 0;
   out_2930332013658119972[12] = 0;
   out_2930332013658119972[13] = 0;
   out_2930332013658119972[14] = 1;
   out_2930332013658119972[15] = 0;
   out_2930332013658119972[16] = 0;
   out_2930332013658119972[17] = 0;
}
void h_30(double *state, double *unused, double *out_7291249934092459708) {
   out_7291249934092459708[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6427072119554315323) {
   out_6427072119554315323[0] = 0;
   out_6427072119554315323[1] = 0;
   out_6427072119554315323[2] = 0;
   out_6427072119554315323[3] = 0;
   out_6427072119554315323[4] = 1;
   out_6427072119554315323[5] = 0;
   out_6427072119554315323[6] = 0;
   out_6427072119554315323[7] = 0;
   out_6427072119554315323[8] = 0;
}
void h_26(double *state, double *unused, double *out_8981697768828140668) {
   out_8981697768828140668[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8407507582424420139) {
   out_8407507582424420139[0] = 0;
   out_8407507582424420139[1] = 0;
   out_8407507582424420139[2] = 0;
   out_8407507582424420139[3] = 0;
   out_8407507582424420139[4] = 0;
   out_8407507582424420139[5] = 0;
   out_8407507582424420139[6] = 0;
   out_8407507582424420139[7] = 1;
   out_8407507582424420139[8] = 0;
}
void h_27(double *state, double *unused, double *out_5552402714342882709) {
   out_5552402714342882709[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8601835431354740234) {
   out_8601835431354740234[0] = 0;
   out_8601835431354740234[1] = 0;
   out_8601835431354740234[2] = 0;
   out_8601835431354740234[3] = 1;
   out_8601835431354740234[4] = 0;
   out_8601835431354740234[5] = 0;
   out_8601835431354740234[6] = 0;
   out_8601835431354740234[7] = 0;
   out_8601835431354740234[8] = 0;
}
void h_29(double *state, double *unused, double *out_2328869936411166476) {
   out_2328869936411166476[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8131545915485260349) {
   out_8131545915485260349[0] = 0;
   out_8131545915485260349[1] = 1;
   out_8131545915485260349[2] = 0;
   out_8131545915485260349[3] = 0;
   out_8131545915485260349[4] = 0;
   out_8131545915485260349[5] = 0;
   out_8131545915485260349[6] = 0;
   out_8131545915485260349[7] = 0;
   out_8131545915485260349[8] = 0;
}
void h_28(double *state, double *unused, double *out_7372524787142414587) {
   out_7372524787142414587[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3049146898415729775) {
   out_3049146898415729775[0] = 1;
   out_3049146898415729775[1] = 0;
   out_3049146898415729775[2] = 0;
   out_3049146898415729775[3] = 0;
   out_3049146898415729775[4] = 0;
   out_3049146898415729775[5] = 0;
   out_3049146898415729775[6] = 0;
   out_3049146898415729775[7] = 0;
   out_3049146898415729775[8] = 0;
}
void h_31(double *state, double *unused, double *out_2790810725183055913) {
   out_2790810725183055913[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5133627574540579966) {
   out_5133627574540579966[0] = 0;
   out_5133627574540579966[1] = 0;
   out_5133627574540579966[2] = 0;
   out_5133627574540579966[3] = 0;
   out_5133627574540579966[4] = 0;
   out_5133627574540579966[5] = 0;
   out_5133627574540579966[6] = 0;
   out_5133627574540579966[7] = 0;
   out_5133627574540579966[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8485107762980471691) {
  err_fun(nom_x, delta_x, out_8485107762980471691);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_752342401365760782) {
  inv_err_fun(nom_x, true_x, out_752342401365760782);
}
void car_H_mod_fun(double *state, double *out_4723183051327282475) {
  H_mod_fun(state, out_4723183051327282475);
}
void car_f_fun(double *state, double dt, double *out_6080379077548842545) {
  f_fun(state,  dt, out_6080379077548842545);
}
void car_F_fun(double *state, double dt, double *out_5795928051360757777) {
  F_fun(state,  dt, out_5795928051360757777);
}
void car_h_25(double *state, double *unused, double *out_7016055871807953819) {
  h_25(state, unused, out_7016055871807953819);
}
void car_H_25(double *state, double *unused, double *out_5102981612663619538) {
  H_25(state, unused, out_5102981612663619538);
}
void car_h_24(double *state, double *unused, double *out_242298661145333250) {
  h_24(state, unused, out_242298661145333250);
}
void car_H_24(double *state, double *unused, double *out_2930332013658119972) {
  H_24(state, unused, out_2930332013658119972);
}
void car_h_30(double *state, double *unused, double *out_7291249934092459708) {
  h_30(state, unused, out_7291249934092459708);
}
void car_H_30(double *state, double *unused, double *out_6427072119554315323) {
  H_30(state, unused, out_6427072119554315323);
}
void car_h_26(double *state, double *unused, double *out_8981697768828140668) {
  h_26(state, unused, out_8981697768828140668);
}
void car_H_26(double *state, double *unused, double *out_8407507582424420139) {
  H_26(state, unused, out_8407507582424420139);
}
void car_h_27(double *state, double *unused, double *out_5552402714342882709) {
  h_27(state, unused, out_5552402714342882709);
}
void car_H_27(double *state, double *unused, double *out_8601835431354740234) {
  H_27(state, unused, out_8601835431354740234);
}
void car_h_29(double *state, double *unused, double *out_2328869936411166476) {
  h_29(state, unused, out_2328869936411166476);
}
void car_H_29(double *state, double *unused, double *out_8131545915485260349) {
  H_29(state, unused, out_8131545915485260349);
}
void car_h_28(double *state, double *unused, double *out_7372524787142414587) {
  h_28(state, unused, out_7372524787142414587);
}
void car_H_28(double *state, double *unused, double *out_3049146898415729775) {
  H_28(state, unused, out_3049146898415729775);
}
void car_h_31(double *state, double *unused, double *out_2790810725183055913) {
  h_31(state, unused, out_2790810725183055913);
}
void car_H_31(double *state, double *unused, double *out_5133627574540579966) {
  H_31(state, unused, out_5133627574540579966);
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
