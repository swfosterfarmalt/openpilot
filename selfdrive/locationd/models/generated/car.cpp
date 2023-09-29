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
void err_fun(double *nom_x, double *delta_x, double *out_3975600348957649681) {
   out_3975600348957649681[0] = delta_x[0] + nom_x[0];
   out_3975600348957649681[1] = delta_x[1] + nom_x[1];
   out_3975600348957649681[2] = delta_x[2] + nom_x[2];
   out_3975600348957649681[3] = delta_x[3] + nom_x[3];
   out_3975600348957649681[4] = delta_x[4] + nom_x[4];
   out_3975600348957649681[5] = delta_x[5] + nom_x[5];
   out_3975600348957649681[6] = delta_x[6] + nom_x[6];
   out_3975600348957649681[7] = delta_x[7] + nom_x[7];
   out_3975600348957649681[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7988676026297517892) {
   out_7988676026297517892[0] = -nom_x[0] + true_x[0];
   out_7988676026297517892[1] = -nom_x[1] + true_x[1];
   out_7988676026297517892[2] = -nom_x[2] + true_x[2];
   out_7988676026297517892[3] = -nom_x[3] + true_x[3];
   out_7988676026297517892[4] = -nom_x[4] + true_x[4];
   out_7988676026297517892[5] = -nom_x[5] + true_x[5];
   out_7988676026297517892[6] = -nom_x[6] + true_x[6];
   out_7988676026297517892[7] = -nom_x[7] + true_x[7];
   out_7988676026297517892[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1700816133426219037) {
   out_1700816133426219037[0] = 1.0;
   out_1700816133426219037[1] = 0;
   out_1700816133426219037[2] = 0;
   out_1700816133426219037[3] = 0;
   out_1700816133426219037[4] = 0;
   out_1700816133426219037[5] = 0;
   out_1700816133426219037[6] = 0;
   out_1700816133426219037[7] = 0;
   out_1700816133426219037[8] = 0;
   out_1700816133426219037[9] = 0;
   out_1700816133426219037[10] = 1.0;
   out_1700816133426219037[11] = 0;
   out_1700816133426219037[12] = 0;
   out_1700816133426219037[13] = 0;
   out_1700816133426219037[14] = 0;
   out_1700816133426219037[15] = 0;
   out_1700816133426219037[16] = 0;
   out_1700816133426219037[17] = 0;
   out_1700816133426219037[18] = 0;
   out_1700816133426219037[19] = 0;
   out_1700816133426219037[20] = 1.0;
   out_1700816133426219037[21] = 0;
   out_1700816133426219037[22] = 0;
   out_1700816133426219037[23] = 0;
   out_1700816133426219037[24] = 0;
   out_1700816133426219037[25] = 0;
   out_1700816133426219037[26] = 0;
   out_1700816133426219037[27] = 0;
   out_1700816133426219037[28] = 0;
   out_1700816133426219037[29] = 0;
   out_1700816133426219037[30] = 1.0;
   out_1700816133426219037[31] = 0;
   out_1700816133426219037[32] = 0;
   out_1700816133426219037[33] = 0;
   out_1700816133426219037[34] = 0;
   out_1700816133426219037[35] = 0;
   out_1700816133426219037[36] = 0;
   out_1700816133426219037[37] = 0;
   out_1700816133426219037[38] = 0;
   out_1700816133426219037[39] = 0;
   out_1700816133426219037[40] = 1.0;
   out_1700816133426219037[41] = 0;
   out_1700816133426219037[42] = 0;
   out_1700816133426219037[43] = 0;
   out_1700816133426219037[44] = 0;
   out_1700816133426219037[45] = 0;
   out_1700816133426219037[46] = 0;
   out_1700816133426219037[47] = 0;
   out_1700816133426219037[48] = 0;
   out_1700816133426219037[49] = 0;
   out_1700816133426219037[50] = 1.0;
   out_1700816133426219037[51] = 0;
   out_1700816133426219037[52] = 0;
   out_1700816133426219037[53] = 0;
   out_1700816133426219037[54] = 0;
   out_1700816133426219037[55] = 0;
   out_1700816133426219037[56] = 0;
   out_1700816133426219037[57] = 0;
   out_1700816133426219037[58] = 0;
   out_1700816133426219037[59] = 0;
   out_1700816133426219037[60] = 1.0;
   out_1700816133426219037[61] = 0;
   out_1700816133426219037[62] = 0;
   out_1700816133426219037[63] = 0;
   out_1700816133426219037[64] = 0;
   out_1700816133426219037[65] = 0;
   out_1700816133426219037[66] = 0;
   out_1700816133426219037[67] = 0;
   out_1700816133426219037[68] = 0;
   out_1700816133426219037[69] = 0;
   out_1700816133426219037[70] = 1.0;
   out_1700816133426219037[71] = 0;
   out_1700816133426219037[72] = 0;
   out_1700816133426219037[73] = 0;
   out_1700816133426219037[74] = 0;
   out_1700816133426219037[75] = 0;
   out_1700816133426219037[76] = 0;
   out_1700816133426219037[77] = 0;
   out_1700816133426219037[78] = 0;
   out_1700816133426219037[79] = 0;
   out_1700816133426219037[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5772475122730241095) {
   out_5772475122730241095[0] = state[0];
   out_5772475122730241095[1] = state[1];
   out_5772475122730241095[2] = state[2];
   out_5772475122730241095[3] = state[3];
   out_5772475122730241095[4] = state[4];
   out_5772475122730241095[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5772475122730241095[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5772475122730241095[7] = state[7];
   out_5772475122730241095[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8078197424882555923) {
   out_8078197424882555923[0] = 1;
   out_8078197424882555923[1] = 0;
   out_8078197424882555923[2] = 0;
   out_8078197424882555923[3] = 0;
   out_8078197424882555923[4] = 0;
   out_8078197424882555923[5] = 0;
   out_8078197424882555923[6] = 0;
   out_8078197424882555923[7] = 0;
   out_8078197424882555923[8] = 0;
   out_8078197424882555923[9] = 0;
   out_8078197424882555923[10] = 1;
   out_8078197424882555923[11] = 0;
   out_8078197424882555923[12] = 0;
   out_8078197424882555923[13] = 0;
   out_8078197424882555923[14] = 0;
   out_8078197424882555923[15] = 0;
   out_8078197424882555923[16] = 0;
   out_8078197424882555923[17] = 0;
   out_8078197424882555923[18] = 0;
   out_8078197424882555923[19] = 0;
   out_8078197424882555923[20] = 1;
   out_8078197424882555923[21] = 0;
   out_8078197424882555923[22] = 0;
   out_8078197424882555923[23] = 0;
   out_8078197424882555923[24] = 0;
   out_8078197424882555923[25] = 0;
   out_8078197424882555923[26] = 0;
   out_8078197424882555923[27] = 0;
   out_8078197424882555923[28] = 0;
   out_8078197424882555923[29] = 0;
   out_8078197424882555923[30] = 1;
   out_8078197424882555923[31] = 0;
   out_8078197424882555923[32] = 0;
   out_8078197424882555923[33] = 0;
   out_8078197424882555923[34] = 0;
   out_8078197424882555923[35] = 0;
   out_8078197424882555923[36] = 0;
   out_8078197424882555923[37] = 0;
   out_8078197424882555923[38] = 0;
   out_8078197424882555923[39] = 0;
   out_8078197424882555923[40] = 1;
   out_8078197424882555923[41] = 0;
   out_8078197424882555923[42] = 0;
   out_8078197424882555923[43] = 0;
   out_8078197424882555923[44] = 0;
   out_8078197424882555923[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8078197424882555923[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8078197424882555923[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8078197424882555923[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8078197424882555923[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8078197424882555923[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8078197424882555923[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8078197424882555923[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8078197424882555923[53] = -9.8000000000000007*dt;
   out_8078197424882555923[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8078197424882555923[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8078197424882555923[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8078197424882555923[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8078197424882555923[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8078197424882555923[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8078197424882555923[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8078197424882555923[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8078197424882555923[62] = 0;
   out_8078197424882555923[63] = 0;
   out_8078197424882555923[64] = 0;
   out_8078197424882555923[65] = 0;
   out_8078197424882555923[66] = 0;
   out_8078197424882555923[67] = 0;
   out_8078197424882555923[68] = 0;
   out_8078197424882555923[69] = 0;
   out_8078197424882555923[70] = 1;
   out_8078197424882555923[71] = 0;
   out_8078197424882555923[72] = 0;
   out_8078197424882555923[73] = 0;
   out_8078197424882555923[74] = 0;
   out_8078197424882555923[75] = 0;
   out_8078197424882555923[76] = 0;
   out_8078197424882555923[77] = 0;
   out_8078197424882555923[78] = 0;
   out_8078197424882555923[79] = 0;
   out_8078197424882555923[80] = 1;
}
void h_25(double *state, double *unused, double *out_188617941975566043) {
   out_188617941975566043[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3516111807592801652) {
   out_3516111807592801652[0] = 0;
   out_3516111807592801652[1] = 0;
   out_3516111807592801652[2] = 0;
   out_3516111807592801652[3] = 0;
   out_3516111807592801652[4] = 0;
   out_3516111807592801652[5] = 0;
   out_3516111807592801652[6] = 1;
   out_3516111807592801652[7] = 0;
   out_3516111807592801652[8] = 0;
}
void h_24(double *state, double *unused, double *out_7581416323553796529) {
   out_7581416323553796529[0] = state[4];
   out_7581416323553796529[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1828295371723697461) {
   out_1828295371723697461[0] = 0;
   out_1828295371723697461[1] = 0;
   out_1828295371723697461[2] = 0;
   out_1828295371723697461[3] = 0;
   out_1828295371723697461[4] = 1;
   out_1828295371723697461[5] = 0;
   out_1828295371723697461[6] = 0;
   out_1828295371723697461[7] = 0;
   out_1828295371723697461[8] = 0;
   out_1828295371723697461[9] = 0;
   out_1828295371723697461[10] = 0;
   out_1828295371723697461[11] = 0;
   out_1828295371723697461[12] = 0;
   out_1828295371723697461[13] = 0;
   out_1828295371723697461[14] = 1;
   out_1828295371723697461[15] = 0;
   out_1828295371723697461[16] = 0;
   out_1828295371723697461[17] = 0;
}
void h_30(double *state, double *unused, double *out_6408995124822580012) {
   out_6408995124822580012[0] = state[4];
}
void H_30(double *state, double *unused, double *out_997778849085553025) {
   out_997778849085553025[0] = 0;
   out_997778849085553025[1] = 0;
   out_997778849085553025[2] = 0;
   out_997778849085553025[3] = 0;
   out_997778849085553025[4] = 1;
   out_997778849085553025[5] = 0;
   out_997778849085553025[6] = 0;
   out_997778849085553025[7] = 0;
   out_997778849085553025[8] = 0;
}
void h_26(double *state, double *unused, double *out_5805679240146102495) {
   out_5805679240146102495[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7257615126466857876) {
   out_7257615126466857876[0] = 0;
   out_7257615126466857876[1] = 0;
   out_7257615126466857876[2] = 0;
   out_7257615126466857876[3] = 0;
   out_7257615126466857876[4] = 0;
   out_7257615126466857876[5] = 0;
   out_7257615126466857876[6] = 0;
   out_7257615126466857876[7] = 1;
   out_7257615126466857876[8] = 0;
}
void h_27(double *state, double *unused, double *out_2415787378833131608) {
   out_2415787378833131608[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1225815222098390192) {
   out_1225815222098390192[0] = 0;
   out_1225815222098390192[1] = 0;
   out_1225815222098390192[2] = 0;
   out_1225815222098390192[3] = 1;
   out_1225815222098390192[4] = 0;
   out_1225815222098390192[5] = 0;
   out_1225815222098390192[6] = 0;
   out_1225815222098390192[7] = 0;
   out_1225815222098390192[8] = 0;
}
void h_29(double *state, double *unused, double *out_5465485271920917577) {
   out_5465485271920917577[0] = state[1];
}
void H_29(double *state, double *unused, double *out_487547504771160841) {
   out_487547504771160841[0] = 0;
   out_487547504771160841[1] = 1;
   out_487547504771160841[2] = 0;
   out_487547504771160841[3] = 0;
   out_487547504771160841[4] = 0;
   out_487547504771160841[5] = 0;
   out_487547504771160841[6] = 0;
   out_487547504771160841[7] = 0;
   out_487547504771160841[8] = 0;
}
void h_28(double *state, double *unused, double *out_2585885888827714037) {
   out_2585885888827714037[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5569946521840691415) {
   out_5569946521840691415[0] = 1;
   out_5569946521840691415[1] = 0;
   out_5569946521840691415[2] = 0;
   out_5569946521840691415[3] = 0;
   out_5569946521840691415[4] = 0;
   out_5569946521840691415[5] = 0;
   out_5569946521840691415[6] = 0;
   out_5569946521840691415[7] = 0;
   out_5569946521840691415[8] = 0;
}
void h_31(double *state, double *unused, double *out_5966829253865094086) {
   out_5966829253865094086[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7883823228700209352) {
   out_7883823228700209352[0] = 0;
   out_7883823228700209352[1] = 0;
   out_7883823228700209352[2] = 0;
   out_7883823228700209352[3] = 0;
   out_7883823228700209352[4] = 0;
   out_7883823228700209352[5] = 0;
   out_7883823228700209352[6] = 0;
   out_7883823228700209352[7] = 0;
   out_7883823228700209352[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3975600348957649681) {
  err_fun(nom_x, delta_x, out_3975600348957649681);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7988676026297517892) {
  inv_err_fun(nom_x, true_x, out_7988676026297517892);
}
void car_H_mod_fun(double *state, double *out_1700816133426219037) {
  H_mod_fun(state, out_1700816133426219037);
}
void car_f_fun(double *state, double dt, double *out_5772475122730241095) {
  f_fun(state,  dt, out_5772475122730241095);
}
void car_F_fun(double *state, double dt, double *out_8078197424882555923) {
  F_fun(state,  dt, out_8078197424882555923);
}
void car_h_25(double *state, double *unused, double *out_188617941975566043) {
  h_25(state, unused, out_188617941975566043);
}
void car_H_25(double *state, double *unused, double *out_3516111807592801652) {
  H_25(state, unused, out_3516111807592801652);
}
void car_h_24(double *state, double *unused, double *out_7581416323553796529) {
  h_24(state, unused, out_7581416323553796529);
}
void car_H_24(double *state, double *unused, double *out_1828295371723697461) {
  H_24(state, unused, out_1828295371723697461);
}
void car_h_30(double *state, double *unused, double *out_6408995124822580012) {
  h_30(state, unused, out_6408995124822580012);
}
void car_H_30(double *state, double *unused, double *out_997778849085553025) {
  H_30(state, unused, out_997778849085553025);
}
void car_h_26(double *state, double *unused, double *out_5805679240146102495) {
  h_26(state, unused, out_5805679240146102495);
}
void car_H_26(double *state, double *unused, double *out_7257615126466857876) {
  H_26(state, unused, out_7257615126466857876);
}
void car_h_27(double *state, double *unused, double *out_2415787378833131608) {
  h_27(state, unused, out_2415787378833131608);
}
void car_H_27(double *state, double *unused, double *out_1225815222098390192) {
  H_27(state, unused, out_1225815222098390192);
}
void car_h_29(double *state, double *unused, double *out_5465485271920917577) {
  h_29(state, unused, out_5465485271920917577);
}
void car_H_29(double *state, double *unused, double *out_487547504771160841) {
  H_29(state, unused, out_487547504771160841);
}
void car_h_28(double *state, double *unused, double *out_2585885888827714037) {
  h_28(state, unused, out_2585885888827714037);
}
void car_H_28(double *state, double *unused, double *out_5569946521840691415) {
  H_28(state, unused, out_5569946521840691415);
}
void car_h_31(double *state, double *unused, double *out_5966829253865094086) {
  h_31(state, unused, out_5966829253865094086);
}
void car_H_31(double *state, double *unused, double *out_7883823228700209352) {
  H_31(state, unused, out_7883823228700209352);
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
