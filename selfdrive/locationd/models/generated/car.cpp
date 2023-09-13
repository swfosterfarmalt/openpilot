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
void err_fun(double *nom_x, double *delta_x, double *out_7443385576447730734) {
   out_7443385576447730734[0] = delta_x[0] + nom_x[0];
   out_7443385576447730734[1] = delta_x[1] + nom_x[1];
   out_7443385576447730734[2] = delta_x[2] + nom_x[2];
   out_7443385576447730734[3] = delta_x[3] + nom_x[3];
   out_7443385576447730734[4] = delta_x[4] + nom_x[4];
   out_7443385576447730734[5] = delta_x[5] + nom_x[5];
   out_7443385576447730734[6] = delta_x[6] + nom_x[6];
   out_7443385576447730734[7] = delta_x[7] + nom_x[7];
   out_7443385576447730734[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3494304989908180493) {
   out_3494304989908180493[0] = -nom_x[0] + true_x[0];
   out_3494304989908180493[1] = -nom_x[1] + true_x[1];
   out_3494304989908180493[2] = -nom_x[2] + true_x[2];
   out_3494304989908180493[3] = -nom_x[3] + true_x[3];
   out_3494304989908180493[4] = -nom_x[4] + true_x[4];
   out_3494304989908180493[5] = -nom_x[5] + true_x[5];
   out_3494304989908180493[6] = -nom_x[6] + true_x[6];
   out_3494304989908180493[7] = -nom_x[7] + true_x[7];
   out_3494304989908180493[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6340163641565670916) {
   out_6340163641565670916[0] = 1.0;
   out_6340163641565670916[1] = 0;
   out_6340163641565670916[2] = 0;
   out_6340163641565670916[3] = 0;
   out_6340163641565670916[4] = 0;
   out_6340163641565670916[5] = 0;
   out_6340163641565670916[6] = 0;
   out_6340163641565670916[7] = 0;
   out_6340163641565670916[8] = 0;
   out_6340163641565670916[9] = 0;
   out_6340163641565670916[10] = 1.0;
   out_6340163641565670916[11] = 0;
   out_6340163641565670916[12] = 0;
   out_6340163641565670916[13] = 0;
   out_6340163641565670916[14] = 0;
   out_6340163641565670916[15] = 0;
   out_6340163641565670916[16] = 0;
   out_6340163641565670916[17] = 0;
   out_6340163641565670916[18] = 0;
   out_6340163641565670916[19] = 0;
   out_6340163641565670916[20] = 1.0;
   out_6340163641565670916[21] = 0;
   out_6340163641565670916[22] = 0;
   out_6340163641565670916[23] = 0;
   out_6340163641565670916[24] = 0;
   out_6340163641565670916[25] = 0;
   out_6340163641565670916[26] = 0;
   out_6340163641565670916[27] = 0;
   out_6340163641565670916[28] = 0;
   out_6340163641565670916[29] = 0;
   out_6340163641565670916[30] = 1.0;
   out_6340163641565670916[31] = 0;
   out_6340163641565670916[32] = 0;
   out_6340163641565670916[33] = 0;
   out_6340163641565670916[34] = 0;
   out_6340163641565670916[35] = 0;
   out_6340163641565670916[36] = 0;
   out_6340163641565670916[37] = 0;
   out_6340163641565670916[38] = 0;
   out_6340163641565670916[39] = 0;
   out_6340163641565670916[40] = 1.0;
   out_6340163641565670916[41] = 0;
   out_6340163641565670916[42] = 0;
   out_6340163641565670916[43] = 0;
   out_6340163641565670916[44] = 0;
   out_6340163641565670916[45] = 0;
   out_6340163641565670916[46] = 0;
   out_6340163641565670916[47] = 0;
   out_6340163641565670916[48] = 0;
   out_6340163641565670916[49] = 0;
   out_6340163641565670916[50] = 1.0;
   out_6340163641565670916[51] = 0;
   out_6340163641565670916[52] = 0;
   out_6340163641565670916[53] = 0;
   out_6340163641565670916[54] = 0;
   out_6340163641565670916[55] = 0;
   out_6340163641565670916[56] = 0;
   out_6340163641565670916[57] = 0;
   out_6340163641565670916[58] = 0;
   out_6340163641565670916[59] = 0;
   out_6340163641565670916[60] = 1.0;
   out_6340163641565670916[61] = 0;
   out_6340163641565670916[62] = 0;
   out_6340163641565670916[63] = 0;
   out_6340163641565670916[64] = 0;
   out_6340163641565670916[65] = 0;
   out_6340163641565670916[66] = 0;
   out_6340163641565670916[67] = 0;
   out_6340163641565670916[68] = 0;
   out_6340163641565670916[69] = 0;
   out_6340163641565670916[70] = 1.0;
   out_6340163641565670916[71] = 0;
   out_6340163641565670916[72] = 0;
   out_6340163641565670916[73] = 0;
   out_6340163641565670916[74] = 0;
   out_6340163641565670916[75] = 0;
   out_6340163641565670916[76] = 0;
   out_6340163641565670916[77] = 0;
   out_6340163641565670916[78] = 0;
   out_6340163641565670916[79] = 0;
   out_6340163641565670916[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2083752042724885918) {
   out_2083752042724885918[0] = state[0];
   out_2083752042724885918[1] = state[1];
   out_2083752042724885918[2] = state[2];
   out_2083752042724885918[3] = state[3];
   out_2083752042724885918[4] = state[4];
   out_2083752042724885918[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2083752042724885918[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2083752042724885918[7] = state[7];
   out_2083752042724885918[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8472269591488097829) {
   out_8472269591488097829[0] = 1;
   out_8472269591488097829[1] = 0;
   out_8472269591488097829[2] = 0;
   out_8472269591488097829[3] = 0;
   out_8472269591488097829[4] = 0;
   out_8472269591488097829[5] = 0;
   out_8472269591488097829[6] = 0;
   out_8472269591488097829[7] = 0;
   out_8472269591488097829[8] = 0;
   out_8472269591488097829[9] = 0;
   out_8472269591488097829[10] = 1;
   out_8472269591488097829[11] = 0;
   out_8472269591488097829[12] = 0;
   out_8472269591488097829[13] = 0;
   out_8472269591488097829[14] = 0;
   out_8472269591488097829[15] = 0;
   out_8472269591488097829[16] = 0;
   out_8472269591488097829[17] = 0;
   out_8472269591488097829[18] = 0;
   out_8472269591488097829[19] = 0;
   out_8472269591488097829[20] = 1;
   out_8472269591488097829[21] = 0;
   out_8472269591488097829[22] = 0;
   out_8472269591488097829[23] = 0;
   out_8472269591488097829[24] = 0;
   out_8472269591488097829[25] = 0;
   out_8472269591488097829[26] = 0;
   out_8472269591488097829[27] = 0;
   out_8472269591488097829[28] = 0;
   out_8472269591488097829[29] = 0;
   out_8472269591488097829[30] = 1;
   out_8472269591488097829[31] = 0;
   out_8472269591488097829[32] = 0;
   out_8472269591488097829[33] = 0;
   out_8472269591488097829[34] = 0;
   out_8472269591488097829[35] = 0;
   out_8472269591488097829[36] = 0;
   out_8472269591488097829[37] = 0;
   out_8472269591488097829[38] = 0;
   out_8472269591488097829[39] = 0;
   out_8472269591488097829[40] = 1;
   out_8472269591488097829[41] = 0;
   out_8472269591488097829[42] = 0;
   out_8472269591488097829[43] = 0;
   out_8472269591488097829[44] = 0;
   out_8472269591488097829[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8472269591488097829[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8472269591488097829[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8472269591488097829[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8472269591488097829[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8472269591488097829[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8472269591488097829[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8472269591488097829[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8472269591488097829[53] = -9.8000000000000007*dt;
   out_8472269591488097829[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8472269591488097829[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8472269591488097829[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8472269591488097829[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8472269591488097829[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8472269591488097829[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8472269591488097829[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8472269591488097829[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8472269591488097829[62] = 0;
   out_8472269591488097829[63] = 0;
   out_8472269591488097829[64] = 0;
   out_8472269591488097829[65] = 0;
   out_8472269591488097829[66] = 0;
   out_8472269591488097829[67] = 0;
   out_8472269591488097829[68] = 0;
   out_8472269591488097829[69] = 0;
   out_8472269591488097829[70] = 1;
   out_8472269591488097829[71] = 0;
   out_8472269591488097829[72] = 0;
   out_8472269591488097829[73] = 0;
   out_8472269591488097829[74] = 0;
   out_8472269591488097829[75] = 0;
   out_8472269591488097829[76] = 0;
   out_8472269591488097829[77] = 0;
   out_8472269591488097829[78] = 0;
   out_8472269591488097829[79] = 0;
   out_8472269591488097829[80] = 1;
}
void h_25(double *state, double *unused, double *out_4469567855986124585) {
   out_4469567855986124585[0] = state[6];
}
void H_25(double *state, double *unused, double *out_9007571436358782224) {
   out_9007571436358782224[0] = 0;
   out_9007571436358782224[1] = 0;
   out_9007571436358782224[2] = 0;
   out_9007571436358782224[3] = 0;
   out_9007571436358782224[4] = 0;
   out_9007571436358782224[5] = 0;
   out_9007571436358782224[6] = 1;
   out_9007571436358782224[7] = 0;
   out_9007571436358782224[8] = 0;
}
void h_24(double *state, double *unused, double *out_7691985016952978134) {
   out_7691985016952978134[0] = state[4];
   out_7691985016952978134[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5881247038548446256) {
   out_5881247038548446256[0] = 0;
   out_5881247038548446256[1] = 0;
   out_5881247038548446256[2] = 0;
   out_5881247038548446256[3] = 0;
   out_5881247038548446256[4] = 1;
   out_5881247038548446256[5] = 0;
   out_5881247038548446256[6] = 0;
   out_5881247038548446256[7] = 0;
   out_5881247038548446256[8] = 0;
   out_5881247038548446256[9] = 0;
   out_5881247038548446256[10] = 0;
   out_5881247038548446256[11] = 0;
   out_5881247038548446256[12] = 0;
   out_5881247038548446256[13] = 0;
   out_5881247038548446256[14] = 1;
   out_5881247038548446256[15] = 0;
   out_5881247038548446256[16] = 0;
   out_5881247038548446256[17] = 0;
}
void h_30(double *state, double *unused, double *out_4626754524337253730) {
   out_4626754524337253730[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8878232489215542154) {
   out_8878232489215542154[0] = 0;
   out_8878232489215542154[1] = 0;
   out_8878232489215542154[2] = 0;
   out_8878232489215542154[3] = 0;
   out_8878232489215542154[4] = 1;
   out_8878232489215542154[5] = 0;
   out_8878232489215542154[6] = 0;
   out_8878232489215542154[7] = 0;
   out_8878232489215542154[8] = 0;
}
void h_26(double *state, double *unused, double *out_2584154961916967781) {
   out_2584154961916967781[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5266068117484726000) {
   out_5266068117484726000[0] = 0;
   out_5266068117484726000[1] = 0;
   out_5266068117484726000[2] = 0;
   out_5266068117484726000[3] = 0;
   out_5266068117484726000[4] = 0;
   out_5266068117484726000[5] = 0;
   out_5266068117484726000[6] = 0;
   out_5266068117484726000[7] = 1;
   out_5266068117484726000[8] = 0;
}
void h_27(double *state, double *unused, double *out_532105411674721060) {
   out_532105411674721060[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6703469177415117243) {
   out_6703469177415117243[0] = 0;
   out_6703469177415117243[1] = 0;
   out_6703469177415117243[2] = 0;
   out_6703469177415117243[3] = 1;
   out_6703469177415117243[4] = 0;
   out_6703469177415117243[5] = 0;
   out_6703469177415117243[6] = 0;
   out_6703469177415117243[7] = 0;
   out_6703469177415117243[8] = 0;
}
void h_29(double *state, double *unused, double *out_374918743323591915) {
   out_374918743323591915[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4990106450545566210) {
   out_4990106450545566210[0] = 0;
   out_4990106450545566210[1] = 1;
   out_4990106450545566210[2] = 0;
   out_4990106450545566210[3] = 0;
   out_4990106450545566210[4] = 0;
   out_4990106450545566210[5] = 0;
   out_4990106450545566210[6] = 0;
   out_4990106450545566210[7] = 0;
   out_4990106450545566210[8] = 0;
}
void h_28(double *state, double *unused, double *out_8449863760332119705) {
   out_8449863760332119705[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6953736722110892461) {
   out_6953736722110892461[0] = 1;
   out_6953736722110892461[1] = 0;
   out_6953736722110892461[2] = 0;
   out_6953736722110892461[3] = 0;
   out_6953736722110892461[4] = 0;
   out_6953736722110892461[5] = 0;
   out_6953736722110892461[6] = 0;
   out_6953736722110892461[7] = 0;
   out_6953736722110892461[8] = 0;
}
void h_31(double *state, double *unused, double *out_7389348291247131868) {
   out_7389348291247131868[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9038217398235742652) {
   out_9038217398235742652[0] = 0;
   out_9038217398235742652[1] = 0;
   out_9038217398235742652[2] = 0;
   out_9038217398235742652[3] = 0;
   out_9038217398235742652[4] = 0;
   out_9038217398235742652[5] = 0;
   out_9038217398235742652[6] = 0;
   out_9038217398235742652[7] = 0;
   out_9038217398235742652[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7443385576447730734) {
  err_fun(nom_x, delta_x, out_7443385576447730734);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3494304989908180493) {
  inv_err_fun(nom_x, true_x, out_3494304989908180493);
}
void car_H_mod_fun(double *state, double *out_6340163641565670916) {
  H_mod_fun(state, out_6340163641565670916);
}
void car_f_fun(double *state, double dt, double *out_2083752042724885918) {
  f_fun(state,  dt, out_2083752042724885918);
}
void car_F_fun(double *state, double dt, double *out_8472269591488097829) {
  F_fun(state,  dt, out_8472269591488097829);
}
void car_h_25(double *state, double *unused, double *out_4469567855986124585) {
  h_25(state, unused, out_4469567855986124585);
}
void car_H_25(double *state, double *unused, double *out_9007571436358782224) {
  H_25(state, unused, out_9007571436358782224);
}
void car_h_24(double *state, double *unused, double *out_7691985016952978134) {
  h_24(state, unused, out_7691985016952978134);
}
void car_H_24(double *state, double *unused, double *out_5881247038548446256) {
  H_24(state, unused, out_5881247038548446256);
}
void car_h_30(double *state, double *unused, double *out_4626754524337253730) {
  h_30(state, unused, out_4626754524337253730);
}
void car_H_30(double *state, double *unused, double *out_8878232489215542154) {
  H_30(state, unused, out_8878232489215542154);
}
void car_h_26(double *state, double *unused, double *out_2584154961916967781) {
  h_26(state, unused, out_2584154961916967781);
}
void car_H_26(double *state, double *unused, double *out_5266068117484726000) {
  H_26(state, unused, out_5266068117484726000);
}
void car_h_27(double *state, double *unused, double *out_532105411674721060) {
  h_27(state, unused, out_532105411674721060);
}
void car_H_27(double *state, double *unused, double *out_6703469177415117243) {
  H_27(state, unused, out_6703469177415117243);
}
void car_h_29(double *state, double *unused, double *out_374918743323591915) {
  h_29(state, unused, out_374918743323591915);
}
void car_H_29(double *state, double *unused, double *out_4990106450545566210) {
  H_29(state, unused, out_4990106450545566210);
}
void car_h_28(double *state, double *unused, double *out_8449863760332119705) {
  h_28(state, unused, out_8449863760332119705);
}
void car_H_28(double *state, double *unused, double *out_6953736722110892461) {
  H_28(state, unused, out_6953736722110892461);
}
void car_h_31(double *state, double *unused, double *out_7389348291247131868) {
  h_31(state, unused, out_7389348291247131868);
}
void car_H_31(double *state, double *unused, double *out_9038217398235742652) {
  H_31(state, unused, out_9038217398235742652);
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
