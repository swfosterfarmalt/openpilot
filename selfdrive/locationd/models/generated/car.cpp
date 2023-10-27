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
void err_fun(double *nom_x, double *delta_x, double *out_7574054726488367863) {
   out_7574054726488367863[0] = delta_x[0] + nom_x[0];
   out_7574054726488367863[1] = delta_x[1] + nom_x[1];
   out_7574054726488367863[2] = delta_x[2] + nom_x[2];
   out_7574054726488367863[3] = delta_x[3] + nom_x[3];
   out_7574054726488367863[4] = delta_x[4] + nom_x[4];
   out_7574054726488367863[5] = delta_x[5] + nom_x[5];
   out_7574054726488367863[6] = delta_x[6] + nom_x[6];
   out_7574054726488367863[7] = delta_x[7] + nom_x[7];
   out_7574054726488367863[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4766889803764304365) {
   out_4766889803764304365[0] = -nom_x[0] + true_x[0];
   out_4766889803764304365[1] = -nom_x[1] + true_x[1];
   out_4766889803764304365[2] = -nom_x[2] + true_x[2];
   out_4766889803764304365[3] = -nom_x[3] + true_x[3];
   out_4766889803764304365[4] = -nom_x[4] + true_x[4];
   out_4766889803764304365[5] = -nom_x[5] + true_x[5];
   out_4766889803764304365[6] = -nom_x[6] + true_x[6];
   out_4766889803764304365[7] = -nom_x[7] + true_x[7];
   out_4766889803764304365[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2258557682853182946) {
   out_2258557682853182946[0] = 1.0;
   out_2258557682853182946[1] = 0;
   out_2258557682853182946[2] = 0;
   out_2258557682853182946[3] = 0;
   out_2258557682853182946[4] = 0;
   out_2258557682853182946[5] = 0;
   out_2258557682853182946[6] = 0;
   out_2258557682853182946[7] = 0;
   out_2258557682853182946[8] = 0;
   out_2258557682853182946[9] = 0;
   out_2258557682853182946[10] = 1.0;
   out_2258557682853182946[11] = 0;
   out_2258557682853182946[12] = 0;
   out_2258557682853182946[13] = 0;
   out_2258557682853182946[14] = 0;
   out_2258557682853182946[15] = 0;
   out_2258557682853182946[16] = 0;
   out_2258557682853182946[17] = 0;
   out_2258557682853182946[18] = 0;
   out_2258557682853182946[19] = 0;
   out_2258557682853182946[20] = 1.0;
   out_2258557682853182946[21] = 0;
   out_2258557682853182946[22] = 0;
   out_2258557682853182946[23] = 0;
   out_2258557682853182946[24] = 0;
   out_2258557682853182946[25] = 0;
   out_2258557682853182946[26] = 0;
   out_2258557682853182946[27] = 0;
   out_2258557682853182946[28] = 0;
   out_2258557682853182946[29] = 0;
   out_2258557682853182946[30] = 1.0;
   out_2258557682853182946[31] = 0;
   out_2258557682853182946[32] = 0;
   out_2258557682853182946[33] = 0;
   out_2258557682853182946[34] = 0;
   out_2258557682853182946[35] = 0;
   out_2258557682853182946[36] = 0;
   out_2258557682853182946[37] = 0;
   out_2258557682853182946[38] = 0;
   out_2258557682853182946[39] = 0;
   out_2258557682853182946[40] = 1.0;
   out_2258557682853182946[41] = 0;
   out_2258557682853182946[42] = 0;
   out_2258557682853182946[43] = 0;
   out_2258557682853182946[44] = 0;
   out_2258557682853182946[45] = 0;
   out_2258557682853182946[46] = 0;
   out_2258557682853182946[47] = 0;
   out_2258557682853182946[48] = 0;
   out_2258557682853182946[49] = 0;
   out_2258557682853182946[50] = 1.0;
   out_2258557682853182946[51] = 0;
   out_2258557682853182946[52] = 0;
   out_2258557682853182946[53] = 0;
   out_2258557682853182946[54] = 0;
   out_2258557682853182946[55] = 0;
   out_2258557682853182946[56] = 0;
   out_2258557682853182946[57] = 0;
   out_2258557682853182946[58] = 0;
   out_2258557682853182946[59] = 0;
   out_2258557682853182946[60] = 1.0;
   out_2258557682853182946[61] = 0;
   out_2258557682853182946[62] = 0;
   out_2258557682853182946[63] = 0;
   out_2258557682853182946[64] = 0;
   out_2258557682853182946[65] = 0;
   out_2258557682853182946[66] = 0;
   out_2258557682853182946[67] = 0;
   out_2258557682853182946[68] = 0;
   out_2258557682853182946[69] = 0;
   out_2258557682853182946[70] = 1.0;
   out_2258557682853182946[71] = 0;
   out_2258557682853182946[72] = 0;
   out_2258557682853182946[73] = 0;
   out_2258557682853182946[74] = 0;
   out_2258557682853182946[75] = 0;
   out_2258557682853182946[76] = 0;
   out_2258557682853182946[77] = 0;
   out_2258557682853182946[78] = 0;
   out_2258557682853182946[79] = 0;
   out_2258557682853182946[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3821644810047975420) {
   out_3821644810047975420[0] = state[0];
   out_3821644810047975420[1] = state[1];
   out_3821644810047975420[2] = state[2];
   out_3821644810047975420[3] = state[3];
   out_3821644810047975420[4] = state[4];
   out_3821644810047975420[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3821644810047975420[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3821644810047975420[7] = state[7];
   out_3821644810047975420[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2591116970938824595) {
   out_2591116970938824595[0] = 1;
   out_2591116970938824595[1] = 0;
   out_2591116970938824595[2] = 0;
   out_2591116970938824595[3] = 0;
   out_2591116970938824595[4] = 0;
   out_2591116970938824595[5] = 0;
   out_2591116970938824595[6] = 0;
   out_2591116970938824595[7] = 0;
   out_2591116970938824595[8] = 0;
   out_2591116970938824595[9] = 0;
   out_2591116970938824595[10] = 1;
   out_2591116970938824595[11] = 0;
   out_2591116970938824595[12] = 0;
   out_2591116970938824595[13] = 0;
   out_2591116970938824595[14] = 0;
   out_2591116970938824595[15] = 0;
   out_2591116970938824595[16] = 0;
   out_2591116970938824595[17] = 0;
   out_2591116970938824595[18] = 0;
   out_2591116970938824595[19] = 0;
   out_2591116970938824595[20] = 1;
   out_2591116970938824595[21] = 0;
   out_2591116970938824595[22] = 0;
   out_2591116970938824595[23] = 0;
   out_2591116970938824595[24] = 0;
   out_2591116970938824595[25] = 0;
   out_2591116970938824595[26] = 0;
   out_2591116970938824595[27] = 0;
   out_2591116970938824595[28] = 0;
   out_2591116970938824595[29] = 0;
   out_2591116970938824595[30] = 1;
   out_2591116970938824595[31] = 0;
   out_2591116970938824595[32] = 0;
   out_2591116970938824595[33] = 0;
   out_2591116970938824595[34] = 0;
   out_2591116970938824595[35] = 0;
   out_2591116970938824595[36] = 0;
   out_2591116970938824595[37] = 0;
   out_2591116970938824595[38] = 0;
   out_2591116970938824595[39] = 0;
   out_2591116970938824595[40] = 1;
   out_2591116970938824595[41] = 0;
   out_2591116970938824595[42] = 0;
   out_2591116970938824595[43] = 0;
   out_2591116970938824595[44] = 0;
   out_2591116970938824595[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2591116970938824595[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2591116970938824595[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2591116970938824595[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2591116970938824595[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2591116970938824595[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2591116970938824595[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2591116970938824595[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2591116970938824595[53] = -9.8000000000000007*dt;
   out_2591116970938824595[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2591116970938824595[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2591116970938824595[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2591116970938824595[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2591116970938824595[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2591116970938824595[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2591116970938824595[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2591116970938824595[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2591116970938824595[62] = 0;
   out_2591116970938824595[63] = 0;
   out_2591116970938824595[64] = 0;
   out_2591116970938824595[65] = 0;
   out_2591116970938824595[66] = 0;
   out_2591116970938824595[67] = 0;
   out_2591116970938824595[68] = 0;
   out_2591116970938824595[69] = 0;
   out_2591116970938824595[70] = 1;
   out_2591116970938824595[71] = 0;
   out_2591116970938824595[72] = 0;
   out_2591116970938824595[73] = 0;
   out_2591116970938824595[74] = 0;
   out_2591116970938824595[75] = 0;
   out_2591116970938824595[76] = 0;
   out_2591116970938824595[77] = 0;
   out_2591116970938824595[78] = 0;
   out_2591116970938824595[79] = 0;
   out_2591116970938824595[80] = 1;
}
void h_25(double *state, double *unused, double *out_157749596158077776) {
   out_157749596158077776[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6889492108556835482) {
   out_6889492108556835482[0] = 0;
   out_6889492108556835482[1] = 0;
   out_6889492108556835482[2] = 0;
   out_6889492108556835482[3] = 0;
   out_6889492108556835482[4] = 0;
   out_6889492108556835482[5] = 0;
   out_6889492108556835482[6] = 1;
   out_6889492108556835482[7] = 0;
   out_6889492108556835482[8] = 0;
}
void h_24(double *state, double *unused, double *out_7347740927150280253) {
   out_7347740927150280253[0] = state[4];
   out_7347740927150280253[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4903046507044530377) {
   out_4903046507044530377[0] = 0;
   out_4903046507044530377[1] = 0;
   out_4903046507044530377[2] = 0;
   out_4903046507044530377[3] = 0;
   out_4903046507044530377[4] = 1;
   out_4903046507044530377[5] = 0;
   out_4903046507044530377[6] = 0;
   out_4903046507044530377[7] = 0;
   out_4903046507044530377[8] = 0;
   out_4903046507044530377[9] = 0;
   out_4903046507044530377[10] = 0;
   out_4903046507044530377[11] = 0;
   out_4903046507044530377[12] = 0;
   out_4903046507044530377[13] = 0;
   out_4903046507044530377[14] = 1;
   out_4903046507044530377[15] = 0;
   out_4903046507044530377[16] = 0;
   out_4903046507044530377[17] = 0;
}
void h_30(double *state, double *unused, double *out_5472357644847062983) {
   out_5472357644847062983[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6760153161413595412) {
   out_6760153161413595412[0] = 0;
   out_6760153161413595412[1] = 0;
   out_6760153161413595412[2] = 0;
   out_6760153161413595412[3] = 0;
   out_6760153161413595412[4] = 1;
   out_6760153161413595412[5] = 0;
   out_6760153161413595412[6] = 0;
   out_6760153161413595412[7] = 0;
   out_6760153161413595412[8] = 0;
}
void h_26(double *state, double *unused, double *out_6415867497748725418) {
   out_6415867497748725418[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3147988789682779258) {
   out_3147988789682779258[0] = 0;
   out_3147988789682779258[1] = 0;
   out_3147988789682779258[2] = 0;
   out_3147988789682779258[3] = 0;
   out_3147988789682779258[4] = 0;
   out_3147988789682779258[5] = 0;
   out_3147988789682779258[6] = 0;
   out_3147988789682779258[7] = 1;
   out_3147988789682779258[8] = 0;
}
void h_27(double *state, double *unused, double *out_1201877934851292052) {
   out_1201877934851292052[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4585389849613170501) {
   out_4585389849613170501[0] = 0;
   out_4585389849613170501[1] = 0;
   out_4585389849613170501[2] = 0;
   out_4585389849613170501[3] = 1;
   out_4585389849613170501[4] = 0;
   out_4585389849613170501[5] = 0;
   out_4585389849613170501[6] = 0;
   out_4585389849613170501[7] = 0;
   out_4585389849613170501[8] = 0;
}
void h_29(double *state, double *unused, double *out_2603842393001749020) {
   out_2603842393001749020[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7270384505727987596) {
   out_7270384505727987596[0] = 0;
   out_7270384505727987596[1] = 1;
   out_7270384505727987596[2] = 0;
   out_7270384505727987596[3] = 0;
   out_7270384505727987596[4] = 0;
   out_7270384505727987596[5] = 0;
   out_7270384505727987596[6] = 0;
   out_7270384505727987596[7] = 0;
   out_7270384505727987596[8] = 0;
}
void h_28(double *state, double *unused, double *out_2892325769586973012) {
   out_2892325769586973012[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2187985488658457022) {
   out_2187985488658457022[0] = 1;
   out_2187985488658457022[1] = 0;
   out_2187985488658457022[2] = 0;
   out_2187985488658457022[3] = 0;
   out_2187985488658457022[4] = 0;
   out_2187985488658457022[5] = 0;
   out_2187985488658457022[6] = 0;
   out_2187985488658457022[7] = 0;
   out_2187985488658457022[8] = 0;
}
void h_31(double *state, double *unused, double *out_4752919809883254530) {
   out_4752919809883254530[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6920138070433795910) {
   out_6920138070433795910[0] = 0;
   out_6920138070433795910[1] = 0;
   out_6920138070433795910[2] = 0;
   out_6920138070433795910[3] = 0;
   out_6920138070433795910[4] = 0;
   out_6920138070433795910[5] = 0;
   out_6920138070433795910[6] = 0;
   out_6920138070433795910[7] = 0;
   out_6920138070433795910[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7574054726488367863) {
  err_fun(nom_x, delta_x, out_7574054726488367863);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4766889803764304365) {
  inv_err_fun(nom_x, true_x, out_4766889803764304365);
}
void car_H_mod_fun(double *state, double *out_2258557682853182946) {
  H_mod_fun(state, out_2258557682853182946);
}
void car_f_fun(double *state, double dt, double *out_3821644810047975420) {
  f_fun(state,  dt, out_3821644810047975420);
}
void car_F_fun(double *state, double dt, double *out_2591116970938824595) {
  F_fun(state,  dt, out_2591116970938824595);
}
void car_h_25(double *state, double *unused, double *out_157749596158077776) {
  h_25(state, unused, out_157749596158077776);
}
void car_H_25(double *state, double *unused, double *out_6889492108556835482) {
  H_25(state, unused, out_6889492108556835482);
}
void car_h_24(double *state, double *unused, double *out_7347740927150280253) {
  h_24(state, unused, out_7347740927150280253);
}
void car_H_24(double *state, double *unused, double *out_4903046507044530377) {
  H_24(state, unused, out_4903046507044530377);
}
void car_h_30(double *state, double *unused, double *out_5472357644847062983) {
  h_30(state, unused, out_5472357644847062983);
}
void car_H_30(double *state, double *unused, double *out_6760153161413595412) {
  H_30(state, unused, out_6760153161413595412);
}
void car_h_26(double *state, double *unused, double *out_6415867497748725418) {
  h_26(state, unused, out_6415867497748725418);
}
void car_H_26(double *state, double *unused, double *out_3147988789682779258) {
  H_26(state, unused, out_3147988789682779258);
}
void car_h_27(double *state, double *unused, double *out_1201877934851292052) {
  h_27(state, unused, out_1201877934851292052);
}
void car_H_27(double *state, double *unused, double *out_4585389849613170501) {
  H_27(state, unused, out_4585389849613170501);
}
void car_h_29(double *state, double *unused, double *out_2603842393001749020) {
  h_29(state, unused, out_2603842393001749020);
}
void car_H_29(double *state, double *unused, double *out_7270384505727987596) {
  H_29(state, unused, out_7270384505727987596);
}
void car_h_28(double *state, double *unused, double *out_2892325769586973012) {
  h_28(state, unused, out_2892325769586973012);
}
void car_H_28(double *state, double *unused, double *out_2187985488658457022) {
  H_28(state, unused, out_2187985488658457022);
}
void car_h_31(double *state, double *unused, double *out_4752919809883254530) {
  h_31(state, unused, out_4752919809883254530);
}
void car_H_31(double *state, double *unused, double *out_6920138070433795910) {
  H_31(state, unused, out_6920138070433795910);
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
