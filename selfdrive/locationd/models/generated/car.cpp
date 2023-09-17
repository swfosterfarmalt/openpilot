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
void err_fun(double *nom_x, double *delta_x, double *out_4793111812780848187) {
   out_4793111812780848187[0] = delta_x[0] + nom_x[0];
   out_4793111812780848187[1] = delta_x[1] + nom_x[1];
   out_4793111812780848187[2] = delta_x[2] + nom_x[2];
   out_4793111812780848187[3] = delta_x[3] + nom_x[3];
   out_4793111812780848187[4] = delta_x[4] + nom_x[4];
   out_4793111812780848187[5] = delta_x[5] + nom_x[5];
   out_4793111812780848187[6] = delta_x[6] + nom_x[6];
   out_4793111812780848187[7] = delta_x[7] + nom_x[7];
   out_4793111812780848187[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8047223645973478750) {
   out_8047223645973478750[0] = -nom_x[0] + true_x[0];
   out_8047223645973478750[1] = -nom_x[1] + true_x[1];
   out_8047223645973478750[2] = -nom_x[2] + true_x[2];
   out_8047223645973478750[3] = -nom_x[3] + true_x[3];
   out_8047223645973478750[4] = -nom_x[4] + true_x[4];
   out_8047223645973478750[5] = -nom_x[5] + true_x[5];
   out_8047223645973478750[6] = -nom_x[6] + true_x[6];
   out_8047223645973478750[7] = -nom_x[7] + true_x[7];
   out_8047223645973478750[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6221619636727725848) {
   out_6221619636727725848[0] = 1.0;
   out_6221619636727725848[1] = 0;
   out_6221619636727725848[2] = 0;
   out_6221619636727725848[3] = 0;
   out_6221619636727725848[4] = 0;
   out_6221619636727725848[5] = 0;
   out_6221619636727725848[6] = 0;
   out_6221619636727725848[7] = 0;
   out_6221619636727725848[8] = 0;
   out_6221619636727725848[9] = 0;
   out_6221619636727725848[10] = 1.0;
   out_6221619636727725848[11] = 0;
   out_6221619636727725848[12] = 0;
   out_6221619636727725848[13] = 0;
   out_6221619636727725848[14] = 0;
   out_6221619636727725848[15] = 0;
   out_6221619636727725848[16] = 0;
   out_6221619636727725848[17] = 0;
   out_6221619636727725848[18] = 0;
   out_6221619636727725848[19] = 0;
   out_6221619636727725848[20] = 1.0;
   out_6221619636727725848[21] = 0;
   out_6221619636727725848[22] = 0;
   out_6221619636727725848[23] = 0;
   out_6221619636727725848[24] = 0;
   out_6221619636727725848[25] = 0;
   out_6221619636727725848[26] = 0;
   out_6221619636727725848[27] = 0;
   out_6221619636727725848[28] = 0;
   out_6221619636727725848[29] = 0;
   out_6221619636727725848[30] = 1.0;
   out_6221619636727725848[31] = 0;
   out_6221619636727725848[32] = 0;
   out_6221619636727725848[33] = 0;
   out_6221619636727725848[34] = 0;
   out_6221619636727725848[35] = 0;
   out_6221619636727725848[36] = 0;
   out_6221619636727725848[37] = 0;
   out_6221619636727725848[38] = 0;
   out_6221619636727725848[39] = 0;
   out_6221619636727725848[40] = 1.0;
   out_6221619636727725848[41] = 0;
   out_6221619636727725848[42] = 0;
   out_6221619636727725848[43] = 0;
   out_6221619636727725848[44] = 0;
   out_6221619636727725848[45] = 0;
   out_6221619636727725848[46] = 0;
   out_6221619636727725848[47] = 0;
   out_6221619636727725848[48] = 0;
   out_6221619636727725848[49] = 0;
   out_6221619636727725848[50] = 1.0;
   out_6221619636727725848[51] = 0;
   out_6221619636727725848[52] = 0;
   out_6221619636727725848[53] = 0;
   out_6221619636727725848[54] = 0;
   out_6221619636727725848[55] = 0;
   out_6221619636727725848[56] = 0;
   out_6221619636727725848[57] = 0;
   out_6221619636727725848[58] = 0;
   out_6221619636727725848[59] = 0;
   out_6221619636727725848[60] = 1.0;
   out_6221619636727725848[61] = 0;
   out_6221619636727725848[62] = 0;
   out_6221619636727725848[63] = 0;
   out_6221619636727725848[64] = 0;
   out_6221619636727725848[65] = 0;
   out_6221619636727725848[66] = 0;
   out_6221619636727725848[67] = 0;
   out_6221619636727725848[68] = 0;
   out_6221619636727725848[69] = 0;
   out_6221619636727725848[70] = 1.0;
   out_6221619636727725848[71] = 0;
   out_6221619636727725848[72] = 0;
   out_6221619636727725848[73] = 0;
   out_6221619636727725848[74] = 0;
   out_6221619636727725848[75] = 0;
   out_6221619636727725848[76] = 0;
   out_6221619636727725848[77] = 0;
   out_6221619636727725848[78] = 0;
   out_6221619636727725848[79] = 0;
   out_6221619636727725848[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3279752275302509260) {
   out_3279752275302509260[0] = state[0];
   out_3279752275302509260[1] = state[1];
   out_3279752275302509260[2] = state[2];
   out_3279752275302509260[3] = state[3];
   out_3279752275302509260[4] = state[4];
   out_3279752275302509260[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3279752275302509260[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3279752275302509260[7] = state[7];
   out_3279752275302509260[8] = state[8];
}
void F_fun(double *state, double dt, double *out_329778069797634847) {
   out_329778069797634847[0] = 1;
   out_329778069797634847[1] = 0;
   out_329778069797634847[2] = 0;
   out_329778069797634847[3] = 0;
   out_329778069797634847[4] = 0;
   out_329778069797634847[5] = 0;
   out_329778069797634847[6] = 0;
   out_329778069797634847[7] = 0;
   out_329778069797634847[8] = 0;
   out_329778069797634847[9] = 0;
   out_329778069797634847[10] = 1;
   out_329778069797634847[11] = 0;
   out_329778069797634847[12] = 0;
   out_329778069797634847[13] = 0;
   out_329778069797634847[14] = 0;
   out_329778069797634847[15] = 0;
   out_329778069797634847[16] = 0;
   out_329778069797634847[17] = 0;
   out_329778069797634847[18] = 0;
   out_329778069797634847[19] = 0;
   out_329778069797634847[20] = 1;
   out_329778069797634847[21] = 0;
   out_329778069797634847[22] = 0;
   out_329778069797634847[23] = 0;
   out_329778069797634847[24] = 0;
   out_329778069797634847[25] = 0;
   out_329778069797634847[26] = 0;
   out_329778069797634847[27] = 0;
   out_329778069797634847[28] = 0;
   out_329778069797634847[29] = 0;
   out_329778069797634847[30] = 1;
   out_329778069797634847[31] = 0;
   out_329778069797634847[32] = 0;
   out_329778069797634847[33] = 0;
   out_329778069797634847[34] = 0;
   out_329778069797634847[35] = 0;
   out_329778069797634847[36] = 0;
   out_329778069797634847[37] = 0;
   out_329778069797634847[38] = 0;
   out_329778069797634847[39] = 0;
   out_329778069797634847[40] = 1;
   out_329778069797634847[41] = 0;
   out_329778069797634847[42] = 0;
   out_329778069797634847[43] = 0;
   out_329778069797634847[44] = 0;
   out_329778069797634847[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_329778069797634847[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_329778069797634847[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_329778069797634847[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_329778069797634847[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_329778069797634847[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_329778069797634847[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_329778069797634847[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_329778069797634847[53] = -9.8000000000000007*dt;
   out_329778069797634847[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_329778069797634847[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_329778069797634847[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_329778069797634847[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_329778069797634847[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_329778069797634847[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_329778069797634847[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_329778069797634847[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_329778069797634847[62] = 0;
   out_329778069797634847[63] = 0;
   out_329778069797634847[64] = 0;
   out_329778069797634847[65] = 0;
   out_329778069797634847[66] = 0;
   out_329778069797634847[67] = 0;
   out_329778069797634847[68] = 0;
   out_329778069797634847[69] = 0;
   out_329778069797634847[70] = 1;
   out_329778069797634847[71] = 0;
   out_329778069797634847[72] = 0;
   out_329778069797634847[73] = 0;
   out_329778069797634847[74] = 0;
   out_329778069797634847[75] = 0;
   out_329778069797634847[76] = 0;
   out_329778069797634847[77] = 0;
   out_329778069797634847[78] = 0;
   out_329778069797634847[79] = 0;
   out_329778069797634847[80] = 1;
}
void h_25(double *state, double *unused, double *out_3060125133492423155) {
   out_3060125133492423155[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6595875180761845420) {
   out_6595875180761845420[0] = 0;
   out_6595875180761845420[1] = 0;
   out_6595875180761845420[2] = 0;
   out_6595875180761845420[3] = 0;
   out_6595875180761845420[4] = 0;
   out_6595875180761845420[5] = 0;
   out_6595875180761845420[6] = 1;
   out_6595875180761845420[7] = 0;
   out_6595875180761845420[8] = 0;
}
void h_24(double *state, double *unused, double *out_2886292295865587153) {
   out_2886292295865587153[0] = state[4];
   out_2886292295865587153[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6982054027919999344) {
   out_6982054027919999344[0] = 0;
   out_6982054027919999344[1] = 0;
   out_6982054027919999344[2] = 0;
   out_6982054027919999344[3] = 0;
   out_6982054027919999344[4] = 1;
   out_6982054027919999344[5] = 0;
   out_6982054027919999344[6] = 0;
   out_6982054027919999344[7] = 0;
   out_6982054027919999344[8] = 0;
   out_6982054027919999344[9] = 0;
   out_6982054027919999344[10] = 0;
   out_6982054027919999344[11] = 0;
   out_6982054027919999344[12] = 0;
   out_6982054027919999344[13] = 0;
   out_6982054027919999344[14] = 1;
   out_6982054027919999344[15] = 0;
   out_6982054027919999344[16] = 0;
   out_6982054027919999344[17] = 0;
}
void h_30(double *state, double *unused, double *out_4703140123328249286) {
   out_4703140123328249286[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7323172562820097998) {
   out_7323172562820097998[0] = 0;
   out_7323172562820097998[1] = 0;
   out_7323172562820097998[2] = 0;
   out_7323172562820097998[3] = 0;
   out_7323172562820097998[4] = 1;
   out_7323172562820097998[5] = 0;
   out_7323172562820097998[6] = 0;
   out_7323172562820097998[7] = 0;
   out_7323172562820097998[8] = 0;
}
void h_26(double *state, double *unused, double *out_8855600155973575318) {
   out_8855600155973575318[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8109365574073649972) {
   out_8109365574073649972[0] = 0;
   out_8109365574073649972[1] = 0;
   out_8109365574073649972[2] = 0;
   out_8109365574073649972[3] = 0;
   out_8109365574073649972[4] = 0;
   out_8109365574073649972[5] = 0;
   out_8109365574073649972[6] = 0;
   out_8109365574073649972[7] = 1;
   out_8109365574073649972[8] = 0;
}
void h_27(double *state, double *unused, double *out_455719812683725504) {
   out_455719812683725504[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5148409251019673087) {
   out_5148409251019673087[0] = 0;
   out_5148409251019673087[1] = 0;
   out_5148409251019673087[2] = 0;
   out_5148409251019673087[3] = 1;
   out_5148409251019673087[4] = 0;
   out_5148409251019673087[5] = 0;
   out_5148409251019673087[6] = 0;
   out_5148409251019673087[7] = 0;
   out_5148409251019673087[8] = 0;
}
void h_29(double *state, double *unused, double *out_6393587958063930246) {
   out_6393587958063930246[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7833403907134490182) {
   out_7833403907134490182[0] = 0;
   out_7833403907134490182[1] = 1;
   out_7833403907134490182[2] = 0;
   out_7833403907134490182[3] = 0;
   out_7833403907134490182[4] = 0;
   out_7833403907134490182[5] = 0;
   out_7833403907134490182[6] = 0;
   out_7833403907134490182[7] = 0;
   out_7833403907134490182[8] = 0;
}
void h_28(double *state, double *unused, double *out_1667108752687590490) {
   out_1667108752687590490[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8649709895009735183) {
   out_8649709895009735183[0] = 1;
   out_8649709895009735183[1] = 0;
   out_8649709895009735183[2] = 0;
   out_8649709895009735183[3] = 0;
   out_8649709895009735183[4] = 0;
   out_8649709895009735183[5] = 0;
   out_8649709895009735183[6] = 0;
   out_8649709895009735183[7] = 0;
   out_8649709895009735183[8] = 0;
}
void h_31(double *state, double *unused, double *out_7312962692256136312) {
   out_7312962692256136312[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7483157471840298496) {
   out_7483157471840298496[0] = 0;
   out_7483157471840298496[1] = 0;
   out_7483157471840298496[2] = 0;
   out_7483157471840298496[3] = 0;
   out_7483157471840298496[4] = 0;
   out_7483157471840298496[5] = 0;
   out_7483157471840298496[6] = 0;
   out_7483157471840298496[7] = 0;
   out_7483157471840298496[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4793111812780848187) {
  err_fun(nom_x, delta_x, out_4793111812780848187);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8047223645973478750) {
  inv_err_fun(nom_x, true_x, out_8047223645973478750);
}
void car_H_mod_fun(double *state, double *out_6221619636727725848) {
  H_mod_fun(state, out_6221619636727725848);
}
void car_f_fun(double *state, double dt, double *out_3279752275302509260) {
  f_fun(state,  dt, out_3279752275302509260);
}
void car_F_fun(double *state, double dt, double *out_329778069797634847) {
  F_fun(state,  dt, out_329778069797634847);
}
void car_h_25(double *state, double *unused, double *out_3060125133492423155) {
  h_25(state, unused, out_3060125133492423155);
}
void car_H_25(double *state, double *unused, double *out_6595875180761845420) {
  H_25(state, unused, out_6595875180761845420);
}
void car_h_24(double *state, double *unused, double *out_2886292295865587153) {
  h_24(state, unused, out_2886292295865587153);
}
void car_H_24(double *state, double *unused, double *out_6982054027919999344) {
  H_24(state, unused, out_6982054027919999344);
}
void car_h_30(double *state, double *unused, double *out_4703140123328249286) {
  h_30(state, unused, out_4703140123328249286);
}
void car_H_30(double *state, double *unused, double *out_7323172562820097998) {
  H_30(state, unused, out_7323172562820097998);
}
void car_h_26(double *state, double *unused, double *out_8855600155973575318) {
  h_26(state, unused, out_8855600155973575318);
}
void car_H_26(double *state, double *unused, double *out_8109365574073649972) {
  H_26(state, unused, out_8109365574073649972);
}
void car_h_27(double *state, double *unused, double *out_455719812683725504) {
  h_27(state, unused, out_455719812683725504);
}
void car_H_27(double *state, double *unused, double *out_5148409251019673087) {
  H_27(state, unused, out_5148409251019673087);
}
void car_h_29(double *state, double *unused, double *out_6393587958063930246) {
  h_29(state, unused, out_6393587958063930246);
}
void car_H_29(double *state, double *unused, double *out_7833403907134490182) {
  H_29(state, unused, out_7833403907134490182);
}
void car_h_28(double *state, double *unused, double *out_1667108752687590490) {
  h_28(state, unused, out_1667108752687590490);
}
void car_H_28(double *state, double *unused, double *out_8649709895009735183) {
  H_28(state, unused, out_8649709895009735183);
}
void car_h_31(double *state, double *unused, double *out_7312962692256136312) {
  h_31(state, unused, out_7312962692256136312);
}
void car_H_31(double *state, double *unused, double *out_7483157471840298496) {
  H_31(state, unused, out_7483157471840298496);
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
