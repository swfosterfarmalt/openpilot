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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4569146530072895553) {
   out_4569146530072895553[0] = delta_x[0] + nom_x[0];
   out_4569146530072895553[1] = delta_x[1] + nom_x[1];
   out_4569146530072895553[2] = delta_x[2] + nom_x[2];
   out_4569146530072895553[3] = delta_x[3] + nom_x[3];
   out_4569146530072895553[4] = delta_x[4] + nom_x[4];
   out_4569146530072895553[5] = delta_x[5] + nom_x[5];
   out_4569146530072895553[6] = delta_x[6] + nom_x[6];
   out_4569146530072895553[7] = delta_x[7] + nom_x[7];
   out_4569146530072895553[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2922497028284663501) {
   out_2922497028284663501[0] = -nom_x[0] + true_x[0];
   out_2922497028284663501[1] = -nom_x[1] + true_x[1];
   out_2922497028284663501[2] = -nom_x[2] + true_x[2];
   out_2922497028284663501[3] = -nom_x[3] + true_x[3];
   out_2922497028284663501[4] = -nom_x[4] + true_x[4];
   out_2922497028284663501[5] = -nom_x[5] + true_x[5];
   out_2922497028284663501[6] = -nom_x[6] + true_x[6];
   out_2922497028284663501[7] = -nom_x[7] + true_x[7];
   out_2922497028284663501[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7900836321283897023) {
   out_7900836321283897023[0] = 1.0;
   out_7900836321283897023[1] = 0;
   out_7900836321283897023[2] = 0;
   out_7900836321283897023[3] = 0;
   out_7900836321283897023[4] = 0;
   out_7900836321283897023[5] = 0;
   out_7900836321283897023[6] = 0;
   out_7900836321283897023[7] = 0;
   out_7900836321283897023[8] = 0;
   out_7900836321283897023[9] = 0;
   out_7900836321283897023[10] = 1.0;
   out_7900836321283897023[11] = 0;
   out_7900836321283897023[12] = 0;
   out_7900836321283897023[13] = 0;
   out_7900836321283897023[14] = 0;
   out_7900836321283897023[15] = 0;
   out_7900836321283897023[16] = 0;
   out_7900836321283897023[17] = 0;
   out_7900836321283897023[18] = 0;
   out_7900836321283897023[19] = 0;
   out_7900836321283897023[20] = 1.0;
   out_7900836321283897023[21] = 0;
   out_7900836321283897023[22] = 0;
   out_7900836321283897023[23] = 0;
   out_7900836321283897023[24] = 0;
   out_7900836321283897023[25] = 0;
   out_7900836321283897023[26] = 0;
   out_7900836321283897023[27] = 0;
   out_7900836321283897023[28] = 0;
   out_7900836321283897023[29] = 0;
   out_7900836321283897023[30] = 1.0;
   out_7900836321283897023[31] = 0;
   out_7900836321283897023[32] = 0;
   out_7900836321283897023[33] = 0;
   out_7900836321283897023[34] = 0;
   out_7900836321283897023[35] = 0;
   out_7900836321283897023[36] = 0;
   out_7900836321283897023[37] = 0;
   out_7900836321283897023[38] = 0;
   out_7900836321283897023[39] = 0;
   out_7900836321283897023[40] = 1.0;
   out_7900836321283897023[41] = 0;
   out_7900836321283897023[42] = 0;
   out_7900836321283897023[43] = 0;
   out_7900836321283897023[44] = 0;
   out_7900836321283897023[45] = 0;
   out_7900836321283897023[46] = 0;
   out_7900836321283897023[47] = 0;
   out_7900836321283897023[48] = 0;
   out_7900836321283897023[49] = 0;
   out_7900836321283897023[50] = 1.0;
   out_7900836321283897023[51] = 0;
   out_7900836321283897023[52] = 0;
   out_7900836321283897023[53] = 0;
   out_7900836321283897023[54] = 0;
   out_7900836321283897023[55] = 0;
   out_7900836321283897023[56] = 0;
   out_7900836321283897023[57] = 0;
   out_7900836321283897023[58] = 0;
   out_7900836321283897023[59] = 0;
   out_7900836321283897023[60] = 1.0;
   out_7900836321283897023[61] = 0;
   out_7900836321283897023[62] = 0;
   out_7900836321283897023[63] = 0;
   out_7900836321283897023[64] = 0;
   out_7900836321283897023[65] = 0;
   out_7900836321283897023[66] = 0;
   out_7900836321283897023[67] = 0;
   out_7900836321283897023[68] = 0;
   out_7900836321283897023[69] = 0;
   out_7900836321283897023[70] = 1.0;
   out_7900836321283897023[71] = 0;
   out_7900836321283897023[72] = 0;
   out_7900836321283897023[73] = 0;
   out_7900836321283897023[74] = 0;
   out_7900836321283897023[75] = 0;
   out_7900836321283897023[76] = 0;
   out_7900836321283897023[77] = 0;
   out_7900836321283897023[78] = 0;
   out_7900836321283897023[79] = 0;
   out_7900836321283897023[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6809919584049877432) {
   out_6809919584049877432[0] = state[0];
   out_6809919584049877432[1] = state[1];
   out_6809919584049877432[2] = state[2];
   out_6809919584049877432[3] = state[3];
   out_6809919584049877432[4] = state[4];
   out_6809919584049877432[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6809919584049877432[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6809919584049877432[7] = state[7];
   out_6809919584049877432[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9126469932184388727) {
   out_9126469932184388727[0] = 1;
   out_9126469932184388727[1] = 0;
   out_9126469932184388727[2] = 0;
   out_9126469932184388727[3] = 0;
   out_9126469932184388727[4] = 0;
   out_9126469932184388727[5] = 0;
   out_9126469932184388727[6] = 0;
   out_9126469932184388727[7] = 0;
   out_9126469932184388727[8] = 0;
   out_9126469932184388727[9] = 0;
   out_9126469932184388727[10] = 1;
   out_9126469932184388727[11] = 0;
   out_9126469932184388727[12] = 0;
   out_9126469932184388727[13] = 0;
   out_9126469932184388727[14] = 0;
   out_9126469932184388727[15] = 0;
   out_9126469932184388727[16] = 0;
   out_9126469932184388727[17] = 0;
   out_9126469932184388727[18] = 0;
   out_9126469932184388727[19] = 0;
   out_9126469932184388727[20] = 1;
   out_9126469932184388727[21] = 0;
   out_9126469932184388727[22] = 0;
   out_9126469932184388727[23] = 0;
   out_9126469932184388727[24] = 0;
   out_9126469932184388727[25] = 0;
   out_9126469932184388727[26] = 0;
   out_9126469932184388727[27] = 0;
   out_9126469932184388727[28] = 0;
   out_9126469932184388727[29] = 0;
   out_9126469932184388727[30] = 1;
   out_9126469932184388727[31] = 0;
   out_9126469932184388727[32] = 0;
   out_9126469932184388727[33] = 0;
   out_9126469932184388727[34] = 0;
   out_9126469932184388727[35] = 0;
   out_9126469932184388727[36] = 0;
   out_9126469932184388727[37] = 0;
   out_9126469932184388727[38] = 0;
   out_9126469932184388727[39] = 0;
   out_9126469932184388727[40] = 1;
   out_9126469932184388727[41] = 0;
   out_9126469932184388727[42] = 0;
   out_9126469932184388727[43] = 0;
   out_9126469932184388727[44] = 0;
   out_9126469932184388727[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9126469932184388727[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9126469932184388727[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9126469932184388727[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9126469932184388727[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9126469932184388727[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9126469932184388727[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9126469932184388727[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9126469932184388727[53] = -9.8000000000000007*dt;
   out_9126469932184388727[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9126469932184388727[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9126469932184388727[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9126469932184388727[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9126469932184388727[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9126469932184388727[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9126469932184388727[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9126469932184388727[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9126469932184388727[62] = 0;
   out_9126469932184388727[63] = 0;
   out_9126469932184388727[64] = 0;
   out_9126469932184388727[65] = 0;
   out_9126469932184388727[66] = 0;
   out_9126469932184388727[67] = 0;
   out_9126469932184388727[68] = 0;
   out_9126469932184388727[69] = 0;
   out_9126469932184388727[70] = 1;
   out_9126469932184388727[71] = 0;
   out_9126469932184388727[72] = 0;
   out_9126469932184388727[73] = 0;
   out_9126469932184388727[74] = 0;
   out_9126469932184388727[75] = 0;
   out_9126469932184388727[76] = 0;
   out_9126469932184388727[77] = 0;
   out_9126469932184388727[78] = 0;
   out_9126469932184388727[79] = 0;
   out_9126469932184388727[80] = 1;
}
void h_25(double *state, double *unused, double *out_3518164862699664303) {
   out_3518164862699664303[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7035607717603393194) {
   out_7035607717603393194[0] = 0;
   out_7035607717603393194[1] = 0;
   out_7035607717603393194[2] = 0;
   out_7035607717603393194[3] = 0;
   out_7035607717603393194[4] = 0;
   out_7035607717603393194[5] = 0;
   out_7035607717603393194[6] = 1;
   out_7035607717603393194[7] = 0;
   out_7035607717603393194[8] = 0;
}
void h_24(double *state, double *unused, double *out_953103084894500303) {
   out_953103084894500303[0] = state[4];
   out_953103084894500303[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8019673588444706862) {
   out_8019673588444706862[0] = 0;
   out_8019673588444706862[1] = 0;
   out_8019673588444706862[2] = 0;
   out_8019673588444706862[3] = 0;
   out_8019673588444706862[4] = 1;
   out_8019673588444706862[5] = 0;
   out_8019673588444706862[6] = 0;
   out_8019673588444706862[7] = 0;
   out_8019673588444706862[8] = 0;
   out_8019673588444706862[9] = 0;
   out_8019673588444706862[10] = 0;
   out_8019673588444706862[11] = 0;
   out_8019673588444706862[12] = 0;
   out_8019673588444706862[13] = 0;
   out_8019673588444706862[14] = 1;
   out_8019673588444706862[15] = 0;
   out_8019673588444706862[16] = 0;
   out_8019673588444706862[17] = 0;
}
void h_30(double *state, double *unused, double *out_7165313954189214872) {
   out_7165313954189214872[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8892803397598909795) {
   out_8892803397598909795[0] = 0;
   out_8892803397598909795[1] = 0;
   out_8892803397598909795[2] = 0;
   out_8892803397598909795[3] = 0;
   out_8892803397598909795[4] = 1;
   out_8892803397598909795[5] = 0;
   out_8892803397598909795[6] = 0;
   out_8892803397598909795[7] = 0;
   out_8892803397598909795[8] = 0;
}
void h_26(double *state, double *unused, double *out_2242113230925674041) {
   out_2242113230925674041[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3294104398729336970) {
   out_3294104398729336970[0] = 0;
   out_3294104398729336970[1] = 0;
   out_3294104398729336970[2] = 0;
   out_3294104398729336970[3] = 0;
   out_3294104398729336970[4] = 0;
   out_3294104398729336970[5] = 0;
   out_3294104398729336970[6] = 0;
   out_3294104398729336970[7] = 1;
   out_3294104398729336970[8] = 0;
}
void h_27(double *state, double *unused, double *out_6122570183508361954) {
   out_6122570183508361954[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4731505458659728213) {
   out_4731505458659728213[0] = 0;
   out_4731505458659728213[1] = 0;
   out_4731505458659728213[2] = 0;
   out_4731505458659728213[3] = 1;
   out_4731505458659728213[4] = 0;
   out_4731505458659728213[5] = 0;
   out_4731505458659728213[6] = 0;
   out_4731505458659728213[7] = 0;
   out_4731505458659728213[8] = 0;
}
void h_29(double *state, double *unused, double *out_6279756851859491099) {
   out_6279756851859491099[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8382572053284517611) {
   out_8382572053284517611[0] = 0;
   out_8382572053284517611[1] = 1;
   out_8382572053284517611[2] = 0;
   out_8382572053284517611[3] = 0;
   out_8382572053284517611[4] = 0;
   out_8382572053284517611[5] = 0;
   out_8382572053284517611[6] = 0;
   out_8382572053284517611[7] = 0;
   out_8382572053284517611[8] = 0;
}
void h_28(double *state, double *unused, double *out_452431478410802598) {
   out_452431478410802598[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4981773003355503431) {
   out_4981773003355503431[0] = 1;
   out_4981773003355503431[1] = 0;
   out_4981773003355503431[2] = 0;
   out_4981773003355503431[3] = 0;
   out_4981773003355503431[4] = 0;
   out_4981773003355503431[5] = 0;
   out_4981773003355503431[6] = 0;
   out_4981773003355503431[7] = 0;
   out_4981773003355503431[8] = 0;
}
void h_31(double *state, double *unused, double *out_7047306560255838128) {
   out_7047306560255838128[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7066253679480353622) {
   out_7066253679480353622[0] = 0;
   out_7066253679480353622[1] = 0;
   out_7066253679480353622[2] = 0;
   out_7066253679480353622[3] = 0;
   out_7066253679480353622[4] = 0;
   out_7066253679480353622[5] = 0;
   out_7066253679480353622[6] = 0;
   out_7066253679480353622[7] = 0;
   out_7066253679480353622[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4569146530072895553) {
  err_fun(nom_x, delta_x, out_4569146530072895553);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2922497028284663501) {
  inv_err_fun(nom_x, true_x, out_2922497028284663501);
}
void car_H_mod_fun(double *state, double *out_7900836321283897023) {
  H_mod_fun(state, out_7900836321283897023);
}
void car_f_fun(double *state, double dt, double *out_6809919584049877432) {
  f_fun(state,  dt, out_6809919584049877432);
}
void car_F_fun(double *state, double dt, double *out_9126469932184388727) {
  F_fun(state,  dt, out_9126469932184388727);
}
void car_h_25(double *state, double *unused, double *out_3518164862699664303) {
  h_25(state, unused, out_3518164862699664303);
}
void car_H_25(double *state, double *unused, double *out_7035607717603393194) {
  H_25(state, unused, out_7035607717603393194);
}
void car_h_24(double *state, double *unused, double *out_953103084894500303) {
  h_24(state, unused, out_953103084894500303);
}
void car_H_24(double *state, double *unused, double *out_8019673588444706862) {
  H_24(state, unused, out_8019673588444706862);
}
void car_h_30(double *state, double *unused, double *out_7165313954189214872) {
  h_30(state, unused, out_7165313954189214872);
}
void car_H_30(double *state, double *unused, double *out_8892803397598909795) {
  H_30(state, unused, out_8892803397598909795);
}
void car_h_26(double *state, double *unused, double *out_2242113230925674041) {
  h_26(state, unused, out_2242113230925674041);
}
void car_H_26(double *state, double *unused, double *out_3294104398729336970) {
  H_26(state, unused, out_3294104398729336970);
}
void car_h_27(double *state, double *unused, double *out_6122570183508361954) {
  h_27(state, unused, out_6122570183508361954);
}
void car_H_27(double *state, double *unused, double *out_4731505458659728213) {
  H_27(state, unused, out_4731505458659728213);
}
void car_h_29(double *state, double *unused, double *out_6279756851859491099) {
  h_29(state, unused, out_6279756851859491099);
}
void car_H_29(double *state, double *unused, double *out_8382572053284517611) {
  H_29(state, unused, out_8382572053284517611);
}
void car_h_28(double *state, double *unused, double *out_452431478410802598) {
  h_28(state, unused, out_452431478410802598);
}
void car_H_28(double *state, double *unused, double *out_4981773003355503431) {
  H_28(state, unused, out_4981773003355503431);
}
void car_h_31(double *state, double *unused, double *out_7047306560255838128) {
  h_31(state, unused, out_7047306560255838128);
}
void car_H_31(double *state, double *unused, double *out_7066253679480353622) {
  H_31(state, unused, out_7066253679480353622);
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
