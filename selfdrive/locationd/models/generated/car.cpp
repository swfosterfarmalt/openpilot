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
void err_fun(double *nom_x, double *delta_x, double *out_5623422911391110822) {
   out_5623422911391110822[0] = delta_x[0] + nom_x[0];
   out_5623422911391110822[1] = delta_x[1] + nom_x[1];
   out_5623422911391110822[2] = delta_x[2] + nom_x[2];
   out_5623422911391110822[3] = delta_x[3] + nom_x[3];
   out_5623422911391110822[4] = delta_x[4] + nom_x[4];
   out_5623422911391110822[5] = delta_x[5] + nom_x[5];
   out_5623422911391110822[6] = delta_x[6] + nom_x[6];
   out_5623422911391110822[7] = delta_x[7] + nom_x[7];
   out_5623422911391110822[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1854474179787496236) {
   out_1854474179787496236[0] = -nom_x[0] + true_x[0];
   out_1854474179787496236[1] = -nom_x[1] + true_x[1];
   out_1854474179787496236[2] = -nom_x[2] + true_x[2];
   out_1854474179787496236[3] = -nom_x[3] + true_x[3];
   out_1854474179787496236[4] = -nom_x[4] + true_x[4];
   out_1854474179787496236[5] = -nom_x[5] + true_x[5];
   out_1854474179787496236[6] = -nom_x[6] + true_x[6];
   out_1854474179787496236[7] = -nom_x[7] + true_x[7];
   out_1854474179787496236[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3278777670123947017) {
   out_3278777670123947017[0] = 1.0;
   out_3278777670123947017[1] = 0;
   out_3278777670123947017[2] = 0;
   out_3278777670123947017[3] = 0;
   out_3278777670123947017[4] = 0;
   out_3278777670123947017[5] = 0;
   out_3278777670123947017[6] = 0;
   out_3278777670123947017[7] = 0;
   out_3278777670123947017[8] = 0;
   out_3278777670123947017[9] = 0;
   out_3278777670123947017[10] = 1.0;
   out_3278777670123947017[11] = 0;
   out_3278777670123947017[12] = 0;
   out_3278777670123947017[13] = 0;
   out_3278777670123947017[14] = 0;
   out_3278777670123947017[15] = 0;
   out_3278777670123947017[16] = 0;
   out_3278777670123947017[17] = 0;
   out_3278777670123947017[18] = 0;
   out_3278777670123947017[19] = 0;
   out_3278777670123947017[20] = 1.0;
   out_3278777670123947017[21] = 0;
   out_3278777670123947017[22] = 0;
   out_3278777670123947017[23] = 0;
   out_3278777670123947017[24] = 0;
   out_3278777670123947017[25] = 0;
   out_3278777670123947017[26] = 0;
   out_3278777670123947017[27] = 0;
   out_3278777670123947017[28] = 0;
   out_3278777670123947017[29] = 0;
   out_3278777670123947017[30] = 1.0;
   out_3278777670123947017[31] = 0;
   out_3278777670123947017[32] = 0;
   out_3278777670123947017[33] = 0;
   out_3278777670123947017[34] = 0;
   out_3278777670123947017[35] = 0;
   out_3278777670123947017[36] = 0;
   out_3278777670123947017[37] = 0;
   out_3278777670123947017[38] = 0;
   out_3278777670123947017[39] = 0;
   out_3278777670123947017[40] = 1.0;
   out_3278777670123947017[41] = 0;
   out_3278777670123947017[42] = 0;
   out_3278777670123947017[43] = 0;
   out_3278777670123947017[44] = 0;
   out_3278777670123947017[45] = 0;
   out_3278777670123947017[46] = 0;
   out_3278777670123947017[47] = 0;
   out_3278777670123947017[48] = 0;
   out_3278777670123947017[49] = 0;
   out_3278777670123947017[50] = 1.0;
   out_3278777670123947017[51] = 0;
   out_3278777670123947017[52] = 0;
   out_3278777670123947017[53] = 0;
   out_3278777670123947017[54] = 0;
   out_3278777670123947017[55] = 0;
   out_3278777670123947017[56] = 0;
   out_3278777670123947017[57] = 0;
   out_3278777670123947017[58] = 0;
   out_3278777670123947017[59] = 0;
   out_3278777670123947017[60] = 1.0;
   out_3278777670123947017[61] = 0;
   out_3278777670123947017[62] = 0;
   out_3278777670123947017[63] = 0;
   out_3278777670123947017[64] = 0;
   out_3278777670123947017[65] = 0;
   out_3278777670123947017[66] = 0;
   out_3278777670123947017[67] = 0;
   out_3278777670123947017[68] = 0;
   out_3278777670123947017[69] = 0;
   out_3278777670123947017[70] = 1.0;
   out_3278777670123947017[71] = 0;
   out_3278777670123947017[72] = 0;
   out_3278777670123947017[73] = 0;
   out_3278777670123947017[74] = 0;
   out_3278777670123947017[75] = 0;
   out_3278777670123947017[76] = 0;
   out_3278777670123947017[77] = 0;
   out_3278777670123947017[78] = 0;
   out_3278777670123947017[79] = 0;
   out_3278777670123947017[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5084800254113784191) {
   out_5084800254113784191[0] = state[0];
   out_5084800254113784191[1] = state[1];
   out_5084800254113784191[2] = state[2];
   out_5084800254113784191[3] = state[3];
   out_5084800254113784191[4] = state[4];
   out_5084800254113784191[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5084800254113784191[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5084800254113784191[7] = state[7];
   out_5084800254113784191[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5106034447749273795) {
   out_5106034447749273795[0] = 1;
   out_5106034447749273795[1] = 0;
   out_5106034447749273795[2] = 0;
   out_5106034447749273795[3] = 0;
   out_5106034447749273795[4] = 0;
   out_5106034447749273795[5] = 0;
   out_5106034447749273795[6] = 0;
   out_5106034447749273795[7] = 0;
   out_5106034447749273795[8] = 0;
   out_5106034447749273795[9] = 0;
   out_5106034447749273795[10] = 1;
   out_5106034447749273795[11] = 0;
   out_5106034447749273795[12] = 0;
   out_5106034447749273795[13] = 0;
   out_5106034447749273795[14] = 0;
   out_5106034447749273795[15] = 0;
   out_5106034447749273795[16] = 0;
   out_5106034447749273795[17] = 0;
   out_5106034447749273795[18] = 0;
   out_5106034447749273795[19] = 0;
   out_5106034447749273795[20] = 1;
   out_5106034447749273795[21] = 0;
   out_5106034447749273795[22] = 0;
   out_5106034447749273795[23] = 0;
   out_5106034447749273795[24] = 0;
   out_5106034447749273795[25] = 0;
   out_5106034447749273795[26] = 0;
   out_5106034447749273795[27] = 0;
   out_5106034447749273795[28] = 0;
   out_5106034447749273795[29] = 0;
   out_5106034447749273795[30] = 1;
   out_5106034447749273795[31] = 0;
   out_5106034447749273795[32] = 0;
   out_5106034447749273795[33] = 0;
   out_5106034447749273795[34] = 0;
   out_5106034447749273795[35] = 0;
   out_5106034447749273795[36] = 0;
   out_5106034447749273795[37] = 0;
   out_5106034447749273795[38] = 0;
   out_5106034447749273795[39] = 0;
   out_5106034447749273795[40] = 1;
   out_5106034447749273795[41] = 0;
   out_5106034447749273795[42] = 0;
   out_5106034447749273795[43] = 0;
   out_5106034447749273795[44] = 0;
   out_5106034447749273795[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5106034447749273795[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5106034447749273795[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5106034447749273795[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5106034447749273795[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5106034447749273795[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5106034447749273795[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5106034447749273795[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5106034447749273795[53] = -9.8000000000000007*dt;
   out_5106034447749273795[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5106034447749273795[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5106034447749273795[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5106034447749273795[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5106034447749273795[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5106034447749273795[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5106034447749273795[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5106034447749273795[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5106034447749273795[62] = 0;
   out_5106034447749273795[63] = 0;
   out_5106034447749273795[64] = 0;
   out_5106034447749273795[65] = 0;
   out_5106034447749273795[66] = 0;
   out_5106034447749273795[67] = 0;
   out_5106034447749273795[68] = 0;
   out_5106034447749273795[69] = 0;
   out_5106034447749273795[70] = 1;
   out_5106034447749273795[71] = 0;
   out_5106034447749273795[72] = 0;
   out_5106034447749273795[73] = 0;
   out_5106034447749273795[74] = 0;
   out_5106034447749273795[75] = 0;
   out_5106034447749273795[76] = 0;
   out_5106034447749273795[77] = 0;
   out_5106034447749273795[78] = 0;
   out_5106034447749273795[79] = 0;
   out_5106034447749273795[80] = 1;
}
void h_25(double *state, double *unused, double *out_1734102347943028106) {
   out_1734102347943028106[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1430810675753686302) {
   out_1430810675753686302[0] = 0;
   out_1430810675753686302[1] = 0;
   out_1430810675753686302[2] = 0;
   out_1430810675753686302[3] = 0;
   out_1430810675753686302[4] = 0;
   out_1430810675753686302[5] = 0;
   out_1430810675753686302[6] = 1;
   out_1430810675753686302[7] = 0;
   out_1430810675753686302[8] = 0;
}
void h_24(double *state, double *unused, double *out_7308874276242624331) {
   out_7308874276242624331[0] = state[4];
   out_7308874276242624331[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4988736274556009438) {
   out_4988736274556009438[0] = 0;
   out_4988736274556009438[1] = 0;
   out_4988736274556009438[2] = 0;
   out_4988736274556009438[3] = 0;
   out_4988736274556009438[4] = 1;
   out_4988736274556009438[5] = 0;
   out_4988736274556009438[6] = 0;
   out_4988736274556009438[7] = 0;
   out_4988736274556009438[8] = 0;
   out_4988736274556009438[9] = 0;
   out_4988736274556009438[10] = 0;
   out_4988736274556009438[11] = 0;
   out_4988736274556009438[12] = 0;
   out_4988736274556009438[13] = 0;
   out_4988736274556009438[14] = 1;
   out_4988736274556009438[15] = 0;
   out_4988736274556009438[16] = 0;
   out_4988736274556009438[17] = 0;
}
void h_30(double *state, double *unused, double *out_8072764249047981626) {
   out_8072764249047981626[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8347501017245303057) {
   out_8347501017245303057[0] = 0;
   out_8347501017245303057[1] = 0;
   out_8347501017245303057[2] = 0;
   out_8347501017245303057[3] = 0;
   out_8347501017245303057[4] = 1;
   out_8347501017245303057[5] = 0;
   out_8347501017245303057[6] = 0;
   out_8347501017245303057[7] = 0;
   out_8347501017245303057[8] = 0;
}
void h_26(double *state, double *unused, double *out_3974938307247720844) {
   out_3974938307247720844[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9133694028498855031) {
   out_9133694028498855031[0] = 0;
   out_9133694028498855031[1] = 0;
   out_9133694028498855031[2] = 0;
   out_9133694028498855031[3] = 0;
   out_9133694028498855031[4] = 0;
   out_9133694028498855031[5] = 0;
   out_9133694028498855031[6] = 0;
   out_9133694028498855031[7] = 1;
   out_9133694028498855031[8] = 0;
}
void h_27(double *state, double *unused, double *out_6854060333861813302) {
   out_6854060333861813302[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6172737705444878146) {
   out_6172737705444878146[0] = 0;
   out_6172737705444878146[1] = 0;
   out_6172737705444878146[2] = 0;
   out_6172737705444878146[3] = 1;
   out_6172737705444878146[4] = 0;
   out_6172737705444878146[5] = 0;
   out_6172737705444878146[6] = 0;
   out_6172737705444878146[7] = 0;
   out_6172737705444878146[8] = 0;
}
void h_29(double *state, double *unused, double *out_5229089761545922453) {
   out_5229089761545922453[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8857732361559695241) {
   out_8857732361559695241[0] = 0;
   out_8857732361559695241[1] = 1;
   out_8857732361559695241[2] = 0;
   out_8857732361559695241[3] = 0;
   out_8857732361559695241[4] = 0;
   out_8857732361559695241[5] = 0;
   out_8857732361559695241[6] = 0;
   out_8857732361559695241[7] = 0;
   out_8857732361559695241[8] = 0;
}
void h_28(double *state, double *unused, double *out_6591010472186892669) {
   out_6591010472186892669[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3775333344490164667) {
   out_3775333344490164667[0] = 1;
   out_3775333344490164667[1] = 0;
   out_3775333344490164667[2] = 0;
   out_3775333344490164667[3] = 0;
   out_3775333344490164667[4] = 0;
   out_3775333344490164667[5] = 0;
   out_3775333344490164667[6] = 0;
   out_3775333344490164667[7] = 0;
   out_3775333344490164667[8] = 0;
}
void h_31(double *state, double *unused, double *out_2009296410227533995) {
   out_2009296410227533995[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1461456637630646730) {
   out_1461456637630646730[0] = 0;
   out_1461456637630646730[1] = 0;
   out_1461456637630646730[2] = 0;
   out_1461456637630646730[3] = 0;
   out_1461456637630646730[4] = 0;
   out_1461456637630646730[5] = 0;
   out_1461456637630646730[6] = 0;
   out_1461456637630646730[7] = 0;
   out_1461456637630646730[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5623422911391110822) {
  err_fun(nom_x, delta_x, out_5623422911391110822);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1854474179787496236) {
  inv_err_fun(nom_x, true_x, out_1854474179787496236);
}
void car_H_mod_fun(double *state, double *out_3278777670123947017) {
  H_mod_fun(state, out_3278777670123947017);
}
void car_f_fun(double *state, double dt, double *out_5084800254113784191) {
  f_fun(state,  dt, out_5084800254113784191);
}
void car_F_fun(double *state, double dt, double *out_5106034447749273795) {
  F_fun(state,  dt, out_5106034447749273795);
}
void car_h_25(double *state, double *unused, double *out_1734102347943028106) {
  h_25(state, unused, out_1734102347943028106);
}
void car_H_25(double *state, double *unused, double *out_1430810675753686302) {
  H_25(state, unused, out_1430810675753686302);
}
void car_h_24(double *state, double *unused, double *out_7308874276242624331) {
  h_24(state, unused, out_7308874276242624331);
}
void car_H_24(double *state, double *unused, double *out_4988736274556009438) {
  H_24(state, unused, out_4988736274556009438);
}
void car_h_30(double *state, double *unused, double *out_8072764249047981626) {
  h_30(state, unused, out_8072764249047981626);
}
void car_H_30(double *state, double *unused, double *out_8347501017245303057) {
  H_30(state, unused, out_8347501017245303057);
}
void car_h_26(double *state, double *unused, double *out_3974938307247720844) {
  h_26(state, unused, out_3974938307247720844);
}
void car_H_26(double *state, double *unused, double *out_9133694028498855031) {
  H_26(state, unused, out_9133694028498855031);
}
void car_h_27(double *state, double *unused, double *out_6854060333861813302) {
  h_27(state, unused, out_6854060333861813302);
}
void car_H_27(double *state, double *unused, double *out_6172737705444878146) {
  H_27(state, unused, out_6172737705444878146);
}
void car_h_29(double *state, double *unused, double *out_5229089761545922453) {
  h_29(state, unused, out_5229089761545922453);
}
void car_H_29(double *state, double *unused, double *out_8857732361559695241) {
  H_29(state, unused, out_8857732361559695241);
}
void car_h_28(double *state, double *unused, double *out_6591010472186892669) {
  h_28(state, unused, out_6591010472186892669);
}
void car_H_28(double *state, double *unused, double *out_3775333344490164667) {
  H_28(state, unused, out_3775333344490164667);
}
void car_h_31(double *state, double *unused, double *out_2009296410227533995) {
  h_31(state, unused, out_2009296410227533995);
}
void car_H_31(double *state, double *unused, double *out_1461456637630646730) {
  H_31(state, unused, out_1461456637630646730);
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
