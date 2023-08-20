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
void err_fun(double *nom_x, double *delta_x, double *out_671052864023945038) {
   out_671052864023945038[0] = delta_x[0] + nom_x[0];
   out_671052864023945038[1] = delta_x[1] + nom_x[1];
   out_671052864023945038[2] = delta_x[2] + nom_x[2];
   out_671052864023945038[3] = delta_x[3] + nom_x[3];
   out_671052864023945038[4] = delta_x[4] + nom_x[4];
   out_671052864023945038[5] = delta_x[5] + nom_x[5];
   out_671052864023945038[6] = delta_x[6] + nom_x[6];
   out_671052864023945038[7] = delta_x[7] + nom_x[7];
   out_671052864023945038[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4718563599199213006) {
   out_4718563599199213006[0] = -nom_x[0] + true_x[0];
   out_4718563599199213006[1] = -nom_x[1] + true_x[1];
   out_4718563599199213006[2] = -nom_x[2] + true_x[2];
   out_4718563599199213006[3] = -nom_x[3] + true_x[3];
   out_4718563599199213006[4] = -nom_x[4] + true_x[4];
   out_4718563599199213006[5] = -nom_x[5] + true_x[5];
   out_4718563599199213006[6] = -nom_x[6] + true_x[6];
   out_4718563599199213006[7] = -nom_x[7] + true_x[7];
   out_4718563599199213006[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4933819263170883354) {
   out_4933819263170883354[0] = 1.0;
   out_4933819263170883354[1] = 0;
   out_4933819263170883354[2] = 0;
   out_4933819263170883354[3] = 0;
   out_4933819263170883354[4] = 0;
   out_4933819263170883354[5] = 0;
   out_4933819263170883354[6] = 0;
   out_4933819263170883354[7] = 0;
   out_4933819263170883354[8] = 0;
   out_4933819263170883354[9] = 0;
   out_4933819263170883354[10] = 1.0;
   out_4933819263170883354[11] = 0;
   out_4933819263170883354[12] = 0;
   out_4933819263170883354[13] = 0;
   out_4933819263170883354[14] = 0;
   out_4933819263170883354[15] = 0;
   out_4933819263170883354[16] = 0;
   out_4933819263170883354[17] = 0;
   out_4933819263170883354[18] = 0;
   out_4933819263170883354[19] = 0;
   out_4933819263170883354[20] = 1.0;
   out_4933819263170883354[21] = 0;
   out_4933819263170883354[22] = 0;
   out_4933819263170883354[23] = 0;
   out_4933819263170883354[24] = 0;
   out_4933819263170883354[25] = 0;
   out_4933819263170883354[26] = 0;
   out_4933819263170883354[27] = 0;
   out_4933819263170883354[28] = 0;
   out_4933819263170883354[29] = 0;
   out_4933819263170883354[30] = 1.0;
   out_4933819263170883354[31] = 0;
   out_4933819263170883354[32] = 0;
   out_4933819263170883354[33] = 0;
   out_4933819263170883354[34] = 0;
   out_4933819263170883354[35] = 0;
   out_4933819263170883354[36] = 0;
   out_4933819263170883354[37] = 0;
   out_4933819263170883354[38] = 0;
   out_4933819263170883354[39] = 0;
   out_4933819263170883354[40] = 1.0;
   out_4933819263170883354[41] = 0;
   out_4933819263170883354[42] = 0;
   out_4933819263170883354[43] = 0;
   out_4933819263170883354[44] = 0;
   out_4933819263170883354[45] = 0;
   out_4933819263170883354[46] = 0;
   out_4933819263170883354[47] = 0;
   out_4933819263170883354[48] = 0;
   out_4933819263170883354[49] = 0;
   out_4933819263170883354[50] = 1.0;
   out_4933819263170883354[51] = 0;
   out_4933819263170883354[52] = 0;
   out_4933819263170883354[53] = 0;
   out_4933819263170883354[54] = 0;
   out_4933819263170883354[55] = 0;
   out_4933819263170883354[56] = 0;
   out_4933819263170883354[57] = 0;
   out_4933819263170883354[58] = 0;
   out_4933819263170883354[59] = 0;
   out_4933819263170883354[60] = 1.0;
   out_4933819263170883354[61] = 0;
   out_4933819263170883354[62] = 0;
   out_4933819263170883354[63] = 0;
   out_4933819263170883354[64] = 0;
   out_4933819263170883354[65] = 0;
   out_4933819263170883354[66] = 0;
   out_4933819263170883354[67] = 0;
   out_4933819263170883354[68] = 0;
   out_4933819263170883354[69] = 0;
   out_4933819263170883354[70] = 1.0;
   out_4933819263170883354[71] = 0;
   out_4933819263170883354[72] = 0;
   out_4933819263170883354[73] = 0;
   out_4933819263170883354[74] = 0;
   out_4933819263170883354[75] = 0;
   out_4933819263170883354[76] = 0;
   out_4933819263170883354[77] = 0;
   out_4933819263170883354[78] = 0;
   out_4933819263170883354[79] = 0;
   out_4933819263170883354[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2322007936687586806) {
   out_2322007936687586806[0] = state[0];
   out_2322007936687586806[1] = state[1];
   out_2322007936687586806[2] = state[2];
   out_2322007936687586806[3] = state[3];
   out_2322007936687586806[4] = state[4];
   out_2322007936687586806[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2322007936687586806[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2322007936687586806[7] = state[7];
   out_2322007936687586806[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3088969633624438936) {
   out_3088969633624438936[0] = 1;
   out_3088969633624438936[1] = 0;
   out_3088969633624438936[2] = 0;
   out_3088969633624438936[3] = 0;
   out_3088969633624438936[4] = 0;
   out_3088969633624438936[5] = 0;
   out_3088969633624438936[6] = 0;
   out_3088969633624438936[7] = 0;
   out_3088969633624438936[8] = 0;
   out_3088969633624438936[9] = 0;
   out_3088969633624438936[10] = 1;
   out_3088969633624438936[11] = 0;
   out_3088969633624438936[12] = 0;
   out_3088969633624438936[13] = 0;
   out_3088969633624438936[14] = 0;
   out_3088969633624438936[15] = 0;
   out_3088969633624438936[16] = 0;
   out_3088969633624438936[17] = 0;
   out_3088969633624438936[18] = 0;
   out_3088969633624438936[19] = 0;
   out_3088969633624438936[20] = 1;
   out_3088969633624438936[21] = 0;
   out_3088969633624438936[22] = 0;
   out_3088969633624438936[23] = 0;
   out_3088969633624438936[24] = 0;
   out_3088969633624438936[25] = 0;
   out_3088969633624438936[26] = 0;
   out_3088969633624438936[27] = 0;
   out_3088969633624438936[28] = 0;
   out_3088969633624438936[29] = 0;
   out_3088969633624438936[30] = 1;
   out_3088969633624438936[31] = 0;
   out_3088969633624438936[32] = 0;
   out_3088969633624438936[33] = 0;
   out_3088969633624438936[34] = 0;
   out_3088969633624438936[35] = 0;
   out_3088969633624438936[36] = 0;
   out_3088969633624438936[37] = 0;
   out_3088969633624438936[38] = 0;
   out_3088969633624438936[39] = 0;
   out_3088969633624438936[40] = 1;
   out_3088969633624438936[41] = 0;
   out_3088969633624438936[42] = 0;
   out_3088969633624438936[43] = 0;
   out_3088969633624438936[44] = 0;
   out_3088969633624438936[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3088969633624438936[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3088969633624438936[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3088969633624438936[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3088969633624438936[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3088969633624438936[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3088969633624438936[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3088969633624438936[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3088969633624438936[53] = -9.8000000000000007*dt;
   out_3088969633624438936[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3088969633624438936[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3088969633624438936[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3088969633624438936[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3088969633624438936[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3088969633624438936[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3088969633624438936[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3088969633624438936[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3088969633624438936[62] = 0;
   out_3088969633624438936[63] = 0;
   out_3088969633624438936[64] = 0;
   out_3088969633624438936[65] = 0;
   out_3088969633624438936[66] = 0;
   out_3088969633624438936[67] = 0;
   out_3088969633624438936[68] = 0;
   out_3088969633624438936[69] = 0;
   out_3088969633624438936[70] = 1;
   out_3088969633624438936[71] = 0;
   out_3088969633624438936[72] = 0;
   out_3088969633624438936[73] = 0;
   out_3088969633624438936[74] = 0;
   out_3088969633624438936[75] = 0;
   out_3088969633624438936[76] = 0;
   out_3088969633624438936[77] = 0;
   out_3088969633624438936[78] = 0;
   out_3088969633624438936[79] = 0;
   out_3088969633624438936[80] = 1;
}
void h_25(double *state, double *unused, double *out_5535747379877375614) {
   out_5535747379877375614[0] = state[6];
}
void H_25(double *state, double *unused, double *out_906809627902024043) {
   out_906809627902024043[0] = 0;
   out_906809627902024043[1] = 0;
   out_906809627902024043[2] = 0;
   out_906809627902024043[3] = 0;
   out_906809627902024043[4] = 0;
   out_906809627902024043[5] = 0;
   out_906809627902024043[6] = 1;
   out_906809627902024043[7] = 0;
   out_906809627902024043[8] = 0;
}
void h_24(double *state, double *unused, double *out_5612233404153587199) {
   out_5612233404153587199[0] = state[4];
   out_5612233404153587199[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2651115970900299093) {
   out_2651115970900299093[0] = 0;
   out_2651115970900299093[1] = 0;
   out_2651115970900299093[2] = 0;
   out_2651115970900299093[3] = 0;
   out_2651115970900299093[4] = 1;
   out_2651115970900299093[5] = 0;
   out_2651115970900299093[6] = 0;
   out_2651115970900299093[7] = 0;
   out_2651115970900299093[8] = 0;
   out_2651115970900299093[9] = 0;
   out_2651115970900299093[10] = 0;
   out_2651115970900299093[11] = 0;
   out_2651115970900299093[12] = 0;
   out_2651115970900299093[13] = 0;
   out_2651115970900299093[14] = 1;
   out_2651115970900299093[15] = 0;
   out_2651115970900299093[16] = 0;
   out_2651115970900299093[17] = 0;
}
void h_30(double *state, double *unused, double *out_6137165537490739168) {
   out_6137165537490739168[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1036148575045264113) {
   out_1036148575045264113[0] = 0;
   out_1036148575045264113[1] = 0;
   out_1036148575045264113[2] = 0;
   out_1036148575045264113[3] = 0;
   out_1036148575045264113[4] = 1;
   out_1036148575045264113[5] = 0;
   out_1036148575045264113[6] = 0;
   out_1036148575045264113[7] = 0;
   out_1036148575045264113[8] = 0;
}
void h_26(double *state, double *unused, double *out_3289035123778336006) {
   out_3289035123778336006[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4648312946776080267) {
   out_4648312946776080267[0] = 0;
   out_4648312946776080267[1] = 0;
   out_4648312946776080267[2] = 0;
   out_4648312946776080267[3] = 0;
   out_4648312946776080267[4] = 0;
   out_4648312946776080267[5] = 0;
   out_4648312946776080267[6] = 0;
   out_4648312946776080267[7] = 1;
   out_4648312946776080267[8] = 0;
}
void h_27(double *state, double *unused, double *out_4932431495200898097) {
   out_4932431495200898097[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3210911886845689024) {
   out_3210911886845689024[0] = 0;
   out_3210911886845689024[1] = 0;
   out_3210911886845689024[2] = 0;
   out_3210911886845689024[3] = 1;
   out_3210911886845689024[4] = 0;
   out_3210911886845689024[5] = 0;
   out_3210911886845689024[6] = 0;
   out_3210911886845689024[7] = 0;
   out_3210911886845689024[8] = 0;
}
void h_29(double *state, double *unused, double *out_1294981966946875747) {
   out_1294981966946875747[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4924274613715240057) {
   out_4924274613715240057[0] = 0;
   out_4924274613715240057[1] = 1;
   out_4924274613715240057[2] = 0;
   out_4924274613715240057[3] = 0;
   out_4924274613715240057[4] = 0;
   out_4924274613715240057[5] = 0;
   out_4924274613715240057[6] = 0;
   out_4924274613715240057[7] = 0;
   out_4924274613715240057[8] = 0;
}
void h_28(double *state, double *unused, double *out_69241772459947548) {
   out_69241772459947548[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2960644342149913806) {
   out_2960644342149913806[0] = 1;
   out_2960644342149913806[1] = 0;
   out_2960644342149913806[2] = 0;
   out_2960644342149913806[3] = 0;
   out_2960644342149913806[4] = 0;
   out_2960644342149913806[5] = 0;
   out_2960644342149913806[6] = 0;
   out_2960644342149913806[7] = 0;
   out_2960644342149913806[8] = 0;
}
void h_31(double *state, double *unused, double *out_2227517876943296827) {
   out_2227517876943296827[0] = state[8];
}
void H_31(double *state, double *unused, double *out_876163666025063615) {
   out_876163666025063615[0] = 0;
   out_876163666025063615[1] = 0;
   out_876163666025063615[2] = 0;
   out_876163666025063615[3] = 0;
   out_876163666025063615[4] = 0;
   out_876163666025063615[5] = 0;
   out_876163666025063615[6] = 0;
   out_876163666025063615[7] = 0;
   out_876163666025063615[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_671052864023945038) {
  err_fun(nom_x, delta_x, out_671052864023945038);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4718563599199213006) {
  inv_err_fun(nom_x, true_x, out_4718563599199213006);
}
void car_H_mod_fun(double *state, double *out_4933819263170883354) {
  H_mod_fun(state, out_4933819263170883354);
}
void car_f_fun(double *state, double dt, double *out_2322007936687586806) {
  f_fun(state,  dt, out_2322007936687586806);
}
void car_F_fun(double *state, double dt, double *out_3088969633624438936) {
  F_fun(state,  dt, out_3088969633624438936);
}
void car_h_25(double *state, double *unused, double *out_5535747379877375614) {
  h_25(state, unused, out_5535747379877375614);
}
void car_H_25(double *state, double *unused, double *out_906809627902024043) {
  H_25(state, unused, out_906809627902024043);
}
void car_h_24(double *state, double *unused, double *out_5612233404153587199) {
  h_24(state, unused, out_5612233404153587199);
}
void car_H_24(double *state, double *unused, double *out_2651115970900299093) {
  H_24(state, unused, out_2651115970900299093);
}
void car_h_30(double *state, double *unused, double *out_6137165537490739168) {
  h_30(state, unused, out_6137165537490739168);
}
void car_H_30(double *state, double *unused, double *out_1036148575045264113) {
  H_30(state, unused, out_1036148575045264113);
}
void car_h_26(double *state, double *unused, double *out_3289035123778336006) {
  h_26(state, unused, out_3289035123778336006);
}
void car_H_26(double *state, double *unused, double *out_4648312946776080267) {
  H_26(state, unused, out_4648312946776080267);
}
void car_h_27(double *state, double *unused, double *out_4932431495200898097) {
  h_27(state, unused, out_4932431495200898097);
}
void car_H_27(double *state, double *unused, double *out_3210911886845689024) {
  H_27(state, unused, out_3210911886845689024);
}
void car_h_29(double *state, double *unused, double *out_1294981966946875747) {
  h_29(state, unused, out_1294981966946875747);
}
void car_H_29(double *state, double *unused, double *out_4924274613715240057) {
  H_29(state, unused, out_4924274613715240057);
}
void car_h_28(double *state, double *unused, double *out_69241772459947548) {
  h_28(state, unused, out_69241772459947548);
}
void car_H_28(double *state, double *unused, double *out_2960644342149913806) {
  H_28(state, unused, out_2960644342149913806);
}
void car_h_31(double *state, double *unused, double *out_2227517876943296827) {
  h_31(state, unused, out_2227517876943296827);
}
void car_H_31(double *state, double *unused, double *out_876163666025063615) {
  H_31(state, unused, out_876163666025063615);
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
