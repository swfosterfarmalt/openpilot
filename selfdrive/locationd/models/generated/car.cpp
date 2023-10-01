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
void err_fun(double *nom_x, double *delta_x, double *out_2652151718471411564) {
   out_2652151718471411564[0] = delta_x[0] + nom_x[0];
   out_2652151718471411564[1] = delta_x[1] + nom_x[1];
   out_2652151718471411564[2] = delta_x[2] + nom_x[2];
   out_2652151718471411564[3] = delta_x[3] + nom_x[3];
   out_2652151718471411564[4] = delta_x[4] + nom_x[4];
   out_2652151718471411564[5] = delta_x[5] + nom_x[5];
   out_2652151718471411564[6] = delta_x[6] + nom_x[6];
   out_2652151718471411564[7] = delta_x[7] + nom_x[7];
   out_2652151718471411564[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2984150088205140915) {
   out_2984150088205140915[0] = -nom_x[0] + true_x[0];
   out_2984150088205140915[1] = -nom_x[1] + true_x[1];
   out_2984150088205140915[2] = -nom_x[2] + true_x[2];
   out_2984150088205140915[3] = -nom_x[3] + true_x[3];
   out_2984150088205140915[4] = -nom_x[4] + true_x[4];
   out_2984150088205140915[5] = -nom_x[5] + true_x[5];
   out_2984150088205140915[6] = -nom_x[6] + true_x[6];
   out_2984150088205140915[7] = -nom_x[7] + true_x[7];
   out_2984150088205140915[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7924466651551420405) {
   out_7924466651551420405[0] = 1.0;
   out_7924466651551420405[1] = 0;
   out_7924466651551420405[2] = 0;
   out_7924466651551420405[3] = 0;
   out_7924466651551420405[4] = 0;
   out_7924466651551420405[5] = 0;
   out_7924466651551420405[6] = 0;
   out_7924466651551420405[7] = 0;
   out_7924466651551420405[8] = 0;
   out_7924466651551420405[9] = 0;
   out_7924466651551420405[10] = 1.0;
   out_7924466651551420405[11] = 0;
   out_7924466651551420405[12] = 0;
   out_7924466651551420405[13] = 0;
   out_7924466651551420405[14] = 0;
   out_7924466651551420405[15] = 0;
   out_7924466651551420405[16] = 0;
   out_7924466651551420405[17] = 0;
   out_7924466651551420405[18] = 0;
   out_7924466651551420405[19] = 0;
   out_7924466651551420405[20] = 1.0;
   out_7924466651551420405[21] = 0;
   out_7924466651551420405[22] = 0;
   out_7924466651551420405[23] = 0;
   out_7924466651551420405[24] = 0;
   out_7924466651551420405[25] = 0;
   out_7924466651551420405[26] = 0;
   out_7924466651551420405[27] = 0;
   out_7924466651551420405[28] = 0;
   out_7924466651551420405[29] = 0;
   out_7924466651551420405[30] = 1.0;
   out_7924466651551420405[31] = 0;
   out_7924466651551420405[32] = 0;
   out_7924466651551420405[33] = 0;
   out_7924466651551420405[34] = 0;
   out_7924466651551420405[35] = 0;
   out_7924466651551420405[36] = 0;
   out_7924466651551420405[37] = 0;
   out_7924466651551420405[38] = 0;
   out_7924466651551420405[39] = 0;
   out_7924466651551420405[40] = 1.0;
   out_7924466651551420405[41] = 0;
   out_7924466651551420405[42] = 0;
   out_7924466651551420405[43] = 0;
   out_7924466651551420405[44] = 0;
   out_7924466651551420405[45] = 0;
   out_7924466651551420405[46] = 0;
   out_7924466651551420405[47] = 0;
   out_7924466651551420405[48] = 0;
   out_7924466651551420405[49] = 0;
   out_7924466651551420405[50] = 1.0;
   out_7924466651551420405[51] = 0;
   out_7924466651551420405[52] = 0;
   out_7924466651551420405[53] = 0;
   out_7924466651551420405[54] = 0;
   out_7924466651551420405[55] = 0;
   out_7924466651551420405[56] = 0;
   out_7924466651551420405[57] = 0;
   out_7924466651551420405[58] = 0;
   out_7924466651551420405[59] = 0;
   out_7924466651551420405[60] = 1.0;
   out_7924466651551420405[61] = 0;
   out_7924466651551420405[62] = 0;
   out_7924466651551420405[63] = 0;
   out_7924466651551420405[64] = 0;
   out_7924466651551420405[65] = 0;
   out_7924466651551420405[66] = 0;
   out_7924466651551420405[67] = 0;
   out_7924466651551420405[68] = 0;
   out_7924466651551420405[69] = 0;
   out_7924466651551420405[70] = 1.0;
   out_7924466651551420405[71] = 0;
   out_7924466651551420405[72] = 0;
   out_7924466651551420405[73] = 0;
   out_7924466651551420405[74] = 0;
   out_7924466651551420405[75] = 0;
   out_7924466651551420405[76] = 0;
   out_7924466651551420405[77] = 0;
   out_7924466651551420405[78] = 0;
   out_7924466651551420405[79] = 0;
   out_7924466651551420405[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8050985811648556845) {
   out_8050985811648556845[0] = state[0];
   out_8050985811648556845[1] = state[1];
   out_8050985811648556845[2] = state[2];
   out_8050985811648556845[3] = state[3];
   out_8050985811648556845[4] = state[4];
   out_8050985811648556845[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8050985811648556845[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8050985811648556845[7] = state[7];
   out_8050985811648556845[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3656719619006401510) {
   out_3656719619006401510[0] = 1;
   out_3656719619006401510[1] = 0;
   out_3656719619006401510[2] = 0;
   out_3656719619006401510[3] = 0;
   out_3656719619006401510[4] = 0;
   out_3656719619006401510[5] = 0;
   out_3656719619006401510[6] = 0;
   out_3656719619006401510[7] = 0;
   out_3656719619006401510[8] = 0;
   out_3656719619006401510[9] = 0;
   out_3656719619006401510[10] = 1;
   out_3656719619006401510[11] = 0;
   out_3656719619006401510[12] = 0;
   out_3656719619006401510[13] = 0;
   out_3656719619006401510[14] = 0;
   out_3656719619006401510[15] = 0;
   out_3656719619006401510[16] = 0;
   out_3656719619006401510[17] = 0;
   out_3656719619006401510[18] = 0;
   out_3656719619006401510[19] = 0;
   out_3656719619006401510[20] = 1;
   out_3656719619006401510[21] = 0;
   out_3656719619006401510[22] = 0;
   out_3656719619006401510[23] = 0;
   out_3656719619006401510[24] = 0;
   out_3656719619006401510[25] = 0;
   out_3656719619006401510[26] = 0;
   out_3656719619006401510[27] = 0;
   out_3656719619006401510[28] = 0;
   out_3656719619006401510[29] = 0;
   out_3656719619006401510[30] = 1;
   out_3656719619006401510[31] = 0;
   out_3656719619006401510[32] = 0;
   out_3656719619006401510[33] = 0;
   out_3656719619006401510[34] = 0;
   out_3656719619006401510[35] = 0;
   out_3656719619006401510[36] = 0;
   out_3656719619006401510[37] = 0;
   out_3656719619006401510[38] = 0;
   out_3656719619006401510[39] = 0;
   out_3656719619006401510[40] = 1;
   out_3656719619006401510[41] = 0;
   out_3656719619006401510[42] = 0;
   out_3656719619006401510[43] = 0;
   out_3656719619006401510[44] = 0;
   out_3656719619006401510[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3656719619006401510[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3656719619006401510[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3656719619006401510[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3656719619006401510[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3656719619006401510[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3656719619006401510[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3656719619006401510[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3656719619006401510[53] = -9.8000000000000007*dt;
   out_3656719619006401510[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3656719619006401510[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3656719619006401510[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3656719619006401510[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3656719619006401510[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3656719619006401510[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3656719619006401510[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3656719619006401510[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3656719619006401510[62] = 0;
   out_3656719619006401510[63] = 0;
   out_3656719619006401510[64] = 0;
   out_3656719619006401510[65] = 0;
   out_3656719619006401510[66] = 0;
   out_3656719619006401510[67] = 0;
   out_3656719619006401510[68] = 0;
   out_3656719619006401510[69] = 0;
   out_3656719619006401510[70] = 1;
   out_3656719619006401510[71] = 0;
   out_3656719619006401510[72] = 0;
   out_3656719619006401510[73] = 0;
   out_3656719619006401510[74] = 0;
   out_3656719619006401510[75] = 0;
   out_3656719619006401510[76] = 0;
   out_3656719619006401510[77] = 0;
   out_3656719619006401510[78] = 0;
   out_3656719619006401510[79] = 0;
   out_3656719619006401510[80] = 1;
}
void h_25(double *state, double *unused, double *out_6684394539326124637) {
   out_6684394539326124637[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7730502843498639271) {
   out_7730502843498639271[0] = 0;
   out_7730502843498639271[1] = 0;
   out_7730502843498639271[2] = 0;
   out_7730502843498639271[3] = 0;
   out_7730502843498639271[4] = 0;
   out_7730502843498639271[5] = 0;
   out_7730502843498639271[6] = 1;
   out_7730502843498639271[7] = 0;
   out_7730502843498639271[8] = 0;
}
void h_24(double *state, double *unused, double *out_7023906729670555183) {
   out_7023906729670555183[0] = state[4];
   out_7023906729670555183[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4062035772102726588) {
   out_4062035772102726588[0] = 0;
   out_4062035772102726588[1] = 0;
   out_4062035772102726588[2] = 0;
   out_4062035772102726588[3] = 0;
   out_4062035772102726588[4] = 1;
   out_4062035772102726588[5] = 0;
   out_4062035772102726588[6] = 0;
   out_4062035772102726588[7] = 0;
   out_4062035772102726588[8] = 0;
   out_4062035772102726588[9] = 0;
   out_4062035772102726588[10] = 0;
   out_4062035772102726588[11] = 0;
   out_4062035772102726588[12] = 0;
   out_4062035772102726588[13] = 0;
   out_4062035772102726588[14] = 1;
   out_4062035772102726588[15] = 0;
   out_4062035772102726588[16] = 0;
   out_4062035772102726588[17] = 0;
}
void h_30(double *state, double *unused, double *out_6236292097832403866) {
   out_6236292097832403866[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7601163896355399201) {
   out_7601163896355399201[0] = 0;
   out_7601163896355399201[1] = 0;
   out_7601163896355399201[2] = 0;
   out_7601163896355399201[3] = 0;
   out_7601163896355399201[4] = 1;
   out_7601163896355399201[5] = 0;
   out_7601163896355399201[6] = 0;
   out_7601163896355399201[7] = 0;
   out_7601163896355399201[8] = 0;
}
void h_26(double *state, double *unused, double *out_7179801950734066301) {
   out_7179801950734066301[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8387356907608951175) {
   out_8387356907608951175[0] = 0;
   out_8387356907608951175[1] = 0;
   out_8387356907608951175[2] = 0;
   out_8387356907608951175[3] = 0;
   out_8387356907608951175[4] = 0;
   out_8387356907608951175[5] = 0;
   out_8387356907608951175[6] = 0;
   out_8387356907608951175[7] = 1;
   out_8387356907608951175[8] = 0;
}
void h_27(double *state, double *unused, double *out_3356692714739200326) {
   out_3356692714739200326[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5426400584554974290) {
   out_5426400584554974290[0] = 0;
   out_5426400584554974290[1] = 0;
   out_5426400584554974290[2] = 0;
   out_5426400584554974290[3] = 1;
   out_5426400584554974290[4] = 0;
   out_5426400584554974290[5] = 0;
   out_5426400584554974290[6] = 0;
   out_5426400584554974290[7] = 0;
   out_5426400584554974290[8] = 0;
}
void h_29(double *state, double *unused, double *out_1954728256588743358) {
   out_1954728256588743358[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8111395240669791385) {
   out_8111395240669791385[0] = 0;
   out_8111395240669791385[1] = 1;
   out_8111395240669791385[2] = 0;
   out_8111395240669791385[3] = 0;
   out_8111395240669791385[4] = 0;
   out_8111395240669791385[5] = 0;
   out_8111395240669791385[6] = 0;
   out_8111395240669791385[7] = 0;
   out_8111395240669791385[8] = 0;
}
void h_28(double *state, double *unused, double *out_4078450719689500487) {
   out_4078450719689500487[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3028996223600260811) {
   out_3028996223600260811[0] = 1;
   out_3028996223600260811[1] = 0;
   out_3028996223600260811[2] = 0;
   out_3028996223600260811[3] = 0;
   out_3028996223600260811[4] = 0;
   out_3028996223600260811[5] = 0;
   out_3028996223600260811[6] = 0;
   out_3028996223600260811[7] = 0;
   out_3028996223600260811[8] = 0;
}
void h_31(double *state, double *unused, double *out_4592706543277130280) {
   out_4592706543277130280[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7761148805375599699) {
   out_7761148805375599699[0] = 0;
   out_7761148805375599699[1] = 0;
   out_7761148805375599699[2] = 0;
   out_7761148805375599699[3] = 0;
   out_7761148805375599699[4] = 0;
   out_7761148805375599699[5] = 0;
   out_7761148805375599699[6] = 0;
   out_7761148805375599699[7] = 0;
   out_7761148805375599699[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2652151718471411564) {
  err_fun(nom_x, delta_x, out_2652151718471411564);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2984150088205140915) {
  inv_err_fun(nom_x, true_x, out_2984150088205140915);
}
void car_H_mod_fun(double *state, double *out_7924466651551420405) {
  H_mod_fun(state, out_7924466651551420405);
}
void car_f_fun(double *state, double dt, double *out_8050985811648556845) {
  f_fun(state,  dt, out_8050985811648556845);
}
void car_F_fun(double *state, double dt, double *out_3656719619006401510) {
  F_fun(state,  dt, out_3656719619006401510);
}
void car_h_25(double *state, double *unused, double *out_6684394539326124637) {
  h_25(state, unused, out_6684394539326124637);
}
void car_H_25(double *state, double *unused, double *out_7730502843498639271) {
  H_25(state, unused, out_7730502843498639271);
}
void car_h_24(double *state, double *unused, double *out_7023906729670555183) {
  h_24(state, unused, out_7023906729670555183);
}
void car_H_24(double *state, double *unused, double *out_4062035772102726588) {
  H_24(state, unused, out_4062035772102726588);
}
void car_h_30(double *state, double *unused, double *out_6236292097832403866) {
  h_30(state, unused, out_6236292097832403866);
}
void car_H_30(double *state, double *unused, double *out_7601163896355399201) {
  H_30(state, unused, out_7601163896355399201);
}
void car_h_26(double *state, double *unused, double *out_7179801950734066301) {
  h_26(state, unused, out_7179801950734066301);
}
void car_H_26(double *state, double *unused, double *out_8387356907608951175) {
  H_26(state, unused, out_8387356907608951175);
}
void car_h_27(double *state, double *unused, double *out_3356692714739200326) {
  h_27(state, unused, out_3356692714739200326);
}
void car_H_27(double *state, double *unused, double *out_5426400584554974290) {
  H_27(state, unused, out_5426400584554974290);
}
void car_h_29(double *state, double *unused, double *out_1954728256588743358) {
  h_29(state, unused, out_1954728256588743358);
}
void car_H_29(double *state, double *unused, double *out_8111395240669791385) {
  H_29(state, unused, out_8111395240669791385);
}
void car_h_28(double *state, double *unused, double *out_4078450719689500487) {
  h_28(state, unused, out_4078450719689500487);
}
void car_H_28(double *state, double *unused, double *out_3028996223600260811) {
  H_28(state, unused, out_3028996223600260811);
}
void car_h_31(double *state, double *unused, double *out_4592706543277130280) {
  h_31(state, unused, out_4592706543277130280);
}
void car_H_31(double *state, double *unused, double *out_7761148805375599699) {
  H_31(state, unused, out_7761148805375599699);
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
