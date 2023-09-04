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
void err_fun(double *nom_x, double *delta_x, double *out_1954447027648292460) {
   out_1954447027648292460[0] = delta_x[0] + nom_x[0];
   out_1954447027648292460[1] = delta_x[1] + nom_x[1];
   out_1954447027648292460[2] = delta_x[2] + nom_x[2];
   out_1954447027648292460[3] = delta_x[3] + nom_x[3];
   out_1954447027648292460[4] = delta_x[4] + nom_x[4];
   out_1954447027648292460[5] = delta_x[5] + nom_x[5];
   out_1954447027648292460[6] = delta_x[6] + nom_x[6];
   out_1954447027648292460[7] = delta_x[7] + nom_x[7];
   out_1954447027648292460[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5248803308693147454) {
   out_5248803308693147454[0] = -nom_x[0] + true_x[0];
   out_5248803308693147454[1] = -nom_x[1] + true_x[1];
   out_5248803308693147454[2] = -nom_x[2] + true_x[2];
   out_5248803308693147454[3] = -nom_x[3] + true_x[3];
   out_5248803308693147454[4] = -nom_x[4] + true_x[4];
   out_5248803308693147454[5] = -nom_x[5] + true_x[5];
   out_5248803308693147454[6] = -nom_x[6] + true_x[6];
   out_5248803308693147454[7] = -nom_x[7] + true_x[7];
   out_5248803308693147454[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7259019347496494652) {
   out_7259019347496494652[0] = 1.0;
   out_7259019347496494652[1] = 0;
   out_7259019347496494652[2] = 0;
   out_7259019347496494652[3] = 0;
   out_7259019347496494652[4] = 0;
   out_7259019347496494652[5] = 0;
   out_7259019347496494652[6] = 0;
   out_7259019347496494652[7] = 0;
   out_7259019347496494652[8] = 0;
   out_7259019347496494652[9] = 0;
   out_7259019347496494652[10] = 1.0;
   out_7259019347496494652[11] = 0;
   out_7259019347496494652[12] = 0;
   out_7259019347496494652[13] = 0;
   out_7259019347496494652[14] = 0;
   out_7259019347496494652[15] = 0;
   out_7259019347496494652[16] = 0;
   out_7259019347496494652[17] = 0;
   out_7259019347496494652[18] = 0;
   out_7259019347496494652[19] = 0;
   out_7259019347496494652[20] = 1.0;
   out_7259019347496494652[21] = 0;
   out_7259019347496494652[22] = 0;
   out_7259019347496494652[23] = 0;
   out_7259019347496494652[24] = 0;
   out_7259019347496494652[25] = 0;
   out_7259019347496494652[26] = 0;
   out_7259019347496494652[27] = 0;
   out_7259019347496494652[28] = 0;
   out_7259019347496494652[29] = 0;
   out_7259019347496494652[30] = 1.0;
   out_7259019347496494652[31] = 0;
   out_7259019347496494652[32] = 0;
   out_7259019347496494652[33] = 0;
   out_7259019347496494652[34] = 0;
   out_7259019347496494652[35] = 0;
   out_7259019347496494652[36] = 0;
   out_7259019347496494652[37] = 0;
   out_7259019347496494652[38] = 0;
   out_7259019347496494652[39] = 0;
   out_7259019347496494652[40] = 1.0;
   out_7259019347496494652[41] = 0;
   out_7259019347496494652[42] = 0;
   out_7259019347496494652[43] = 0;
   out_7259019347496494652[44] = 0;
   out_7259019347496494652[45] = 0;
   out_7259019347496494652[46] = 0;
   out_7259019347496494652[47] = 0;
   out_7259019347496494652[48] = 0;
   out_7259019347496494652[49] = 0;
   out_7259019347496494652[50] = 1.0;
   out_7259019347496494652[51] = 0;
   out_7259019347496494652[52] = 0;
   out_7259019347496494652[53] = 0;
   out_7259019347496494652[54] = 0;
   out_7259019347496494652[55] = 0;
   out_7259019347496494652[56] = 0;
   out_7259019347496494652[57] = 0;
   out_7259019347496494652[58] = 0;
   out_7259019347496494652[59] = 0;
   out_7259019347496494652[60] = 1.0;
   out_7259019347496494652[61] = 0;
   out_7259019347496494652[62] = 0;
   out_7259019347496494652[63] = 0;
   out_7259019347496494652[64] = 0;
   out_7259019347496494652[65] = 0;
   out_7259019347496494652[66] = 0;
   out_7259019347496494652[67] = 0;
   out_7259019347496494652[68] = 0;
   out_7259019347496494652[69] = 0;
   out_7259019347496494652[70] = 1.0;
   out_7259019347496494652[71] = 0;
   out_7259019347496494652[72] = 0;
   out_7259019347496494652[73] = 0;
   out_7259019347496494652[74] = 0;
   out_7259019347496494652[75] = 0;
   out_7259019347496494652[76] = 0;
   out_7259019347496494652[77] = 0;
   out_7259019347496494652[78] = 0;
   out_7259019347496494652[79] = 0;
   out_7259019347496494652[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_970439124039022076) {
   out_970439124039022076[0] = state[0];
   out_970439124039022076[1] = state[1];
   out_970439124039022076[2] = state[2];
   out_970439124039022076[3] = state[3];
   out_970439124039022076[4] = state[4];
   out_970439124039022076[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_970439124039022076[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_970439124039022076[7] = state[7];
   out_970439124039022076[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2728382579438229754) {
   out_2728382579438229754[0] = 1;
   out_2728382579438229754[1] = 0;
   out_2728382579438229754[2] = 0;
   out_2728382579438229754[3] = 0;
   out_2728382579438229754[4] = 0;
   out_2728382579438229754[5] = 0;
   out_2728382579438229754[6] = 0;
   out_2728382579438229754[7] = 0;
   out_2728382579438229754[8] = 0;
   out_2728382579438229754[9] = 0;
   out_2728382579438229754[10] = 1;
   out_2728382579438229754[11] = 0;
   out_2728382579438229754[12] = 0;
   out_2728382579438229754[13] = 0;
   out_2728382579438229754[14] = 0;
   out_2728382579438229754[15] = 0;
   out_2728382579438229754[16] = 0;
   out_2728382579438229754[17] = 0;
   out_2728382579438229754[18] = 0;
   out_2728382579438229754[19] = 0;
   out_2728382579438229754[20] = 1;
   out_2728382579438229754[21] = 0;
   out_2728382579438229754[22] = 0;
   out_2728382579438229754[23] = 0;
   out_2728382579438229754[24] = 0;
   out_2728382579438229754[25] = 0;
   out_2728382579438229754[26] = 0;
   out_2728382579438229754[27] = 0;
   out_2728382579438229754[28] = 0;
   out_2728382579438229754[29] = 0;
   out_2728382579438229754[30] = 1;
   out_2728382579438229754[31] = 0;
   out_2728382579438229754[32] = 0;
   out_2728382579438229754[33] = 0;
   out_2728382579438229754[34] = 0;
   out_2728382579438229754[35] = 0;
   out_2728382579438229754[36] = 0;
   out_2728382579438229754[37] = 0;
   out_2728382579438229754[38] = 0;
   out_2728382579438229754[39] = 0;
   out_2728382579438229754[40] = 1;
   out_2728382579438229754[41] = 0;
   out_2728382579438229754[42] = 0;
   out_2728382579438229754[43] = 0;
   out_2728382579438229754[44] = 0;
   out_2728382579438229754[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2728382579438229754[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2728382579438229754[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2728382579438229754[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2728382579438229754[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2728382579438229754[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2728382579438229754[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2728382579438229754[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2728382579438229754[53] = -9.8000000000000007*dt;
   out_2728382579438229754[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2728382579438229754[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2728382579438229754[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2728382579438229754[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2728382579438229754[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2728382579438229754[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2728382579438229754[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2728382579438229754[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2728382579438229754[62] = 0;
   out_2728382579438229754[63] = 0;
   out_2728382579438229754[64] = 0;
   out_2728382579438229754[65] = 0;
   out_2728382579438229754[66] = 0;
   out_2728382579438229754[67] = 0;
   out_2728382579438229754[68] = 0;
   out_2728382579438229754[69] = 0;
   out_2728382579438229754[70] = 1;
   out_2728382579438229754[71] = 0;
   out_2728382579438229754[72] = 0;
   out_2728382579438229754[73] = 0;
   out_2728382579438229754[74] = 0;
   out_2728382579438229754[75] = 0;
   out_2728382579438229754[76] = 0;
   out_2728382579438229754[77] = 0;
   out_2728382579438229754[78] = 0;
   out_2728382579438229754[79] = 0;
   out_2728382579438229754[80] = 1;
}
void h_25(double *state, double *unused, double *out_8115337547472667507) {
   out_8115337547472667507[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5296403723882923727) {
   out_5296403723882923727[0] = 0;
   out_5296403723882923727[1] = 0;
   out_5296403723882923727[2] = 0;
   out_5296403723882923727[3] = 0;
   out_5296403723882923727[4] = 0;
   out_5296403723882923727[5] = 0;
   out_5296403723882923727[6] = 1;
   out_5296403723882923727[7] = 0;
   out_5296403723882923727[8] = 0;
}
void h_24(double *state, double *unused, double *out_8239611247265747344) {
   out_8239611247265747344[0] = state[4];
   out_8239611247265747344[1] = state[5];
}
void H_24(double *state, double *unused, double *out_476082219226935464) {
   out_476082219226935464[0] = 0;
   out_476082219226935464[1] = 0;
   out_476082219226935464[2] = 0;
   out_476082219226935464[3] = 0;
   out_476082219226935464[4] = 1;
   out_476082219226935464[5] = 0;
   out_476082219226935464[6] = 0;
   out_476082219226935464[7] = 0;
   out_476082219226935464[8] = 0;
   out_476082219226935464[9] = 0;
   out_476082219226935464[10] = 0;
   out_476082219226935464[11] = 0;
   out_476082219226935464[12] = 0;
   out_476082219226935464[13] = 0;
   out_476082219226935464[14] = 1;
   out_476082219226935464[15] = 0;
   out_476082219226935464[16] = 0;
   out_476082219226935464[17] = 0;
}
void h_30(double *state, double *unused, double *out_222398052588144148) {
   out_222398052588144148[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5167064776739683657) {
   out_5167064776739683657[0] = 0;
   out_5167064776739683657[1] = 0;
   out_5167064776739683657[2] = 0;
   out_5167064776739683657[3] = 0;
   out_5167064776739683657[4] = 1;
   out_5167064776739683657[5] = 0;
   out_5167064776739683657[6] = 0;
   out_5167064776739683657[7] = 0;
   out_5167064776739683657[8] = 0;
}
void h_26(double *state, double *unused, double *out_4779275215072701497) {
   out_4779275215072701497[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1554900405008867503) {
   out_1554900405008867503[0] = 0;
   out_1554900405008867503[1] = 0;
   out_1554900405008867503[2] = 0;
   out_1554900405008867503[3] = 0;
   out_1554900405008867503[4] = 0;
   out_1554900405008867503[5] = 0;
   out_1554900405008867503[6] = 0;
   out_1554900405008867503[7] = 1;
   out_1554900405008867503[8] = 0;
}
void h_27(double *state, double *unused, double *out_5825489477862353347) {
   out_5825489477862353347[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2992301464939258746) {
   out_2992301464939258746[0] = 0;
   out_2992301464939258746[1] = 0;
   out_2992301464939258746[2] = 0;
   out_2992301464939258746[3] = 1;
   out_2992301464939258746[4] = 0;
   out_2992301464939258746[5] = 0;
   out_2992301464939258746[6] = 0;
   out_2992301464939258746[7] = 0;
   out_2992301464939258746[8] = 0;
}
void h_29(double *state, double *unused, double *out_2382007268220553503) {
   out_2382007268220553503[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5677296121054075841) {
   out_5677296121054075841[0] = 0;
   out_5677296121054075841[1] = 1;
   out_5677296121054075841[2] = 0;
   out_5677296121054075841[3] = 0;
   out_5677296121054075841[4] = 0;
   out_5677296121054075841[5] = 0;
   out_5677296121054075841[6] = 0;
   out_5677296121054075841[7] = 0;
   out_5677296121054075841[8] = 0;
}
void h_28(double *state, double *unused, double *out_4135041643126672387) {
   out_4135041643126672387[0] = state[0];
}
void H_28(double *state, double *unused, double *out_594897103984545267) {
   out_594897103984545267[0] = 1;
   out_594897103984545267[1] = 0;
   out_594897103984545267[2] = 0;
   out_594897103984545267[3] = 0;
   out_594897103984545267[4] = 0;
   out_594897103984545267[5] = 0;
   out_594897103984545267[6] = 0;
   out_594897103984545267[7] = 0;
   out_594897103984545267[8] = 0;
}
void h_31(double *state, double *unused, double *out_7840143485188161618) {
   out_7840143485188161618[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5327049685759884155) {
   out_5327049685759884155[0] = 0;
   out_5327049685759884155[1] = 0;
   out_5327049685759884155[2] = 0;
   out_5327049685759884155[3] = 0;
   out_5327049685759884155[4] = 0;
   out_5327049685759884155[5] = 0;
   out_5327049685759884155[6] = 0;
   out_5327049685759884155[7] = 0;
   out_5327049685759884155[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1954447027648292460) {
  err_fun(nom_x, delta_x, out_1954447027648292460);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5248803308693147454) {
  inv_err_fun(nom_x, true_x, out_5248803308693147454);
}
void car_H_mod_fun(double *state, double *out_7259019347496494652) {
  H_mod_fun(state, out_7259019347496494652);
}
void car_f_fun(double *state, double dt, double *out_970439124039022076) {
  f_fun(state,  dt, out_970439124039022076);
}
void car_F_fun(double *state, double dt, double *out_2728382579438229754) {
  F_fun(state,  dt, out_2728382579438229754);
}
void car_h_25(double *state, double *unused, double *out_8115337547472667507) {
  h_25(state, unused, out_8115337547472667507);
}
void car_H_25(double *state, double *unused, double *out_5296403723882923727) {
  H_25(state, unused, out_5296403723882923727);
}
void car_h_24(double *state, double *unused, double *out_8239611247265747344) {
  h_24(state, unused, out_8239611247265747344);
}
void car_H_24(double *state, double *unused, double *out_476082219226935464) {
  H_24(state, unused, out_476082219226935464);
}
void car_h_30(double *state, double *unused, double *out_222398052588144148) {
  h_30(state, unused, out_222398052588144148);
}
void car_H_30(double *state, double *unused, double *out_5167064776739683657) {
  H_30(state, unused, out_5167064776739683657);
}
void car_h_26(double *state, double *unused, double *out_4779275215072701497) {
  h_26(state, unused, out_4779275215072701497);
}
void car_H_26(double *state, double *unused, double *out_1554900405008867503) {
  H_26(state, unused, out_1554900405008867503);
}
void car_h_27(double *state, double *unused, double *out_5825489477862353347) {
  h_27(state, unused, out_5825489477862353347);
}
void car_H_27(double *state, double *unused, double *out_2992301464939258746) {
  H_27(state, unused, out_2992301464939258746);
}
void car_h_29(double *state, double *unused, double *out_2382007268220553503) {
  h_29(state, unused, out_2382007268220553503);
}
void car_H_29(double *state, double *unused, double *out_5677296121054075841) {
  H_29(state, unused, out_5677296121054075841);
}
void car_h_28(double *state, double *unused, double *out_4135041643126672387) {
  h_28(state, unused, out_4135041643126672387);
}
void car_H_28(double *state, double *unused, double *out_594897103984545267) {
  H_28(state, unused, out_594897103984545267);
}
void car_h_31(double *state, double *unused, double *out_7840143485188161618) {
  h_31(state, unused, out_7840143485188161618);
}
void car_H_31(double *state, double *unused, double *out_5327049685759884155) {
  H_31(state, unused, out_5327049685759884155);
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
