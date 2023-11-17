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
void err_fun(double *nom_x, double *delta_x, double *out_7598390566383516577) {
   out_7598390566383516577[0] = delta_x[0] + nom_x[0];
   out_7598390566383516577[1] = delta_x[1] + nom_x[1];
   out_7598390566383516577[2] = delta_x[2] + nom_x[2];
   out_7598390566383516577[3] = delta_x[3] + nom_x[3];
   out_7598390566383516577[4] = delta_x[4] + nom_x[4];
   out_7598390566383516577[5] = delta_x[5] + nom_x[5];
   out_7598390566383516577[6] = delta_x[6] + nom_x[6];
   out_7598390566383516577[7] = delta_x[7] + nom_x[7];
   out_7598390566383516577[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5639318755963839761) {
   out_5639318755963839761[0] = -nom_x[0] + true_x[0];
   out_5639318755963839761[1] = -nom_x[1] + true_x[1];
   out_5639318755963839761[2] = -nom_x[2] + true_x[2];
   out_5639318755963839761[3] = -nom_x[3] + true_x[3];
   out_5639318755963839761[4] = -nom_x[4] + true_x[4];
   out_5639318755963839761[5] = -nom_x[5] + true_x[5];
   out_5639318755963839761[6] = -nom_x[6] + true_x[6];
   out_5639318755963839761[7] = -nom_x[7] + true_x[7];
   out_5639318755963839761[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4346635802366151207) {
   out_4346635802366151207[0] = 1.0;
   out_4346635802366151207[1] = 0;
   out_4346635802366151207[2] = 0;
   out_4346635802366151207[3] = 0;
   out_4346635802366151207[4] = 0;
   out_4346635802366151207[5] = 0;
   out_4346635802366151207[6] = 0;
   out_4346635802366151207[7] = 0;
   out_4346635802366151207[8] = 0;
   out_4346635802366151207[9] = 0;
   out_4346635802366151207[10] = 1.0;
   out_4346635802366151207[11] = 0;
   out_4346635802366151207[12] = 0;
   out_4346635802366151207[13] = 0;
   out_4346635802366151207[14] = 0;
   out_4346635802366151207[15] = 0;
   out_4346635802366151207[16] = 0;
   out_4346635802366151207[17] = 0;
   out_4346635802366151207[18] = 0;
   out_4346635802366151207[19] = 0;
   out_4346635802366151207[20] = 1.0;
   out_4346635802366151207[21] = 0;
   out_4346635802366151207[22] = 0;
   out_4346635802366151207[23] = 0;
   out_4346635802366151207[24] = 0;
   out_4346635802366151207[25] = 0;
   out_4346635802366151207[26] = 0;
   out_4346635802366151207[27] = 0;
   out_4346635802366151207[28] = 0;
   out_4346635802366151207[29] = 0;
   out_4346635802366151207[30] = 1.0;
   out_4346635802366151207[31] = 0;
   out_4346635802366151207[32] = 0;
   out_4346635802366151207[33] = 0;
   out_4346635802366151207[34] = 0;
   out_4346635802366151207[35] = 0;
   out_4346635802366151207[36] = 0;
   out_4346635802366151207[37] = 0;
   out_4346635802366151207[38] = 0;
   out_4346635802366151207[39] = 0;
   out_4346635802366151207[40] = 1.0;
   out_4346635802366151207[41] = 0;
   out_4346635802366151207[42] = 0;
   out_4346635802366151207[43] = 0;
   out_4346635802366151207[44] = 0;
   out_4346635802366151207[45] = 0;
   out_4346635802366151207[46] = 0;
   out_4346635802366151207[47] = 0;
   out_4346635802366151207[48] = 0;
   out_4346635802366151207[49] = 0;
   out_4346635802366151207[50] = 1.0;
   out_4346635802366151207[51] = 0;
   out_4346635802366151207[52] = 0;
   out_4346635802366151207[53] = 0;
   out_4346635802366151207[54] = 0;
   out_4346635802366151207[55] = 0;
   out_4346635802366151207[56] = 0;
   out_4346635802366151207[57] = 0;
   out_4346635802366151207[58] = 0;
   out_4346635802366151207[59] = 0;
   out_4346635802366151207[60] = 1.0;
   out_4346635802366151207[61] = 0;
   out_4346635802366151207[62] = 0;
   out_4346635802366151207[63] = 0;
   out_4346635802366151207[64] = 0;
   out_4346635802366151207[65] = 0;
   out_4346635802366151207[66] = 0;
   out_4346635802366151207[67] = 0;
   out_4346635802366151207[68] = 0;
   out_4346635802366151207[69] = 0;
   out_4346635802366151207[70] = 1.0;
   out_4346635802366151207[71] = 0;
   out_4346635802366151207[72] = 0;
   out_4346635802366151207[73] = 0;
   out_4346635802366151207[74] = 0;
   out_4346635802366151207[75] = 0;
   out_4346635802366151207[76] = 0;
   out_4346635802366151207[77] = 0;
   out_4346635802366151207[78] = 0;
   out_4346635802366151207[79] = 0;
   out_4346635802366151207[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8097694292397678708) {
   out_8097694292397678708[0] = state[0];
   out_8097694292397678708[1] = state[1];
   out_8097694292397678708[2] = state[2];
   out_8097694292397678708[3] = state[3];
   out_8097694292397678708[4] = state[4];
   out_8097694292397678708[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8097694292397678708[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8097694292397678708[7] = state[7];
   out_8097694292397678708[8] = state[8];
}
void F_fun(double *state, double dt, double *out_41596102729771974) {
   out_41596102729771974[0] = 1;
   out_41596102729771974[1] = 0;
   out_41596102729771974[2] = 0;
   out_41596102729771974[3] = 0;
   out_41596102729771974[4] = 0;
   out_41596102729771974[5] = 0;
   out_41596102729771974[6] = 0;
   out_41596102729771974[7] = 0;
   out_41596102729771974[8] = 0;
   out_41596102729771974[9] = 0;
   out_41596102729771974[10] = 1;
   out_41596102729771974[11] = 0;
   out_41596102729771974[12] = 0;
   out_41596102729771974[13] = 0;
   out_41596102729771974[14] = 0;
   out_41596102729771974[15] = 0;
   out_41596102729771974[16] = 0;
   out_41596102729771974[17] = 0;
   out_41596102729771974[18] = 0;
   out_41596102729771974[19] = 0;
   out_41596102729771974[20] = 1;
   out_41596102729771974[21] = 0;
   out_41596102729771974[22] = 0;
   out_41596102729771974[23] = 0;
   out_41596102729771974[24] = 0;
   out_41596102729771974[25] = 0;
   out_41596102729771974[26] = 0;
   out_41596102729771974[27] = 0;
   out_41596102729771974[28] = 0;
   out_41596102729771974[29] = 0;
   out_41596102729771974[30] = 1;
   out_41596102729771974[31] = 0;
   out_41596102729771974[32] = 0;
   out_41596102729771974[33] = 0;
   out_41596102729771974[34] = 0;
   out_41596102729771974[35] = 0;
   out_41596102729771974[36] = 0;
   out_41596102729771974[37] = 0;
   out_41596102729771974[38] = 0;
   out_41596102729771974[39] = 0;
   out_41596102729771974[40] = 1;
   out_41596102729771974[41] = 0;
   out_41596102729771974[42] = 0;
   out_41596102729771974[43] = 0;
   out_41596102729771974[44] = 0;
   out_41596102729771974[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_41596102729771974[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_41596102729771974[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_41596102729771974[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_41596102729771974[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_41596102729771974[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_41596102729771974[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_41596102729771974[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_41596102729771974[53] = -9.8000000000000007*dt;
   out_41596102729771974[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_41596102729771974[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_41596102729771974[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_41596102729771974[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_41596102729771974[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_41596102729771974[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_41596102729771974[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_41596102729771974[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_41596102729771974[62] = 0;
   out_41596102729771974[63] = 0;
   out_41596102729771974[64] = 0;
   out_41596102729771974[65] = 0;
   out_41596102729771974[66] = 0;
   out_41596102729771974[67] = 0;
   out_41596102729771974[68] = 0;
   out_41596102729771974[69] = 0;
   out_41596102729771974[70] = 1;
   out_41596102729771974[71] = 0;
   out_41596102729771974[72] = 0;
   out_41596102729771974[73] = 0;
   out_41596102729771974[74] = 0;
   out_41596102729771974[75] = 0;
   out_41596102729771974[76] = 0;
   out_41596102729771974[77] = 0;
   out_41596102729771974[78] = 0;
   out_41596102729771974[79] = 0;
   out_41596102729771974[80] = 1;
}
void h_25(double *state, double *unused, double *out_3103901371871802669) {
   out_3103901371871802669[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3849843004749534923) {
   out_3849843004749534923[0] = 0;
   out_3849843004749534923[1] = 0;
   out_3849843004749534923[2] = 0;
   out_3849843004749534923[3] = 0;
   out_3849843004749534923[4] = 0;
   out_3849843004749534923[5] = 0;
   out_3849843004749534923[6] = 1;
   out_3849843004749534923[7] = 0;
   out_3849843004749534923[8] = 0;
}
void h_24(double *state, double *unused, double *out_8600469465376390611) {
   out_8600469465376390611[0] = state[4];
   out_8600469465376390611[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1677193405744035357) {
   out_1677193405744035357[0] = 0;
   out_1677193405744035357[1] = 0;
   out_1677193405744035357[2] = 0;
   out_1677193405744035357[3] = 0;
   out_1677193405744035357[4] = 1;
   out_1677193405744035357[5] = 0;
   out_1677193405744035357[6] = 0;
   out_1677193405744035357[7] = 0;
   out_1677193405744035357[8] = 0;
   out_1677193405744035357[9] = 0;
   out_1677193405744035357[10] = 0;
   out_1677193405744035357[11] = 0;
   out_1677193405744035357[12] = 0;
   out_1677193405744035357[13] = 0;
   out_1677193405744035357[14] = 1;
   out_1677193405744035357[15] = 0;
   out_1677193405744035357[16] = 0;
   out_1677193405744035357[17] = 0;
}
void h_30(double *state, double *unused, double *out_2946714703520673524) {
   out_2946714703520673524[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6368175963256783550) {
   out_6368175963256783550[0] = 0;
   out_6368175963256783550[1] = 0;
   out_6368175963256783550[2] = 0;
   out_6368175963256783550[3] = 0;
   out_6368175963256783550[4] = 1;
   out_6368175963256783550[5] = 0;
   out_6368175963256783550[6] = 0;
   out_6368175963256783550[7] = 0;
   out_6368175963256783550[8] = 0;
}
void h_26(double *state, double *unused, double *out_2119702815197669199) {
   out_2119702815197669199[0] = state[7];
}
void H_26(double *state, double *unused, double *out_108339685875478699) {
   out_108339685875478699[0] = 0;
   out_108339685875478699[1] = 0;
   out_108339685875478699[2] = 0;
   out_108339685875478699[3] = 0;
   out_108339685875478699[4] = 0;
   out_108339685875478699[5] = 0;
   out_108339685875478699[6] = 0;
   out_108339685875478699[7] = 1;
   out_108339685875478699[8] = 0;
}
void h_27(double *state, double *unused, double *out_8105574639532648314) {
   out_8105574639532648314[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8591770034440726767) {
   out_8591770034440726767[0] = 0;
   out_8591770034440726767[1] = 0;
   out_8591770034440726767[2] = 0;
   out_8591770034440726767[3] = 1;
   out_8591770034440726767[4] = 0;
   out_8591770034440726767[5] = 0;
   out_8591770034440726767[6] = 0;
   out_8591770034440726767[7] = 0;
   out_8591770034440726767[8] = 0;
}
void h_29(double *state, double *unused, double *out_7948387971181519169) {
   out_7948387971181519169[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6878407307571175734) {
   out_6878407307571175734[0] = 0;
   out_6878407307571175734[1] = 1;
   out_6878407307571175734[2] = 0;
   out_6878407307571175734[3] = 0;
   out_6878407307571175734[4] = 0;
   out_6878407307571175734[5] = 0;
   out_6878407307571175734[6] = 0;
   out_6878407307571175734[7] = 0;
   out_6878407307571175734[8] = 0;
}
void h_28(double *state, double *unused, double *out_5068788588088315629) {
   out_5068788588088315629[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1796008290501645160) {
   out_1796008290501645160[0] = 1;
   out_1796008290501645160[1] = 0;
   out_1796008290501645160[2] = 0;
   out_1796008290501645160[3] = 0;
   out_1796008290501645160[4] = 0;
   out_1796008290501645160[5] = 0;
   out_1796008290501645160[6] = 0;
   out_1796008290501645160[7] = 0;
   out_1796008290501645160[8] = 0;
}
void h_31(double *state, double *unused, double *out_2828707309587296780) {
   out_2828707309587296780[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3880488966626495351) {
   out_3880488966626495351[0] = 0;
   out_3880488966626495351[1] = 0;
   out_3880488966626495351[2] = 0;
   out_3880488966626495351[3] = 0;
   out_3880488966626495351[4] = 0;
   out_3880488966626495351[5] = 0;
   out_3880488966626495351[6] = 0;
   out_3880488966626495351[7] = 0;
   out_3880488966626495351[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7598390566383516577) {
  err_fun(nom_x, delta_x, out_7598390566383516577);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5639318755963839761) {
  inv_err_fun(nom_x, true_x, out_5639318755963839761);
}
void car_H_mod_fun(double *state, double *out_4346635802366151207) {
  H_mod_fun(state, out_4346635802366151207);
}
void car_f_fun(double *state, double dt, double *out_8097694292397678708) {
  f_fun(state,  dt, out_8097694292397678708);
}
void car_F_fun(double *state, double dt, double *out_41596102729771974) {
  F_fun(state,  dt, out_41596102729771974);
}
void car_h_25(double *state, double *unused, double *out_3103901371871802669) {
  h_25(state, unused, out_3103901371871802669);
}
void car_H_25(double *state, double *unused, double *out_3849843004749534923) {
  H_25(state, unused, out_3849843004749534923);
}
void car_h_24(double *state, double *unused, double *out_8600469465376390611) {
  h_24(state, unused, out_8600469465376390611);
}
void car_H_24(double *state, double *unused, double *out_1677193405744035357) {
  H_24(state, unused, out_1677193405744035357);
}
void car_h_30(double *state, double *unused, double *out_2946714703520673524) {
  h_30(state, unused, out_2946714703520673524);
}
void car_H_30(double *state, double *unused, double *out_6368175963256783550) {
  H_30(state, unused, out_6368175963256783550);
}
void car_h_26(double *state, double *unused, double *out_2119702815197669199) {
  h_26(state, unused, out_2119702815197669199);
}
void car_H_26(double *state, double *unused, double *out_108339685875478699) {
  H_26(state, unused, out_108339685875478699);
}
void car_h_27(double *state, double *unused, double *out_8105574639532648314) {
  h_27(state, unused, out_8105574639532648314);
}
void car_H_27(double *state, double *unused, double *out_8591770034440726767) {
  H_27(state, unused, out_8591770034440726767);
}
void car_h_29(double *state, double *unused, double *out_7948387971181519169) {
  h_29(state, unused, out_7948387971181519169);
}
void car_H_29(double *state, double *unused, double *out_6878407307571175734) {
  H_29(state, unused, out_6878407307571175734);
}
void car_h_28(double *state, double *unused, double *out_5068788588088315629) {
  h_28(state, unused, out_5068788588088315629);
}
void car_H_28(double *state, double *unused, double *out_1796008290501645160) {
  H_28(state, unused, out_1796008290501645160);
}
void car_h_31(double *state, double *unused, double *out_2828707309587296780) {
  h_31(state, unused, out_2828707309587296780);
}
void car_H_31(double *state, double *unused, double *out_3880488966626495351) {
  H_31(state, unused, out_3880488966626495351);
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
