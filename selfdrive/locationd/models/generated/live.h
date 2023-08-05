#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6232700668869886557);
void live_err_fun(double *nom_x, double *delta_x, double *out_1478261554725453242);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7358232558062317751);
void live_H_mod_fun(double *state, double *out_4935659974528521667);
void live_f_fun(double *state, double dt, double *out_1491263050808390229);
void live_F_fun(double *state, double dt, double *out_8117133176680196658);
void live_h_4(double *state, double *unused, double *out_405206402584394540);
void live_H_4(double *state, double *unused, double *out_4994686547697486725);
void live_h_9(double *state, double *unused, double *out_2151508975143286110);
void live_H_9(double *state, double *unused, double *out_6164838590747617421);
void live_h_10(double *state, double *unused, double *out_9176198998440989683);
void live_H_10(double *state, double *unused, double *out_244147991604393521);
void live_h_12(double *state, double *unused, double *out_1284073685258936711);
void live_H_12(double *state, double *unused, double *out_8432601117980103096);
void live_h_35(double *state, double *unused, double *out_3644461868441878761);
void live_H_35(double *state, double *unused, double *out_8361348605070094101);
void live_h_32(double *state, double *unused, double *out_7564506732801241555);
void live_H_32(double *state, double *unused, double *out_8492513390010834570);
void live_h_13(double *state, double *unused, double *out_7899918884651125118);
void live_H_13(double *state, double *unused, double *out_5068259913659727943);
void live_h_14(double *state, double *unused, double *out_2151508975143286110);
void live_H_14(double *state, double *unused, double *out_6164838590747617421);
void live_h_33(double *state, double *unused, double *out_4405361491101812079);
void live_H_33(double *state, double *unused, double *out_6934838464000599911);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}