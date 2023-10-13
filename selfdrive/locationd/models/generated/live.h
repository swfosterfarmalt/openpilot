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
void live_H(double *in_vec, double *out_7991541939242048212);
void live_err_fun(double *nom_x, double *delta_x, double *out_9165175297852018824);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8840460989217465937);
void live_H_mod_fun(double *state, double *out_2323407358908051241);
void live_f_fun(double *state, double dt, double *out_7679666672239594573);
void live_F_fun(double *state, double dt, double *out_6529899552359769468);
void live_h_4(double *state, double *unused, double *out_3869304003052469608);
void live_H_4(double *state, double *unused, double *out_1451348106210356524);
void live_h_9(double *state, double *unused, double *out_2177619537968645471);
void live_H_9(double *state, double *unused, double *out_1692537752839947169);
void live_h_10(double *state, double *unused, double *out_7579447694123810588);
void live_H_10(double *state, double *unused, double *out_5136410709345790733);
void live_h_12(double *state, double *unused, double *out_1960893434720126470);
void live_H_12(double *state, double *unused, double *out_6470804514242318319);
void live_h_35(double *state, double *unused, double *out_4281719968276769877);
void live_H_35(double *state, double *unused, double *out_9216367546567332028);
void live_h_32(double *state, double *unused, double *out_1943421361472635268);
void live_H_32(double *state, double *unused, double *out_7289538189809210787);
void live_h_13(double *state, double *unused, double *out_1922592336643947345);
void live_H_13(double *state, double *unused, double *out_4134273078622778629);
void live_h_14(double *state, double *unused, double *out_2177619537968645471);
void live_H_14(double *state, double *unused, double *out_1692537752839947169);
void live_h_33(double *state, double *unused, double *out_8025325674593916277);
void live_H_33(double *state, double *unused, double *out_6079819522503361984);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}