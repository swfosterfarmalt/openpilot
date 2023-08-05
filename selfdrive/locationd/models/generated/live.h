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
void live_H(double *in_vec, double *out_6610048557944625674);
void live_err_fun(double *nom_x, double *delta_x, double *out_7748278816848259557);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2213082149044762718);
void live_H_mod_fun(double *state, double *out_5094183247678276312);
void live_f_fun(double *state, double dt, double *out_1900829864058528240);
void live_F_fun(double *state, double dt, double *out_3644154628780867195);
void live_h_4(double *state, double *unused, double *out_7072665034909624515);
void live_H_4(double *state, double *unused, double *out_4741796549636674704);
void live_h_9(double *state, double *unused, double *out_5823877824687254756);
void live_H_9(double *state, double *unused, double *out_4982986196266265349);
void live_h_10(double *state, double *unused, double *out_3210457795255247761);
void live_H_10(double *state, double *unused, double *out_6457423153758060774);
void live_h_12(double *state, double *unused, double *out_999243380165389066);
void live_H_12(double *state, double *unused, double *out_8685491116040915117);
void live_h_35(double *state, double *unused, double *out_6872417239258100351);
void live_H_35(double *state, double *unused, double *out_5939928083715901408);
void live_h_32(double *state, double *unused, double *out_6069528323998052603);
void live_H_32(double *state, double *unused, double *out_3999089746382892607);
void live_h_13(double *state, double *unused, double *out_5593807323605185206);
void live_H_13(double *state, double *unused, double *out_5691826018188565238);
void live_h_14(double *state, double *unused, double *out_5823877824687254756);
void live_H_14(double *state, double *unused, double *out_4982986196266265349);
void live_h_33(double *state, double *unused, double *out_2822426070322703074);
void live_H_33(double *state, double *unused, double *out_7187728462061411932);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}