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
void live_H(double *in_vec, double *out_3237850412590264377);
void live_err_fun(double *nom_x, double *delta_x, double *out_786389705708054918);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2061415588084110731);
void live_H_mod_fun(double *state, double *out_8608232382931709485);
void live_f_fun(double *state, double dt, double *out_4785434985982575598);
void live_F_fun(double *state, double dt, double *out_1171459914458974777);
void live_h_4(double *state, double *unused, double *out_5681251961842903636);
void live_H_4(double *state, double *unused, double *out_1380979307874200219);
void live_h_9(double *state, double *unused, double *out_5112082236684918754);
void live_H_9(double *state, double *unused, double *out_8668198243138647689);
void live_h_10(double *state, double *unused, double *out_4955041470653423657);
void live_H_10(double *state, double *unused, double *out_17546421107381143);
void live_h_12(double *state, double *unused, double *out_1905694267703785960);
void live_H_12(double *state, double *unused, double *out_5000279069168532777);
void live_h_35(double *state, double *unused, double *out_4899215905558210316);
void live_H_35(double *state, double *unused, double *out_9145998748231175723);
void live_h_32(double *state, double *unused, double *out_5135024665828606004);
void live_H_32(double *state, double *unused, double *out_8833377053343730695);
void live_h_13(double *state, double *unused, double *out_2570952339250706509);
void live_H_13(double *state, double *unused, double *out_4744662095952477789);
void live_h_14(double *state, double *unused, double *out_5112082236684918754);
void live_H_14(double *state, double *unused, double *out_8668198243138647689);
void live_h_33(double *state, double *unused, double *out_4491193780097901364);
void live_H_33(double *state, double *unused, double *out_7898198369885665199);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}