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
void live_H(double *in_vec, double *out_4952847842636845307);
void live_err_fun(double *nom_x, double *delta_x, double *out_7174874578269485343);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2815856922138082094);
void live_H_mod_fun(double *state, double *out_6996944131011363031);
void live_f_fun(double *state, double dt, double *out_4379143464075805020);
void live_F_fun(double *state, double dt, double *out_4126089116567622472);
void live_h_4(double *state, double *unused, double *out_7517708438626322194);
void live_H_4(double *state, double *unused, double *out_4871242145388500877);
void live_h_9(double *state, double *unused, double *out_6840173855033849765);
void live_H_9(double *state, double *unused, double *out_5112431792018091522);
void live_h_10(double *state, double *unused, double *out_173744706950380686);
void live_H_10(double *state, double *unused, double *out_1457760568505601749);
void live_h_12(double *state, double *unused, double *out_5440948760933695840);
void live_H_12(double *state, double *unused, double *out_8556045520289088944);
void live_h_35(double *state, double *unused, double *out_6215788252178693963);
void live_H_35(double *state, double *unused, double *out_5810482487964075235);
void live_h_32(double *state, double *unused, double *out_7181094678608317786);
void live_H_32(double *state, double *unused, double *out_8295256810012369493);
void live_h_13(double *state, double *unused, double *out_5340487374202210186);
void live_H_13(double *state, double *unused, double *out_7628484752367355419);
void live_h_14(double *state, double *unused, double *out_6840173855033849765);
void live_H_14(double *state, double *unused, double *out_5112431792018091522);
void live_h_33(double *state, double *unused, double *out_5804146122758019849);
void live_H_33(double *state, double *unused, double *out_2659925483325217631);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}