#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2788921404904260909);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6290624704054750684);
void gnss_H_mod_fun(double *state, double *out_4733939560862216063);
void gnss_f_fun(double *state, double dt, double *out_3892250880040031115);
void gnss_F_fun(double *state, double dt, double *out_505639274279674341);
void gnss_h_6(double *state, double *sat_pos, double *out_3534555043245932423);
void gnss_H_6(double *state, double *sat_pos, double *out_5909739894810249108);
void gnss_h_20(double *state, double *sat_pos, double *out_1177420800671986150);
void gnss_H_20(double *state, double *sat_pos, double *out_2444663620237150446);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_391663488407230341);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5791096403187492688);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_391663488407230341);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5791096403187492688);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}