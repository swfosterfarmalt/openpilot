#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_284607730628362834);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7549528948775121586);
void gnss_H_mod_fun(double *state, double *out_961775498725370832);
void gnss_f_fun(double *state, double dt, double *out_543635440202532621);
void gnss_F_fun(double *state, double dt, double *out_2295083499474340999);
void gnss_h_6(double *state, double *sat_pos, double *out_5929645947376494773);
void gnss_H_6(double *state, double *sat_pos, double *out_520461601455328205);
void gnss_h_20(double *state, double *sat_pos, double *out_7758809767259933663);
void gnss_H_20(double *state, double *sat_pos, double *out_898721150884741153);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_219142218163687136);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7272996389187845126);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_219142218163687136);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7272996389187845126);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}