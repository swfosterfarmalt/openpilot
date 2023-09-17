#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3903931754286709959);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6187082929283308233);
void gnss_H_mod_fun(double *state, double *out_3389584686144192417);
void gnss_f_fun(double *state, double dt, double *out_9055272195158205822);
void gnss_F_fun(double *state, double dt, double *out_681731754219421529);
void gnss_h_6(double *state, double *sat_pos, double *out_1841729915697983286);
void gnss_H_6(double *state, double *sat_pos, double *out_8761235728672435821);
void gnss_h_20(double *state, double *sat_pos, double *out_2924715623646539284);
void gnss_H_20(double *state, double *sat_pos, double *out_2535761186771767843);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1370555120028092744);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5519331308243009628);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1370555120028092744);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5519331308243009628);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}