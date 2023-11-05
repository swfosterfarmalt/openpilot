#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2179469339109170571);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7956168666464883728);
void gnss_H_mod_fun(double *state, double *out_514528688558190830);
void gnss_f_fun(double *state, double dt, double *out_6047950613379053758);
void gnss_F_fun(double *state, double dt, double *out_1077854140641575516);
void gnss_h_6(double *state, double *sat_pos, double *out_2024765830105387342);
void gnss_H_6(double *state, double *sat_pos, double *out_7891237798228026138);
void gnss_h_20(double *state, double *sat_pos, double *out_1046796656331890993);
void gnss_H_20(double *state, double *sat_pos, double *out_5982310217900809888);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5735942229984248599);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2797736668572699440);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5735942229984248599);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2797736668572699440);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}