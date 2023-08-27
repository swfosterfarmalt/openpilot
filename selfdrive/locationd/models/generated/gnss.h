#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2759909455403238108);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8906084683496585849);
void gnss_H_mod_fun(double *state, double *out_4553317514464520854);
void gnss_f_fun(double *state, double dt, double *out_3315877890026810001);
void gnss_F_fun(double *state, double dt, double *out_647918121564994917);
void gnss_h_6(double *state, double *sat_pos, double *out_3218075092645176804);
void gnss_H_6(double *state, double *sat_pos, double *out_8344807646612916812);
void gnss_h_20(double *state, double *sat_pos, double *out_324931965682456940);
void gnss_H_20(double *state, double *sat_pos, double *out_2633905436684386266);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7688980450062471422);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7177043048130116370);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7688980450062471422);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7177043048130116370);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}