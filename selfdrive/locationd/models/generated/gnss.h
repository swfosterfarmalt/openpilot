#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1199080895228679500);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3733508033735132626);
void gnss_H_mod_fun(double *state, double *out_3459453620548230826);
void gnss_f_fun(double *state, double dt, double *out_9104597615753488104);
void gnss_F_fun(double *state, double dt, double *out_8533997876613764308);
void gnss_h_6(double *state, double *sat_pos, double *out_3522191147772980852);
void gnss_H_6(double *state, double *sat_pos, double *out_41023783411261112);
void gnss_h_20(double *state, double *sat_pos, double *out_2634656617657040111);
void gnss_H_20(double *state, double *sat_pos, double *out_3182068259027006405);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3983293769857751604);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1182800211671042582);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3983293769857751604);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1182800211671042582);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}