#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7494899235073949606);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_608002279011974407);
void gnss_H_mod_fun(double *state, double *out_7002084543314776290);
void gnss_f_fun(double *state, double dt, double *out_7089057631738940520);
void gnss_F_fun(double *state, double dt, double *out_4393676974186888507);
void gnss_h_6(double *state, double *sat_pos, double *out_3305816208013416001);
void gnss_H_6(double *state, double *sat_pos, double *out_4780980724345831917);
void gnss_h_20(double *state, double *sat_pos, double *out_2426811405452364869);
void gnss_H_20(double *state, double *sat_pos, double *out_8252454322377627110);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8737519804286952927);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3207489715113657734);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8737519804286952927);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3207489715113657734);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}