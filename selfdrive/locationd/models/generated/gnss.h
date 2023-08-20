#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4321674817797598520);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3485155282649920022);
void gnss_H_mod_fun(double *state, double *out_5535577955172455216);
void gnss_f_fun(double *state, double dt, double *out_1238601709221895068);
void gnss_F_fun(double *state, double dt, double *out_5092949869051383406);
void gnss_h_6(double *state, double *sat_pos, double *out_5638122544984721887);
void gnss_H_6(double *state, double *sat_pos, double *out_7628797803822472301);
void gnss_h_20(double *state, double *sat_pos, double *out_8758860236983935183);
void gnss_H_20(double *state, double *sat_pos, double *out_8519583756205282960);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_780098669313691505);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3736749610851893537);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_780098669313691505);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3736749610851893537);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}