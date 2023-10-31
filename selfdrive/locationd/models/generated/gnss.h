#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3580552561584141668);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3985669647031736288);
void gnss_H_mod_fun(double *state, double *out_3674751707753416237);
void gnss_f_fun(double *state, double dt, double *out_5581775936242371520);
void gnss_F_fun(double *state, double dt, double *out_9136268394026262564);
void gnss_h_6(double *state, double *sat_pos, double *out_8396154811134387028);
void gnss_H_6(double *state, double *sat_pos, double *out_8585197235663590162);
void gnss_h_20(double *state, double *sat_pos, double *out_3794563826398603374);
void gnss_H_20(double *state, double *sat_pos, double *out_2931996085169558241);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7026932990701218247);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3463380596821841936);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7026932990701218247);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3463380596821841936);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}