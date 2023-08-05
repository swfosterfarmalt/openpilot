#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_444352741485845956);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1833548362137252330);
void gnss_H_mod_fun(double *state, double *out_3884835978187609184);
void gnss_f_fun(double *state, double dt, double *out_2799479213853634378);
void gnss_F_fun(double *state, double dt, double *out_3097430740682718197);
void gnss_h_6(double *state, double *sat_pos, double *out_7192572793318069684);
void gnss_H_6(double *state, double *sat_pos, double *out_5970708195009703631);
void gnss_h_20(double *state, double *sat_pos, double *out_8398852110558080067);
void gnss_H_20(double *state, double *sat_pos, double *out_8813653629957763591);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2767556765111865509);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4773051478760733724);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2767556765111865509);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4773051478760733724);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}