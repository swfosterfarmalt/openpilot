#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6447414538914227673);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5000797098598118139);
void gnss_H_mod_fun(double *state, double *out_4309541226740549276);
void gnss_f_fun(double *state, double dt, double *out_8646939531375188806);
void gnss_F_fun(double *state, double dt, double *out_4181600022368642599);
void gnss_h_6(double *state, double *sat_pos, double *out_2257566452131893418);
void gnss_H_6(double *state, double *sat_pos, double *out_997360191990300889);
void gnss_h_20(double *state, double *sat_pos, double *out_788991058837861864);
void gnss_H_20(double *state, double *sat_pos, double *out_5102792766153159856);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2714770949607692313);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4952928235801302959);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2714770949607692313);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4952928235801302959);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}