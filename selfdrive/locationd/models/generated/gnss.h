#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6450837299122467107);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1281683845617115365);
void gnss_H_mod_fun(double *state, double *out_1101293044550311056);
void gnss_f_fun(double *state, double dt, double *out_944238033724073937);
void gnss_F_fun(double *state, double dt, double *out_6526830519377011082);
void gnss_h_6(double *state, double *sat_pos, double *out_2527351935937827091);
void gnss_H_6(double *state, double *sat_pos, double *out_7158454718814365747);
void gnss_h_20(double *state, double *sat_pos, double *out_2101410374362704668);
void gnss_H_20(double *state, double *sat_pos, double *out_4847885205478089006);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_386792629721769053);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4543080048435198643);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_386792629721769053);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4543080048435198643);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}