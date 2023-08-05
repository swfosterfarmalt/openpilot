#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5797809671637013283);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2749815054075637968);
void gnss_H_mod_fun(double *state, double *out_2172530486751409334);
void gnss_f_fun(double *state, double dt, double *out_3803902687444144824);
void gnss_F_fun(double *state, double dt, double *out_448765062510618999);
void gnss_h_6(double *state, double *sat_pos, double *out_7666487914334335596);
void gnss_H_6(double *state, double *sat_pos, double *out_2843628536195130770);
void gnss_h_20(double *state, double *sat_pos, double *out_3527336028405721629);
void gnss_H_20(double *state, double *sat_pos, double *out_1506007362187013094);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1146256510176586591);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2837864402044772102);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1146256510176586591);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2837864402044772102);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}