#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5909540454426075247);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8457296220736712046);
void gnss_H_mod_fun(double *state, double *out_8531965417689122995);
void gnss_f_fun(double *state, double dt, double *out_6195388292700041643);
void gnss_F_fun(double *state, double dt, double *out_7909854684745487152);
void gnss_h_6(double *state, double *sat_pos, double *out_3920509931570091493);
void gnss_H_6(double *state, double *sat_pos, double *out_8872851097972471612);
void gnss_h_20(double *state, double *sat_pos, double *out_7175736005951966427);
void gnss_H_20(double *state, double *sat_pos, double *out_3709328815680046837);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2137916395300013369);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7240879494761677841);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2137916395300013369);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7240879494761677841);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}