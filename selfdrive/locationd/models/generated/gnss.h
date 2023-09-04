#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4516170446114263232);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2808882605666685159);
void gnss_H_mod_fun(double *state, double *out_2215232348373469859);
void gnss_f_fun(double *state, double dt, double *out_1776385195625842248);
void gnss_F_fun(double *state, double dt, double *out_900808918083314584);
void gnss_h_6(double *state, double *sat_pos, double *out_8025372775447727249);
void gnss_H_6(double *state, double *sat_pos, double *out_4754726028637191613);
void gnss_h_20(double *state, double *sat_pos, double *out_7770384175390878934);
void gnss_H_20(double *state, double *sat_pos, double *out_3064333999394004580);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6740989689516723293);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3412074168452321369);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6740989689516723293);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3412074168452321369);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}