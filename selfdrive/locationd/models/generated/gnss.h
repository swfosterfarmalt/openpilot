#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3463381613600226321);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4676674826669643480);
void gnss_H_mod_fun(double *state, double *out_4538830618860333370);
void gnss_f_fun(double *state, double dt, double *out_1811539429430141585);
void gnss_F_fun(double *state, double dt, double *out_6885361808005668560);
void gnss_h_6(double *state, double *sat_pos, double *out_4720690199573780707);
void gnss_H_6(double *state, double *sat_pos, double *out_3501032272112231556);
void gnss_h_20(double *state, double *sat_pos, double *out_7764732769973401328);
void gnss_H_20(double *state, double *sat_pos, double *out_2224619370774720672);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_367888971529516748);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8328773051766743003);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_367888971529516748);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8328773051766743003);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}