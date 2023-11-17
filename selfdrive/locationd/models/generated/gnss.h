#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7898758279284104251);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_258779395976866605);
void gnss_H_mod_fun(double *state, double *out_1068558286524968);
void gnss_f_fun(double *state, double dt, double *out_6635033905056555325);
void gnss_F_fun(double *state, double dt, double *out_8502563063737433073);
void gnss_h_6(double *state, double *sat_pos, double *out_4881918134730526271);
void gnss_H_6(double *state, double *sat_pos, double *out_1972889481654701448);
void gnss_h_20(double *state, double *sat_pos, double *out_5245777435176666735);
void gnss_H_20(double *state, double *sat_pos, double *out_380673705059921505);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1916653750505923582);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_788355260993627668);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1916653750505923582);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_788355260993627668);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}