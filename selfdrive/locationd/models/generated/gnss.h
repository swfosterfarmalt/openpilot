#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4044646051050604849);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5682799120079336425);
void gnss_H_mod_fun(double *state, double *out_8388095332595561320);
void gnss_f_fun(double *state, double dt, double *out_5292647404086881635);
void gnss_F_fun(double *state, double dt, double *out_7434165543291924497);
void gnss_h_6(double *state, double *sat_pos, double *out_9172353468857903526);
void gnss_H_6(double *state, double *sat_pos, double *out_824859287253399737);
void gnss_h_20(double *state, double *sat_pos, double *out_2435912712014351082);
void gnss_H_20(double *state, double *sat_pos, double *out_652244719691705438);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2377929683236467987);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8047101356961090957);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2377929683236467987);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8047101356961090957);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}