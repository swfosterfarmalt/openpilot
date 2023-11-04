#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9018383677196817484);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7459293352492598460);
void gnss_H_mod_fun(double *state, double *out_2463199541683517663);
void gnss_f_fun(double *state, double dt, double *out_8399668097283148026);
void gnss_F_fun(double *state, double dt, double *out_3552796253967856461);
void gnss_h_6(double *state, double *sat_pos, double *out_6298886154741125804);
void gnss_H_6(double *state, double *sat_pos, double *out_2109310268915088104);
void gnss_h_20(double *state, double *sat_pos, double *out_3273347345638054669);
void gnss_H_20(double *state, double *sat_pos, double *out_8076940145235720636);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8571555988265576038);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_697782563777508081);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8571555988265576038);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_697782563777508081);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}