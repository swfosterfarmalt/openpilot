#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5002540976583435154);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8923382853611226714);
void gnss_H_mod_fun(double *state, double *out_7483018014028836974);
void gnss_f_fun(double *state, double dt, double *out_3233547939003755521);
void gnss_F_fun(double *state, double dt, double *out_6091345688566986461);
void gnss_h_6(double *state, double *sat_pos, double *out_1049109194514504534);
void gnss_H_6(double *state, double *sat_pos, double *out_824724143852124941);
void gnss_h_20(double *state, double *sat_pos, double *out_874640839125696915);
void gnss_H_20(double *state, double *sat_pos, double *out_4929145430862346338);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3022852619830856734);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6808293484879381181);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3022852619830856734);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6808293484879381181);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}