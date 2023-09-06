#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4307062985303373349);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4110902357590046395);
void gnss_H_mod_fun(double *state, double *out_6392659668905738982);
void gnss_f_fun(double *state, double dt, double *out_7119050100122636677);
void gnss_F_fun(double *state, double dt, double *out_8526931677833646522);
void gnss_h_6(double *state, double *sat_pos, double *out_5691792470437897435);
void gnss_H_6(double *state, double *sat_pos, double *out_3368042863135061588);
void gnss_h_20(double *state, double *sat_pos, double *out_3944654474512252825);
void gnss_H_20(double *state, double *sat_pos, double *out_1517584534183337728);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6702181570619932104);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6478109308117993176);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6702181570619932104);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6478109308117993176);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}