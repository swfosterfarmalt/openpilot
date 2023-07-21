#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7670801227763844157);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5262035334049799005);
void gnss_H_mod_fun(double *state, double *out_655458551453000420);
void gnss_f_fun(double *state, double dt, double *out_1162856319156072173);
void gnss_F_fun(double *state, double dt, double *out_976946745335017488);
void gnss_h_6(double *state, double *sat_pos, double *out_2857555702628399245);
void gnss_H_6(double *state, double *sat_pos, double *out_1866219706673432532);
void gnss_h_20(double *state, double *sat_pos, double *out_2796042144131901280);
void gnss_H_20(double *state, double *sat_pos, double *out_5089222249954200270);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8606158719625759086);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3514710730864635405);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8606158719625759086);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3514710730864635405);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}