#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4009183954963761830);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5668670552804156843);
void gnss_H_mod_fun(double *state, double *out_7793541619644674581);
void gnss_f_fun(double *state, double dt, double *out_2953486288822651743);
void gnss_F_fun(double *state, double dt, double *out_7502400574187806738);
void gnss_h_6(double *state, double *sat_pos, double *out_5515239915342403667);
void gnss_H_6(double *state, double *sat_pos, double *out_691111391283485275);
void gnss_h_20(double *state, double *sat_pos, double *out_6421220721641647458);
void gnss_H_20(double *state, double *sat_pos, double *out_4436024380608800520);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8868946558143862461);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8408613646792434496);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8868946558143862461);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8408613646792434496);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}