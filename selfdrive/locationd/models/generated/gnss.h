#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9006316484952766497);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7014221750480788203);
void gnss_H_mod_fun(double *state, double *out_1743468875505645939);
void gnss_f_fun(double *state, double dt, double *out_2447433791296152011);
void gnss_F_fun(double *state, double dt, double *out_1083846469902090950);
void gnss_h_6(double *state, double *sat_pos, double *out_6943420607144317399);
void gnss_H_6(double *state, double *sat_pos, double *out_904843385930103144);
void gnss_h_20(double *state, double *sat_pos, double *out_5773685015670522167);
void gnss_H_20(double *state, double *sat_pos, double *out_5283321743023419019);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3961974541600867607);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2426909385454584489);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3961974541600867607);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2426909385454584489);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}