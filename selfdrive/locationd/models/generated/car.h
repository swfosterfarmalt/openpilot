#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5369626594687161812);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1748580837222196880);
void car_H_mod_fun(double *state, double *out_2693258022412199625);
void car_f_fun(double *state, double dt, double *out_7366003018438160507);
void car_F_fun(double *state, double dt, double *out_9173119452061934407);
void car_h_25(double *state, double *unused, double *out_4312494488748408182);
void car_H_25(double *state, double *unused, double *out_1156567547106706327);
void car_h_24(double *state, double *unused, double *out_4071417845229739232);
void car_H_24(double *state, double *unused, double *out_1016082051898793239);
void car_h_30(double *state, double *unused, double *out_3568778162005641003);
void car_H_30(double *state, double *unused, double *out_3674900505613954954);
void car_h_26(double *state, double *unused, double *out_8033318351721624937);
void car_H_26(double *state, double *unused, double *out_2584935771767349897);
void car_h_27(double *state, double *unused, double *out_689178778912437463);
void car_H_27(double *state, double *unused, double *out_1147534711836958654);
void car_h_29(double *state, double *unused, double *out_6844625974753097592);
void car_H_29(double *state, double *unused, double *out_4185131849928347138);
void car_h_28(double *state, double *unused, double *out_254331073989224972);
void car_H_28(double *state, double *unused, double *out_897267167141183436);
void car_h_31(double *state, double *unused, double *out_3450770768072264259);
void car_H_31(double *state, double *unused, double *out_1187213508983666755);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}