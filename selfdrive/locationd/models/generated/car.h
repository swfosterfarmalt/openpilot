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
void car_err_fun(double *nom_x, double *delta_x, double *out_9197230539031214470);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5205798968448591509);
void car_H_mod_fun(double *state, double *out_6671470295335338631);
void car_f_fun(double *state, double dt, double *out_1262138838755003200);
void car_F_fun(double *state, double dt, double *out_125455060826810986);
void car_h_25(double *state, double *unused, double *out_15986601659185824);
void car_H_25(double *state, double *unused, double *out_546342770720130705);
void car_h_24(double *state, double *unused, double *out_5620042477706618172);
void car_H_24(double *state, double *unused, double *out_8598523939230746457);
void car_h_30(double *state, double *unused, double *out_9121730683024665358);
void car_H_30(double *state, double *unused, double *out_3981353559407477493);
void car_h_26(double *state, double *unused, double *out_5587104823615023375);
void car_H_26(double *state, double *unused, double *out_3195160548153925519);
void car_h_27(double *state, double *unused, double *out_5017659869320031469);
void car_H_27(double *state, double *unused, double *out_6156116871207902404);
void car_h_29(double *state, double *unused, double *out_8879343345748533121);
void car_H_29(double *state, double *unused, double *out_3471122215093085309);
void car_h_28(double *state, double *unused, double *out_2193249616934190042);
void car_H_28(double *state, double *unused, double *out_1507491943527759058);
void car_h_31(double *state, double *unused, double *out_259207460625320065);
void car_H_31(double *state, double *unused, double *out_3821368650387276995);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}