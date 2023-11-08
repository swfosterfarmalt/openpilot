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
void car_err_fun(double *nom_x, double *delta_x, double *out_7167355006837459689);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_171878989924026497);
void car_H_mod_fun(double *state, double *out_2596456598414479484);
void car_f_fun(double *state, double dt, double *out_2608799569241697690);
void car_F_fun(double *state, double dt, double *out_2281501308483998435);
void car_h_25(double *state, double *unused, double *out_320415058552087998);
void car_H_25(double *state, double *unused, double *out_6663260843557340531);
void car_h_24(double *state, double *unused, double *out_4906346470436907167);
void car_H_24(double *state, double *unused, double *out_7508257452341035930);
void car_h_30(double *state, double *unused, double *out_595835607152529081);
void car_H_30(double *state, double *unused, double *out_4866792888660594330);
void car_h_26(double *state, double *unused, double *out_6196407366479202698);
void car_H_26(double *state, double *unused, double *out_2921757524683284307);
void car_h_27(double *state, double *unused, double *out_6078399972545825954);
void car_H_27(double *state, double *unused, double *out_7041556200461019241);
void car_h_29(double *state, double *unused, double *out_5597508874813374726);
void car_H_29(double *state, double *unused, double *out_8754918927330570274);
void car_h_28(double *state, double *unused, double *out_1076726704884980309);
void car_H_28(double *state, double *unused, double *out_4609426129309450768);
void car_h_31(double *state, double *unused, double *out_3316807983385999158);
void car_H_31(double *state, double *unused, double *out_6693906805434300959);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}