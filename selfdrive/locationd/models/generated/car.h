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
void car_err_fun(double *nom_x, double *delta_x, double *out_3502673967241664248);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5131477847977226664);
void car_H_mod_fun(double *state, double *out_2530919945440300104);
void car_f_fun(double *state, double dt, double *out_7205150727819091354);
void car_F_fun(double *state, double dt, double *out_7417487238667392724);
void car_h_25(double *state, double *unused, double *out_2841136473189617083);
void car_H_25(double *state, double *unused, double *out_8264885873019158718);
void car_h_24(double *state, double *unused, double *out_466890035114431795);
void car_H_24(double *state, double *unused, double *out_3606286394098874797);
void car_h_30(double *state, double *unused, double *out_5040136177564432102);
void car_H_30(double *state, double *unused, double *out_3265167859198776143);
void car_h_26(double *state, double *unused, double *out_38462909903586457);
void car_H_26(double *state, double *unused, double *out_4523382554145102494);
void car_h_27(double *state, double *unused, double *out_3784646326091279518);
void car_H_27(double *state, double *unused, double *out_5439931170999201054);
void car_h_29(double *state, double *unused, double *out_4096626324662769667);
void car_H_29(double *state, double *unused, double *out_7153293897868752087);
void car_h_28(double *state, double *unused, double *out_8263056230204422952);
void car_H_28(double *state, double *unused, double *out_6211051158771268955);
void car_h_31(double *state, double *unused, double *out_2004104806923773306);
void car_H_31(double *state, double *unused, double *out_8295531834896119146);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}