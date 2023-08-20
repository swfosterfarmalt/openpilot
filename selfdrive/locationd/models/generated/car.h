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
void car_err_fun(double *nom_x, double *delta_x, double *out_671052864023945038);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4718563599199213006);
void car_H_mod_fun(double *state, double *out_4933819263170883354);
void car_f_fun(double *state, double dt, double *out_2322007936687586806);
void car_F_fun(double *state, double dt, double *out_3088969633624438936);
void car_h_25(double *state, double *unused, double *out_5535747379877375614);
void car_H_25(double *state, double *unused, double *out_906809627902024043);
void car_h_24(double *state, double *unused, double *out_5612233404153587199);
void car_H_24(double *state, double *unused, double *out_2651115970900299093);
void car_h_30(double *state, double *unused, double *out_6137165537490739168);
void car_H_30(double *state, double *unused, double *out_1036148575045264113);
void car_h_26(double *state, double *unused, double *out_3289035123778336006);
void car_H_26(double *state, double *unused, double *out_4648312946776080267);
void car_h_27(double *state, double *unused, double *out_4932431495200898097);
void car_H_27(double *state, double *unused, double *out_3210911886845689024);
void car_h_29(double *state, double *unused, double *out_1294981966946875747);
void car_H_29(double *state, double *unused, double *out_4924274613715240057);
void car_h_28(double *state, double *unused, double *out_69241772459947548);
void car_H_28(double *state, double *unused, double *out_2960644342149913806);
void car_h_31(double *state, double *unused, double *out_2227517876943296827);
void car_H_31(double *state, double *unused, double *out_876163666025063615);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}