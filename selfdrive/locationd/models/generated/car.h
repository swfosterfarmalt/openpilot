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
void car_err_fun(double *nom_x, double *delta_x, double *out_4569146530072895553);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2922497028284663501);
void car_H_mod_fun(double *state, double *out_7900836321283897023);
void car_f_fun(double *state, double dt, double *out_6809919584049877432);
void car_F_fun(double *state, double dt, double *out_9126469932184388727);
void car_h_25(double *state, double *unused, double *out_3518164862699664303);
void car_H_25(double *state, double *unused, double *out_7035607717603393194);
void car_h_24(double *state, double *unused, double *out_953103084894500303);
void car_H_24(double *state, double *unused, double *out_8019673588444706862);
void car_h_30(double *state, double *unused, double *out_7165313954189214872);
void car_H_30(double *state, double *unused, double *out_8892803397598909795);
void car_h_26(double *state, double *unused, double *out_2242113230925674041);
void car_H_26(double *state, double *unused, double *out_3294104398729336970);
void car_h_27(double *state, double *unused, double *out_6122570183508361954);
void car_H_27(double *state, double *unused, double *out_4731505458659728213);
void car_h_29(double *state, double *unused, double *out_6279756851859491099);
void car_H_29(double *state, double *unused, double *out_8382572053284517611);
void car_h_28(double *state, double *unused, double *out_452431478410802598);
void car_H_28(double *state, double *unused, double *out_4981773003355503431);
void car_h_31(double *state, double *unused, double *out_7047306560255838128);
void car_H_31(double *state, double *unused, double *out_7066253679480353622);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}