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
void car_err_fun(double *nom_x, double *delta_x, double *out_7574054726488367863);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4766889803764304365);
void car_H_mod_fun(double *state, double *out_2258557682853182946);
void car_f_fun(double *state, double dt, double *out_3821644810047975420);
void car_F_fun(double *state, double dt, double *out_2591116970938824595);
void car_h_25(double *state, double *unused, double *out_157749596158077776);
void car_H_25(double *state, double *unused, double *out_6889492108556835482);
void car_h_24(double *state, double *unused, double *out_7347740927150280253);
void car_H_24(double *state, double *unused, double *out_4903046507044530377);
void car_h_30(double *state, double *unused, double *out_5472357644847062983);
void car_H_30(double *state, double *unused, double *out_6760153161413595412);
void car_h_26(double *state, double *unused, double *out_6415867497748725418);
void car_H_26(double *state, double *unused, double *out_3147988789682779258);
void car_h_27(double *state, double *unused, double *out_1201877934851292052);
void car_H_27(double *state, double *unused, double *out_4585389849613170501);
void car_h_29(double *state, double *unused, double *out_2603842393001749020);
void car_H_29(double *state, double *unused, double *out_7270384505727987596);
void car_h_28(double *state, double *unused, double *out_2892325769586973012);
void car_H_28(double *state, double *unused, double *out_2187985488658457022);
void car_h_31(double *state, double *unused, double *out_4752919809883254530);
void car_H_31(double *state, double *unused, double *out_6920138070433795910);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}