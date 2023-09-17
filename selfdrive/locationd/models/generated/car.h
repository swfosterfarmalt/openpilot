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
void car_err_fun(double *nom_x, double *delta_x, double *out_4793111812780848187);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8047223645973478750);
void car_H_mod_fun(double *state, double *out_6221619636727725848);
void car_f_fun(double *state, double dt, double *out_3279752275302509260);
void car_F_fun(double *state, double dt, double *out_329778069797634847);
void car_h_25(double *state, double *unused, double *out_3060125133492423155);
void car_H_25(double *state, double *unused, double *out_6595875180761845420);
void car_h_24(double *state, double *unused, double *out_2886292295865587153);
void car_H_24(double *state, double *unused, double *out_6982054027919999344);
void car_h_30(double *state, double *unused, double *out_4703140123328249286);
void car_H_30(double *state, double *unused, double *out_7323172562820097998);
void car_h_26(double *state, double *unused, double *out_8855600155973575318);
void car_H_26(double *state, double *unused, double *out_8109365574073649972);
void car_h_27(double *state, double *unused, double *out_455719812683725504);
void car_H_27(double *state, double *unused, double *out_5148409251019673087);
void car_h_29(double *state, double *unused, double *out_6393587958063930246);
void car_H_29(double *state, double *unused, double *out_7833403907134490182);
void car_h_28(double *state, double *unused, double *out_1667108752687590490);
void car_H_28(double *state, double *unused, double *out_8649709895009735183);
void car_h_31(double *state, double *unused, double *out_7312962692256136312);
void car_H_31(double *state, double *unused, double *out_7483157471840298496);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}