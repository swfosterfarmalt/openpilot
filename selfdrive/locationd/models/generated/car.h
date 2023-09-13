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
void car_err_fun(double *nom_x, double *delta_x, double *out_7443385576447730734);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3494304989908180493);
void car_H_mod_fun(double *state, double *out_6340163641565670916);
void car_f_fun(double *state, double dt, double *out_2083752042724885918);
void car_F_fun(double *state, double dt, double *out_8472269591488097829);
void car_h_25(double *state, double *unused, double *out_4469567855986124585);
void car_H_25(double *state, double *unused, double *out_9007571436358782224);
void car_h_24(double *state, double *unused, double *out_7691985016952978134);
void car_H_24(double *state, double *unused, double *out_5881247038548446256);
void car_h_30(double *state, double *unused, double *out_4626754524337253730);
void car_H_30(double *state, double *unused, double *out_8878232489215542154);
void car_h_26(double *state, double *unused, double *out_2584154961916967781);
void car_H_26(double *state, double *unused, double *out_5266068117484726000);
void car_h_27(double *state, double *unused, double *out_532105411674721060);
void car_H_27(double *state, double *unused, double *out_6703469177415117243);
void car_h_29(double *state, double *unused, double *out_374918743323591915);
void car_H_29(double *state, double *unused, double *out_4990106450545566210);
void car_h_28(double *state, double *unused, double *out_8449863760332119705);
void car_H_28(double *state, double *unused, double *out_6953736722110892461);
void car_h_31(double *state, double *unused, double *out_7389348291247131868);
void car_H_31(double *state, double *unused, double *out_9038217398235742652);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}