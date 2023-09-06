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
void car_err_fun(double *nom_x, double *delta_x, double *out_2574937435638925744);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4900970792020499484);
void car_H_mod_fun(double *state, double *out_8451084579596259389);
void car_f_fun(double *state, double dt, double *out_7474040492891634637);
void car_F_fun(double *state, double dt, double *out_3691743665564862162);
void car_h_25(double *state, double *unused, double *out_928977834463644485);
void car_H_25(double *state, double *unused, double *out_2178384050109672028);
void car_h_24(double *state, double *unused, double *out_4056782944777882268);
void car_H_24(double *state, double *unused, double *out_5736309648911995164);
void car_h_30(double *state, double *unused, double *out_4566427362717666835);
void car_H_30(double *state, double *unused, double *out_2049045102966431958);
void car_h_26(double *state, double *unused, double *out_6689255928088982829);
void car_H_26(double *state, double *unused, double *out_1563119268764384196);
void car_h_27(double *state, double *unused, double *out_6807263322022359573);
void car_H_27(double *state, double *unused, double *out_125718208833992953);
void car_h_29(double *state, double *unused, double *out_1832614154696182311);
void car_H_29(double *state, double *unused, double *out_2559276447280824142);
void car_h_28(double *state, double *unused, double *out_3051318069882350635);
void car_H_28(double *state, double *unused, double *out_2523122569788706432);
void car_h_31(double *state, double *unused, double *out_1530395992077008039);
void car_H_31(double *state, double *unused, double *out_2209030011986632456);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}