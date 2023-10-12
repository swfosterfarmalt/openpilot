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
void car_err_fun(double *nom_x, double *delta_x, double *out_7739919700438188629);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6319375409011776283);
void car_H_mod_fun(double *state, double *out_2844378829659981252);
void car_f_fun(double *state, double dt, double *out_4884965143351282392);
void car_F_fun(double *state, double dt, double *out_6963307994085528976);
void car_h_25(double *state, double *unused, double *out_4344813781663258756);
void car_H_25(double *state, double *unused, double *out_7241202889850392582);
void car_h_24(double *state, double *unused, double *out_3932708101001244534);
void car_H_24(double *state, double *unused, double *out_1044291530223732530);
void car_h_30(double *state, double *unused, double *out_5170465887451101612);
void car_H_30(double *state, double *unused, double *out_8687208225351910407);
void car_h_26(double *state, double *unused, double *out_8274604918597604359);
void car_H_26(double *state, double *unused, double *out_7901015214098358433);
void car_h_27(double *state, double *unused, double *out_53138299618370256);
void car_H_27(double *state, double *unused, double *out_7584772536557216298);
void car_h_29(double *state, double *unused, double *out_104048368732758889);
void car_H_29(double *state, double *unused, double *out_8176976881037518223);
void car_h_28(double *state, double *unused, double *out_5054811567279215901);
void car_H_28(double *state, double *unused, double *out_5187368175602502819);
void car_h_31(double *state, double *unused, double *out_8585984496296497739);
void car_H_31(double *state, double *unused, double *out_2873491468742984882);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}