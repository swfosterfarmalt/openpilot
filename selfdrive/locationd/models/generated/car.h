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
void car_err_fun(double *nom_x, double *delta_x, double *out_3975600348957649681);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7988676026297517892);
void car_H_mod_fun(double *state, double *out_1700816133426219037);
void car_f_fun(double *state, double dt, double *out_5772475122730241095);
void car_F_fun(double *state, double dt, double *out_8078197424882555923);
void car_h_25(double *state, double *unused, double *out_188617941975566043);
void car_H_25(double *state, double *unused, double *out_3516111807592801652);
void car_h_24(double *state, double *unused, double *out_7581416323553796529);
void car_H_24(double *state, double *unused, double *out_1828295371723697461);
void car_h_30(double *state, double *unused, double *out_6408995124822580012);
void car_H_30(double *state, double *unused, double *out_997778849085553025);
void car_h_26(double *state, double *unused, double *out_5805679240146102495);
void car_H_26(double *state, double *unused, double *out_7257615126466857876);
void car_h_27(double *state, double *unused, double *out_2415787378833131608);
void car_H_27(double *state, double *unused, double *out_1225815222098390192);
void car_h_29(double *state, double *unused, double *out_5465485271920917577);
void car_H_29(double *state, double *unused, double *out_487547504771160841);
void car_h_28(double *state, double *unused, double *out_2585885888827714037);
void car_H_28(double *state, double *unused, double *out_5569946521840691415);
void car_h_31(double *state, double *unused, double *out_5966829253865094086);
void car_H_31(double *state, double *unused, double *out_7883823228700209352);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}