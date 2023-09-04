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
void car_err_fun(double *nom_x, double *delta_x, double *out_1954447027648292460);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5248803308693147454);
void car_H_mod_fun(double *state, double *out_7259019347496494652);
void car_f_fun(double *state, double dt, double *out_970439124039022076);
void car_F_fun(double *state, double dt, double *out_2728382579438229754);
void car_h_25(double *state, double *unused, double *out_8115337547472667507);
void car_H_25(double *state, double *unused, double *out_5296403723882923727);
void car_h_24(double *state, double *unused, double *out_8239611247265747344);
void car_H_24(double *state, double *unused, double *out_476082219226935464);
void car_h_30(double *state, double *unused, double *out_222398052588144148);
void car_H_30(double *state, double *unused, double *out_5167064776739683657);
void car_h_26(double *state, double *unused, double *out_4779275215072701497);
void car_H_26(double *state, double *unused, double *out_1554900405008867503);
void car_h_27(double *state, double *unused, double *out_5825489477862353347);
void car_H_27(double *state, double *unused, double *out_2992301464939258746);
void car_h_29(double *state, double *unused, double *out_2382007268220553503);
void car_H_29(double *state, double *unused, double *out_5677296121054075841);
void car_h_28(double *state, double *unused, double *out_4135041643126672387);
void car_H_28(double *state, double *unused, double *out_594897103984545267);
void car_h_31(double *state, double *unused, double *out_7840143485188161618);
void car_H_31(double *state, double *unused, double *out_5327049685759884155);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}