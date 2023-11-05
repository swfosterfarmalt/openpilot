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
void car_err_fun(double *nom_x, double *delta_x, double *out_7068129843327768808);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_733494678229042308);
void car_H_mod_fun(double *state, double *out_7571385414202809552);
void car_f_fun(double *state, double dt, double *out_9021702938268023425);
void car_F_fun(double *state, double dt, double *out_3272499767417673680);
void car_h_25(double *state, double *unused, double *out_7443089287366681720);
void car_H_25(double *state, double *unused, double *out_2778685672947439072);
void car_h_24(double *state, double *unused, double *out_9187770942234931660);
void car_H_24(double *state, double *unused, double *out_1967823654019069962);
void car_h_30(double *state, double *unused, double *out_8893720679995564887);
void car_H_30(double *state, double *unused, double *out_5297018631454687699);
void car_h_26(double *state, double *unused, double *out_1682811193741343376);
void car_H_26(double *state, double *unused, double *out_962817645926617152);
void car_h_27(double *state, double *unused, double *out_8399249465534172245);
void car_H_27(double *state, double *unused, double *out_3122255319654262788);
void car_h_29(double *state, double *unused, double *out_3892047412334719242);
void car_H_29(double *state, double *unused, double *out_5807249975769079883);
void car_h_28(double *state, double *unused, double *out_964106031105471432);
void car_H_28(double *state, double *unused, double *out_724850958699549309);
void car_h_31(double *state, double *unused, double *out_3122382135588820711);
void car_H_31(double *state, double *unused, double *out_1589025748159968628);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}