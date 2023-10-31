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
void car_err_fun(double *nom_x, double *delta_x, double *out_7056533279073179144);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_116170648062875201);
void car_H_mod_fun(double *state, double *out_2198752619133686475);
void car_f_fun(double *state, double dt, double *out_2859451684574302086);
void car_F_fun(double *state, double dt, double *out_1750058964340999480);
void car_h_25(double *state, double *unused, double *out_2976450765751548964);
void car_H_25(double *state, double *unused, double *out_1918401128996020553);
void car_h_24(double *state, double *unused, double *out_9061254321776730835);
void car_H_24(double *state, double *unused, double *out_1447943646952681829);
void car_h_30(double *state, double *unused, double *out_5924176756107033925);
void car_H_30(double *state, double *unused, double *out_1789062181852780483);
void car_h_26(double *state, double *unused, double *out_8960962807551366610);
void car_H_26(double *state, double *unused, double *out_1823102189878035671);
void car_h_27(double *state, double *unused, double *out_2032940912849886529);
void car_H_27(double *state, double *unused, double *out_385701129947644428);
void car_h_29(double *state, double *unused, double *out_4980666903205371490);
void car_H_29(double *state, double *unused, double *out_2299293526167172667);
void car_h_28(double *state, double *unused, double *out_8609923727063989452);
void car_H_28(double *state, double *unused, double *out_7181462873886726035);
void car_h_31(double *state, double *unused, double *out_3802102871539391820);
void car_H_31(double *state, double *unused, double *out_1949047090872980981);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}