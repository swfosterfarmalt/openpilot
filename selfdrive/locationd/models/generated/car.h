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
void car_err_fun(double *nom_x, double *delta_x, double *out_3229655548484910964);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_154977718050279836);
void car_H_mod_fun(double *state, double *out_1011263767238227610);
void car_f_fun(double *state, double dt, double *out_5721512869151697320);
void car_F_fun(double *state, double dt, double *out_1692340814292537798);
void car_h_25(double *state, double *unused, double *out_2597451784216131708);
void car_H_25(double *state, double *unused, double *out_4643745684543313011);
void car_h_24(double *state, double *unused, double *out_5427474373712947275);
void car_H_24(double *state, double *unused, double *out_7148792931058052848);
void car_h_30(double *state, double *unused, double *out_2322257721931625819);
void car_H_30(double *state, double *unused, double *out_4514406737400072941);
void car_h_26(double *state, double *unused, double *out_7323930989592471464);
void car_H_26(double *state, double *unused, double *out_902242365669256787);
void car_h_27(double *state, double *unused, double *out_3540961637117794143);
void car_H_27(double *state, double *unused, double *out_2339643425599648030);
void car_h_29(double *state, double *unused, double *out_3265767574833288254);
void car_H_29(double *state, double *unused, double *out_5024638081714465125);
void car_h_28(double *state, double *unused, double *out_4108383536193540199);
void car_H_28(double *state, double *unused, double *out_57760935355065449);
void car_h_31(double *state, double *unused, double *out_4444331606499267924);
void car_H_31(double *state, double *unused, double *out_4674391646420273439);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}