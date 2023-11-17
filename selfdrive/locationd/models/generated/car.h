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
void car_err_fun(double *nom_x, double *delta_x, double *out_7598390566383516577);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5639318755963839761);
void car_H_mod_fun(double *state, double *out_4346635802366151207);
void car_f_fun(double *state, double dt, double *out_8097694292397678708);
void car_F_fun(double *state, double dt, double *out_41596102729771974);
void car_h_25(double *state, double *unused, double *out_3103901371871802669);
void car_H_25(double *state, double *unused, double *out_3849843004749534923);
void car_h_24(double *state, double *unused, double *out_8600469465376390611);
void car_H_24(double *state, double *unused, double *out_1677193405744035357);
void car_h_30(double *state, double *unused, double *out_2946714703520673524);
void car_H_30(double *state, double *unused, double *out_6368175963256783550);
void car_h_26(double *state, double *unused, double *out_2119702815197669199);
void car_H_26(double *state, double *unused, double *out_108339685875478699);
void car_h_27(double *state, double *unused, double *out_8105574639532648314);
void car_H_27(double *state, double *unused, double *out_8591770034440726767);
void car_h_29(double *state, double *unused, double *out_7948387971181519169);
void car_H_29(double *state, double *unused, double *out_6878407307571175734);
void car_h_28(double *state, double *unused, double *out_5068788588088315629);
void car_H_28(double *state, double *unused, double *out_1796008290501645160);
void car_h_31(double *state, double *unused, double *out_2828707309587296780);
void car_H_31(double *state, double *unused, double *out_3880488966626495351);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}