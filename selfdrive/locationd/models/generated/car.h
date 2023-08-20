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
void car_err_fun(double *nom_x, double *delta_x, double *out_3195962060965127192);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5174661106618167489);
void car_H_mod_fun(double *state, double *out_6206683640939866959);
void car_f_fun(double *state, double dt, double *out_256968889868331023);
void car_F_fun(double *state, double dt, double *out_4251600485297607562);
void car_h_25(double *state, double *unused, double *out_1546507495354430623);
void car_H_25(double *state, double *unused, double *out_6942253115235371962);
void car_h_24(double *state, double *unused, double *out_962415991559381739);
void car_H_24(double *state, double *unused, double *out_4716545331256503400);
void car_h_30(double *state, double *unused, double *out_3061883139040810868);
void car_H_30(double *state, double *unused, double *out_7071592062378612032);
void car_h_26(double *state, double *unused, double *out_8063556406701656513);
void car_H_26(double *state, double *unused, double *out_7762987639600123430);
void car_h_27(double *state, double *unused, double *out_7945549012768279769);
void car_H_27(double *state, double *unused, double *out_9200388699530514673);
void car_h_29(double *state, double *unused, double *out_8403750374926841431);
void car_H_29(double *state, double *unused, double *out_6561360718064219848);
void car_h_28(double *state, double *unused, double *out_2943875745107434124);
void car_H_28(double *state, double *unused, double *out_4597730446498893597);
void car_h_31(double *state, double *unused, double *out_5183957023608452973);
void car_H_31(double *state, double *unused, double *out_6911607153358411534);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}