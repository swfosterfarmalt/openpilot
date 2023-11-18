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
void car_err_fun(double *nom_x, double *delta_x, double *out_2474516941466880566);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9123673598000259248);
void car_H_mod_fun(double *state, double *out_2745915890047717913);
void car_f_fun(double *state, double dt, double *out_2687781910986821476);
void car_F_fun(double *state, double dt, double *out_1020427047685603609);
void car_h_25(double *state, double *unused, double *out_89412062583394035);
void car_H_25(double *state, double *unused, double *out_1503933484884863987);
void car_h_24(double *state, double *unused, double *out_7955775995665634223);
void car_H_24(double *state, double *unused, double *out_6777427767797876581);
void car_h_30(double *state, double *unused, double *out_10697938629569704);
void car_H_30(double *state, double *unused, double *out_4022266443392112614);
void car_h_26(double *state, double *unused, double *out_4990975329031275941);
void car_H_26(double *state, double *unused, double *out_2237569833989192237);
void car_h_27(double *state, double *unused, double *out_1439665929093890077);
void car_H_27(double *state, double *unused, double *out_6245860514576055831);
void car_h_29(double *state, double *unused, double *out_6767830816989016773);
void car_H_29(double *state, double *unused, double *out_4532497787706504798);
void car_h_28(double *state, double *unused, double *out_6441339196754735722);
void car_H_28(double *state, double *unused, double *out_549901229363025776);
void car_h_31(double *state, double *unused, double *out_7852677319404066476);
void car_H_31(double *state, double *unused, double *out_2863777936222543713);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}