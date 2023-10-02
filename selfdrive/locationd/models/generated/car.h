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
void car_err_fun(double *nom_x, double *delta_x, double *out_8485107762980471691);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_752342401365760782);
void car_H_mod_fun(double *state, double *out_4723183051327282475);
void car_f_fun(double *state, double dt, double *out_6080379077548842545);
void car_F_fun(double *state, double dt, double *out_5795928051360757777);
void car_h_25(double *state, double *unused, double *out_7016055871807953819);
void car_H_25(double *state, double *unused, double *out_5102981612663619538);
void car_h_24(double *state, double *unused, double *out_242298661145333250);
void car_H_24(double *state, double *unused, double *out_2930332013658119972);
void car_h_30(double *state, double *unused, double *out_7291249934092459708);
void car_H_30(double *state, double *unused, double *out_6427072119554315323);
void car_h_26(double *state, double *unused, double *out_8981697768828140668);
void car_H_26(double *state, double *unused, double *out_8407507582424420139);
void car_h_27(double *state, double *unused, double *out_5552402714342882709);
void car_H_27(double *state, double *unused, double *out_8601835431354740234);
void car_h_29(double *state, double *unused, double *out_2328869936411166476);
void car_H_29(double *state, double *unused, double *out_8131545915485260349);
void car_h_28(double *state, double *unused, double *out_7372524787142414587);
void car_H_28(double *state, double *unused, double *out_3049146898415729775);
void car_h_31(double *state, double *unused, double *out_2790810725183055913);
void car_H_31(double *state, double *unused, double *out_5133627574540579966);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}