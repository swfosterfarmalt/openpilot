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
void car_err_fun(double *nom_x, double *delta_x, double *out_7673939181608642735);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6364931973222736938);
void car_H_mod_fun(double *state, double *out_2545105741110675571);
void car_f_fun(double *state, double dt, double *out_8600163977073292651);
void car_F_fun(double *state, double dt, double *out_2114740120359314092);
void car_h_25(double *state, double *unused, double *out_4848877104582332744);
void car_H_25(double *state, double *unused, double *out_2295858152052168291);
void car_h_24(double *state, double *unused, double *out_71651515018363944);
void car_H_24(double *state, double *unused, double *out_3790315666801304028);
void car_h_30(double *state, double *unused, double *out_2914388152238339697);
void car_H_30(double *state, double *unused, double *out_4620832189439448464);
void car_h_26(double *state, double *unused, double *out_7916061419899185342);
void car_H_26(double *state, double *unused, double *out_6037361470926224515);
void car_h_27(double *state, double *unused, double *out_8478633103568119553);
void car_H_27(double *state, double *unused, double *out_2446068877639023553);
void car_h_29(double *state, double *unused, double *out_2311072267561862180);
void car_H_29(double *state, double *unused, double *out_5131063533753840648);
void car_h_28(double *state, double *unused, double *out_3529776182748030504);
void car_H_28(double *state, double *unused, double *out_48664516684310074);
void car_h_31(double *state, double *unused, double *out_7430752929156084569);
void car_H_31(double *state, double *unused, double *out_2265212190175207863);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}