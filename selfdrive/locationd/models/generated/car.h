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
void car_err_fun(double *nom_x, double *delta_x, double *out_5388900906574639090);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8931120564270050854);
void car_H_mod_fun(double *state, double *out_4693862061305480966);
void car_f_fun(double *state, double dt, double *out_3234639509369413484);
void car_F_fun(double *state, double dt, double *out_8767551863233107118);
void car_h_25(double *state, double *unused, double *out_1318005858210708024);
void car_H_25(double *state, double *unused, double *out_3383746956418488080);
void car_h_24(double *state, double *unused, double *out_3565217635435425234);
void car_H_24(double *state, double *unused, double *out_4034644677258438813);
void car_h_30(double *state, double *unused, double *out_7178654823053031968);
void car_H_30(double *state, double *unused, double *out_7911443286546096278);
void car_h_26(double *state, double *unused, double *out_4557986449716406676);
void car_H_26(double *state, double *unused, double *out_7125250275292544304);
void car_h_27(double *state, double *unused, double *out_6319679125871553669);
void car_H_27(double *state, double *unused, double *out_5687849215362153061);
void car_h_29(double *state, double *unused, double *out_2176981555392186323);
void car_H_29(double *state, double *unused, double *out_7401211942231704094);
void car_h_28(double *state, double *unused, double *out_2903405836222483653);
void car_H_28(double *state, double *unused, double *out_5437581670666377843);
void car_h_31(double *state, double *unused, double *out_1042811795926202135);
void car_H_31(double *state, double *unused, double *out_3353100994541527652);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}