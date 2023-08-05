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
void car_err_fun(double *nom_x, double *delta_x, double *out_6639263734267575362);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8779595821562168901);
void car_H_mod_fun(double *state, double *out_1887421269059698674);
void car_f_fun(double *state, double dt, double *out_5758214277974107615);
void car_F_fun(double *state, double dt, double *out_4628961427457330570);
void car_h_25(double *state, double *unused, double *out_8692943286833080319);
void car_H_25(double *state, double *unused, double *out_2901388286093755664);
void car_h_24(double *state, double *unused, double *out_139111919068464966);
void car_H_24(double *state, double *unused, double *out_6317290601546600727);
void car_h_30(double *state, double *unused, double *out_196282605569340327);
void car_H_30(double *state, double *unused, double *out_1626308044033852534);
void car_h_26(double *state, double *unused, double *out_785801487765957785);
void car_H_26(double *state, double *unused, double *out_840115032780300560);
void car_h_27(double *state, double *unused, double *out_4608910723760823760);
void car_H_27(double *state, double *unused, double *out_3801071355834277445);
void car_h_29(double *state, double *unused, double *out_5197955873230185972);
void car_H_29(double *state, double *unused, double *out_1116076699719460350);
void car_h_28(double *state, double *unused, double *out_7421462827030677143);
void car_H_28(double *state, double *unused, double *out_847553571845865901);
void car_h_31(double *state, double *unused, double *out_8417749224548574430);
void car_H_31(double *state, double *unused, double *out_1466323135013652036);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}