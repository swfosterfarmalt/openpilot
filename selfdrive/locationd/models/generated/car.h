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
void car_err_fun(double *nom_x, double *delta_x, double *out_2652151718471411564);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2984150088205140915);
void car_H_mod_fun(double *state, double *out_7924466651551420405);
void car_f_fun(double *state, double dt, double *out_8050985811648556845);
void car_F_fun(double *state, double dt, double *out_3656719619006401510);
void car_h_25(double *state, double *unused, double *out_6684394539326124637);
void car_H_25(double *state, double *unused, double *out_7730502843498639271);
void car_h_24(double *state, double *unused, double *out_7023906729670555183);
void car_H_24(double *state, double *unused, double *out_4062035772102726588);
void car_h_30(double *state, double *unused, double *out_6236292097832403866);
void car_H_30(double *state, double *unused, double *out_7601163896355399201);
void car_h_26(double *state, double *unused, double *out_7179801950734066301);
void car_H_26(double *state, double *unused, double *out_8387356907608951175);
void car_h_27(double *state, double *unused, double *out_3356692714739200326);
void car_H_27(double *state, double *unused, double *out_5426400584554974290);
void car_h_29(double *state, double *unused, double *out_1954728256588743358);
void car_H_29(double *state, double *unused, double *out_8111395240669791385);
void car_h_28(double *state, double *unused, double *out_4078450719689500487);
void car_H_28(double *state, double *unused, double *out_3028996223600260811);
void car_h_31(double *state, double *unused, double *out_4592706543277130280);
void car_H_31(double *state, double *unused, double *out_7761148805375599699);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}