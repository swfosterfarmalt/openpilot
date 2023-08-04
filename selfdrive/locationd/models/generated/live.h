#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2881037365619628279);
void live_err_fun(double *nom_x, double *delta_x, double *out_9185585876490325269);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4515645717665193781);
void live_H_mod_fun(double *state, double *out_7360777769815407851);
void live_f_fun(double *state, double dt, double *out_707519906875901283);
void live_F_fun(double *state, double dt, double *out_7537946981436927173);
void live_h_4(double *state, double *unused, double *out_7985008971958282204);
void live_H_4(double *state, double *unused, double *out_6639602133857460936);
void live_h_9(double *state, double *unused, double *out_7416348393553739455);
void live_H_9(double *state, double *unused, double *out_6880791780487051581);
void live_h_10(double *state, double *unused, double *out_8336492447638165131);
void live_H_10(double *state, double *unused, double *out_4399766487559122564);
void live_h_12(double *state, double *unused, double *out_748897964763280025);
void live_H_12(double *state, double *unused, double *out_6787685531820128885);
void live_h_35(double *state, double *unused, double *out_3350233371873118465);
void live_H_35(double *state, double *unused, double *out_8440479882479483304);
void live_h_32(double *state, double *unused, double *out_1211367634796053892);
void live_H_32(double *state, double *unused, double *out_1315395392884311885);
void live_h_13(double *state, double *unused, double *out_2955681637201582983);
void live_H_13(double *state, double *unused, double *out_4537722988519974402);
void live_h_14(double *state, double *unused, double *out_7416348393553739455);
void live_H_14(double *state, double *unused, double *out_6880791780487051581);
void live_h_33(double *state, double *unused, double *out_3008793661358065678);
void live_H_33(double *state, double *unused, double *out_5289922877840625700);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}