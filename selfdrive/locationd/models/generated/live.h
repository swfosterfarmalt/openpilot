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
void live_H(double *in_vec, double *out_8519043703690293172);
void live_err_fun(double *nom_x, double *delta_x, double *out_3680497765668344875);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_490859156954727028);
void live_H_mod_fun(double *state, double *out_1252192294812973493);
void live_f_fun(double *state, double dt, double *out_474865800768300853);
void live_F_fun(double *state, double dt, double *out_5017497839438550032);
void live_h_4(double *state, double *unused, double *out_7949008646907126126);
void live_H_4(double *state, double *unused, double *out_6964111654991874231);
void live_h_9(double *state, double *unused, double *out_3344885598970166155);
void live_H_9(double *state, double *unused, double *out_7205301301621464876);
void live_h_10(double *state, double *unused, double *out_2861810977634033663);
void live_H_10(double *state, double *unused, double *out_9187903489143333369);
void live_h_12(double *state, double *unused, double *out_1782214676500418139);
void live_H_12(double *state, double *unused, double *out_7585210680039467898);
void live_h_35(double *state, double *unused, double *out_7515623329853509537);
void live_H_35(double *state, double *unused, double *out_8115970361345070009);
void live_h_32(double *state, double *unused, double *out_1599319830146521781);
void live_H_32(double *state, double *unused, double *out_3415909208670365251);
void live_h_13(double *state, double *unused, double *out_8458160363789135668);
void live_H_13(double *state, double *unused, double *out_4639420343340967437);
void live_h_14(double *state, double *unused, double *out_3344885598970166155);
void live_H_14(double *state, double *unused, double *out_7205301301621464876);
void live_h_33(double *state, double *unused, double *out_5558343595682314927);
void live_H_33(double *state, double *unused, double *out_4965413356706212405);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}