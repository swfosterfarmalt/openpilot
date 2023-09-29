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
void live_H(double *in_vec, double *out_470604318717566801);
void live_err_fun(double *nom_x, double *delta_x, double *out_6558990226915077565);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6401476958856669047);
void live_H_mod_fun(double *state, double *out_5973545985384515953);
void live_f_fun(double *state, double dt, double *out_6808343377148390101);
void live_F_fun(double *state, double dt, double *out_2412275078392409249);
void live_h_4(double *state, double *unused, double *out_2364343428603774062);
void live_H_4(double *state, double *unused, double *out_848492494935281580);
void live_h_9(double *state, double *unused, double *out_1191057817494216336);
void live_H_9(double *state, double *unused, double *out_1089682141564872225);
void live_h_10(double *state, double *unused, double *out_9105489698903399924);
void live_H_10(double *state, double *unused, double *out_486666004787405723);
void live_h_12(double *state, double *unused, double *out_2414492042108452084);
void live_H_12(double *state, double *unused, double *out_5867948902967243375);
void live_h_35(double *state, double *unused, double *out_6297167056393129579);
void live_H_35(double *state, double *unused, double *out_8613511935292257084);
void live_h_32(double *state, double *unused, double *out_4999194695924638870);
void live_H_32(double *state, double *unused, double *out_4031927766209681974);
void live_h_13(double *state, double *unused, double *out_7175463478001282133);
void live_H_13(double *state, double *unused, double *out_2154158160156252988);
void live_h_14(double *state, double *unused, double *out_1191057817494216336);
void live_H_14(double *state, double *unused, double *out_1089682141564872225);
void live_h_33(double *state, double *unused, double *out_1914367665099725448);
void live_H_33(double *state, double *unused, double *out_7365711556946746560);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}