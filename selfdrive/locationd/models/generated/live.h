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
void live_H(double *in_vec, double *out_7941023759042693369);
void live_err_fun(double *nom_x, double *delta_x, double *out_1859974344818569590);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9004749950139765637);
void live_H_mod_fun(double *state, double *out_2720961815570706232);
void live_f_fun(double *state, double dt, double *out_3053570887846599330);
void live_F_fun(double *state, double dt, double *out_4672631377469707742);
void live_h_4(double *state, double *unused, double *out_3957759162106459955);
void live_H_4(double *state, double *unused, double *out_8744211529056501276);
void live_h_9(double *state, double *unused, double *out_6576856407365962507);
void live_H_9(double *state, double *unused, double *out_2415313609388602870);
void live_h_10(double *state, double *unused, double *out_4747328977182000186);
void live_H_10(double *state, double *unused, double *out_6344708103702225709);
void live_h_12(double *state, double *unused, double *out_2351388817550320165);
void live_H_12(double *state, double *unused, double *out_9081433519605456673);
void live_h_35(double *state, double *unused, double *out_4544883733830743788);
void live_H_35(double *state, double *unused, double *out_6335870487280442964);
void live_h_32(double *state, double *unused, double *out_6942880184821978167);
void live_H_32(double *state, double *unused, double *out_9222351910128897460);
void live_h_13(double *state, double *unused, double *out_7655900047573857255);
void live_H_13(double *state, double *unused, double *out_7863485562721535985);
void live_h_14(double *state, double *unused, double *out_6576856407365962507);
void live_H_14(double *state, double *unused, double *out_2415313609388602870);
void live_h_33(double *state, double *unused, double *out_8024822863313080523);
void live_H_33(double *state, double *unused, double *out_3185313482641585360);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}