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
void live_H(double *in_vec, double *out_6451142613360380251);
void live_err_fun(double *nom_x, double *delta_x, double *out_7538598397836621666);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5762235078521032163);
void live_H_mod_fun(double *state, double *out_2616824879059836125);
void live_f_fun(double *state, double dt, double *out_2519803541471883852);
void live_F_fun(double *state, double dt, double *out_4993501443679153384);
void live_h_4(double *state, double *unused, double *out_773519739452149909);
void live_H_4(double *state, double *unused, double *out_3488853440007765243);
void live_h_9(double *state, double *unused, double *out_7910407917020635879);
void live_H_9(double *state, double *unused, double *out_599991887727685901);
void live_h_10(double *state, double *unused, double *out_3151163302577113248);
void live_H_10(double *state, double *unused, double *out_4364406165511907898);
void live_h_12(double *state, double *unused, double *out_9102017503874830796);
void live_H_12(double *state, double *unused, double *out_4178274873674685249);
void live_h_35(double *state, double *unused, double *out_661952609671183647);
void live_H_35(double *state, double *unused, double *out_122191382635157867);
void live_h_32(double *state, double *unused, double *out_7910574425261860947);
void live_H_32(double *state, double *unused, double *out_2582833130266592070);
void live_h_13(double *state, double *unused, double *out_1123454992331711379);
void live_H_13(double *state, double *unused, double *out_7377454176579306646);
void live_h_14(double *state, double *unused, double *out_7910407917020635879);
void live_H_14(double *state, double *unused, double *out_599991887727685901);
void live_h_33(double *state, double *unused, double *out_8148064412384831821);
void live_H_33(double *state, double *unused, double *out_8372349163070995054);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}