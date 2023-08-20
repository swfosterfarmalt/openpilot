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
void live_H(double *in_vec, double *out_8859560392771966978);
void live_err_fun(double *nom_x, double *delta_x, double *out_275793430844566277);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1256350007897599276);
void live_H_mod_fun(double *state, double *out_6601594514128786682);
void live_f_fun(double *state, double dt, double *out_8845733240490315512);
void live_F_fun(double *state, double dt, double *out_1382002735738886191);
void live_h_4(double *state, double *unused, double *out_6295913573181500801);
void live_H_4(double *state, double *unused, double *out_7346355438007863230);
void live_h_9(double *state, double *unused, double *out_8505409878227279928);
void live_H_9(double *state, double *unused, double *out_3813169700437240916);
void live_h_10(double *state, double *unused, double *out_2471859477179688568);
void live_H_10(double *state, double *unused, double *out_7463458794690068065);
void live_h_12(double *state, double *unused, double *out_3832372823363074171);
void live_H_12(double *state, double *unused, double *out_3433260322019237894);
void live_h_35(double *state, double *unused, double *out_6235703645985212609);
void live_H_35(double *state, double *unused, double *out_687697289694224185);
void live_h_32(double *state, double *unused, double *out_6171406099382606695);
void live_H_32(double *state, double *unused, double *out_7602561793388340541);
void live_h_13(double *state, double *unused, double *out_5743512044801874994);
void live_H_13(double *state, double *unused, double *out_9039445725242075232);
void live_h_14(double *state, double *unused, double *out_8505409878227279928);
void live_H_14(double *state, double *unused, double *out_3813169700437240916);
void live_h_33(double *state, double *unused, double *out_1281986419790075968);
void live_H_33(double *state, double *unused, double *out_2462859714944633419);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}