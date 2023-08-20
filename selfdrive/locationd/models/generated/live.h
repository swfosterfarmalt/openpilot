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
void live_H(double *in_vec, double *out_5765856962138507314);
void live_err_fun(double *nom_x, double *delta_x, double *out_8128801794583824831);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8392023562780547307);
void live_H_mod_fun(double *state, double *out_5878743350081727116);
void live_f_fun(double *state, double dt, double *out_4858778294711581647);
void live_F_fun(double *state, double dt, double *out_4437991197939110159);
void live_h_4(double *state, double *unused, double *out_1526533485984711682);
void live_H_4(double *state, double *unused, double *out_8614362426384290758);
void live_h_9(double *state, double *unused, double *out_558098318269232442);
void live_H_9(double *state, double *unused, double *out_8855552073013881403);
void live_h_10(double *state, double *unused, double *out_640959415076880377);
void live_H_10(double *state, double *unused, double *out_6298957188028998798);
void live_h_12(double *state, double *unused, double *out_4925027671785651199);
void live_H_12(double *state, double *unused, double *out_9211282622277667191);
void live_h_35(double *state, double *unused, double *out_1853877955385180261);
void live_H_35(double *state, double *unused, double *out_6465719589952653482);
void live_h_32(double *state, double *unused, double *out_8903766325307788735);
void live_H_32(double *state, double *unused, double *out_5066159980062781778);
void live_h_13(double *state, double *unused, double *out_5672385168945637700);
void live_H_13(double *state, double *unused, double *out_5540611197974743508);
void live_h_14(double *state, double *unused, double *out_558098318269232442);
void live_H_14(double *state, double *unused, double *out_8855552073013881403);
void live_h_33(double *state, double *unused, double *out_3830038481069695916);
void live_H_33(double *state, double *unused, double *out_3315162585313795878);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}