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
void live_H(double *in_vec, double *out_2414735134555351652);
void live_err_fun(double *nom_x, double *delta_x, double *out_4765299119652427496);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3190586852095350362);
void live_H_mod_fun(double *state, double *out_8614950881650688864);
void live_f_fun(double *state, double dt, double *out_2595414180991291416);
void live_F_fun(double *state, double dt, double *out_6803033353997994321);
void live_h_4(double *state, double *unused, double *out_616930058420824);
void live_H_4(double *state, double *unused, double *out_5710146728240620416);
void live_h_9(double *state, double *unused, double *out_3011225497219900464);
void live_H_9(double *state, double *unused, double *out_1577072207023827054);
void live_h_10(double *state, double *unused, double *out_1629810256331739409);
void live_H_10(double *state, double *unused, double *out_5489835648690208687);
void live_h_12(double *state, double *unused, double *out_20494760359511289);
void live_H_12(double *state, double *unused, double *out_6355338968426198204);
void live_h_35(double *state, double *unused, double *out_2434426115653514957);
void live_H_35(double *state, double *unused, double *out_9100902000751211913);
void live_h_32(double *state, double *unused, double *out_4703891601766069245);
void live_H_32(double *state, double *unused, double *out_2212319885927272571);
void live_h_13(double *state, double *unused, double *out_3772027060503193218);
void live_H_13(double *state, double *unused, double *out_3597831966201927386);
void live_h_14(double *state, double *unused, double *out_3011225497219900464);
void live_H_14(double *state, double *unused, double *out_1577072207023827054);
void live_h_33(double *state, double *unused, double *out_447286506532369353);
void live_H_33(double *state, double *unused, double *out_6195285068319482099);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}