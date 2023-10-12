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
void live_H(double *in_vec, double *out_1920292565159361144);
void live_err_fun(double *nom_x, double *delta_x, double *out_8960971141736065854);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6092473996019335073);
void live_H_mod_fun(double *state, double *out_6450239707838081688);
void live_f_fun(double *state, double dt, double *out_7416629977670457027);
void live_F_fun(double *state, double dt, double *out_64636203677338758);
void live_h_4(double *state, double *unused, double *out_4229943665795564963);
void live_H_4(double *state, double *unused, double *out_6822771310761048891);
void live_h_9(double *state, double *unused, double *out_2084672950082204567);
void live_H_9(double *state, double *unused, double *out_7063960957390639536);
void live_h_10(double *state, double *unused, double *out_5476039309526243161);
void live_H_10(double *state, double *unused, double *out_4889185259062790022);
void live_h_12(double *state, double *unused, double *out_3198692804067788211);
void live_H_12(double *state, double *unused, double *out_7443870335808642558);
void live_h_35(double *state, double *unused, double *out_1218819416543384806);
void live_H_35(double *state, double *unused, double *out_8257310705575895349);
void live_h_32(double *state, double *unused, double *out_1140620113963710055);
void live_H_32(double *state, double *unused, double *out_3274568864439539911);
void live_h_13(double *state, double *unused, double *out_6838285208320409473);
void live_H_13(double *state, double *unused, double *out_765406635205951091);
void live_h_14(double *state, double *unused, double *out_2084672950082204567);
void live_H_14(double *state, double *unused, double *out_7063960957390639536);
void live_h_33(double *state, double *unused, double *out_1760391852948304019);
void live_H_33(double *state, double *unused, double *out_5106753700937037745);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}