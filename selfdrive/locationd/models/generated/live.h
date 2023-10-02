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
void live_H(double *in_vec, double *out_8185154824396842924);
void live_err_fun(double *nom_x, double *delta_x, double *out_6874723704538611314);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_715471445446418310);
void live_H_mod_fun(double *state, double *out_7903858007706249740);
void live_f_fun(double *state, double dt, double *out_4931375126789355244);
void live_F_fun(double *state, double dt, double *out_3000197326783185579);
void live_h_4(double *state, double *unused, double *out_799707160660174299);
void live_H_4(double *state, double *unused, double *out_7964895101051458796);
void live_h_9(double *state, double *unused, double *out_2705653560226498363);
void live_H_9(double *state, double *unused, double *out_8206084747681049441);
void live_h_10(double *state, double *unused, double *out_7997323943072708299);
void live_H_10(double *state, double *unused, double *out_8868272782527553003);
void live_h_12(double *state, double *unused, double *out_6565642392846210650);
void live_H_12(double *state, double *unused, double *out_8585994126099052463);
void live_h_35(double *state, double *unused, double *out_787659766134000101);
void live_H_35(double *state, double *unused, double *out_7115186915285485444);
void live_h_32(double *state, double *unused, double *out_1300325630068484101);
void live_H_32(double *state, double *unused, double *out_1313882543077872886);
void live_h_13(double *state, double *unused, double *out_903929101481248629);
void live_H_13(double *state, double *unused, double *out_116252820802879318);
void live_h_14(double *state, double *unused, double *out_2705653560226498363);
void live_H_14(double *state, double *unused, double *out_8206084747681049441);
void live_h_33(double *state, double *unused, double *out_6043942239175649670);
void live_H_33(double *state, double *unused, double *out_3964629910646627840);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}