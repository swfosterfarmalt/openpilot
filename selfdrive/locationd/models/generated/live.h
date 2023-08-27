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
void live_H(double *in_vec, double *out_8295645905409934087);
void live_err_fun(double *nom_x, double *delta_x, double *out_8619578534154616486);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3513142964193659494);
void live_H_mod_fun(double *state, double *out_4486130015261313537);
void live_f_fun(double *state, double dt, double *out_1181359135653298229);
void live_F_fun(double *state, double dt, double *out_5541218906871329770);
void live_h_4(double *state, double *unused, double *out_5193603763907660647);
void live_H_4(double *state, double *unused, double *out_4937118780596984391);
void live_h_9(double *state, double *unused, double *out_3105314858659580175);
void live_H_9(double *state, double *unused, double *out_5178308427226575036);
void live_h_10(double *state, double *unused, double *out_2225569820954965318);
void live_H_10(double *state, double *unused, double *out_7551586305984993157);
void live_h_12(double *state, double *unused, double *out_9147171086662171935);
void live_H_12(double *state, double *unused, double *out_8490168885080605430);
void live_h_35(double *state, double *unused, double *out_8823103167589140492);
void live_H_35(double *state, double *unused, double *out_5744605852755591721);
void live_h_32(double *state, double *unused, double *out_4181631937077063);
void live_H_32(double *state, double *unused, double *out_4085081600297682697);
void live_h_13(double *state, double *unused, double *out_7154068656228527500);
void live_H_13(double *state, double *unused, double *out_8864001332903443877);
void live_h_14(double *state, double *unused, double *out_3105314858659580175);
void live_H_14(double *state, double *unused, double *out_5178308427226575036);
void live_h_33(double *state, double *unused, double *out_3247796329789380701);
void live_H_33(double *state, double *unused, double *out_6992406231101102245);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}