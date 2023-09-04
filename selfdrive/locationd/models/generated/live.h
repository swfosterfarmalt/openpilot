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
void live_H(double *in_vec, double *out_6264240166570111056);
void live_err_fun(double *nom_x, double *delta_x, double *out_929829361008702232);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8810076277123414669);
void live_H_mod_fun(double *state, double *out_5102563603905716445);
void live_f_fun(double *state, double dt, double *out_7329154140838552077);
void live_F_fun(double *state, double dt, double *out_4718842532029652348);
void live_h_4(double *state, double *unused, double *out_7415800318484510059);
void live_H_4(double *state, double *unused, double *out_4555738591742659618);
void live_h_9(double *state, double *unused, double *out_5501259363844600019);
void live_H_9(double *state, double *unused, double *out_4314548945113068973);
void live_h_10(double *state, double *unused, double *out_7659614083091357968);
void live_H_10(double *state, double *unused, double *out_4099442533871365456);
void live_h_12(double *state, double *unused, double *out_24599607377042501);
void live_H_12(double *state, double *unused, double *out_463717816289302177);
void live_h_35(double *state, double *unused, double *out_7610217345300344909);
void live_H_35(double *state, double *unused, double *out_1189076534370052242);
void live_h_32(double *state, double *unused, double *out_7061699293644735915);
void live_H_32(double *state, double *unused, double *out_8103941038064168598);
void live_h_13(double *state, double *unused, double *out_9072829778971797242);
void live_H_13(double *state, double *unused, double *out_4163188090060936784);
void live_h_14(double *state, double *unused, double *out_5501259363844600019);
void live_H_14(double *state, double *unused, double *out_4314548945113068973);
void live_h_33(double *state, double *unused, double *out_4368452122107625980);
void live_H_33(double *state, double *unused, double *out_1961480470268805362);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}