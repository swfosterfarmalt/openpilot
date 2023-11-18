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
void live_H(double *in_vec, double *out_903410881553498820);
void live_err_fun(double *nom_x, double *delta_x, double *out_6347198416308078133);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6606451253515979607);
void live_H_mod_fun(double *state, double *out_5508914255187383521);
void live_f_fun(double *state, double dt, double *out_2698707008917840806);
void live_F_fun(double *state, double dt, double *out_612426947716396943);
void live_h_4(double *state, double *unused, double *out_8928792572923818065);
void live_H_4(double *state, double *unused, double *out_5597980534556278949);
void live_h_9(double *state, double *unused, double *out_1468415542480490961);
void live_H_9(double *state, double *unused, double *out_5356790887926688304);
void live_h_10(double *state, double *unused, double *out_7065133993630064388);
void live_H_10(double *state, double *unused, double *out_1213535654927450255);
void live_h_12(double *state, double *unused, double *out_92474216324681544);
void live_H_12(double *state, double *unused, double *out_578524126524317154);
void live_h_35(double *state, double *unused, double *out_8543445846310041789);
void live_H_35(double *state, double *unused, double *out_2167038905800696555);
void live_h_32(double *state, double *unused, double *out_3874621113343202864);
void live_H_32(double *state, double *unused, double *out_1854417210913251527);
void live_h_13(double *state, double *unused, double *out_6665212794720325510);
void live_H_13(double *state, double *unused, double *out_4793867377691150803);
void live_h_14(double *state, double *unused, double *out_1468415542480490961);
void live_H_14(double *state, double *unused, double *out_5356790887926688304);
void live_h_33(double *state, double *unused, double *out_3519495767417048963);
void live_H_33(double *state, double *unused, double *out_5317595910439554159);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}