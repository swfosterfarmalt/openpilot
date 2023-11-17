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
void live_H(double *in_vec, double *out_2381728513021776623);
void live_err_fun(double *nom_x, double *delta_x, double *out_5908421990898430924);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5372329235597557523);
void live_H_mod_fun(double *state, double *out_1274189134229994757);
void live_f_fun(double *state, double dt, double *out_5206476803248674863);
void live_F_fun(double *state, double dt, double *out_7634219856181846541);
void live_h_4(double *state, double *unused, double *out_6175535945132992619);
void live_H_4(double *state, double *unused, double *out_5466614738994487577);
void live_h_9(double *state, double *unused, double *out_7708084397290276278);
void live_H_9(double *state, double *unused, double *out_5707804385624078222);
void live_h_10(double *state, double *unused, double *out_5970349915769922415);
void live_H_10(double *state, double *unused, double *out_1038736123818861462);
void live_h_12(double *state, double *unused, double *out_8325619333716319982);
void live_H_12(double *state, double *unused, double *out_7960672926683102244);
void live_h_35(double *state, double *unused, double *out_6228897039356605775);
void live_H_35(double *state, double *unused, double *out_8833276796367094953);
void live_h_32(double *state, double *unused, double *out_535028966847562763);
void live_H_32(double *state, double *unused, double *out_1918412292672978597);
void live_h_13(double *state, double *unused, double *out_308609886787442161);
void live_H_13(double *state, double *unused, double *out_889109028162732878);
void live_h_14(double *state, double *unused, double *out_7708084397290276278);
void live_H_14(double *state, double *unused, double *out_5707804385624078222);
void live_h_33(double *state, double *unused, double *out_7948037784891248791);
void live_H_33(double *state, double *unused, double *out_6462910272703599059);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}