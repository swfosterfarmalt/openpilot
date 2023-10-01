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
void live_H(double *in_vec, double *out_4131254223325676384);
void live_err_fun(double *nom_x, double *delta_x, double *out_2742659515808312139);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5409060373077877762);
void live_H_mod_fun(double *state, double *out_4545639670895671074);
void live_f_fun(double *state, double dt, double *out_8720865560414031758);
void live_F_fun(double *state, double dt, double *out_2018563564658859806);
void live_h_4(double *state, double *unused, double *out_1216254421693794346);
void live_H_4(double *state, double *unused, double *out_2068224320832719797);
void live_h_9(double *state, double *unused, double *out_7559075047251418560);
void live_H_9(double *state, double *unused, double *out_1827034674203129152);
void live_h_10(double *state, double *unused, double *out_5783482280623166038);
void live_H_10(double *state, double *unused, double *out_8841027575448190877);
void live_h_12(double *state, double *unused, double *out_6193413751734227853);
void live_H_12(double *state, double *unused, double *out_1447125295785126130);
void live_h_35(double *state, double *unused, double *out_3186779468869586602);
void live_H_35(double *state, double *unused, double *out_1298437736539887579);
void live_h_32(double *state, double *unused, double *out_8772917863253775244);
void live_H_32(double *state, double *unused, double *out_1588043686308566917);
void live_h_13(double *state, double *unused, double *out_1001338837700093085);
void live_H_13(double *state, double *unused, double *out_5476705467195109763);
void live_h_14(double *state, double *unused, double *out_7559075047251418560);
void live_H_14(double *state, double *unused, double *out_1827034674203129152);
void live_h_33(double *state, double *unused, double *out_8087452208506297307);
void live_H_33(double *state, double *unused, double *out_4448994741178745183);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}