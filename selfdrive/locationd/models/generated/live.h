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
void live_H(double *in_vec, double *out_2337898968706533344);
void live_err_fun(double *nom_x, double *delta_x, double *out_6878075311318465643);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1794092814993268924);
void live_H_mod_fun(double *state, double *out_6748437875193912696);
void live_f_fun(double *state, double dt, double *out_1742564737881391388);
void live_F_fun(double *state, double dt, double *out_5510140683284074867);
void live_h_4(double *state, double *unused, double *out_3896021957130681052);
void live_H_4(double *state, double *unused, double *out_2796592115458417611);
void live_h_9(double *state, double *unused, double *out_3279097196838248586);
void live_H_9(double *state, double *unused, double *out_8362933022986686535);
void live_h_10(double *state, double *unused, double *out_5743771204428898197);
void live_H_10(double *state, double *unused, double *out_2519184783685511941);
void live_h_12(double *state, double *unused, double *out_9145814329225030014);
void live_H_12(double *state, double *unused, double *out_3584666261584315385);
void live_h_35(double *state, double *unused, double *out_1854802132409489495);
void live_H_35(double *state, double *unused, double *out_839103229259301676);
void live_h_32(double *state, double *unused, double *out_1289607573080015306);
void live_H_32(double *state, double *unused, double *out_1944554935159115917);
void live_h_13(double *state, double *unused, double *out_5515199641584361897);
void live_H_13(double *state, double *unused, double *out_4096940203060364411);
void live_h_14(double *state, double *unused, double *out_3279097196838248586);
void live_H_14(double *state, double *unused, double *out_8362933022986686535);
void live_h_33(double *state, double *unused, double *out_6887162999505427965);
void live_H_33(double *state, double *unused, double *out_2311453775379555928);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}