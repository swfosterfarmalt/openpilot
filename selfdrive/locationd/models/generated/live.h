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
void live_H(double *in_vec, double *out_656014244611583561);
void live_err_fun(double *nom_x, double *delta_x, double *out_3499855352556526276);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6360762681463094448);
void live_H_mod_fun(double *state, double *out_845361931018872706);
void live_f_fun(double *state, double dt, double *out_6508799808593326694);
void live_F_fun(double *state, double dt, double *out_2481137801995991042);
void live_h_4(double *state, double *unused, double *out_9104477390874771765);
void live_H_4(double *state, double *unused, double *out_8223427181738939725);
void live_h_9(double *state, double *unused, double *out_6205118936888486508);
void live_H_9(double *state, double *unused, double *out_5334565629458860383);
void live_h_10(double *state, double *unused, double *out_2335932696573388509);
void live_H_10(double *state, double *unused, double *out_7506038249863649267);
void live_h_12(double *state, double *unused, double *out_1661391825458651885);
void live_H_12(double *state, double *unused, double *out_556298868056489233);
void live_h_35(double *state, double *unused, double *out_919867919034478329);
void live_H_35(double *state, double *unused, double *out_2189264164268524476);
void live_h_32(double *state, double *unused, double *out_1665444862883567993);
void live_H_32(double *state, double *unused, double *out_2151740611464582412);
void live_h_13(double *state, double *unused, double *out_49245112528698080);
void live_H_13(double *state, double *unused, double *out_549865965523821400);
void live_h_14(double *state, double *unused, double *out_6205118936888486508);
void live_H_14(double *state, double *unused, double *out_5334565629458860383);
void live_h_33(double *state, double *unused, double *out_879381520641385049);
void live_H_33(double *state, double *unused, double *out_5339821168907382080);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}