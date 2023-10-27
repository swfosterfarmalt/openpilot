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
void live_H(double *in_vec, double *out_9191720918565895292);
void live_err_fun(double *nom_x, double *delta_x, double *out_4097545666249594925);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5294404423203983721);
void live_H_mod_fun(double *state, double *out_3638656807510898365);
void live_f_fun(double *state, double dt, double *out_4064418078486338310);
void live_F_fun(double *state, double dt, double *out_4074196571012395889);
void live_h_4(double *state, double *unused, double *out_8366072108827298312);
void live_H_4(double *state, double *unused, double *out_499387579376300602);
void live_h_9(double *state, double *unused, double *out_2956207272003695430);
void live_H_9(double *state, double *unused, double *out_258197932746709957);
void live_h_10(double *state, double *unused, double *out_9173627037127896632);
void live_H_10(double *state, double *unused, double *out_6757501493105471108);
void live_h_12(double *state, double *unused, double *out_9016299843067629392);
void live_H_12(double *state, double *unused, double *out_4520068828655661193);
void live_h_35(double *state, double *unused, double *out_8897869277968490229);
void live_H_35(double *state, double *unused, double *out_2867274477996306774);
void live_h_32(double *state, double *unused, double *out_2172986558537042211);
void live_H_32(double *state, double *unused, double *out_4047590025697809582);
void live_h_13(double *state, double *unused, double *out_6823959863893977632);
void live_H_13(double *state, double *unused, double *out_5256338612527669940);
void live_h_14(double *state, double *unused, double *out_2956207272003695430);
void live_H_14(double *state, double *unused, double *out_258197932746709957);
void live_h_33(double *state, double *unused, double *out_4690348640723019468);
void live_H_33(double *state, double *unused, double *out_6017831482635164378);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}