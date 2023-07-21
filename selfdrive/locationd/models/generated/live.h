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
void live_H(double *in_vec, double *out_8268278872832769695);
void live_err_fun(double *nom_x, double *delta_x, double *out_3779192168373667116);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6912764199932702258);
void live_H_mod_fun(double *state, double *out_1958029865805661768);
void live_f_fun(double *state, double dt, double *out_5787325954291269014);
void live_F_fun(double *state, double dt, double *out_8867243437119667204);
void live_h_4(double *state, double *unused, double *out_1732874906513663232);
void live_H_4(double *state, double *unused, double *out_2781950829758574507);
void live_h_9(double *state, double *unused, double *out_591592173537986614);
void live_H_9(double *state, double *unused, double *out_8377574308686529639);
void live_h_10(double *state, double *unused, double *out_1344679237068414749);
void live_H_10(double *state, double *unused, double *out_8745788590229668561);
void live_h_12(double *state, double *unused, double *out_8829866971693237244);
void live_H_12(double *state, double *unused, double *out_3599307547284158489);
void live_h_35(double *state, double *unused, double *out_4003887171127857483);
void live_H_35(double *state, double *unused, double *out_5252101897943512908);
void live_h_32(double *state, double *unused, double *out_7409445084752508975);
void live_H_32(double *state, double *unused, double *out_1929913649459272813);
void live_h_13(double *state, double *unused, double *out_3319251004507905160);
void live_H_13(double *state, double *unused, double *out_463920091638251512);
void live_h_14(double *state, double *unused, double *out_591592173537986614);
void live_H_14(double *state, double *unused, double *out_8377574308686529639);
void live_h_33(double *state, double *unused, double *out_8235721860435663780);
void live_H_33(double *state, double *unused, double *out_2101544893304655304);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}