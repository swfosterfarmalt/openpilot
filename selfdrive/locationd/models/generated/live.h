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
void live_H(double *in_vec, double *out_3808128492149288823);
void live_err_fun(double *nom_x, double *delta_x, double *out_7187904047825591631);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5909848640433995487);
void live_H_mod_fun(double *state, double *out_6245519984707546482);
void live_f_fun(double *state, double dt, double *out_3243602048370032302);
void live_F_fun(double *state, double dt, double *out_5080430979257473046);
void live_h_4(double *state, double *unused, double *out_5465210553308981186);
void live_H_4(double *state, double *unused, double *out_8665156220454918411);
void live_h_9(double *state, double *unused, double *out_4580217727067371428);
void live_H_9(double *state, double *unused, double *out_4507988484100140928);
void live_h_10(double *state, double *unused, double *out_4894321095925723070);
void live_H_10(double *state, double *unused, double *out_2200864402958350746);
void live_h_12(double *state, double *unused, double *out_6116745976112172927);
void live_H_12(double *state, double *unused, double *out_9160488828207039538);
void live_h_35(double *state, double *unused, double *out_5998315411013033764);
void live_H_35(double *state, double *unused, double *out_6414925795882025829);
void live_h_32(double *state, double *unused, double *out_920711856693618752);
void live_H_32(double *state, double *unused, double *out_3631267631463608236);
void live_h_13(double *state, double *unused, double *out_8421617918659120550);
void live_H_13(double *state, double *unused, double *out_8520443505670961767);
void live_h_14(double *state, double *unused, double *out_4580217727067371428);
void live_H_14(double *state, double *unused, double *out_4507988484100140928);
void live_h_33(double *state, double *unused, double *out_1157661319096080697);
void live_H_33(double *state, double *unused, double *out_3264368791243168225);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}