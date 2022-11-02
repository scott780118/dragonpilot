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
void live_H(double *in_vec, double *out_8919387032363340024);
void live_err_fun(double *nom_x, double *delta_x, double *out_6241232987128523181);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8668464867797320256);
void live_H_mod_fun(double *state, double *out_3223984679836796094);
void live_f_fun(double *state, double dt, double *out_8792432846504919457);
void live_F_fun(double *state, double dt, double *out_1660227815357983615);
void live_h_4(double *state, double *unused, double *out_6280336770264779317);
void live_H_4(double *state, double *unused, double *out_7610399415638725269);
void live_h_9(double *state, double *unused, double *out_4987524776592364348);
void live_H_9(double *state, double *unused, double *out_323180480374277799);
void live_h_10(double *state, double *unused, double *out_5067339660044823415);
void live_H_10(double *state, double *unused, double *out_4060124344518808344);
void live_h_12(double *state, double *unused, double *out_4166309286904166984);
void live_H_12(double *state, double *unused, double *out_2590943007606763474);
void live_h_35(double *state, double *unused, double *out_2478603440640092291);
void live_H_35(double *state, double *unused, double *out_4243737358266117893);
void live_h_32(double *state, double *unused, double *out_1090084174258433073);
void live_H_32(double *state, double *unused, double *out_4112572573325377424);
void live_h_13(double *state, double *unused, double *out_4895825668815333413);
void live_H_13(double *state, double *unused, double *out_2504921632991610304);
void live_h_14(double *state, double *unused, double *out_4987524776592364348);
void live_H_14(double *state, double *unused, double *out_323180480374277799);
void live_h_33(double *state, double *unused, double *out_1289111515151973379);
void live_H_33(double *state, double *unused, double *out_1093180353627260289);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}