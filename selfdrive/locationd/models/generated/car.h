#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8827834186464641590);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4357544145669623999);
void car_H_mod_fun(double *state, double *out_5054864097630402037);
void car_f_fun(double *state, double dt, double *out_3794276025868419788);
void car_F_fun(double *state, double dt, double *out_3600320196893165164);
void car_h_25(double *state, double *unused, double *out_133870440206676857);
void car_H_25(double *state, double *unused, double *out_1939961542814685148);
void car_h_24(double *state, double *unused, double *out_2241976900638027477);
void car_H_24(double *state, double *unused, double *out_232688056190814418);
void car_h_30(double *state, double *unused, double *out_8630531121470416849);
void car_H_30(double *state, double *unused, double *out_4458294501321933775);
void car_h_26(double *state, double *unused, double *out_5214257831821346833);
void car_H_26(double *state, double *unused, double *out_1801541776059371076);
void car_h_27(double *state, double *unused, double *out_5011019622908970680);
void car_H_27(double *state, double *unused, double *out_6681888572505876992);
void car_h_29(double *state, double *unused, double *out_3417171434825285621);
void car_H_29(double *state, double *unused, double *out_4968525845636325959);
void car_h_28(double *state, double *unused, double *out_4114166344552671977);
void car_H_28(double *state, double *unused, double *out_113873171433204615);
void car_h_31(double *state, double *unused, double *out_6578481571228170128);
void car_H_31(double *state, double *unused, double *out_2427749878292722552);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}