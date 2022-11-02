#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3504239062773879246);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_533357682304880391);
void gnss_H_mod_fun(double *state, double *out_1488114587680907839);
void gnss_f_fun(double *state, double dt, double *out_6096373899760264690);
void gnss_F_fun(double *state, double dt, double *out_6952422209739945707);
void gnss_h_6(double *state, double *sat_pos, double *out_8474727300148108745);
void gnss_H_6(double *state, double *sat_pos, double *out_5179266658141670640);
void gnss_h_20(double *state, double *sat_pos, double *out_6087112215128222949);
void gnss_H_20(double *state, double *sat_pos, double *out_5090632472623700831);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5550647620280080127);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4904720883080239818);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5550647620280080127);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4904720883080239818);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}