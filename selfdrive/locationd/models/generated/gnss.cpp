#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3504239062773879246) {
   out_3504239062773879246[0] = delta_x[0] + nom_x[0];
   out_3504239062773879246[1] = delta_x[1] + nom_x[1];
   out_3504239062773879246[2] = delta_x[2] + nom_x[2];
   out_3504239062773879246[3] = delta_x[3] + nom_x[3];
   out_3504239062773879246[4] = delta_x[4] + nom_x[4];
   out_3504239062773879246[5] = delta_x[5] + nom_x[5];
   out_3504239062773879246[6] = delta_x[6] + nom_x[6];
   out_3504239062773879246[7] = delta_x[7] + nom_x[7];
   out_3504239062773879246[8] = delta_x[8] + nom_x[8];
   out_3504239062773879246[9] = delta_x[9] + nom_x[9];
   out_3504239062773879246[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_533357682304880391) {
   out_533357682304880391[0] = -nom_x[0] + true_x[0];
   out_533357682304880391[1] = -nom_x[1] + true_x[1];
   out_533357682304880391[2] = -nom_x[2] + true_x[2];
   out_533357682304880391[3] = -nom_x[3] + true_x[3];
   out_533357682304880391[4] = -nom_x[4] + true_x[4];
   out_533357682304880391[5] = -nom_x[5] + true_x[5];
   out_533357682304880391[6] = -nom_x[6] + true_x[6];
   out_533357682304880391[7] = -nom_x[7] + true_x[7];
   out_533357682304880391[8] = -nom_x[8] + true_x[8];
   out_533357682304880391[9] = -nom_x[9] + true_x[9];
   out_533357682304880391[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1488114587680907839) {
   out_1488114587680907839[0] = 1.0;
   out_1488114587680907839[1] = 0;
   out_1488114587680907839[2] = 0;
   out_1488114587680907839[3] = 0;
   out_1488114587680907839[4] = 0;
   out_1488114587680907839[5] = 0;
   out_1488114587680907839[6] = 0;
   out_1488114587680907839[7] = 0;
   out_1488114587680907839[8] = 0;
   out_1488114587680907839[9] = 0;
   out_1488114587680907839[10] = 0;
   out_1488114587680907839[11] = 0;
   out_1488114587680907839[12] = 1.0;
   out_1488114587680907839[13] = 0;
   out_1488114587680907839[14] = 0;
   out_1488114587680907839[15] = 0;
   out_1488114587680907839[16] = 0;
   out_1488114587680907839[17] = 0;
   out_1488114587680907839[18] = 0;
   out_1488114587680907839[19] = 0;
   out_1488114587680907839[20] = 0;
   out_1488114587680907839[21] = 0;
   out_1488114587680907839[22] = 0;
   out_1488114587680907839[23] = 0;
   out_1488114587680907839[24] = 1.0;
   out_1488114587680907839[25] = 0;
   out_1488114587680907839[26] = 0;
   out_1488114587680907839[27] = 0;
   out_1488114587680907839[28] = 0;
   out_1488114587680907839[29] = 0;
   out_1488114587680907839[30] = 0;
   out_1488114587680907839[31] = 0;
   out_1488114587680907839[32] = 0;
   out_1488114587680907839[33] = 0;
   out_1488114587680907839[34] = 0;
   out_1488114587680907839[35] = 0;
   out_1488114587680907839[36] = 1.0;
   out_1488114587680907839[37] = 0;
   out_1488114587680907839[38] = 0;
   out_1488114587680907839[39] = 0;
   out_1488114587680907839[40] = 0;
   out_1488114587680907839[41] = 0;
   out_1488114587680907839[42] = 0;
   out_1488114587680907839[43] = 0;
   out_1488114587680907839[44] = 0;
   out_1488114587680907839[45] = 0;
   out_1488114587680907839[46] = 0;
   out_1488114587680907839[47] = 0;
   out_1488114587680907839[48] = 1.0;
   out_1488114587680907839[49] = 0;
   out_1488114587680907839[50] = 0;
   out_1488114587680907839[51] = 0;
   out_1488114587680907839[52] = 0;
   out_1488114587680907839[53] = 0;
   out_1488114587680907839[54] = 0;
   out_1488114587680907839[55] = 0;
   out_1488114587680907839[56] = 0;
   out_1488114587680907839[57] = 0;
   out_1488114587680907839[58] = 0;
   out_1488114587680907839[59] = 0;
   out_1488114587680907839[60] = 1.0;
   out_1488114587680907839[61] = 0;
   out_1488114587680907839[62] = 0;
   out_1488114587680907839[63] = 0;
   out_1488114587680907839[64] = 0;
   out_1488114587680907839[65] = 0;
   out_1488114587680907839[66] = 0;
   out_1488114587680907839[67] = 0;
   out_1488114587680907839[68] = 0;
   out_1488114587680907839[69] = 0;
   out_1488114587680907839[70] = 0;
   out_1488114587680907839[71] = 0;
   out_1488114587680907839[72] = 1.0;
   out_1488114587680907839[73] = 0;
   out_1488114587680907839[74] = 0;
   out_1488114587680907839[75] = 0;
   out_1488114587680907839[76] = 0;
   out_1488114587680907839[77] = 0;
   out_1488114587680907839[78] = 0;
   out_1488114587680907839[79] = 0;
   out_1488114587680907839[80] = 0;
   out_1488114587680907839[81] = 0;
   out_1488114587680907839[82] = 0;
   out_1488114587680907839[83] = 0;
   out_1488114587680907839[84] = 1.0;
   out_1488114587680907839[85] = 0;
   out_1488114587680907839[86] = 0;
   out_1488114587680907839[87] = 0;
   out_1488114587680907839[88] = 0;
   out_1488114587680907839[89] = 0;
   out_1488114587680907839[90] = 0;
   out_1488114587680907839[91] = 0;
   out_1488114587680907839[92] = 0;
   out_1488114587680907839[93] = 0;
   out_1488114587680907839[94] = 0;
   out_1488114587680907839[95] = 0;
   out_1488114587680907839[96] = 1.0;
   out_1488114587680907839[97] = 0;
   out_1488114587680907839[98] = 0;
   out_1488114587680907839[99] = 0;
   out_1488114587680907839[100] = 0;
   out_1488114587680907839[101] = 0;
   out_1488114587680907839[102] = 0;
   out_1488114587680907839[103] = 0;
   out_1488114587680907839[104] = 0;
   out_1488114587680907839[105] = 0;
   out_1488114587680907839[106] = 0;
   out_1488114587680907839[107] = 0;
   out_1488114587680907839[108] = 1.0;
   out_1488114587680907839[109] = 0;
   out_1488114587680907839[110] = 0;
   out_1488114587680907839[111] = 0;
   out_1488114587680907839[112] = 0;
   out_1488114587680907839[113] = 0;
   out_1488114587680907839[114] = 0;
   out_1488114587680907839[115] = 0;
   out_1488114587680907839[116] = 0;
   out_1488114587680907839[117] = 0;
   out_1488114587680907839[118] = 0;
   out_1488114587680907839[119] = 0;
   out_1488114587680907839[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6096373899760264690) {
   out_6096373899760264690[0] = dt*state[3] + state[0];
   out_6096373899760264690[1] = dt*state[4] + state[1];
   out_6096373899760264690[2] = dt*state[5] + state[2];
   out_6096373899760264690[3] = state[3];
   out_6096373899760264690[4] = state[4];
   out_6096373899760264690[5] = state[5];
   out_6096373899760264690[6] = dt*state[7] + state[6];
   out_6096373899760264690[7] = dt*state[8] + state[7];
   out_6096373899760264690[8] = state[8];
   out_6096373899760264690[9] = state[9];
   out_6096373899760264690[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6952422209739945707) {
   out_6952422209739945707[0] = 1;
   out_6952422209739945707[1] = 0;
   out_6952422209739945707[2] = 0;
   out_6952422209739945707[3] = dt;
   out_6952422209739945707[4] = 0;
   out_6952422209739945707[5] = 0;
   out_6952422209739945707[6] = 0;
   out_6952422209739945707[7] = 0;
   out_6952422209739945707[8] = 0;
   out_6952422209739945707[9] = 0;
   out_6952422209739945707[10] = 0;
   out_6952422209739945707[11] = 0;
   out_6952422209739945707[12] = 1;
   out_6952422209739945707[13] = 0;
   out_6952422209739945707[14] = 0;
   out_6952422209739945707[15] = dt;
   out_6952422209739945707[16] = 0;
   out_6952422209739945707[17] = 0;
   out_6952422209739945707[18] = 0;
   out_6952422209739945707[19] = 0;
   out_6952422209739945707[20] = 0;
   out_6952422209739945707[21] = 0;
   out_6952422209739945707[22] = 0;
   out_6952422209739945707[23] = 0;
   out_6952422209739945707[24] = 1;
   out_6952422209739945707[25] = 0;
   out_6952422209739945707[26] = 0;
   out_6952422209739945707[27] = dt;
   out_6952422209739945707[28] = 0;
   out_6952422209739945707[29] = 0;
   out_6952422209739945707[30] = 0;
   out_6952422209739945707[31] = 0;
   out_6952422209739945707[32] = 0;
   out_6952422209739945707[33] = 0;
   out_6952422209739945707[34] = 0;
   out_6952422209739945707[35] = 0;
   out_6952422209739945707[36] = 1;
   out_6952422209739945707[37] = 0;
   out_6952422209739945707[38] = 0;
   out_6952422209739945707[39] = 0;
   out_6952422209739945707[40] = 0;
   out_6952422209739945707[41] = 0;
   out_6952422209739945707[42] = 0;
   out_6952422209739945707[43] = 0;
   out_6952422209739945707[44] = 0;
   out_6952422209739945707[45] = 0;
   out_6952422209739945707[46] = 0;
   out_6952422209739945707[47] = 0;
   out_6952422209739945707[48] = 1;
   out_6952422209739945707[49] = 0;
   out_6952422209739945707[50] = 0;
   out_6952422209739945707[51] = 0;
   out_6952422209739945707[52] = 0;
   out_6952422209739945707[53] = 0;
   out_6952422209739945707[54] = 0;
   out_6952422209739945707[55] = 0;
   out_6952422209739945707[56] = 0;
   out_6952422209739945707[57] = 0;
   out_6952422209739945707[58] = 0;
   out_6952422209739945707[59] = 0;
   out_6952422209739945707[60] = 1;
   out_6952422209739945707[61] = 0;
   out_6952422209739945707[62] = 0;
   out_6952422209739945707[63] = 0;
   out_6952422209739945707[64] = 0;
   out_6952422209739945707[65] = 0;
   out_6952422209739945707[66] = 0;
   out_6952422209739945707[67] = 0;
   out_6952422209739945707[68] = 0;
   out_6952422209739945707[69] = 0;
   out_6952422209739945707[70] = 0;
   out_6952422209739945707[71] = 0;
   out_6952422209739945707[72] = 1;
   out_6952422209739945707[73] = dt;
   out_6952422209739945707[74] = 0;
   out_6952422209739945707[75] = 0;
   out_6952422209739945707[76] = 0;
   out_6952422209739945707[77] = 0;
   out_6952422209739945707[78] = 0;
   out_6952422209739945707[79] = 0;
   out_6952422209739945707[80] = 0;
   out_6952422209739945707[81] = 0;
   out_6952422209739945707[82] = 0;
   out_6952422209739945707[83] = 0;
   out_6952422209739945707[84] = 1;
   out_6952422209739945707[85] = dt;
   out_6952422209739945707[86] = 0;
   out_6952422209739945707[87] = 0;
   out_6952422209739945707[88] = 0;
   out_6952422209739945707[89] = 0;
   out_6952422209739945707[90] = 0;
   out_6952422209739945707[91] = 0;
   out_6952422209739945707[92] = 0;
   out_6952422209739945707[93] = 0;
   out_6952422209739945707[94] = 0;
   out_6952422209739945707[95] = 0;
   out_6952422209739945707[96] = 1;
   out_6952422209739945707[97] = 0;
   out_6952422209739945707[98] = 0;
   out_6952422209739945707[99] = 0;
   out_6952422209739945707[100] = 0;
   out_6952422209739945707[101] = 0;
   out_6952422209739945707[102] = 0;
   out_6952422209739945707[103] = 0;
   out_6952422209739945707[104] = 0;
   out_6952422209739945707[105] = 0;
   out_6952422209739945707[106] = 0;
   out_6952422209739945707[107] = 0;
   out_6952422209739945707[108] = 1;
   out_6952422209739945707[109] = 0;
   out_6952422209739945707[110] = 0;
   out_6952422209739945707[111] = 0;
   out_6952422209739945707[112] = 0;
   out_6952422209739945707[113] = 0;
   out_6952422209739945707[114] = 0;
   out_6952422209739945707[115] = 0;
   out_6952422209739945707[116] = 0;
   out_6952422209739945707[117] = 0;
   out_6952422209739945707[118] = 0;
   out_6952422209739945707[119] = 0;
   out_6952422209739945707[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8474727300148108745) {
   out_8474727300148108745[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_5179266658141670640) {
   out_5179266658141670640[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5179266658141670640[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5179266658141670640[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5179266658141670640[3] = 0;
   out_5179266658141670640[4] = 0;
   out_5179266658141670640[5] = 0;
   out_5179266658141670640[6] = 1;
   out_5179266658141670640[7] = 0;
   out_5179266658141670640[8] = 0;
   out_5179266658141670640[9] = 0;
   out_5179266658141670640[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6087112215128222949) {
   out_6087112215128222949[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5090632472623700831) {
   out_5090632472623700831[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5090632472623700831[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5090632472623700831[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5090632472623700831[3] = 0;
   out_5090632472623700831[4] = 0;
   out_5090632472623700831[5] = 0;
   out_5090632472623700831[6] = 1;
   out_5090632472623700831[7] = 0;
   out_5090632472623700831[8] = 0;
   out_5090632472623700831[9] = 1;
   out_5090632472623700831[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5550647620280080127) {
   out_5550647620280080127[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4904720883080239818) {
   out_4904720883080239818[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[6] = 0;
   out_4904720883080239818[7] = 1;
   out_4904720883080239818[8] = 0;
   out_4904720883080239818[9] = 0;
   out_4904720883080239818[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5550647620280080127) {
   out_5550647620280080127[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4904720883080239818) {
   out_4904720883080239818[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4904720883080239818[6] = 0;
   out_4904720883080239818[7] = 1;
   out_4904720883080239818[8] = 0;
   out_4904720883080239818[9] = 0;
   out_4904720883080239818[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3504239062773879246) {
  err_fun(nom_x, delta_x, out_3504239062773879246);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_533357682304880391) {
  inv_err_fun(nom_x, true_x, out_533357682304880391);
}
void gnss_H_mod_fun(double *state, double *out_1488114587680907839) {
  H_mod_fun(state, out_1488114587680907839);
}
void gnss_f_fun(double *state, double dt, double *out_6096373899760264690) {
  f_fun(state,  dt, out_6096373899760264690);
}
void gnss_F_fun(double *state, double dt, double *out_6952422209739945707) {
  F_fun(state,  dt, out_6952422209739945707);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8474727300148108745) {
  h_6(state, sat_pos, out_8474727300148108745);
}
void gnss_H_6(double *state, double *sat_pos, double *out_5179266658141670640) {
  H_6(state, sat_pos, out_5179266658141670640);
}
void gnss_h_20(double *state, double *sat_pos, double *out_6087112215128222949) {
  h_20(state, sat_pos, out_6087112215128222949);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5090632472623700831) {
  H_20(state, sat_pos, out_5090632472623700831);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5550647620280080127) {
  h_7(state, sat_pos_vel, out_5550647620280080127);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4904720883080239818) {
  H_7(state, sat_pos_vel, out_4904720883080239818);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5550647620280080127) {
  h_21(state, sat_pos_vel, out_5550647620280080127);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4904720883080239818) {
  H_21(state, sat_pos_vel, out_4904720883080239818);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
