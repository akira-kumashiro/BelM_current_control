#include "stdafx.h"

#define AD_GROUP_0 0//AD変換グループ0 VuVw->BインバータIuIw
#define AD_GROUP_1 1//AD変換グループ1 IuIw->AインバータIuIw
#define AD_GROUP_2 2//AD変換グループ2 VdcIdc->AインバータVdcIdvc
#define AD_GROUP_3 3//AD変換グループ3 AUX0AUX1->BインバータVdcIdc
#define AD_GROUP_4 4//AD変換グループ4 CH9CH8->空き
#define AD_GROUP_5 5//AD変換グループ5 CH11CH10->距離センサ1距離センサ2
#define AD_GROUP_6 6//AD変換グループ6 CH13CH12->ホール素子1ホール素子2
#define AD_GROUP_7 7//AD変換グループ7 CH15CH14->ホール素子3ホール素子4

#define DA_CH_0 0//DAチャンネルグループ0
#define DA_CH_1 1//DAチャンネルグループ1
#define DA_CH_2 2//DAチャンネルグループ2
#define DA_CH_3 3//DAチャンネルグループ3
#define DA_CH_4 4//DAチャンネルグループ4

#define SEQ1 1//シーケンサ1
#define SEQ2 2//シーケンサ2
#define CASCADE_SEQ 3//カスケードシーケンサ

#define SINGLE_SCAN 0//ADスキャン方法
#define CONTINUOUS_SCAN 1//ADスキャン方法
#define ONE_CYCLE_SCAN 3//ADスキャン方法

#define	PI(n)	(3.14159265358979 * (n))
#define ROOT2 1.414213562373095
#define ROOT3 1.732050807568877
#define mylimit(data,limit) ((data) > (limit)) ? (limit) : ((data) < -(limit) ? -(limit) : (data))//dataを±limitに制限
#define FtoOmega(theta) (6.28318530717958 * (theta))//ω=2πf
#define TtoOmega(T) (6.28318530717958 / (T))//ω=2π/T
#define v_raw_converterA(v_in) (v_in)*(voltage_ad_gainA)//AインバータのAD変換した値を実際の電圧値に変換
#define i_raw_converterA(c_in) (c_in)*(current_ad_gainA)//AインバータのAD変換した値を実際の電流値に変換
#define v_raw_converterB(v_in) (v_in)*(voltage_ad_gainB)//AインバータのAD変換した値を実際の電圧値に変換
#define i_raw_converterB(c_in) (c_in)*(current_ad_gainB)//AインバータのAD変換した値を実際の電流値に変換
#define INV_1R_CURRENT_LIMIT 4.89
#define INV_5R_CURRENT_LIMIT 24.4
//#define INV_5R_CURRENT_LIMIT 35//電磁接触器を使って電流保護する用

FLOAT32 radianLimit(FLOAT32 rad);
FLOAT32 myExp(FLOAT32 x);
//FLOAT32 myExpCalc(FLOAT32 x, INT32 n, FLOAT32 _numerator, FLOAT32 _denominator,FLOAT32 y);
void adInput(INT32 seq_No, INT32 ad_group, FLOAT32 *signalA, FLOAT32 *signalB);
FLOAT32 getHallRawValue(FLOAT32 _hallA, FLOAT32 _hallB, FLOAT32 _hallC, FLOAT32 _hallD);
void uvw2dq(FLOAT32 _u, FLOAT32 _w, FLOAT32 _theta, FLOAT32 *_d, FLOAT32 *_q);
void dq2uvw(FLOAT32 _d, FLOAT32 _q, FLOAT32 _theta, FLOAT32 *_u, FLOAT32 *_v, FLOAT32 *_w);
void pi_control(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _max, volatile FLOAT32 *_p_out, volatile FLOAT32 *_i_out, volatile FLOAT32 *_itemp);
FLOAT32 pi_control_release(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _max, volatile FLOAT32 *_itemp);
//void pid_control(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _d_gain, FLOAT32 _max, volatile FLOAT32 *_p_out, volatile FLOAT32 *_i_out, volatile FLOAT32 *_itemp, volatile FLOAT32 *_d_out, volatile FLOAT32 *_err_prev);
void pwm_output(FLOAT32 _d_ref, FLOAT32 _q_ref, FLOAT32 _half_Vdc, FLOAT32 _theta, INT32 _inverter_no);
