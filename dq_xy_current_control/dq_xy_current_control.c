/****************************************************************************************************************************
 Title			:	Sample Program for PE-PRO/F28335A
 Sub title		:	Demonstration program for triangle comparison based 3-phase PWM generation
					as the gate signal from port0
					Vdcを検知(ADCINA2)して指定した電圧を出力
 Copyright		:	2010 Myway Corporation

 Explanation	:	This program generates triangle comparison based 3-phase PWM signal
 					by using the PWM generation functions for the gate signal output from port0.

 Used board		:	PE-PRO/F28335A
*****************************************************************************************************************************/
#define		DAC_ENABLE
#include "stdafx.h"
#include "MyFunction.h"
#include "MyFunction.c"

#define INVERTER_MODEL_A 5//Aインバータのモデル1R->1、5R->5、9R->9
#define INVERTER_MODEL_B 1//Bインバータのモデル1R->1、5R->5、9R->9

#define dispSensorCalcA(voltage) (voltage + 4.5) * 0.2 + 3.293e-5 * (myExp(1.8234 * (voltage + 4.5)) - 1)//B10793
#define dispSensorCalcB(voltage) (voltage + 4.65) * 0.2 + 1.7e-5 * (myExp(1.959 * (voltage + 4.65)) - 1)//B10794new 20rpm.xlsx
//#define dispSensorCalcB(voltage) (voltage + 4.5) * 0.2 + 1.211e-5 * (myExp(1.959 * (voltage + 4.5)) - 1)//B10794old

#define AD_RANGE 5.0 //AD変換の数値レンジ

#define	PERIOD	20000							// 1/(50Hz) = 20ms = 20000us								
#define	SAMPL	200								// Number of sampling points								
#define PERIOD2 100							//ホール素子割り込み周期[us]
#define RESOLUSION 4000							//エンコーダ分解能*4
#define Vdc_ref 1.0                                // DCリンク電圧[v]
#define Vdc_thres 20.0//電流制御するVdcの閾値#define PERIOD_OF_PI_START_DERAY 1.0e-4//PI制御を始めるまでの待ち時間0.15[sec]
#define NUM_OF_OFFSET_DETECT 100//電流センサオフセット検出のステップ回数
#define PERIOD_OF_PI_START_DERAY 1.0e-4//PI制御を始めるまでの待ち時間0.15[sec]
#define PERIOD_OF_XY_CURRENT_TEST 10.0//xy電流制御の電流出力時間[sec]
#define HALL_COMP_ORDER_PREC 15//ホール素子校正の高調波次数
#define HALL_COMP_ORDER 9//ホール素子校正の高調波次数
#define HALL_CUTOFF_FREQ 1000//ホール素子計算値のLPFカットオフ周波数

#define PolePairs 1.0//極対数
#define PSY_a 0.02976//PSYa[Wb] 測定値更新済み 20770708result.xlsx
#define Ra_2p 0.443//R:相抵抗[Ω] 2p 測定値更新済み 20220927各相抵抗再測定.xlsx
#define Ra_4p 0.622//R:相抵抗[Ω] 4p 測定値更新済み 20220927各相抵抗再測定.xlsx
#define Ld_2p 0.000980// Ld:d軸2pインダクタンス[H] 20220930LdLq測定・解析.xlsx
#define Lq_2p 0.000980// Lq:q軸2pインダクタンス[H] 20220930LdLq測定・解析.xlsx
#define Ld_4p 0.001059// Ld:d軸4pインダクタンス[H] 20220930LdLq測定・解析.xlsx
#define Lq_4p 0.001059// Lq:q軸4pインダクタンス[H] 20220930LdLq測定・解析.xlsx
#define FmodElec 13.0//変調波の周波数(電気)Fe[Hz]
#define Fcutoff 1000.0//電流制御の閉ループカットオフ周波数Fc[Hz]
#define Fcarrier 20000.0//キャリア周波数Fs[Hz]
#define	DEADTIME	4000							// Dead time [ns]	0<=deadtime<=24000	これ以上小さくしない！
#define Kp_gain2p 1.0//2pPゲイン調整用
#define Ki_gain2p 4.0//2pIゲイン調整用
#define Kp_gain4p 1.0//4pPゲイン調整用
#define Ki_gain4p 1.0//4pIゲイン調整用
#define MAX_RPM 10.0e+3//回転数計算用のレンジ設定

/*--------------------hall関連設定ここから--------------------*/
FLOAT32 hall_theta_raw=0.0;//ホール素子->回転角度[rad]
FLOAT32 hall_theta_calc=0.0;//ホール素子回転角度オフセット+高調波補正値[rad]
FLOAT32 hall_theta_calc_prev=0.0;
INT32 rotation_num = 0;//回転回数
INT32 one_rotation_period = 0;//1回転の時間
volatile FLOAT32 rotational_speed = 0.0;

FLOAT32 hallErrorCompAmp[HALL_COMP_ORDER], hallErrorCompPh[HALL_COMP_ORDER];//ホール素子角度誤差の校正　0次～14次 振幅・角度はdeg入力でrad変換
INT32 hallErrorCompUse[HALL_COMP_ORDER];//指定した次数を使う場合は1、使わない場合は0

//volatile FLOAT32 theta_sus_0=210;//ラジアル電流の初期位相(deg)
//volatile FLOAT32 theta_sus_0=207.5;//ラジアル電流の初期位相(deg) -20230120
//volatile FLOAT32 theta_sus_0=217.5;//ラジアル電流の初期位相(deg) 20230123
//volatile FLOAT32 theta_sus_0=197.5;//ラジアル電流の初期位相(deg) 20230125
volatile FLOAT32 theta_sus_0=193.6;//ラジアル電流の初期位相(deg)
/*--------------------hall関連設定ここまで--------------------*/

FLOAT32 disp_x = 0.0, disp_y = 0.0;//変位センサ変換後(mm)

/*--------------------電流・電圧AD入力変数ここから--------------------*/

const FLOAT32 voltage_ad_gainA = (INVERTER_MODEL_A == 1) ? 100.0 : ((INVERTER_MODEL_A == 5) ? 100.0 : ((INVERTER_MODEL_A == 9) ? 80.0 : 0.0));//Aインバータ電圧センサのゲイン
const FLOAT32 current_ad_gainA = (INVERTER_MODEL_A == 1) ? 1.25 : ((INVERTER_MODEL_A == 5) ? 6.25 : ((INVERTER_MODEL_A == 9) ? 10.0 : 0.0));//Aインバータ電流センサのゲイン
const FLOAT32 voltage_ad_gainB = (INVERTER_MODEL_B == 1) ? 100.0 : ((INVERTER_MODEL_B == 5) ? 100.0 : ((INVERTER_MODEL_B == 9) ? 80.0 : 0.0));//Bインバータ電圧センサのゲイン
const FLOAT32 current_ad_gainB = (INVERTER_MODEL_B == 1) ? 1.25 : ((INVERTER_MODEL_B == 5) ? 6.25 : ((INVERTER_MODEL_B == 9) ? 10.0 : 0.0));//Bインバータ電流センサのゲイン

/*--------------------電流・電圧AD入力変数ここまで--------------------*/

/*--------------------電流・電圧測定値ここから--------------------*/
volatile FLOAT32 iuA=0.0;//(A)
volatile FLOAT32 iwA=0.0;//(A)

volatile FLOAT32 vdcA=0.0;//(V)
volatile FLOAT32 idcA=0.0;//(A)

volatile FLOAT32 iuA_offset=0.0;//Aインバータ電流センサオフセット値(A)
volatile FLOAT32 iwA_offset=0.0;//Aインバータ電流センサオフセット値(A)

volatile FLOAT32 iuB=0.0;//(A)
volatile FLOAT32 iwB=0.0;//(A)

volatile FLOAT32 vdcB=0.0;//(V)
volatile FLOAT32 idcB=0.0;//(A)

volatile FLOAT32 iuB_offset=0.0;//Bインバータ電流センサオフセット値(A)
volatile FLOAT32 iwB_offset=0.0;//Bインバータ電流センサオフセット値(A)

/*--------------------電流・電圧測定値ここまで--------------------*/

/*--------------------電流制御用変数ここから--------------------*/
//FLOAT32 ixA, iyA, idB, iqB;                              							// 実測されたdq軸電流[A]

const FLOAT32 KPd2p = FtoOmega(Fcutoff) * Ld_2p * Kp_gain2p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ld
const FLOAT32 KPq2p = FtoOmega(Fcutoff) * Lq_2p * Kp_gain2p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Lq
const FLOAT32 KId2p = FtoOmega(Fcutoff) * Ra_2p / Fcarrier * Ki_gain2p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 KIq2p = FtoOmega(Fcutoff) * Ra_2p / Fcarrier * Ki_gain2p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]

const FLOAT32 KPx4p = FtoOmega(Fcutoff) * Ld_4p * Kp_gain4p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ld
const FLOAT32 KPy4p = FtoOmega(Fcutoff) * Lq_4p * Kp_gain4p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Lq
const FLOAT32 KIx4p = FtoOmega(Fcutoff) * Ra_4p / Fcarrier * Ki_gain4p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 KIy4p = FtoOmega(Fcutoff) * Ra_4p / Fcarrier * Ki_gain4p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]

const FLOAT32 ixA_max = INV_5R_CURRENT_LIMIT * ROOT3 / ROOT2 / 2.0;
const FLOAT32 iyA_max = INV_5R_CURRENT_LIMIT * ROOT3 / ROOT2 / 2.0;      							// idq_max:dq軸電流の最大値[A]
const FLOAT32 idB_max = 3.0, iqB_max = 3.0;      							// idq_max:dq軸電流の最大値[A]
const INT32 max_num_of_delay = PERIOD_OF_PI_START_DERAY * Fcarrier;//I制御開始までの遅延ステップ数
const INT32 max_num_of_xy_current_output = PERIOD_OF_XY_CURRENT_TEST*Fcarrier;//xy電流出力のステップ数

INT32 delay_counterA = 0, delay_counterB = 0;//I制御開始までの遅延カウンタ
//INT32 offest_calc_counterB = 0;//電流センサオフセット値計算のカウンタ
volatile FLOAT32 vxA_max = 0.0, vyA_max = 0.0, vdB_max = 0.0, vqB_max = 0.0;//dq軸電圧の最大値[V]
volatile FLOAT32 ixA_ref = 0.0, iyA_ref = 0.0, ixA_ref_grad = 0.0, iyA_ref_grad = 0.0, idB_ref = 0.0, iqB_ref = 0.0;// dq軸電流の指令値[A]
FLOAT32 ixA_ref_temp, iyA_ref_temp;//変更検出用
//volatile FLOAT32 ixA_Pout, ixA_Iout, iyA_Pout, iyA_Iout;//dq軸電流PI出力
//volatile FLOAT32 idB_Pout, idB_Iout, iqB_Pout, iqB_Iout;//dq軸電流PI出力
//volatile FLOAT32 idB_out, iqB_out, ixA_out, iyA_out;//dq軸電流PI出力
volatile FLOAT32 ixA_Itmp = 0.0, iyA_Itmp = 0.0, idB_Itmp = 0.0, iqB_Itmp = 0.0;
volatile FLOAT32 vxA_ref, vyA_ref, vdB_ref, vqB_ref;//dq軸電圧指令値[V]
volatile FLOAT32 Half_VdcA = 0.0, Half_VdcB = 0.0;                            	// Half_VdcB:DCリンク電圧*1/2[V]
volatile INT32 enable_PI_controlA = 1, enable_PI_controlB = 0;//I制御停止用のVdcチェック変数 VdcがVdc_thresよりも大きいと1、それ以外0
volatile INT32 enable_angle_control = 0;//角度位置を指定する制御をする(1)、しない(0)
//volatile INT32 enable_grad_x_control = 0, enable_grad_y_control = 0;//指令値をゆっくり変化
volatile INT32 enable_speed_control = 0;//速度制御ONOFF
volatile INT32 enable_position_control = 0;//ラジアル制御ONOFF
volatile FLOAT32 reference_rpm = 0.0;//速度指令値
FLOAT32 reference_rpm_calc = 0.0;//実際に指令する回転数(指令回転数の変動が大きくなりすぎないようにrpm_incrementごとに増加させる)
//const FLOAT32 rotational_accelaretion = 200.0;//rpm/sec
const FLOAT32 rpm_increment = 100.0 / Fcarrier;//(rpm/sec)/Hz
volatile FLOAT32 reference_angle = 0.0;//角度制御するときの角度位置(deg)
//volatile FLOAT32 KPt = 3, KIt = 0.1, KDt = 0.1;//角度位置制御のPIDゲイン
//volatile FLOAT32 KPt = 3, KIt = 0.6, KDt = 0.04;//角度位置制御のPIDゲイン
const FLOAT32 KPts = 12.8359, KIts = 141.0516, KDts = 0.29202;//角度位置制御のPIDゲイン(角度)
const FLOAT32 KPtr = 0.090878, KItr = 0.028874, KDtr = 0.071506;//角度位置制御のPIDゲイン(速度)
volatile FLOAT32 theta_err, theta_error_prev, theta_Pout, theta_Iout, theta_Itmp, theta_Dtmp, theta_Dout, theta_ref;
//volatile FLOAT32 KPsus = 47933, KIsus = 1.30272e+006, KDsus = 249;//磁気支持制御のPIDゲイン 20221222 LSRなし
//volatile FLOAT32 KPsus = 34168, KIsus = 1.1327e+006, KDsus = 258;//磁気支持制御のPIDゲイン 20221223 LSRなし
const FLOAT32 KPsus = 139145, KIsus = 10770510, KDsus = 449, Tfsus = 1 / (PI(2)*5.0e+2);//磁気支持制御のPIDゲイン 20221223 LSRあり
//const FLOAT32 i_control_limit = 0.04;//mm これ以内ならI制御実行(誤差がでかいとIが暴走して振動がでかくなるので
const FLOAT32 i_control_limit = 0.1;//mm 20230206変更
//volatile FLOAT32 KPsus = 115987.3, KIsus = 2961375.2, KDsus = 449.0, Tfsus = 2.6653e-6;//磁気支持制御のPIDゲイン 20230105 LSRあり Tfsus:LPF時定数 浮上してない
//volatile FLOAT32 KPsus = 117261.2, KIsus = 10672908.6, KDsus = 313.7, Tfsus = 3.8529e-5;//磁気支持制御のPIDゲイン 20230105 LSRあり Tfsus:LPF時定数 浮いたけどタッチダウンしまくり
//volatile FLOAT32 xsus_ref = 0.625,ysus_ref = 0.645;//20rpm.xls
//volatile FLOAT32 xsus_ref = 0.61,ysus_ref = 0.63;//20221223 手動調節 w/o LSR
//volatile FLOAT32 xsus_ref = 0.67,ysus_ref = 0.63;//20221227 手動調節 w/LSR
//volatile FLOAT32 xsus_ref = 0.675+0.0129,ysus_ref = 0.618+0.0070;//20230111 回転時の中心位置 w/LSR
volatile FLOAT32 xsus_ref = 0.6879 - 0.0133,ysus_ref = 0.625 - 0.0080;//20230111 回転時の中心位置 w/LSR
volatile FLOAT32 xsus_error_LPF, ysus_error_LPF, asus_error_LPF, bsus_error_LPF;
volatile INT32 enable_xy_filtering = 0;
volatile FLOAT32 xsus_err, xsus_error_prev, xsus_Pout, xsus_Iout, xsus_Itmp, xsus_Dtmp, xsus_Dout,  ysus_err, ysus_error_prev, ysus_Pout, ysus_Iout, ysus_Itmp, ysus_Dtmp, ysus_Dout;
//FLOAT32 xsus_Dtmp_lpf, ysus_Dtmp_lpf;
/*--------------------電流制御用変数ここまで--------------------*/

void hallErrorCompInit(void);
FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle);
void getAdStatusSensor(void);//timer1割り込み
void getAdStatusAinv(void);
void getAdStatusBinv(void);
void current_controlA(void);//pwm割り込み
void current_controlB(void);//pwm割り込み
void positionControlB(void);
void positionControlA(void);
void speedControl(void);
void currentSensorInitB(void);
void currentSensorInitA(void);
//inline static void setAGeinZero(void);
inline static void setBThetaGeinZero (void);
inline static void setSusGeinZero (void);
//inline static void setBGeinZero(void);

#pragma INTERRUPT(current_controlA)//pwm割り込み
#pragma INTERRUPT(current_controlB)//pwm割り込み
#pragma INTERRUPT(getAdStatusSensor)//timer1割り込み

void hallErrorCompInit(void)//ホール素子校正の初期設定
{
	INT32 i;
	FLOAT32 _hallErrorCompAmpPrec[HALL_COMP_ORDER_PREC], _hallErrorCompPhPrec[HALL_COMP_ORDER_PREC];
	INT32 _hallErrorCompUsePrec[HALL_COMP_ORDER_PREC];

	//hallErrorCompPhはdegで代入してradに変換
	_hallErrorCompAmpPrec[0]=-103.194+7.488;	_hallErrorCompPhPrec[0]=0.0;	_hallErrorCompUsePrec[0]=1;//-103.194
	_hallErrorCompAmpPrec[1]=0.633275735;	_hallErrorCompPhPrec[1]= -117.8906118;	_hallErrorCompUsePrec[1]=1;
	_hallErrorCompAmpPrec[2]=3.086499659;	_hallErrorCompPhPrec[2]= -81.0681271;	_hallErrorCompUsePrec[2]=1;
	_hallErrorCompAmpPrec[3]=0.075699977;	_hallErrorCompPhPrec[3]= 23.14646234;	_hallErrorCompUsePrec[3]=0;
	_hallErrorCompAmpPrec[4]=4.366958749;	_hallErrorCompPhPrec[4]= -81.76701926;	_hallErrorCompUsePrec[4]=1;
	_hallErrorCompAmpPrec[5]=0.173986398;	_hallErrorCompPhPrec[5]= 107.3194347;	_hallErrorCompUsePrec[5]=0;
	_hallErrorCompAmpPrec[6]=0.521664715;	_hallErrorCompPhPrec[6]= 117.9616102;	_hallErrorCompUsePrec[6]=1;
	_hallErrorCompAmpPrec[7]=0.027259279;	_hallErrorCompPhPrec[7]= 53.82444327;	_hallErrorCompUsePrec[7]=0;
	_hallErrorCompAmpPrec[8]=0.583975898;	_hallErrorCompPhPrec[8]= 120.4875918;	_hallErrorCompUsePrec[8]=1;
	_hallErrorCompAmpPrec[9]=0.066215787;	_hallErrorCompPhPrec[9]= -61.50186586;	_hallErrorCompUsePrec[9]=0;
	_hallErrorCompAmpPrec[10]=0.151485583;	_hallErrorCompPhPrec[10]= -31.82436231;	_hallErrorCompUsePrec[10]=0;
	_hallErrorCompAmpPrec[11]=0.015517434;	_hallErrorCompPhPrec[11]= -152.0030841;	_hallErrorCompUsePrec[11]=0;
	_hallErrorCompAmpPrec[12]=0.120998567;	_hallErrorCompPhPrec[12]= -38.31503892;	_hallErrorCompUsePrec[12]=0;
	_hallErrorCompAmpPrec[13]=0.023401454;	_hallErrorCompPhPrec[13]= 130.660386;	_hallErrorCompUsePrec[13]=0;
	_hallErrorCompAmpPrec[14]=0.052686297;	_hallErrorCompPhPrec[14]= 174.0505764;	_hallErrorCompUsePrec[14]=0;

	for(i = 0; i < HALL_COMP_ORDER; i++)
	{
		hallErrorCompAmp[i] = _hallErrorCompAmpPrec[i] * PI(1) / 180.0;
		hallErrorCompPh[i] = _hallErrorCompPhPrec[i] * PI(1) / 180.0;//radに変換
		hallErrorCompUse[i] = _hallErrorCompUsePrec[i];
	}
}

FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle)//hallRawAngleはrad
{
	FLOAT32 hallCalcAngle = hallRawAngle;
	INT32 i;
	for(i = 0; i < HALL_COMP_ORDER; i++)//フーリエ変換したもので真値を計算
	{
		if(hallErrorCompUse[i])//無視する次数じゃないとき
			hallCalcAngle -= hallErrorCompAmp[i] * mwcos((FLOAT32)i * hallRawAngle+hallErrorCompPh[i]);
	}
	
	return radianLimit(hallCalcAngle);
}

//AD変換した値を取得して実際の値に変換して格納
void getAdStatusSensor(void)
{
	FLOAT32 _hallA = 0.0, _hallB = 0.0, _hallC = 0.0, _hallD = 0.0;
	FLOAT32 _disp_x_raw = 0.0, _disp_y_raw = 0.0;
	FLOAT32 rotational_speed_temp = 0.0;
	//const INT32 min_rotational_speed = 300;//rpm
	const INT32 max_period = 4000;
	//const FLOAT32 temp = (60 / min_rotational_speed / (PERIOD / SAMPL * 1.0e-6));
	//max_period = (INT32)temp;
	/* AD変換結果の取得 */
	adInput(SEQ1, AD_GROUP_6, &_hallA, &_hallB);
	adInput(SEQ2, AD_GROUP_7, &_hallC, &_hallD);
	adInput(SEQ1, AD_GROUP_5, &_disp_y_raw, &_disp_x_raw);

	if(mwabs(_disp_x_raw) >= 5.0)
		_disp_x_raw = 0.0;
	if(mwabs(_disp_y_raw) >= 5.0)
		_disp_y_raw = 0.0;
	
	disp_y = dispSensorCalcA(_disp_y_raw);
	disp_x = dispSensorCalcB(_disp_x_raw);

	hall_theta_raw = getHallRawValue(_hallA, _hallB, _hallC, _hallD);
	hall_theta_calc_prev = hall_theta_calc;
	hall_theta_calc = getHallErrorCalc(hall_theta_raw);

	if(enable_speed_control == 1)//速度制御の時のみ
	{
		if(one_rotation_period < max_period)
			one_rotation_period++;
		else if (mwabs(reference_rpm_calc) < 1.0)
			rotational_speed = 0.0;
	}
	else//速度制御しないときは0のまま
	{
		one_rotation_period = 0.0;
		rotational_speed = 0.0;
	}

	if(mwabs(hall_theta_calc - hall_theta_calc_prev) > PI(1))//前の角度との差分がpi以上あったとき
	{
		if(one_rotation_period > (60 / MAX_RPM / (PERIOD / SAMPL * 1.0e-6)))
		{
			rotational_speed_temp = 60.0 / (one_rotation_period * (PERIOD / SAMPL * 1.0e-6));//rpm=60/(カウント数*1カウントの時間)
		}
		
		if((hall_theta_calc - hall_theta_calc_prev) > 0)//差分が正の時→のこぎり波の傾きは負　なので逆回転
		{
			if(mwabs(rotational_speed_temp)>1.0)
			{
				rotational_speed = - rotational_speed_temp;
				one_rotation_period = 0;
			}
			rotation_num--;
		}
		else//差分が負のとき→のこぎり波の傾きは正　なので正回転
		{
			if(mwabs(rotational_speed_temp)>1.0)
			{
				rotational_speed = rotational_speed_temp;
				one_rotation_period = 0;
			}
			rotation_num++;
		}
	}
	//pro_da_out(DA_CH_0, hall_theta_raw);							// Output data for viewing waveforms in DAC					
	//pro_da_out(DA_CH_0, rotational_speed);//速度試験用 〇後で戻す
	//pro_da_out(DA_CH_0, reference_rpm_calc);//校正用
	//pro_da_out(DA_CH_2, hall_theta_calc);//〇後で戻す
}

void getAdStatusAinv(void)
{
	FLOAT32 _iuA_raw, _iwA_raw, _vdcA_raw, _idcA_raw;
	adInput(SEQ2, AD_GROUP_1, &_iuA_raw, &_iwA_raw);
	iuA = i_raw_converterA(_iuA_raw);
	iwA = i_raw_converterA(_iwA_raw);

	adInput(SEQ1, AD_GROUP_2, &_vdcA_raw, &_idcA_raw);
	vdcA = (_vdcA_raw >= 0.0) ? v_raw_converterA(_vdcA_raw) : 0.0;
	idcA = i_raw_converterA(_idcA_raw);

	Half_VdcA = (vdcA >= 0.0) ? vdcA / 2 : 0.0;//DCリンク電圧を更新
	vxA_max = ROOT3 / ROOT2 * Half_VdcA;//dq軸電圧最大値を計算
	vyA_max = ROOT3 / ROOT2 * Half_VdcA;
}

void getAdStatusBinv(void)
{
	FLOAT32 _iuB_raw, _iwB_raw, _vdcB_raw, _idcB_raw;
	adInput(SEQ1, AD_GROUP_0, &_iuB_raw, &_iwB_raw);
	iuB=i_raw_converterB(_iuB_raw);
	iwB=i_raw_converterB(_iwB_raw);

	adInput(SEQ2, AD_GROUP_3, &_vdcB_raw, &_idcB_raw);
	vdcB=(_vdcB_raw>=0.0)?v_raw_converterB(_vdcB_raw):0.0;
	idcB=i_raw_converterB(_idcB_raw);
	
	Half_VdcB = (vdcB >= 0.0) ? vdcB / 2 : 0.0;//DCリンク電圧を更新
	vdB_max = ROOT3 / ROOT2 * Half_VdcB;//dq軸電圧最大値を計算
	vqB_max = ROOT3 / ROOT2 * Half_VdcB;
}

// キャリア同期割り込みで，dq軸電流をPI制御する関数
/*void current_controlA(void) 
{
    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    getAdStatusAinv();//インバータから電流電圧取得

	//オフセット値を減算
	iuA -= iuA_offset;
	iwA -= iwA_offset;

	*//*if( vdcA > Vdc_thres )//閾値電圧よりも大きい場合
	{		
		if(delay_counterA >= max_num_of_delay)//カウンタが規定回数を超えた場合
		{
			enable_PI_controlA =  1;//I制御開始
			if(delay_counterA < max_num_of_delay * 1.5)//閾値電圧付近を行き来しても大丈夫なように最大値の1.5倍までカウントアップ
				delay_counterA++;
		}
		else
			delay_counterA++;//カウンタアップ
	}
	else
	{
		if(delay_counterA < 0)//待ち時間の1.5倍が過ぎても閾値電圧を下回っている場合
		{
			delay_counterA = 0;
			enable_PI_controlA = 0;//I制御停止
			//setAGeinZero();
			ixA_Itmp = 0.0;
			iyA_Itmp = 0.0;
			ixA_Pout = 0.0;
			iyA_Pout = 0.0;
		}
		else
		{
			delay_counterA--;//カウンタ減少
		}
	}*//*

	if(enable_PI_controlA)
	{
		pi_control(ixA, ixA_ref, KPx4p, KIx4p, ixA_max, &ixA_Pout, &ixA_Iout, &ixA_Itmp);
		pi_control(iyA, iyA_ref, KPy4p, KIy4p, iyA_max, &iyA_Pout, &iyA_Iout, &iyA_Itmp);
	}
	else
	{
		idB_Itmp = 0.0;
		iqB_Itmp = 0.0;
		idB_Pout = 0.0;
		iqB_Pout = 0.0;
		idB_Iout = 0.0;
		iqB_Iout = 0.0;
	}

    // dq軸電圧指令値を計算
	vxA_ref = ixA_Pout + ixA_Iout;
	vyA_ref = iyA_Pout + iyA_Iout;

	vxA_ref=mylimit(vxA_ref, vxA_max);
	vyA_ref=mylimit(vyA_ref, vyA_max);
	
	//inverter0_set_uvw( 0.0, 0.0, 0.0 );
	
	pwm_output(vxA_ref, vyA_ref, Half_VdcA, hall_theta_calc);
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter0_clear_up_flag();						// Clearing the interrupt flag.											
}*/
// キャリア同期割り込みで，dq軸電流をPI制御する関数
void current_controlA(void) 
{
	FLOAT32 _ixA, _iyA;
    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    getAdStatusAinv();//インバータから電流電圧取得

	//オフセット値を減算
	iuA -= iuA_offset;
	iwA -= iwA_offset;

	/*if( vdcB > Vdc_thres )//閾値電圧よりも大きい場合 未編集
	{		
		if(delay_counterB >= max_num_of_delay)//カウンタが規定回数を超えた場合
		{
			enable_PI_controlB =  1;//I制御開始
			if(delay_counterB < max_num_of_delay * 1.5)//閾値電圧付近を行き来しても大丈夫なように最大値の1.5倍までカウントアップ
				delay_counterB++;
		}
		else
			delay_counterB++;//カウンタアップ
	}
	else
	{
		if(delay_counterB < 0)//待ち時間の1.5倍が過ぎても閾値電圧を下回っている場合
		{
			delay_counterB = 0;
			enable_PI_controlB = 0;//I制御停止
			
			idB_Itmp = 0.0;
			iqB_Itmp = 0.0;
			idB_Pout = 0.0;
			iqB_Pout = 0.0;
			//setBThetaGeinZero();
			rotaton_num = 0;
		}
		else
		{
			delay_counterB--;//カウンタ減少
		}
	}*/
	
    // iuB, iwB → idB, iqB (絶対変換)
	//uvw2dq(iuA, iwA, radianLimit(hall_theta_calc+theta_sus_0*PI(1)/180), &_ixA, &_iyA);
	uvw2dq(iuA, iwA, hall_theta_calc+theta_sus_0*PI(1)/180, &_ixA, &_iyA);

	//pro_da_out(DA_CH_1,_ixA);	//速度試験用
	//pro_da_out(DA_CH_2,_iyA);//速度試験用
	pro_da_out(DA_CH_0,_ixA);//□あとでコメントアウト
	pro_da_out(DA_CH_1,_iyA);//□あとでコメントアウト

	if(enable_position_control)
	{
		positionControlA();
	}
	else
	{
		setSusGeinZero();
	}

	if(enable_position_control)
	{
		positionControlA();
		enable_PI_controlA=1;
	}
	else
	{
		setSusGeinZero();
		if(delay_counterA<max_num_of_xy_current_output)//一瞬電流出力
		{
			enable_PI_controlA=1;
			/*if(enable_grad_x_control == 1)
			{
				ixA_ref-=ixA_ref_grad*2/max_num_of_xy_current_output;
			}
			if(enable_grad_y_control == 1)
			{
				iyA_ref-=iyA_ref_grad*2/max_num_of_xy_current_output;
			}*/
		}
		else
		{
			if(ixA_ref!=ixA_ref_temp||iyA_ref!=iyA_ref_temp)//指示電流の変更を検出した場合
			{
				delay_counterA=0;
				/*if(enable_grad_x_control==1)
				{
					ixA_ref_grad=ixA_ref;
				}
				if(enable_grad_y_control==1)
				{
					iyA_ref_grad=iyA_ref;
				}*/
			}
			enable_PI_controlA=0;
		}
	}

	pro_da_out(DA_CH_2,ixA_ref);//□あとでコメントアウト
	pro_da_out(DA_CH_3,iyA_ref);//□あとでコメントアウト

	if(enable_PI_controlA)
	{
		//pi_control(ixA, ixA_ref, KPx4p, KIx4p, ixA_max, &ixA_Pout, &ixA_Iout, &ixA_Itmp);
		//pi_control(iyA, iyA_ref, KPy4p, KIy4p, iyA_max, &iyA_Pout, &iyA_Iout, &iyA_Itmp);
		vxA_ref = pi_control_release(_ixA, ixA_ref, KPx4p, KIx4p, ixA_max, &ixA_Itmp);
		vyA_ref = pi_control_release(_iyA, iyA_ref, KPy4p, KIy4p, iyA_max, &iyA_Itmp);
		ixA_ref_temp=ixA_ref;
		iyA_ref_temp=iyA_ref;
		delay_counterA++;//電流出力カウンタ
	}
	else
	{
		ixA_Itmp = 0.0;
		iyA_Itmp = 0.0;
		vxA_ref = 0.0;
		vyA_ref = 0.0;
		//ixA_Pout = 0.0;
		//iyA_Pout = 0.0;
		//ixA_Iout = 0.0;
		//iyA_Iout = 0.0;
	}

    // dq軸電圧指令値を計算
	//vxA_ref = ixA_Pout + ixA_Iout;
	//vyA_ref = iyA_Pout + iyA_Iout;
	//vxA_ref = ixA_out;
	//vyA_ref = iyA_out;

    vxA_ref = mylimit(vxA_ref, vxA_max);
    vyA_ref = mylimit(vyA_ref, vyA_max);

	//inverter1_set_uvw(0.0,0.0,0.0);
	//pwm_output(vxA_ref, vyA_ref, Half_VdcA, radianLimit(hall_theta_calc+theta_sus_0*PI(1)/180),0);
	pwm_output(vxA_ref, vyA_ref, Half_VdcA, hall_theta_calc+theta_sus_0*PI(1)/180,0);
	
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter0_clear_up_flag();						// Clearing the interrupt flag.											
}

// キャリア同期割り込みで，dq軸電流をPI制御する関数
void current_controlB(void) 
{
	FLOAT32 _idB, _iqB;
    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    getAdStatusBinv();//インバータから電流電圧取得

	//オフセット値を減算
	iuB -= iuB_offset;
	iwB -= iwB_offset;

	if( vdcB > Vdc_thres )//閾値電圧よりも大きい場合
	{		
		if(delay_counterB >= max_num_of_delay)//カウンタが規定回数を超えた場合
		{
			enable_PI_controlB =  1;//I制御開始
			if(delay_counterB < max_num_of_delay * 1.5)//閾値電圧付近を行き来しても大丈夫なように最大値の1.5倍までカウントアップ
				delay_counterB++;
		}
		else
			delay_counterB++;//カウンタアップ
	}
	else
	{
		if(delay_counterB < 0)//待ち時間の1.5倍が過ぎても閾値電圧を下回っている場合
		{
			delay_counterB = 0;
			enable_PI_controlB = 0;//I制御停止
			
			idB_Itmp = 0.0;
			iqB_Itmp = 0.0;
			vdB_ref = 0.0;
			vqB_ref = 0.0;
			//idB_Pout = 0.0;
			//iqB_Pout = 0.0;
			setBThetaGeinZero();
			rotation_num = 0;
		}
		else
		{
			delay_counterB--;//カウンタ減少
		}
	}
	
    // iuB, iwB → idB, iqB (絶対変換)
	uvw2dq(iuB, iwB, hall_theta_calc, &_idB, &_iqB);

	if(enable_angle_control + enable_speed_control)
	{
		positionControlB();
	}
	else
	{
		setBThetaGeinZero();
	}

	if(enable_PI_controlB)
	{
		//pi_control(idB, idB_ref, KPd2p, KId2p, idB_max, &idB_Pout, &idB_Iout, &idB_Itmp);
		//pi_control(iqB, iqB_ref, KPq2p, KIq2p, iqB_max, &iqB_Pout, &iqB_Iout, &iqB_Itmp);
		vdB_ref = pi_control_release(_idB, idB_ref, KPd2p, KId2p, idB_max, &idB_Itmp);
		vqB_ref = pi_control_release(_iqB, iqB_ref, KPq2p, KIq2p, iqB_max, &iqB_Itmp);
	}
	else
	{
		idB_Itmp = 0.0;
		iqB_Itmp = 0.0;
		vdB_ref = 0.0;
		vqB_ref = 0.0;
		//idB_Pout = 0.0;
		//iqB_Pout = 0.0;
		//idB_Iout = 0.0;
		//iqB_Iout = 0.0;
	}
    /*// PI制御 d軸
    idB_ref = mylimit(idB_ref, idB_max);
    // d軸電流 P制御
    idB_err = idB_ref - idB;
    idB_Pout = KPd2p * idB_err * (FLOAT32)enable_PI_controlB;
    // d軸電流 I制御
    idB_Itmp += idB_err * (FLOAT32)enable_PI_controlB;
    idB_Itmp = mylimit(idB_Itmp, 30000.0);
    idB_Iout = KId2p * idB_Itmp;//なんかバグる

    // PI制御 q軸
    iqB_ref = mylimit(iqB_ref, iqB_max);
    // q軸電流 P制御
    iqB_err = iqB_ref - iqB;
    iqB_Pout = KPq2p * iqB_err * (FLOAT32)enable_PI_controlB;
    // q軸電流 I制御
    iqB_Itmp += iqB_err * (FLOAT32)enable_PI_controlB;
    iqB_Itmp = mylimit(iqB_Itmp, 30000.0);
    iqB_Iout = KIq2p * iqB_Itmp;*/

    // dq軸干渉項の計算
    //vodB = -w_e * Lq_2p * iqB;
    //voqB = w_e * (PSY_a + Ld_2p * idB);

    // dq軸電圧指令値を計算
	//vdB_ref = idB_Pout + idB_Iout;// + vodB * Decoupling_Mode2p;
	//vqB_ref = iqB_Pout + iqB_Iout;// + voqB * Decoupling_Mode2p;
	//vdB_ref = idB_out;// + vodB * Decoupling_Mode2p;
	//vqB_ref = iqB_out;// + voqB * Decoupling_Mode2p;

    vdB_ref = mylimit(vdB_ref, vdB_max);
    vqB_ref = mylimit(vqB_ref, vqB_max);
    //vdB_limd = mylimit(vdB_ref, vdB_max);
    //vqB_limd = mylimit(vqB_ref, vqB_max);

    // uvw電圧を出力
    // vd, vq → vu, vv, vw (絶対変換)
	//dq2uvw(vdB_limd, vqB_limd, hall_theta_calc, &vuB_ref, &vvB_ref, &vwB_ref);
	/*dq2uvw(vdB_ref, vqB_ref, hall_theta_calc, &vuB_ref, &vvB_ref, &vwB_ref);
	
	//三相電圧指令値を正規化(-1～1)
    mvuB = mylimit(vuB_ref / Half_VdcB, 1.0);
    mvvB = mylimit(vvB_ref / Half_VdcB, 1.0);
    mvwB = mylimit(vwB_ref / Half_VdcB, 1.0);

	inverter1_set_uvw( mvuB, mvvB, mvwB );//インバータ出力*/
	//inverter1_set_uvw(0.0,0.0,0.0);
	pwm_output(vdB_ref, vqB_ref, Half_VdcB, hall_theta_calc,1);
	
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter1_clear_up_flag();						// Clearing the interrupt flag.											
}

void positionControlB(void)
{
	const FLOAT32 filter_factor = 1 / (PI(2) * 500 / Fcarrier + 1);//参考 https://haizairenmei.com/2018/10/27/arduino_noise/
	FLOAT32 KPt = enable_speed_control ? KPtr : KPts;
	FLOAT32 KIt = enable_speed_control ? KItr : KIts;
	FLOAT32 KDt = enable_speed_control ? KDtr : KDts;
	const FLOAT32 i_control_limit_theta = enable_speed_control ? 100 : 3.0 * PI(1.0) / 180.0;

	theta_error_prev = theta_err;
	speedControl();
	// PI制御 d軸
	theta_ref = radianLimit(reference_angle * PI(1.0) / 180.0);
	// d軸電流 P制御
	theta_err = theta_ref - (hall_theta_calc + rotation_num * PI(2));
	//theta_Dtmp = theta_err - theta_error_prev;
	
	theta_Dtmp = filter_factor * theta_Dtmp + (1.0 - filter_factor) * (theta_err - theta_error_prev);

	theta_Pout = KPt * theta_err * (FLOAT32)enable_angle_control;
	// d軸電流 I制御
	theta_Itmp += theta_err * (FLOAT32)enable_angle_control / Fcarrier;
	if(enable_speed_control == 0)
	{
		if(mwabs(theta_err) >= i_control_limit_theta)//速度制御アリの時にこのif文が有効だと急に制御が変わってタッチダウンする(2023/2/6)
		{
			theta_Itmp = 0.0;
		}
		theta_Itmp = mylimit(theta_Itmp, (iqB_max / KIt * 10.0));
	}
	else
	{
		theta_Itmp = mylimit(theta_Itmp, 3000.0);
	}
	
	theta_Iout = KIt * theta_Itmp;

	theta_Dout = theta_Dtmp * KDt * Fcarrier * (FLOAT32)enable_angle_control;

	iqB_ref = theta_Iout + theta_Pout + theta_Dout;
	iqB_ref = mylimit(iqB_ref, enable_position_control ? 1.5 : 2.0);
	//iqB_ref = mylimit(theta_Iout + theta_Pout + theta_Dout,1.0);
	//idB_ref = mylimit(1/mwabs(theta_err),1.0);
}

void positionControlA(void)
{
	//const FLOAT32 filter_factor = 1 / (PI(2) * Fcutoff / Fcarrier + 1);
	const FLOAT32 filter_factor = Tfsus * Fcarrier / (1 + Fcarrier * Tfsus);//LPF時定数から計算
	const FLOAT32 xy_filter_factor = 1 / (Fcarrier * PI(2) * rotational_speed / 60 / 10 + 1);
	FLOAT32 xsus_Dcalc, ysus_Dcalc;
	//FLOAT32 xsus_error_LPF_prev, ysus_error_LPF_prev, asus_error_LPF_prev, bsus_error_LPF_prev;
	FLOAT32 asus_err, bsus_err;
	FLOAT32 xsus_filtered_diff_error, ysus_filtered_diff_error;

	//前回の誤差を保存
	xsus_error_prev = xsus_err;
	ysus_error_prev = ysus_err;
	
	//mmをmに変換
	xsus_err = (xsus_ref - disp_x) / 1000.0;
	ysus_err = (ysus_ref - disp_y) / 1000.0;
	//xsus_Dtmp = xsus_err - xsus_error_prev;
	//ysus_Dtmp = ysus_err - ysus_error_prev;

	//前回の誤差を保存
	//xsus_error_LPF_prev = xsus_error_LPF;
	//ysus_error_LPF_prev = ysus_error_LPF;
	
	//xsus_error_LPF += xy_filter_factor * (xsus_err - xsus_error_LPF_prev);//xyの誤差の平均値を取得
	//ysus_error_LPF += xy_filter_factor * (ysus_err - ysus_error_LPF_prev);
	xsus_error_LPF = xy_filter_factor * xsus_error_LPF + (1 - xy_filter_factor) * xsus_err;
	ysus_error_LPF = xy_filter_factor * ysus_error_LPF + (1 - xy_filter_factor) * ysus_err;

	//xyをdq座標に変換
	ab2dq(xsus_err, ysus_err, hall_theta_calc, &asus_err, &bsus_err);

	//LPF on ab error
	//asus_error_LPF_prev = asus_error_LPF;
	//bsus_error_LPF_prev = bsus_error_LPF;
	//asus_error_LPF += xy_filter_factor * (asus_err - asus_error_LPF_prev);
	//bsus_error_LPF += xy_filter_factor * (bsus_err - bsus_error_LPF_prev);
	asus_error_LPF = xy_filter_factor * asus_error_LPF + (1 - xy_filter_factor) * asus_err;
	bsus_error_LPF = xy_filter_factor * bsus_error_LPF + (1 - xy_filter_factor) * bsus_err;

	//inverse dq transform on filtered ab error
	dq2ab(asus_error_LPF, bsus_error_LPF, hall_theta_calc, &xsus_filtered_diff_error, &ysus_filtered_diff_error);

	//when rotational speed is faster than 600 rpm
	if(rotational_speed > 600.0 )
	{
		//calc diference of 1st comp of error and raw error
		if(enable_xy_filtering == 1)
		{
			xsus_err = xsus_filtered_diff_error;
			ysus_err = ysus_filtered_diff_error;
		}
	}

	//P制御
	xsus_Pout = KPsus * xsus_err * (FLOAT32)enable_position_control;
	ysus_Pout = KPsus * ysus_err * (FLOAT32)enable_position_control;
	
	//I制御
	xsus_Itmp += xsus_err * (FLOAT32)enable_position_control / Fcarrier;
	ysus_Itmp += ysus_err * (FLOAT32)enable_position_control / Fcarrier;
	if(mwsqrt2(xsus_err, ysus_err) >= i_control_limit / 1000.0)
	{
		xsus_Itmp = 0.0;
		ysus_Itmp = 0.0;
	}
	//xsus_Itmp = mylimit(xsus_Itmp, 3.0);
	//ysus_Itmp = mylimit(ysus_Itmp, 3.0);
	xsus_Itmp = mylimit(xsus_Itmp, 3000.0);
	ysus_Itmp = mylimit(ysus_Itmp, 3000.0);
	xsus_Iout = KIsus * xsus_Itmp;
	ysus_Iout = KIsus * ysus_Itmp;

	//D制御
	xsus_Dcalc = (xsus_err - xsus_error_prev) * Fcarrier;
	ysus_Dcalc = (ysus_err - ysus_error_prev) * Fcarrier;
	//LPF
	xsus_Dtmp = filter_factor * xsus_Dtmp + (1.0 - filter_factor) * xsus_Dcalc;
	ysus_Dtmp = filter_factor * ysus_Dtmp + (1.0 - filter_factor) * ysus_Dcalc;
	xsus_Dtmp = xsus_Dtmp * (FLOAT32)enable_position_control;
	ysus_Dtmp = ysus_Dtmp * (FLOAT32)enable_position_control;
	xsus_Dout = xsus_Dtmp * KDsus;
	ysus_Dout = ysus_Dtmp * KDsus;

	ixA_ref = mylimit(xsus_Pout + xsus_Iout + xsus_Dout, ixA_max);
	iyA_ref = mylimit(ysus_Pout + ysus_Iout + ysus_Dout, iyA_max);
}

void speedControl(void)
{
	FLOAT32 theta_diff_one_cycle;
	if(reference_rpm != reference_rpm_calc)
	{
		if(mwabs(reference_rpm - reference_rpm_calc) <= rpm_increment)
		{
			reference_rpm_calc = reference_rpm;
		}
		else
		{
			if(reference_rpm - reference_rpm_calc > 0)
				reference_rpm_calc += rpm_increment;
			else
				reference_rpm_calc -= rpm_increment;
		}
	}
	theta_diff_one_cycle = reference_rpm_calc / 60 * 360 / Fcarrier;//(deg)
	reference_angle += theta_diff_one_cycle * enable_speed_control;
	if(mwabs(reference_angle) > 180)
	{
		if(reference_angle > 0)
		{
			reference_angle -= 360.0;
			rotation_num--;
		}
		else
		{
			reference_angle += 360.0;
			rotation_num++;
		}
	}
	//reference_angle = radianLimit(reference_angle / 180 * PI(1)) / PI(1) * 180;
}

void currentSensorInitB(void)
{
	INT32 i;
	const INT32 waitTime = (INT32)(1.0 / Fcarrier * 10e+6);//usec

	getAdStatusBinv();

	for(i = 0; i < NUM_OF_OFFSET_DETECT; i++)
	{
		iuB_offset += iuB / (FLOAT32)NUM_OF_OFFSET_DETECT;//オフセット値の平均値を積算
		iwB_offset += iwB / (FLOAT32)NUM_OF_OFFSET_DETECT;
		enable_PI_controlB = 0;//I制御停止
	
		inverter1_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力停止

		wait(waitTime);
	}
}

void currentSensorInitA(void)
{
	INT32 i;
	const INT32 waitTime = (1.0/Fcarrier*10e+6);//usec

	getAdStatusAinv();

	for(i = 0; i < NUM_OF_OFFSET_DETECT; i++)
	{
		iuA_offset += iuA / (FLOAT32)NUM_OF_OFFSET_DETECT;//オフセット値の平均値を積算
		iwA_offset += iwA / (FLOAT32)NUM_OF_OFFSET_DETECT;
		//enable_PI_controlA = 0;//I制御停止
	
		inverter0_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力停止

		wait(waitTime);
	}
}
/*
inline static void setAGeinZero(void)
{
	//PI制御の計算値を初期化
	
}

inline static void setBGeinZero(void)
{
	//PI制御の計算値を初期化
}*/

inline static void setBThetaGeinZero (void)
{
	theta_err = 0.0;
	theta_Iout = 0.0;
	theta_Itmp = 0.0;
	theta_Pout = 0.0;
	theta_error_prev = 0.0;
	theta_Dout = 0.0;
	theta_Dtmp = 0.0;
}

inline static void setSusGeinZero (void)
{
	xsus_err = 0.0;
	xsus_Iout = 0.0;
	xsus_Itmp = 0.0;
	xsus_Pout = 0.0;
	xsus_error_prev = 0.0;
	xsus_Dout = 0.0;

	ysus_err = 0.0;
	ysus_Iout = 0.0;
	ysus_Itmp = 0.0;
	ysus_Pout = 0.0;
	ysus_error_prev = 0.0;
	ysus_Dout = 0.0;
	ysus_Dtmp = 0.0;
}

int main( void )
{
	int_disable();									/* Disabling all interrupt												*/

	hallErrorCompInit();

	system_init();
	#if !defined(F28335LMT)
	watch_init();
	#endif
	
	inverter0_init( Fcarrier, DEADTIME );					// Setting carrier frequency and dead time		 						
	inverter0_set_uvw( 0.0, 0.0, 0.0 );				// Setting initial PWM references										
	inverter0_init_up_vector( current_controlA );	// Carrier interrupt routine definition									
	inverter0_enable_up_int();						// Enabling carrier interrupt at the top point of the carrier mountain	
	
	inverter1_init( Fcarrier, DEADTIME );					/* Setting carrier frequency and dead time		 						*/
	inverter1_set_uvw( 0.0, 0.0, 0.0 );				/* Setting initial PWM references										*/
	inverter1_init_up_vector( current_controlB );	/* Carrier interrupt routine definition									*/
	inverter1_enable_up_int();						/* Enabling carrier interrupt at the top point of the carrier mountain	*/
	
	/*--------------------da出力(enc/hall)設定ここから--------------------*/
	pro_da_init();								/* Initialization of DA converter							*/
	//pro_da_set_range(DA_CH_0, PI(1));//DACの正規化レンジは-pi->piに設定
	//pro_da_set_range(DA_CH_0, MAX_RPM);//〇後で戻す
	//pro_da_set_range(DA_CH_1,20.0);//速度試験用
	//pro_da_set_range(DA_CH_2,20.0);//速度試験用
	//pro_da_set_range(DA_CH_1, PI(1));					/* Setting range DA output level							*/
	//pro_da_set_range(DA_CH_2, PI(1));//〇後で戻す
	//pro_da_set_range(DA_CH_3, AD_RANGE);
	pro_da_set_range(DA_CH_0,15);//□あとでコメントアウト
	pro_da_set_range(DA_CH_1,15);//□あとでコメントアウト
	pro_da_set_range(DA_CH_2,15);//□あとでコメントアウト
	pro_da_set_range(DA_CH_3,15);//□あとでコメントアウト
	/*--------------------da出力(enc/hall)設定ここまで--------------------*/
	
	/*--------------------ad入力(inverter/hall)設定ここから--------------------*/
	ad_init( SINGLE_SCAN );							/*	Initializing A/D conversion module and startign the conversion on Continuous scan mode		*/
	ad_stop();										/*	Stopping A/D conversion once												 			*/

	ad_set_range(AD_GROUP_0, AD_RANGE, AD_RANGE);//21,22番ピン初期化
	ad_set_range(AD_GROUP_1, AD_RANGE, AD_RANGE);//23,24番ピン初期化
	ad_set_range(AD_GROUP_2, AD_RANGE, AD_RANGE);						//VdcはADCINA2に接続されているので、チャンネルグループ番号chg=2 レンジは-5~5
													//ADCINB2に接続されたIdcも勝手に初期化される(仕様)
	ad_set_range(AD_GROUP_3, AD_RANGE, AD_RANGE);
	ad_set_range(AD_GROUP_5, AD_RANGE, AD_RANGE);
	ad_set_range(AD_GROUP_6, AD_RANGE, AD_RANGE);//7,8番ピン初期化
	ad_set_range(AD_GROUP_7, AD_RANGE, AD_RANGE);//5,6番ピン初期化
	/*--------------------ad(inverter/hall)設定ここまで--------------------*/

	/*--------------------timer1割り込み(enc)設定ここから--------------------*/
	timer1_init( PERIOD / SAMPL );				/* Initialization of Timer0 and setting interrupt period	*/
												/* After the initialization, Timer0 is stopped,				*/
												/* and Timer0 interrupt is inactive.						*/

	timer1_init_vector( getAdStatusSensor );			/* Timer0 interrupt routine definition						*/
	timer1_start();								/* Starting Timer0 counter									*/
	timer1_enable_int();						/* Enabling Timer0 interrupt								*/
	/*--------------------timer1割り込み(enc)設定ここまで--------------------*/

	/*--------------------abz2割り込み(enc)設定ここから--------------------*/
	//abz2_init();							/* Initialization of abz2 function							*/
	//abz2_init_vector( abz2_interrupt );		/* Z interrupt routine definition							*/
	//abz2_enable_int();						/* Enabling the Z interrupt from ABZ2 port					*/
	/*--------------------abz2割り込み(enc)設定ここまで--------------------*/

	int_enable();									/* Enabling all interrupts												*/
	
	wait(100);										/* Waiting more than half of the carrier period							*/

	currentSensorInitA();
	currentSensorInitB();

	inverter0_start_pwm();
	inverter1_start_pwm();							/* Starting 3-phase PWM signal output									*/
	
	while(1)
	{
		//cnt = abz2_read();					/* Reading count of encoder									*/
		//enc_theta_raw=(FLOAT32)cnt/(FLOAT32)RESOLUSION*PI(2)-PI(1);//エンコーダ計算値をradに変換

	}												/* Repeating endlessly													*/
	/* NOTREACHED */
	return 0;
}
