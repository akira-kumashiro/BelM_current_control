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
#include	<mwio.h>								/*	Header file of I/O library for PEOS/F28335							*/
#include	<pro_f28335.h>							/* Header file for PE-PRO/F28335A										*/
#include	<pro_f28335.c>							/*	Peculiar module for PE-PRO/F28335A									*/

#define INVERTER_MODEL_A 5//Aインバータのモデル1R->1、5R->5、9R->9
#define INVERTER_MODEL_B 1//Bインバータのモデル1R->1、5R->5、9R->9
#define Decoupling_Mode2p 0//2pのdq軸干渉項を計算する(1)しない(0)
#define Decoupling_Mode4p 0//4pのdq軸干渉項を計算する(1)しない(0)

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

#define AD_RANGE 5.0 //AD変換の数値レンジ

#define	PERIOD	20000							// 1/(50Hz) = 20ms = 20000us								
#define	SAMPL	200								// Number of sampling points								
#define PERIOD2 100							//ホール素子割り込み周期[us]
#define RESOLUSION 4000							//エンコーダ分解能*4
#define Vdc_ref 1.0                                // DCリンク電圧[v]
#define Vdc_thres 20.0//電流制御するVdcの閾値
//#define NUNBER_OF_PI_START_DELAY 500//PI制御を始めるまでの待ち時間[回]
#define PERIOD_OF_PI_START_DERAY 1.0e-4//PI制御を始めるまでの待ち時間0.15[sec]
#define NUM_OF_OFFSET_DETECT 100//電流センサオフセット検出のステップ回数
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
#define dispSensorCalcA(voltage) (voltage + 4.5) * 0.2 + 3.293e-5 * (myExp(1.8234 * (voltage + 4.5)) - 1)//B10793
#define dispSensorCalcB(voltage) (voltage + 4.5) * 0.2 + 1.211e-5 * (myExp(1.959 * (voltage + 4.5)) - 1)//B10794

/*--------------------enc関連設定ここから--------------------*/
INT32 cnt = 0;//エンコーダのカウンタAB相
INT32 zero = 0;//エンコーダのZ相

FLOAT32 enc_theta_raw=0.0;//DAC出力[rad]
INT32 cnt_min = 10000;//AB相最小値
INT32 cnt_max = 0;//AB相最大値
FLOAT32 cnt_ave=0.0;//AB相平均値
FLOAT32 enc_theta_calc=0.0;//エンコーダ角度位置計算値[rad]
const FLOAT32 enc_theta_offset=(-51.246-7.488)*PI(1)/180;//エンコーダ角度オフセット値　0～2pi[rad]
/*--------------------enc関連設定ここまで--------------------*/

/*--------------------hall関連設定ここから--------------------*/
FLOAT32 hallA=0.0;//ホール素子1
FLOAT32 hallB=0.0;//ホール素子2
FLOAT32 hallC=0.0;//ホール素子3
FLOAT32 hallD=0.0;//ホール素子4

FLOAT32 hallX=0.0;//ホール素子X
FLOAT32 hallY=0.0;//ホール素子Y

FLOAT32 hallR=0.0;//ホール素子->磁束強さ
FLOAT32 hall_theta_raw=0.0;//ホール素子->回転角度[rad]
FLOAT32 hall_theta_calc=0.0;//ホール素子回転角度オフセット+高調波補正値[rad]
FLOAT32 hall_theta_calc_prev=0.0;
INT32 rotaton_num = 0;//回転回数

FLOAT32 hallErrorCompAmpPrec[HALL_COMP_ORDER_PREC], hallErrorCompPhPrec[HALL_COMP_ORDER_PREC];//ホール素子角度誤差の校正　0次～14次 振幅・角度はdeg入力でrad変換
INT32 hallErrorCompUsePrec[HALL_COMP_ORDER_PREC];//指定した次数を使う場合は1、使わない場合は0
FLOAT32 hallErrorCompAmp[HALL_COMP_ORDER], hallErrorCompPh[HALL_COMP_ORDER];//ホール素子角度誤差の校正　0次～14次 振幅・角度はdeg入力でrad変換
INT32 hallErrorCompUse[HALL_COMP_ORDER];//指定した次数を使う場合は1、使わない場合は0

/*--------------------hall関連設定ここまで--------------------*/

FLOAT32 disp_x_raw = 0.0, disp_y_raw = 0.0;//変位センサAD入力(V)
FLOAT32 disp_x = 0.0, disp_y = 0.0;//変位センサ変換後(mm)

/*--------------------電流・電圧AD入力変数ここから--------------------*/
FLOAT32 iuA_raw=0.0;//(V) (1R) Iu=+-6.25A => iuB_raw=+-5V, (5R) Iu=+-31.25A => iuB_raw=+-5V, (9R) Iu=+-50A => iuB_raw=+-5V
FLOAT32 iwA_raw=0.0;//(V) (1R) Iw=+-6.25A => iwB_raw=+-5V, (5R) Iw=+-31.25A => iwB_raw=+-5V, (9R) Iw=+-50A => iwB_raw=+-5V

FLOAT32 vdcA_raw=0.0;//(V) (1R/5R) Vdc=+500V => vdcB_raw=5V, (9R) Vdc=+400V => vdcB_raw=5V
FLOAT32 idcA_raw=0.0;//(V) (1R) Idc=+-6.25A => idcB_raw=+-5V, (5R) Idc=+-31.25A => idcB_raw=+-5V, (9R) Idc=+-50A => idcB_raw=+-5V

FLOAT32 iuB_raw=0.0;//(V) (1R) Iu=+-6.25A => iuB_raw=+-5V, (5R) Iu=+-31.25A => iuB_raw=+-5V, (9R) Iu=+-50A => iuB_raw=+-5V
FLOAT32 iwB_raw=0.0;//(V) (1R) Iw=+-6.25A => iwB_raw=+-5V, (5R) Iw=+-31.25A => iwB_raw=+-5V, (9R) Iw=+-50A => iwB_raw=+-5V

FLOAT32 vdcB_raw=0.0;//(V) (1R/5R) Vdc=+500V => vdcB_raw=5V, (9R) Vdc=+400V => vdcB_raw=5V
FLOAT32 idcB_raw=0.0;//(V) (1R) Idc=+-6.25A => idcB_raw=+-5V, (5R) Idc=+-31.25A => idcB_raw=+-5V, (9R) Idc=+-50A => idcB_raw=+-5V

const FLOAT32 voltage_ad_gainA = (INVERTER_MODEL_A == 1) ? 100.0 : ((INVERTER_MODEL_A == 5) ? 100.0 : ((INVERTER_MODEL_A == 9) ? 80.0 : 0.0));//Aインバータ電圧センサのゲイン
const FLOAT32 current_ad_gainA = (INVERTER_MODEL_A == 1) ? 1.25 : ((INVERTER_MODEL_A == 5) ? 6.25 : ((INVERTER_MODEL_A == 9) ? 10.0 : 0.0));//Aインバータ電流センサのゲイン
const FLOAT32 voltage_ad_gainB = (INVERTER_MODEL_B == 1) ? 100.0 : ((INVERTER_MODEL_B == 5) ? 100.0 : ((INVERTER_MODEL_B == 9) ? 80.0 : 0.0));//Bインバータ電圧センサのゲイン
const FLOAT32 current_ad_gainB = (INVERTER_MODEL_B == 1) ? 1.25 : ((INVERTER_MODEL_B == 5) ? 6.25 : ((INVERTER_MODEL_B == 9) ? 10.0 : 0.0));//Bインバータ電流センサのゲイン

/*--------------------電流・電圧AD入力変数ここまで--------------------*/

/*--------------------電流・電圧測定値ここから--------------------*/
/*volatile FLOAT32 vu=0.0;//(V) 1R/5Rではuv間電圧
volatile FLOAT32 vw=0.0;//(V) 1R/5RではWv間電圧*/
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
FLOAT32 idA, iqA, idB, iqB;                              							// 実測されたdq軸電流[A]
FLOAT32 vuA_ref, vvA_ref, vwA_ref;             							// 三相電圧指令値[V]
FLOAT32 vuB_ref, vvB_ref, vwB_ref;             							// 三相電圧指令値[V]

const FLOAT32 KPd2p = FtoOmega(Fcutoff) * Ld_2p * Kp_gain2p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ld
const FLOAT32 KPq2p = FtoOmega(Fcutoff) * Lq_2p * Kp_gain2p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Lq
const FLOAT32 KId2p = FtoOmega(Fcutoff) * Ra_2p / Fcarrier * Ki_gain2p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 KIq2p = FtoOmega(Fcutoff) * Ra_2p / Fcarrier * Ki_gain2p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]

const FLOAT32 KPd4p = FtoOmega(Fcutoff) * Ld_4p * Kp_gain4p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ld
const FLOAT32 KPq4p = FtoOmega(Fcutoff) * Lq_4p * Kp_gain4p;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Lq
const FLOAT32 KId4p = FtoOmega(Fcutoff) * Ra_4p / Fcarrier * Ki_gain4p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 KIq4p = FtoOmega(Fcutoff) * Ra_4p / Fcarrier * Ki_gain4p;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]

const FLOAT32 idA_max = 1.0, iqA_max = 1.0;      							// idq_max:dq軸電流の最大値[A]
const FLOAT32 idB_max = 1.0, iqB_max = 1.0;      							// idq_max:dq軸電流の最大値[A]
const INT32 max_num_of_delay = PERIOD_OF_PI_START_DERAY * Fcarrier;//I制御開始までの遅延ステップ数

volatile INT32 delay_counterA = 0, delay_counterB = 0;//I制御開始までの遅延カウンタ
volatile INT32 offest_calc_counterB = 0;//電流センサオフセット値計算のカウンタ
FLOAT32 iaA=0, ibA=0, iaB=0, ibB=0;//ab軸電流
FLOAT32 vaA=0, vbA=0, vaB=0, vbB=0;//ab軸電圧
volatile FLOAT32 vdA_max = 0.0, vqA_max = 0.0, vdB_max = 0.0, vqB_max = 0.0;//dq軸電圧の最大値[V]
volatile FLOAT32 idA_ref = 0.0, iqA_ref = 0.0, idB_ref = 0.0, iqB_ref = 0.0;// dq軸電流の指令値[A]
volatile FLOAT32 idA_err, iqA_err, idB_err, iqB_err;//dq軸電流の偏差
volatile FLOAT32 idA_Pout, idA_Iout, iqA_Pout, iqA_Iout;//dq軸電流PI出力
volatile FLOAT32 idB_Pout, idB_Iout, iqB_Pout, iqB_Iout;//dq軸電流PI出力
volatile FLOAT32 idA_Itmp = 0.0, iqA_Itmp = 0.0, idB_Itmp = 0.0, iqB_Itmp = 0.0;
volatile FLOAT32 vdA_ref, vqA_ref, vdB_ref, vqB_ref;//dq軸電圧指令値[V]
volatile FLOAT32 vodA, voqA, vodB, voqB;//dq軸干渉項の保障電圧[V]
volatile FLOAT32 vdA_limd, vqA_limd, vdB_limd, vqB_limd;// dq軸電圧指令値リミット後[V]
volatile FLOAT32 Half_VdcA = 0.0, Half_VdcB = 0.0;                            	// Half_VdcB:DCリンク電圧*1/2[V]
volatile FLOAT32 mvuA, mvvA, mvwA, mvuB, mvvB, mvwB;                       					// 変調波指令値(-1～1)
volatile FLOAT32 w_e = FtoOmega(FmodElec);          		          	// 変調波の電気角速度[rad/s]
volatile INT32 enable_I_controlA = 0, enable_PI_controlB = 0;//I制御停止用のVdcチェック変数 VdcがVdc_thresよりも大きいと1、それ以外0
volatile INT32 enable_angle_control = 0;//角度位置を指定する制御をする(1)、しない(0)
volatile FLOAT32 reference_angle = 0.0;//角度制御するときの角度位置(deg)
volatile FLOAT32 KPt = 0.8, KIt = 0.01, KDt = 0.1;//角度位置制御のPIDゲイン
volatile FLOAT32 theta_err, theta_error_prev, theta_Pout, theta_Iout, theta_Itmp, theta_Dtmp, theta_Dout, theta_ref;
/*--------------------電流制御用変数ここまで--------------------*/

//FLOAT32 v_raw_converterA(FLOAT32 v_in);//inverter_model 1R->1, 5R->5, 9R->9
//FLOAT32 i_raw_converterA(FLOAT32 c_in);//inverter_model 1R->1, 5R->5, 9R->9
FLOAT32 radianLimit(FLOAT32 rad);
void getHallValue(void);
void abz2_interrupt( void );//abz2割り込み
void getEncSawWave( void );//timer1割り込み
void hallErrorCompInit(void);
FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle);
void getAdStatusSensor(void);
void getAdStatusAinv(void);
void getAdStatusBinv(void);
void current_controlB(void);//pwm割り込み
void currentSensorInitB(void);
void currentSensorInitA(void);
FLOAT32 myExp(FLOAT32 x);
FLOAT32 myExpCalc(FLOAT32 x, INT32 n, FLOAT32 _numerator, FLOAT32 _denominator,FLOAT32 y);

#pragma INTERRUPT(current_controlB)//pwm割り込み
#pragma INTERRUPT(abz2_interrupt)//abz2割り込み
#pragma	INTERRUPT(getEncSawWave)//timer1割り込み

//AD変換した値を実際の電圧値に変更
/*FLOAT32 v_raw_converterA(FLOAT32 v_in)//inverter_model 1R->1, 9R->9
{
	switch(INVERTER_MODEL)
	{
		case 1:								//1Rと5Rは同じ
		case 5:		return (v_in*100.0);	//Vdc=+500V => vdcB_raw=5V
		case 9:		return (v_in*80.0);	//Vdc=+400V => vdcB_raw=5V
		default:	return 0.0;
	}
}*/

//AD変換した値を実際の電流値に変更
/*FLOAT32 i_raw_converterA(FLOAT32 c_in)//inverter_model 1R->1, 9R->9
{
	switch(INVERTER_MODEL)
	{
		case 1:		return (c_in*1.25);	//Idc=+-6.25A => idcB_raw=+-5V
		case 5:		return (c_in*6.25);	//Idc=+-31.25A => idcB_raw=+-5V
		case 9:		return (c_in*10.0);	//Idc=+-50A => idcB_raw=+-5V
		default:	return 0.0;
	}
}*/

FLOAT32 radianLimit(FLOAT32 rad)//-PI～PIに制限
{
	FLOAT32 rad_out=rad;

	if(mwabs(rad)>PI(1))//radの絶対値がPI以上の時
		rad_out = (rad > 0) ? radianLimit(rad-PI(2)) : radianLimit(rad+PI(2));//radが正なら2PI引く、radが負なら2PI足す
	
	return rad_out;
}

//ホール素子の入力値hallA～hallDから校正値を計算してhallOUTに出力
void getHallValue(void)
{
	hallX=(hallA-hallC)/2;//ホール素子のXを平均化で導出(AとCは理想的には2.5からの差が同じ絶対値で符号が逆)
	hallY=(hallB-hallD)/2;//ホール素子のYを平均化で導出(BとDは理想的には2.5からの差が同じ絶対値で符号が逆)

	xy2ra(hallX,hallY,&hallR,&hall_theta_raw);//直交座標を極座標に変換して回転角度を導出
	hall_theta_calc_prev=hall_theta_calc;
	hall_theta_calc=getHallErrorCalc(hall_theta_raw);
	if(mwabs(hall_theta_calc-hall_theta_calc_prev)>PI(1))
	{
		rotaton_num += ((hall_theta_calc - hall_theta_calc_prev) > 0) ? -1 : 1;
	}
	pro_da_out(DA_CH_0,hall_theta_raw);							/* Output data for viewing waveforms in DAC					*/
	pro_da_out(DA_CH_2,hall_theta_calc);
}

//エンコーダ計算
void abz2_interrupt( void )
{
	int_ack();
	//wait(1000);

	if(cnt>100)//チャタリング防止
	{
		zero++;//回転回数++
		cnt_ave=cnt_ave*(zero-1)/zero;
		cnt_ave+=cnt/zero;//平均値更新
		
		if(cnt<cnt_min)//最小値更新
			cnt_min=cnt;
		if(cnt>cnt_max)//最大値更新
			cnt_max=cnt;
	}

	abz2_clear_int_flag();					/* Clearing the interrupt flag.								*/
}

//エンコーダの角度をDACから出力
void getEncSawWave(void)
{
	int_ack();

	enc_theta_calc=radianLimit(enc_theta_raw+enc_theta_offset);//エンコーダ角度のオフセットを計算
	
	pro_da_out(DA_CH_1,enc_theta_calc);							/* Output data for viewing waveforms in DAC					*/

	timer1_clear_int_flag();					/* Clearing the interrupt flag.								*/
}

void hallErrorCompInit(void)//ホール素子校正の初期設定
{
	INT32 i;
	//hallErrorCompPhはdegで代入してradに変換
	hallErrorCompAmpPrec[0]=-103.194+7.488;	hallErrorCompPhPrec[0]=0.0;	hallErrorCompUsePrec[0]=1;//-103.194
	hallErrorCompAmpPrec[1]=0.633275735;	hallErrorCompPhPrec[1]= -117.8906118;	hallErrorCompUsePrec[1]=1;
	hallErrorCompAmpPrec[2]=3.086499659;	hallErrorCompPhPrec[2]= -81.0681271;	hallErrorCompUsePrec[2]=1;
	hallErrorCompAmpPrec[3]=0.075699977;	hallErrorCompPhPrec[3]= 23.14646234;	hallErrorCompUsePrec[3]=0;
	hallErrorCompAmpPrec[4]=4.366958749;	hallErrorCompPhPrec[4]= -81.76701926;	hallErrorCompUsePrec[4]=1;
	hallErrorCompAmpPrec[5]=0.173986398;	hallErrorCompPhPrec[5]= 107.3194347;	hallErrorCompUsePrec[5]=0;
	hallErrorCompAmpPrec[6]=0.521664715;	hallErrorCompPhPrec[6]= 117.9616102;	hallErrorCompUsePrec[6]=1;
	hallErrorCompAmpPrec[7]=0.027259279;	hallErrorCompPhPrec[7]= 53.82444327;	hallErrorCompUsePrec[7]=0;
	hallErrorCompAmpPrec[8]=0.583975898;	hallErrorCompPhPrec[8]= 120.4875918;	hallErrorCompUsePrec[8]=1;
	hallErrorCompAmpPrec[9]=0.066215787;	hallErrorCompPhPrec[9]= -61.50186586;	hallErrorCompUsePrec[9]=0;
	hallErrorCompAmpPrec[10]=0.151485583;	hallErrorCompPhPrec[10]= -31.82436231;	hallErrorCompUsePrec[10]=0;
	hallErrorCompAmpPrec[11]=0.015517434;	hallErrorCompPhPrec[11]= -152.0030841;	hallErrorCompUsePrec[11]=0;
	hallErrorCompAmpPrec[12]=0.120998567;	hallErrorCompPhPrec[12]= -38.31503892;	hallErrorCompUsePrec[12]=0;
	hallErrorCompAmpPrec[13]=0.023401454;	hallErrorCompPhPrec[13]= 130.660386;	hallErrorCompUsePrec[13]=0;
	hallErrorCompAmpPrec[14]=0.052686297;	hallErrorCompPhPrec[14]= 174.0505764;	hallErrorCompUsePrec[14]=0;

	for(i=0;i<HALL_COMP_ORDER;i++)
	{
		hallErrorCompAmp[i]=hallErrorCompAmpPrec[i]*PI(1)/180.0;
		hallErrorCompPh[i]=hallErrorCompPhPrec[i]*PI(1)/180.0;//radに変換
		hallErrorCompUse[i]=hallErrorCompUsePrec[i];
	}
}

FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle)//hallRawAngleはrad
{
	FLOAT32 hallCalcAngle=hallRawAngle;
	INT32 i;
	for(i=0;i<HALL_COMP_ORDER;i++)//フーリエ変換したもので真値を計算
	{
		if(hallErrorCompUse[i])//無視する次数じゃないとき
			hallCalcAngle-=hallErrorCompAmp[i]*mwcos((FLOAT32)i*hallRawAngle+hallErrorCompPh[i]);
	}
	
	return radianLimit(hallCalcAngle);
}

//AD変換した値を取得して実際の値に変換して格納
void getAdStatusSensor(void)
{
	/* AD変換結果の取得 */
	ad_start(SEQ1, AD_GROUP_6);							/*	Starting A/D conversion of channel group6 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &hallA, &hallB );				/*	Reading A/D conversion output data of channel group6										*/

	ad_start(SEQ2, AD_GROUP_7);							/*	Starting A/D conversion of channel group7 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ2, 0, &hallC, &hallD );				/*	Reading A/D conversion output data of channel group7										*/
	
	getHallValue();//ホール素子で角度位置取得

	ad_start(SEQ1, AD_GROUP_5);							/*	Starting A/D conversion of channel group7 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &disp_y_raw, &disp_x_raw );				/*	Reading A/D conversion output data of channel group7										*/

	disp_y = dispSensorCalcA(disp_y_raw);
	disp_x = dispSensorCalcB(disp_x_raw);
}

void getAdStatusAinv(void)
{
	ad_start(SEQ2, AD_GROUP_1);							/*	Starting A/D conversion of channel group1 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ2, 0, &iuA_raw, &iwA_raw );				/*	Reading A/D conversion output data of channel group1										*/
	iuA=i_raw_converterA(iuA_raw);
	iwA=i_raw_converterA(iwA_raw);

	ad_start(SEQ1, AD_GROUP_2);							/*	Starting A/D conversion of channel group2 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &vdcA_raw, &idcA_raw );				/*	Reading A/D conversion output data of channel group2										*/
	vdcA=(vdcA_raw>=0.0)?v_raw_converterA(vdcA_raw):0.0;
	idcA=i_raw_converterA(idcA_raw);

	getAdStatusSensor();
}

void getAdStatusBinv(void)
{
	ad_start(SEQ1, AD_GROUP_0);							//	Starting A/D conversion of channel group0 and setting it to  SEQ1							
	while(ad_in_st() != 0){}						//	Checking status of A/D converter device and waiting for completing A/D conversion 	
	ad_in( SEQ1, 0, &iuB_raw, &iwB_raw );				//	Reading A/D conversion output data of channel group0										
	iuB=i_raw_converterB(iuB_raw);
	iwB=i_raw_converterB(iwB_raw);

	ad_start(SEQ2, AD_GROUP_3);							/*	Starting A/D conversion of channel group2 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ2, 0, &vdcB_raw, &idcB_raw );				/*	Reading A/D conversion output data of channel group2										*/
	vdcB=(vdcB_raw>=0.0)?v_raw_converterB(vdcB_raw):0.0;
	idcB=i_raw_converterB(idcB_raw);

	getAdStatusSensor();
}

// キャリア同期割り込みで，dq軸電流をPI制御する関数
void current_controlB(void) 
{
    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    getAdStatusBinv();//インバータから電流電圧取得
	Half_VdcB = (vdcB >= 0.0) ? vdcB / 2 : 0.0;//DCリンク電圧を更新
	vdB_max = ROOT3 / ROOT2 * Half_VdcB;//dq軸電圧最大値を計算
	vqB_max = ROOT3 / ROOT2 * Half_VdcB;

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
			//PI制御の計算値を初期化
			idB_Itmp = 0.0;
			iqB_Itmp = 0.0;
			idB_Pout = 0.0;
			iqB_Pout = 0.0;
			theta_err = 0.0;
			theta_Iout = 0.0;
			theta_Itmp = 0.0;
			theta_Pout = 0.0;
			theta_error_prev = 0.0;
			theta_Dout = 0.0;
			theta_Dtmp = 0.0;
			rotaton_num = 0;
		}
		else
		{
			delay_counterB--;//カウンタ減少
		}
	}

    // iuB, iwB → idB, iqB (絶対変換)
    uw2ab(iuB, iwB, &iaB, &ibB);
	ab2dq(iaB, ibB, hall_theta_calc, &idB, &iqB);

	if(enable_angle_control == 1)
	{
		theta_error_prev = theta_err;
		// PI制御 d軸
		theta_ref = radianLimit(reference_angle * PI(1.0) / 180.0);
		// d軸電流 P制御
		theta_err = theta_ref - (hall_theta_calc + rotaton_num * PI(2));
		theta_Dtmp = theta_err - theta_error_prev;
		/*if(mwabs(theta_Dtmp) > PI(1))
		{
			//theta_err += (theta_Dtmp > 0) ? -PI(2) : PI(2);
			theta_Dtmp += (theta_Dtmp > 0) ? -PI(2) : PI(2);
		}*/
		theta_Pout = KPt * theta_err * (FLOAT32)enable_angle_control;
		// d軸電流 I制御
		theta_Itmp += theta_err * (FLOAT32)enable_angle_control / Fcutoff * 10;
		theta_Itmp = mylimit(theta_Itmp, 30000.0);
		theta_Iout = KIt * theta_Itmp;

		theta_Dout = theta_Dtmp * KDt * Fcutoff / 10;

		iqB_ref = mylimit(theta_Iout + theta_Pout + theta_Dout,0.5);
		//idB_ref = mylimit(1/mwabs(theta_err),1.0);
	}
	else
	{
		theta_err = 0.0;
		theta_Iout = 0.0;
		theta_Itmp = 0.0;
		theta_Pout = 0.0;
		theta_error_prev = 0.0;
		theta_Dout = 0.0;
		theta_Dtmp = 0.0;
	}


    // PI制御 d軸
    idB_ref = mylimit(idB_ref, idB_max);
    // d軸電流 P制御
    idB_err = idB_ref - idB;
    idB_Pout = KPd2p * idB_err * (FLOAT32)enable_PI_controlB;
    // d軸電流 I制御
    idB_Itmp += idB_err * (FLOAT32)enable_PI_controlB;
    idB_Itmp = mylimit(idB_Itmp, 30000.0);
    idB_Iout = KId2p * idB_Itmp;

    // PI制御 q軸
    iqB_ref = mylimit(iqB_ref, iqB_max);
    // q軸電流 P制御
    iqB_err = iqB_ref - iqB;
    iqB_Pout = KPq2p * iqB_err * (FLOAT32)enable_PI_controlB;
    // q軸電流 I制御
    iqB_Itmp += iqB_err * (FLOAT32)enable_PI_controlB;
    iqB_Itmp = mylimit(iqB_Itmp, 30000.0);
    iqB_Iout = KIq2p * iqB_Itmp;

    // dq軸干渉項の計算
    vodB = -w_e * Lq_2p * iqB;
    voqB = w_e * (PSY_a + Ld_2p * idB);

    // dq軸電圧指令値を計算
	vdB_ref = idB_Pout + idB_Iout + vodB * Decoupling_Mode2p;
	vqB_ref = iqB_Pout + iqB_Iout + voqB * Decoupling_Mode2p;

    vdB_limd = mylimit(vdB_ref, vdB_max);
    vqB_limd = mylimit(vqB_ref, vqB_max);

    // uvw電圧を出力
    // vd, vq → vu, vv, vw (絶対変換)
	dq2ab(vdB_limd, vqB_limd, hall_theta_calc, &vaB, &vbB);
	ab2uvw(vaB, vbB, &vuB_ref, &vvB_ref, &vwB_ref);

	//三相電圧指令値を正規化(-1～1)
    mvuB = mylimit(vuB_ref / Half_VdcB, 1.0);
    mvvB = mylimit(vvB_ref / Half_VdcB, 1.0);
    mvwB = mylimit(vwB_ref / Half_VdcB, 1.0);

	inverter1_set_uvw( mvuB, mvvB, mvwB );//インバータ出力
	
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter1_clear_up_flag();						// Clearing the interrupt flag.											
}

void currentSensorInitB(void)
{
	INT32 i;
	const INT32 waitTime = (INT32)(1.0/Fcarrier*10e+6);//usec

	getAdStatusBinv();

	for(i=0;i<NUM_OF_OFFSET_DETECT;i++)
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

	for(i=0;i<NUM_OF_OFFSET_DETECT;i++)
	{
		iuA_offset += iuA / (FLOAT32)NUM_OF_OFFSET_DETECT;//オフセット値の平均値を積算
		iwA_offset += iwA / (FLOAT32)NUM_OF_OFFSET_DETECT;
		enable_I_controlA = 0;//I制御停止
	
		inverter0_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力停止

		wait(waitTime);
	}
}

FLOAT32 myExp(FLOAT32 x)//指数関数の計算exp(x)
{
	return myExpCalc(x, 1, 1.0, 1.0, 1.0);
}
FLOAT32 myExpCalc(FLOAT32 x, INT32 n, FLOAT32 _numerator, FLOAT32 _denominator, FLOAT32 y)
{
	FLOAT32 denominator = _denominator * (FLOAT32)n;
	FLOAT32 numerator = _numerator * x;
	FLOAT32 a = numerator / denominator;
	if(mwabs(a) <= (FLOAT32)1.0e-5)//最大誤差
		return y;
	else
		return (y + myExpCalc(x, ++n, numerator, denominator, a));
}

int main( void )
{
	int_disable();									/* Disabling all interrupt												*/

	hallErrorCompInit();

	system_init();
	#if !defined(F28335LMT)
	watch_init();
	#endif

	inverter1_init( Fcarrier, DEADTIME );					/* Setting carrier frequency and dead time		 						*/
	inverter1_set_uvw( 0.0, 0.0, 0.0 );				/* Setting initial PWM references										*/
	inverter1_init_up_vector( current_controlB );	/* Carrier interrupt routine definition									*/
	inverter1_enable_up_int();						/* Enabling carrier interrupt at the top point of the carrier mountain	*/
	
	/*--------------------da出力(enc/hall)設定ここから--------------------*/
	pro_da_init();								/* Initialization of DA converter							*/
	pro_da_set_range(DA_CH_0, PI(1));//DACの正規化レンジは-pi->piに設定
	pro_da_set_range(DA_CH_1, PI(1));					/* Setting range DA output level							*/
	pro_da_set_range(DA_CH_2, PI(1));
	pro_da_set_range(DA_CH_3, AD_RANGE);
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

	timer1_init_vector( getEncSawWave );			/* Timer0 interrupt routine definition						*/
	timer1_start();								/* Starting Timer0 counter									*/
	timer1_enable_int();						/* Enabling Timer0 interrupt								*/
	/*--------------------timer1割り込み(enc)設定ここまで--------------------*/

	/*--------------------abz2割り込み(enc)設定ここから--------------------*/
	abz2_init();							/* Initialization of abz2 function							*/
	abz2_init_vector( abz2_interrupt );		/* Z interrupt routine definition							*/
	abz2_enable_int();						/* Enabling the Z interrupt from ABZ2 port					*/
	/*--------------------abz2割り込み(enc)設定ここまで--------------------*/

	int_enable();									/* Enabling all interrupts												*/
	
	wait(100);										/* Waiting more than half of the carrier period							*/

	currentSensorInitB();

	inverter1_start_pwm();							/* Starting 3-phase PWM signal output									*/
	
	while(1)
	{
		cnt = abz2_read();					/* Reading count of encoder									*/
		enc_theta_raw=(FLOAT32)cnt/(FLOAT32)RESOLUSION*PI(2)-PI(1);//エンコーダ計算値をradに変換

	}												/* Repeating endlessly													*/
	/* NOTREACHED */
	return 0;
}
