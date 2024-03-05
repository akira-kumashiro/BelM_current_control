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

#define dispSensorCalcA(voltage) (voltage) * 0.2 + 3.293e-5 * (myExp(1.8234 * (voltage)) - 1)//B10793
#define dispSensorCalcB(voltage) (voltage) * 0.2 + 1.211e-5 * (myExp(1.959 * (voltage)) - 1)//B10794

#define AD_RANGE 5.0 //AD変換の数値レンジ

#define	PERIOD	20000							// 1/(50Hz) = 20ms = 20000us								
#define	SAMPL	200								// Number of sampling points								
#define PERIOD2 100							//ホール素子割り込み周期[us]
#define RESOLUSION 4000							//エンコーダ分解能*4
#define Vdc_ref 1.0                                // DCリンク電圧[v]
#define Vdc_thres 20.0//電流制御するVdcの閾値#define PERIOD_OF_PI_START_DERAY 1.0e-4//PI制御を始めるまでの待ち時間0.15[sec]
#define NUM_OF_OFFSET_DETECT 100//電流センサオフセット検出のステップ回数
#define PERIOD_OF_PI_START_DERAY 1.0e-4//PI制御を始めるまでの待ち時間0.15[sec]
#define HALL_COMP_ORDER_PREC 15//ホール素子校正の高調波次数
#define HALL_COMP_ORDER 9//ホール素子校正の高調波次数
#define HALL_CUTOFF_FREQ 1000//ホール素子計算値のLPFカットオフ周波数

#define FmodElec 13.0//変調波の周波数(電気)Fe[Hz]
#define Fcutoff 1000.0//電流制御の閉ループカットオフ周波数Fc[Hz]
#define Fcarrier 20000.0//キャリア周波数Fs[Hz]
#define	DEADTIME	4000							// Dead time [ns]	0<=deadtime<=24000	これ以上小さくしない！

/*--------------------enc関連設定ここから--------------------
INT32 cnt = 0;//エンコーダのカウンタAB相
INT32 zero = 0;//エンコーダのZ相

FLOAT32 enc_theta_raw=0.0;//DAC出力[rad]
INT32 cnt_min = 10000;//AB相最小値
INT32 cnt_max = 0;//AB相最大値
FLOAT32 cnt_ave=0.0;//AB相平均値
FLOAT32 enc_theta_calc=0.0;//エンコーダ角度位置計算値[rad]
const FLOAT32 enc_theta_offset=(-51.246-7.488)*PI(1)/180;//エンコーダ角度オフセット値　0～2pi[rad]
--------------------enc関連設定ここまで--------------------*/

/*--------------------hall関連設定ここから--------------------*/
FLOAT32 hall_theta_raw=0.0;//ホール素子->回転角度[rad]
FLOAT32 hall_theta_calc=0.0;//ホール素子回転角度オフセット+高調波補正値[rad]
FLOAT32 hall_theta_calc_prev=0.0;
INT32 rotaton_num = 0;//回転回数

FLOAT32 hallErrorCompAmp[HALL_COMP_ORDER], hallErrorCompPh[HALL_COMP_ORDER];//ホール素子角度誤差の校正　0次～14次 振幅・角度はdeg入力でrad変換
INT32 hallErrorCompUse[HALL_COMP_ORDER];//指定した次数を使う場合は1、使わない場合は0

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
volatile FLOAT32 vdA_max = 0.0, vqA_max = 0.0, vdB_max = 0.0, vqB_max = 0.0;//dq軸電圧の最大値[V]
volatile FLOAT32 Half_VdcA = 0.0, Half_VdcB = 0.0;                            	// Half_VdcB:DCリンク電圧*1/2[V]

const INT32 max_num_of_delay = PERIOD_OF_PI_START_DERAY * Fcarrier;//I制御開始までの遅延ステップ数

volatile INT32 delay_counterA = 0, delay_counterB = 0;//I制御開始までの遅延カウンタ
volatile INT32 offest_calc_counterB = 0;//電流センサオフセット値計算のカウンタ
/*--------------------電流制御用変数ここまで--------------------*/

//void abz2_interrupt( void );//abz2割り込み
//void getEncSawWave( void );//timer1割り込み
void hallErrorCompInit(void);
FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle);
void getAdStatusSensor(void);
void getAdStatusAinv(void);
void getAdStatusBinv(void);
void current_controlA(void);//pwm割り込み
void current_controlB(void);//pwm割り込み
void currentSensorInitB(void);
void currentSensorInitA(void);

#pragma INTERRUPT(current_controlA)//pwm割り込み
#pragma INTERRUPT(current_controlB)//pwm割り込み
//#pragma INTERRUPT(abz2_interrupt)//abz2割り込み
//#pragma	INTERRUPT(getEncSawWave)//timer1割り込み

//エンコーダ計算
/*void abz2_interrupt( void )
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

	abz2_clear_int_flag();					// Clearing the interrupt flag.								
}

//エンコーダの角度をDACから出力
void getEncSawWave(void)
{
	int_ack();

	enc_theta_calc=radianLimit(enc_theta_raw+enc_theta_offset);//エンコーダ角度のオフセットを計算
	
	pro_da_out(DA_CH_1,enc_theta_calc);							// Output data for viewing waveforms in DAC					

	timer1_clear_int_flag();					// Clearing the interrupt flag.								
}*/

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
	FLOAT32 _disp_x_raw = -4.5, _disp_y_raw = -4.5;
	/* AD変換結果の取得 */
	adInput(SEQ1, AD_GROUP_6, &_hallA, &_hallB);
	adInput(SEQ2, AD_GROUP_7, &_hallC, &_hallD);
	adInput(SEQ1, AD_GROUP_5, &_disp_y_raw, &_disp_x_raw);

	//getHallRawValue(_hallA, _hallB, _hallC, _hallD);//ホール素子で角度位置取得


	if(mwabs(_disp_x_raw) >= 5.0)
		_disp_x_raw = 0.0;
	if(mwabs(_disp_y_raw) >= 5.0)
		_disp_y_raw = 0.0;

	_disp_x_raw += 4.5;
	_disp_y_raw += 4.5;

	//disp_x = _disp_x_raw * 0.2;
	//disp_y = _disp_y_raw * 0.2;
	disp_y = dispSensorCalcA(_disp_y_raw);
	disp_x = dispSensorCalcB(_disp_x_raw);

	hall_theta_raw = getHallRawValue(_hallA, _hallB, _hallC, _hallD);
	hall_theta_calc_prev = hall_theta_calc;
	hall_theta_calc = getHallErrorCalc(hall_theta_raw);
	if(mwabs(hall_theta_calc - hall_theta_calc_prev) > PI(1))
	{
		rotaton_num += ((hall_theta_calc - hall_theta_calc_prev) > 0) ? -1 : 1;
	}
	pro_da_out(DA_CH_0, hall_theta_raw);							// Output data for viewing waveforms in DAC					
	pro_da_out(DA_CH_2, hall_theta_calc);
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

	getAdStatusSensor();

	Half_VdcA = (vdcA >= 0.0) ? vdcA / 2 : 0.0;//DCリンク電圧を更新
	vdA_max = ROOT3 / ROOT2 * Half_VdcA;//dq軸電圧最大値を計算
	vqA_max = ROOT3 / ROOT2 * Half_VdcA;
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

	getAdStatusSensor();
	
	Half_VdcB = (vdcB >= 0.0) ? vdcB / 2 : 0.0;//DCリンク電圧を更新
	vdB_max = ROOT3 / ROOT2 * Half_VdcB;//dq軸電圧最大値を計算
	vqB_max = ROOT3 / ROOT2 * Half_VdcB;
}

// キャリア同期割り込みで，dq軸電流をPI制御する関数
void current_controlA(void) 
{
    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    getAdStatusAinv();//インバータから電流電圧取得

	//オフセット値を減算
	iuA -= iuA_offset;
	iwA -= iwA_offset;

	if( vdcA > Vdc_thres )//閾値電圧よりも大きい場合
	{		
		if(delay_counterA >= max_num_of_delay)//カウンタが規定回数を超えた場合
		{
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
			rotaton_num = 0;
		}
		else
		{
			delay_counterA--;//カウンタ減少
		}
	}

    /*// iuB, iwB → idB, iqB (絶対変換)
    uw2ab(iuA, iwA, &iaA, &ibA);
	ab2dq(iaA, ibA, hall_theta_calc, &idA, &iqA);

    // PI制御 d軸
    idA_ref = mylimit(idA_ref, idA_max);
    // d軸電流 P制御
    idA_err = idA_ref - idA;
    idA_Pout = KPd4p * idA_err * (FLOAT32)enable_PI_controlA;
    // d軸電流 I制御
    idA_Itmp += idA_err * (FLOAT32)enable_PI_controlA;
    idA_Itmp = mylimit(idA_Itmp, 30000.0);
    idA_Iout = KId4p * idA_Itmp;

    // PI制御 q軸
    iqA_ref = mylimit(iqA_ref, iqA_max);
    // q軸電流 P制御
    iqA_err = iqA_ref - iqA;
    iqA_Pout = KPq4p * iqA_err * (FLOAT32)enable_PI_controlA;
    // q軸電流 I制御
    iqA_Itmp += iqA_err * (FLOAT32)enable_PI_controlA;
    iqA_Itmp = mylimit(iqA_Itmp, 30000.0);
    iqA_Iout = KIq4p * iqA_Itmp;

    // dq軸干渉項の計算
    vodA = -w_e * Lq_4p * iqA;
    voqA = w_e * (PSY_a + Ld_4p * idA);

    // dq軸電圧指令値を計算
	vdA_ref = idA_Pout + idA_Iout + vodA * Decoupling_Mode4p;
	vqA_ref = iqA_Pout + iqA_Iout + voqA * Decoupling_Mode4p;

    vdA_limd = mylimit(vdA_ref, vdA_max);
    vqA_limd = mylimit(vqA_ref, vqA_max);

    // uvw電圧を出力
    // vd, vq → vu, vv, vw (絶対変換)
	dq2ab(vdA_limd, vqA_limd, hall_theta_calc, &vaA, &vbA);
	ab2uvw(vaA, vbA, &vuA_ref, &vvA_ref, &vwA_ref);

	//三相電圧指令値を正規化(-1～1)
    mvuA = mylimit(vuA_ref / Half_VdcA, 1.0);
    mvvA = mylimit(vvA_ref / Half_VdcA, 1.0);
    mvwA = mylimit(vwA_ref / Half_VdcA, 1.0);
*/
	inverter0_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力
	
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter0_clear_up_flag();						// Clearing the interrupt flag.											
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
			rotaton_num = 0;
		}
		else
		{
			delay_counterB--;//カウンタ減少
		}
	}

    // iuB, iwB → idB, iqB (絶対変換)
    /*uw2ab(iuB, iwB, &iaB, &ibB);
	ab2dq(iaB, ibB, hall_theta_calc, &idB, &iqB);

	if(enable_angle_control)
		positionControlB();
	else
		setBThetaGeinZero();

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
*/
	inverter1_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力
	
	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter1_clear_up_flag();						// Clearing the interrupt flag.											
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
	
		inverter0_set_uvw( 0.0, 0.0, 0.0 );//インバータ出力停止

		wait(waitTime);
	}
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
	//timer1_init( PERIOD / SAMPL );				/* Initialization of Timer0 and setting interrupt period	*/
												/* After the initialization, Timer0 is stopped,				*/
												/* and Timer0 interrupt is inactive.						*/

	//timer1_init_vector( getEncSawWave );			/* Timer0 interrupt routine definition						*/
	//timer1_start();								/* Starting Timer0 counter									*/
	//timer1_enable_int();						/* Enabling Timer0 interrupt								*/
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
