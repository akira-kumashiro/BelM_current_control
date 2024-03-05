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

#define INVERTER_MODEL 1//インバータのモデル1R->1、5R->5、9R->9
#define Decoupling_Mode 0//dq軸干渉項を計算する(1)しない(0)

//#define	FS			10000							/* Carrier frequency [Hz]	FS>=1145									*/
#define	DEADTIME	4000							/* Dead time [ns]	0<=deadtime<=24000	これ以上小さくしない！			*/
//#define	SAMPL		200								/* Number of sampling points											*/
//#define FREQ		100								//変調波周波数(Hz)

#define AD_GROUP_0 0//AD変換グループ0
#define AD_GROUP_1 1//AD変換グループ1
#define AD_GROUP_2 2//AD変換グループ2
#define AD_GROUP_3 3//AD変換グループ3
#define AD_GROUP_4 4//AD変換グループ4
#define AD_GROUP_5 5//AD変換グループ5
#define AD_GROUP_6 6//AD変換グループ6
#define AD_GROUP_7 7//AD変換グループ7

#define SEQ1 1//シーケンサ1
#define SEQ2 2//シーケンサ2
#define CASCADE_SEQ 3//カスケードシーケンサ

#define SINGLE_SCAN 0
#define CONTINUOUS_SCAN 1
#define ONE_CYCLE_SCAN 3

#define AD_RANGE 5.0 //AD変換の数値レンジ

#define	PERIOD	20000							/* 1/(50Hz) = 20ms = 20000us								*/
#define	SAMPL	200								/* Number of sampling points								*/
#define PERIOD2 100							//ホール素子割り込み周期[us]
#define RESOLUSION 4000							//エンコーダ分解能*4
#define Vdc_ref 0.0                                // DCリンク電圧[v]
#define HALL_COMP_ORDER 15//ホール素子校正の高調波次数

#define PolePairs 1.0//極対数
#define PSY_a 0.02976//PSYa[Wb] 測定値更新済み
#define Res_a 0.443//R:相抵抗[Ω] //2p 測定値更新済み
#define Ld_axis 0.001066// Ld:d軸インダクタンス[H]
#define Lq_axis 0.001066// Lq:q軸インダクタンス[H]
#define FmodElec 13.0//変調波の周波数(電気)Fe[Hz]
#define Fcutoff 1000.0//電流制御の閉ループカットオフ周波数Fc[Hz]
#define Fcarrier 20000.0//キャリア周波数Fs[Hz]

#define	PI(n)	(3.14159265358979 * (n))
#define ROOT2 1.414213562373095
#define ROOT3 1.732050807568877
#define mylimit(data,limit) ((data) > (limit)) ? (limit) : ((data) < -(limit) ? -(limit) : (data))//dataを±limitに制限
#define FtoOmega(theta) (6.28318530717958 * (theta))//ω=2πf
#define TtoOmega(T) (6.28318530717958 / (T))//ω=2π/T

/*
struct _uvw
{
	FLOAT32 u;
	FLOAT32 v;
	FLOAT32 w;
};
typedef struct _uvw FLOAT32_UVW;

struct _dq
{
	FLOAT32 d;
	FLOAT32 q;
};
typedef struct _dq FLOAT32_DQ;

struct _ab
{
	FLOAT32 a;
	FLOAT32 b;
};
typedef struct _ab FLOAT32_AB;
*/

/*--------------------enc関連設定ここから--------------------*/
INT32 cnt = 0;//エンコーダのカウンタAB相
INT32 zero = 0;//エンコーダのZ相

FLOAT32 enc_theta_raw=0.0;//DAC出力[rad]
INT32 cnt_min = 10000;//AB相最小値
INT32 cnt_max = 0;//AB相最大値
FLOAT32 cnt_ave=0.0;//AB相平均値
FLOAT32 enc_theta_calc=0.0;//エンコーダ角度位置計算値[rad]
const FLOAT32 enc_theta_offset=-51.246*PI(1)/180;//エンコーダ角度オフセット値　0～2pi[rad]
/*--------------------enc関連設定ここまで--------------------*/

/*--------------------hall関連設定ここから--------------------*/
FLOAT32 hallA=0.0;//ホール素子1
FLOAT32 hallB=0.0;//ホール素子2
FLOAT32 hallC=0.0;//ホール素子3
FLOAT32 hallD=0.0;//ホール素子4

FLOAT32 hallX=0.0;//ホール素子X
FLOAT32 hallY=0.0;//ホール素子Y

FLOAT32 hallR=0.0;//ホール素子->磁束強さ
FLOAT32 hallT=0.0;//ホール素子->回転角度[rad]
FLOAT32 hallOUT=0.0;//ホール素子回転角度オフセット+高調波補正値[rad]

FLOAT32 hallErrorCompAmp[HALL_COMP_ORDER], hallErrorCompPh[HALL_COMP_ORDER];//ホール素子角度誤差の校正　0次～14次 振幅・角度はdeg入力でrad変換
INT32 hallErrorCompUse[HALL_COMP_ORDER];//指定した次数を使う場合は1、使わない場合は0
/*--------------------hall関連設定ここまで--------------------*/

//FLOAT32 vref=10;//(V) rmsではないことに注意

/*--------------------PWM用変数ここから--------------------*/
//FLOAT32			u	= 0.0;
//FLOAT32			v	= 0.0;
//FLOAT32			w	= 0.0;
//FLOAT32			wt	= 0.0;
//FLOAT32			dwt	= PI(2.0)/(Fcarrier/FmodElec);// / SAMPL;
//FLOAT32			m	= 1.0;//最大duty比
/*--------------------PWM用変数ここまで--------------------*/

/*--------------------電流・電圧AD入力変数ここから--------------------*/
FLOAT32 vu_raw=0.0;//(V) (1R/5R) Vuv=+-500V => vu_raw=+-5V, (9R) Vu=+400V => vu_raw=+-5V
FLOAT32 vw_raw=0.0;//(V) (1R/5R) Vuv=+-500V => vu_raw=+-5V, (9R) Vu=+400V => vu_raw=+-5V

FLOAT32 iu_raw=0.0;//(V) (1R) Iu=+-6.25A => iu_raw=+-5V, (5R) Iu=+-31.25A => iu_raw=+-5V, (9R) Iu=+-50A => iu_raw=+-5V
FLOAT32 iw_raw=0.0;//(V) (1R) Iw=+-6.25A => iw_raw=+-5V, (5R) Iw=+-31.25A => iw_raw=+-5V, (9R) Iw=+-50A => iw_raw=+-5V

FLOAT32 vdc_raw=0.0;//(V) (1R/5R) Vdc=+500V => vdc_raw=5V, (9R) Vdc=+400V => vdc_raw=5V
FLOAT32 idc_raw=0.0;//(V) (1R) Idc=+-6.25A => idc_raw=+-5V, (5R) Idc=+-31.25A => idc_raw=+-5V, (9R) Idc=+-50A => idc_raw=+-5V
/*--------------------電流・電圧AD入力変数ここまで--------------------*/

/*--------------------電流・電圧測定値ここから--------------------*/
volatile FLOAT32 vu=0.0;//(V) 1R/5Rではuv間電圧
volatile FLOAT32 vw=0.0;//(V) 1R/5RではWv間電圧

volatile FLOAT32 iu=0.0;//(A)
volatile FLOAT32 iw=0.0;//(A)

volatile FLOAT32 vdc=0.0;//(V)
volatile FLOAT32 idc=0.0;//(A)
/*--------------------電流・電圧測定値ここまで--------------------*/

/*--------------------電流制御用変数ここから--------------------*/
FLOAT32 id, iq;                              							// 実測されたdq軸電流[A]
FLOAT32 vu_ref, vv_ref, vw_ref;             							// 三相電圧指令値[V]
FLOAT32 u = 0, v = 0, w = 0;

const FLOAT32 KPd = FtoOmega(Fcutoff) * Ld_axis;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ld
const FLOAT32 KId = FtoOmega(Fcutoff) * Res_a / Fcarrier;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 KPq = FtoOmega(Fcutoff) * Lq_axis;						// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Lq
const FLOAT32 KIq = FtoOmega(Fcutoff) * Res_a / Fcarrier;				// dq軸電流制御のゲイン カットオフ角速度[rad/s] x Ra x サンプリング周期[s]
const FLOAT32 vd_lim = ROOT3 / ROOT2 * Vdc_ref / 2;						//vd_lim:d軸電圧の最大値[V]
const FLOAT32 vq_lim = ROOT3 / ROOT2 * Vdc_ref / 2;						//vq_lim:q軸電圧の最大値[V]
//const FLOAT32_DQ v_lim = { ROOT3 / ROOT2 * Vdc_ref / 2, ROOT3 / ROOT2 * Vdc_ref / 2};

const FLOAT32 id_lim = 1, iq_lim = 1;      							// idq_lim:dq軸電流の最大値[A]

volatile FLOAT32 id_ref = 0.1, iq_ref = 0;     							// dq軸電流の指令値[A]
volatile FLOAT32 id_ref_lim, iq_ref_lim;
volatile FLOAT32 id_err, iq_err;
volatile FLOAT32 id_Pout, id_Iout, iq_Pout, iq_Iout;
volatile FLOAT32 id_Itmp, iq_Itmp;
volatile FLOAT32 vd_ref, vq_ref, vod, voq;            					// dq軸電圧指令値[V], dq軸干渉項の保障電圧[V]
volatile FLOAT32 vd_ref_lim, vq_ref_lim;
volatile FLOAT32 Half_Vdc = Vdc_ref / 2;                            	// Half_Vdc:DCリンク電圧*1/2[V]
volatile FLOAT32 mvu, mvv, mvw;                       					// 変調波指令値(-1～1)
volatile FLOAT32 mvu_lim, mvv_lim, mvw_lim;
volatile FLOAT32 w_e = FtoOmega(FmodElec);          		          	// 変調波の電気角速度[rad/s]
//volatile float theta_offset;                      					// u相巻線軸・d軸が重なる位置とz相との誤差角[deg](ここはdegなので注意!!)

// 三角波(キャリア)
//volatile FLOAT32 Ts=1.0 / Fcarrier;                              //サンプリング周期[s]
//volatile FLOAT32 t;                                   // t:初回割り込み時からの経過時間[s]
/*--------------------電流制御用変数ここまで--------------------*/

//void interrupt_up_inv0(void);
FLOAT32 v_raw_converter(FLOAT32 v_in);//inverter_model 1R->1, 5R->5, 9R->9
FLOAT32 i_raw_converter(FLOAT32 c_in);//inverter_model 1R->1, 5R->5, 9R->9
FLOAT32 radianLimit(FLOAT32 rad);
//void getHallValueInt(void);//timer0割り込み
void getHallValue(void);
void abz2_interrupt( void );//abz2割り込み
void getEncSawWave( void );//timer1割り込み
void hallErrorCompInit(void);
FLOAT32 getHallErrorCalc(FLOAT32 hallRawAngle);
void current_control(void);//pwm割り込み

//#pragma	INTERRUPT( interrupt_up_inv0 )
#pragma INTERRUPT(current_control)//pwm割り込み
//#pragma	INTERRUPT(getHallValue)//timer0割り込み
#pragma INTERRUPT(abz2_interrupt)//abz2割り込み
#pragma	INTERRUPT(getEncSawWave)//timer1割り込み

/*void interrupt_up_inv0(void)
{
	int_ack();

	wt += dwt;										// Phase angle															

	if (wt > PI(1.0)) { wt -= PI(2.0); }			// When the phase angle reaches pi, it is turned to be -pi				
	
	if(vdc<=0.0){m=0.0;}							//vdc計算値がおかしいとき、最大duty比=0
	else{m=vref/vdc*(2.0/ROOT3);}					//PWM出力はduty比-1~1なのでDC電圧Vdcと指令電圧vrefからduty比の最大値を計算
													//注意：最大AC電圧(対電源側中性点・瞬時値)はVdc/2
													//線間実効電圧=(2/3)^(1/2)*Vdc*duty

	if(m>1.0){m=1.0;}								//vdc/2<vrefのとき、最大duty比が1以内になるように正規化

	u = m*mwsin(wt + PI(2.0 / 3.0));				// Generation of 3-phase sine wave										
	v = m*mwsin(wt                );
	w = m*mwsin(wt - PI(2.0 / 3.0));

	inverter0_set_uvw( u, v, w );					// Setting PWM references												

	#if !defined(F28335LMT)
	watch_data();									// Sending data for viewing waveforms in WAVE							
	#endif
	inverter0_clear_up_flag();						// Clearing the interrupt flag.											
}
*/

//AD変換した値を実際の電圧値に変更
FLOAT32 v_raw_converter(FLOAT32 v_in)//inverter_model 1R->1, 9R->9
{
	switch(INVERTER_MODEL)
	{
		case 1:								//1Rと5Rは同じ
		case 5:		return (v_in*100.0);	//Vdc=+500V => vdc_raw=5V
		case 9:		return (v_in*80.0);	//Vdc=+400V => vdc_raw=5V
		default:	return 0.0;
	}
}

//AD変換した値を実際の電流値に変更
FLOAT32 i_raw_converter(FLOAT32 c_in)//inverter_model 1R->1, 9R->9
{
	switch(INVERTER_MODEL)
	{
		case 1:		return (c_in*1.25);	//Idc=+-6.25A => idc_raw=+-5V
		case 5:		return (c_in*6.25);	//Idc=+-31.25A => idc_raw=+-5V
		case 9:		return (c_in*10.0);	//Idc=+-50A => idc_raw=+-5V
		default:	return 0.0;
	}
}

FLOAT32 radianLimit(FLOAT32 rad)//-PI～PIに制限
{
	FLOAT32 rad_out=rad;

	if(mwabs(rad)>PI(1))//radの絶対値がPI以上の時
		rad_out = (rad > 0) ? radianLimit(rad-PI(2)) : radianLimit(rad+PI(2));//radが正なら2PI引く、radが負なら2PI足す
	
	return rad_out;
}

/*void getHallValueInt(void)
{
	int_ack();

	getHallValue();

	timer0_clear_int_flag();				// Clearing the interrupt flag.
}*/

void getHallValue(void)
{
	ad_start(SEQ2, AD_GROUP_6);							/*	Starting A/D conversion of channel group6 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ2, 0, &hallA, &hallB );				/*	Reading A/D conversion output data of channel group6										*/

	ad_start(SEQ2, AD_GROUP_7);							/*	Starting A/D conversion of channel group7 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ2, 0, &hallC, &hallD );				/*	Reading A/D conversion output data of channel group7										*/

	hallX=(hallA-hallC)/2;//ホール素子のXを平均化で導出(AとCは理想的には2.5からの差が同じ絶対値で符号が逆)
	hallY=(hallB-hallD)/2;//ホール素子のYを平均化で導出(BとDは理想的には2.5からの差が同じ絶対値で符号が逆)

	xy2ra(hallX,hallY,&hallR,&hallT);//直交座標を極座標に変換して回転角度を導出
	hallOUT=getHallErrorCalc(hallT);
	pro_da_out(0,hallT);							/* Output data for viewing waveforms in DAC					*/
}

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

void getEncSawWave(void)
{
	int_ack();

	enc_theta_calc=radianLimit(enc_theta_raw+enc_theta_offset);//エンコーダ角度のオフセットを計算
	
	pro_da_out(1,enc_theta_calc);							/* Output data for viewing waveforms in DAC					*/

	timer1_clear_int_flag();					/* Clearing the interrupt flag.								*/
}

void hallErrorCompInit(void)//ホール素子校正の初期設定
{
	INT32 i;
	//hallErrorCompPhはdegで代入してradに変換
	hallErrorCompAmp[0]=-103.194;	hallErrorCompPh[0]=0.0;	hallErrorCompUse[0]=1;
	hallErrorCompAmp[1]=0.633275735;	hallErrorCompPh[1]= -117.8906118;	hallErrorCompUse[1]=1;
	hallErrorCompAmp[2]=3.086499659;	hallErrorCompPh[2]= -81.0681271;	hallErrorCompUse[2]=1;
	hallErrorCompAmp[3]=0.075699977;	hallErrorCompPh[3]= 23.14646234;	hallErrorCompUse[3]=0;
	hallErrorCompAmp[4]=4.366958749;	hallErrorCompPh[4]= -81.76701926;	hallErrorCompUse[4]=1;
	hallErrorCompAmp[5]=0.173986398;	hallErrorCompPh[5]= 107.3194347;	hallErrorCompUse[5]=0;
	hallErrorCompAmp[6]=0.521664715;	hallErrorCompPh[6]= 117.9616102;	hallErrorCompUse[6]=1;
	hallErrorCompAmp[7]=0.027259279;	hallErrorCompPh[7]= 53.82444327;	hallErrorCompUse[7]=0;
	hallErrorCompAmp[8]=0.583975898;	hallErrorCompPh[8]= 120.4875918;	hallErrorCompUse[8]=1;
	hallErrorCompAmp[9]=0.066215787;	hallErrorCompPh[9]= -61.50186586;	hallErrorCompUse[9]=0;
	hallErrorCompAmp[10]=0.151485583;	hallErrorCompPh[10]= -31.82436231;	hallErrorCompUse[10]=0;
	hallErrorCompAmp[11]=0.015517434;	hallErrorCompPh[11]= -152.0030841;	hallErrorCompUse[11]=0;
	hallErrorCompAmp[12]=0.120998567;	hallErrorCompPh[12]= -38.31503892;	hallErrorCompUse[12]=0;
	hallErrorCompAmp[13]=0.023401454;	hallErrorCompPh[13]= 130.660386;	hallErrorCompUse[13]=0;
	hallErrorCompAmp[14]=0.052686297;	hallErrorCompPh[14]= 174.0505764;	hallErrorCompUse[14]=0;

	for(i=0;i<HALL_COMP_ORDER;i++)
	{
		hallErrorCompAmp[i]=hallErrorCompAmp[i]*PI(1)/180.0;
		hallErrorCompPh[i]=hallErrorCompPh[i]*PI(1)/180.0;//radに変換
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

/* キャリア同期割り込みで，dq軸電流をPI制御する関数 */
void current_control(void) 
{
    /* 関数内で利用する変数の定義 */
    FLOAT32 ia=0, ib=0;
    FLOAT32 va=0, vb=0;

    int_ack(); // 割り込みのフラグを下す（Mywayがルーチン冒頭での呼び出しを推奨）

    /* AD変換結果の取得 */
	ad_start(SEQ1, AD_GROUP_0);							/*	Starting A/D conversion of channel group0 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &vu_raw, &vw_raw );				/*	Reading A/D conversion output data of channel group0										*/
	vu=v_raw_converter(vu_raw);
	vw=v_raw_converter(vw_raw);

	ad_start(SEQ1, AD_GROUP_1);							/*	Starting A/D conversion of channel group1 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &iu_raw, &iw_raw );				/*	Reading A/D conversion output data of channel group1										*/
	iu=i_raw_converter(iu_raw);
	iw=i_raw_converter(iw_raw);

	ad_start(SEQ1, AD_GROUP_2);							/*	Starting A/D conversion of channel group2 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( SEQ1, 0, &vdc_raw, &idc_raw );				/*	Reading A/D conversion output data of channel group2										*/
	vdc=v_raw_converter(vdc_raw);
	idc=i_raw_converter(idc_raw);

	Half_Vdc=vdc/2;

	getHallValue();//ホール素子で角度位置取得

    /* iu, iw → id, iq (絶対変換) */
    uw2ab(iu, iw, &ia, &ib);
    ab2dq(ia, ib, hallOUT, &id, &iq);

    /* PI制御 */
    id_ref_lim = mylimit(id_ref, id_lim);
    // d軸電流 P制御
    id_err = id_ref_lim - id;
    id_Pout = KPd * id_err;
    // d軸電流 I制御
    id_Itmp += id_err;
    id_Itmp = mylimit(id_Itmp, 30000.0);
    id_Iout = KId * id_Itmp;

    iq_ref_lim = mylimit(iq_ref, iq_lim);
    // q軸電流 P制御
    iq_err = iq_ref_lim - iq;
    iq_Pout = KPq * iq_err;
    // q軸電流 I制御
    iq_Itmp += iq_err;
    iq_Itmp = mylimit(iq_Itmp, 30000.0);
    iq_Iout = KIq * iq_Itmp;

    // dq軸干渉項の計算
    vod = -w_e * Lq_axis * iq;
    voq = w_e * (PSY_a + Ld_axis * id);

    // dq軸電圧指令値を計算
    vd_ref = id_Pout + id_Iout + vod * Decoupling_Mode;
    vq_ref = iq_Pout + iq_Iout + voq * Decoupling_Mode;

    vd_ref_lim = mylimit(vd_ref, vd_lim);
    vq_ref_lim = mylimit(vq_ref, vq_lim);


    /* uvw電圧を出力 */
    // vd, vq → vu, vv, vw (絶対変換)
    dq2ab(vd_ref_lim, vq_ref_lim, hallOUT, &va, &vb);
    ab2uvw(va, vb, &vu_ref, &vv_ref, &vw_ref);

	//三相電圧指令値を正規化(-1～1)
    mvu = vu_ref / Half_Vdc;
    mvv = vv_ref / Half_Vdc;
    mvw = vw_ref / Half_Vdc;

    // duty比を-1～1以内に
    mvu_lim = mylimit(mvu, 1.0);
    mvv_lim = mylimit(mvv, 1.0);
    mvw_lim = mylimit(mvw, 1.0);

    // 変調波との三角波比較を実行
	u = mvu_lim*mwsin(hallOUT + PI(2.0 / 3.0));				/* Generation of 3-phase sine wave										*/
	v = mvv_lim*mwsin(hallOUT                );
	w = mvw_lim*mwsin(hallOUT - PI(2.0 / 3.0));

	inverter0_set_uvw( u, v, w );					/* Setting PWM references												*/

	#if !defined(F28335LMT)
	watch_data();									/* Sending data for viewing waveforms in WAVE							*/
	#endif
	inverter0_clear_up_flag();						/* Clearing the interrupt flag.											*/
}


int main( void )
{
	int_disable();									/* Disabling all interrupt												*/

	hallErrorCompInit();

	system_init();
	#if !defined(F28335LMT)
	watch_init();
	#endif

	inverter0_init( Fcarrier, DEADTIME );					/* Setting carrier frequency and dead time		 						*/
	inverter0_set_uvw( 0.0, 0.0, 0.0 );				/* Setting initial PWM references										*/
	inverter0_init_up_vector( current_control );	/* Carrier interrupt routine definition									*/
	inverter0_enable_up_int();						/* Enabling carrier interrupt at the top point of the carrier mountain	*/

	/*--------------------ad入力(inverter/hall)設定ここから--------------------*/
	ad_init( SINGLE_SCAN );							/*	Initializing A/D conversion module and startign the conversion on Continuous scan mode		*/
	ad_stop();										/*	Stopping A/D conversion once												 			*/

	ad_set_range(AD_GROUP_0, AD_RANGE, AD_RANGE);//21,22番ピン初期化
	ad_set_range(AD_GROUP_1, AD_RANGE, AD_RANGE);//23,24番ピン初期化
	ad_set_range(AD_GROUP_2, AD_RANGE, AD_RANGE);						//VdcはADCINA2に接続されているので、チャンネルグループ番号chg=2 レンジは-5~5
													//ADCINB2に接続されたIdcも勝手に初期化される(仕様)
	ad_set_range(AD_GROUP_6, AD_RANGE, AD_RANGE);//7,8番ピン初期化
	ad_set_range(AD_GROUP_7, AD_RANGE, AD_RANGE);//5,6番ピン初期化
	/*--------------------ad(inverter/hall)設定ここまで--------------------*/

	/*--------------------timer0割り込み(hall)設定ここから--------------------*/
	//timer0_init( PERIOD2 );					/* Initialization of Timer0 and setting interrupt period										*/
											/* After the initialization, Timer0 is stopped,													*/
											/* and Timer0 interrupt is inactive.															*/

	//timer0_init_vector( getHallValueInt );		/* Timer0 interrupt routine definition															*/
	//timer0_start();							/* Starting Timer0 counter																		*/
	//timer0_enable_int();					/* Enabling Timer0 interrupt																	*/
	/*--------------------timer0割り込み(hall)設定ここまで--------------------*/


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

	inverter0_start_pwm();							/* Starting 3-phase PWM signal output									*/
	
	while(1)
	{
		cnt = abz2_read();					/* Reading count of encoder									*/
		enc_theta_raw=(FLOAT32)cnt/(FLOAT32)RESOLUSION*PI(2)-PI(1);//エンコーダ計算値をradに変換

	}												/* Repeating endlessly													*/
	/* NOTREACHED */
	return 0;
}
