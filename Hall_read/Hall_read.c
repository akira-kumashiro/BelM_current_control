/****************************************************************************************************************************
 Title			:	Sample Program for PE-PRO/F28335A
 Sub title		:	ホール素子の出力値を取得
 Copyright		:	2010 Myway Corporation

 Explanation	:	ホール素子の出力値を取得

 Used board		:	PE-PRO/F28335A
*****************************************************************************************************************************/
#define		DAC_ENABLE
#include	<mwio.h>								/*	Header file of I/O library for PEOS/F28335							*/
#include	<pro_f28335.h>							/* Header file for PE-PRO/F28335A										*/
#include	<pro_f28335.c>							/*	Peculiar module for PE-PRO/F28335A									*/

#define	PI(n)	(3.14159265358979 * (n))

FLOAT32 hallA=0.0;//ホール素子1
FLOAT32 hallB=0.0;//ホール素子2
FLOAT32 hallC=0.0;//ホール素子3
FLOAT32 hallD=0.0;//ホール素子4

FLOAT32 hallX=0.0;//ホール素子X
FLOAT32 hallY=0.0;//ホール素子Y

FLOAT32 hallR=0.0;//ホール素子->磁束強さ
FLOAT32 hallT=0.0;//ホール素子->回転角度

void	getHallValue(void);

#pragma	INTERRUPT(getHallValue)

void	getHallValue(void)
{
	int_ack();

	ad_start(1, 6);							/*	Starting A/D conversion of channel group6 and setting it to  SEQ1							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( 1, 0, &hallA, &hallB );				/*	Reading A/D conversion output data of channel group6										*/

	ad_start(2, 7);							/*	Starting A/D conversion of channel group7 and setting it to  SEQ2							*/
	while(ad_in_st() != 0){}						/*	Checking status of A/D converter device and waiting for completing A/D conversion 	*/
	ad_in( 2, 0, &hallC, &hallD );				/*	Reading A/D conversion output data of channel group7										*/

	hallX=(hallA-hallC)/2;//ホール素子のXを平均化で導出(AとCは理想的には2.5からの差が同じ絶対値で符号が逆)
	hallY=(hallB-hallD)/2;//ホール素子のYを平均化で導出(BとDは理想的には2.5からの差が同じ絶対値で符号が逆)

	xy2ra(hallX,hallY,&hallR,&hallT);//直交座標を極座標に変換して回転角度を導出

	timer0_clear_int_flag();				/* Clearing the interrupt flag.																	*/
}


int main( void )
{
	int_disable();									/* Disabling all interrupt												*/

	system_init();
	#if !defined(F28335LMT)
	watch_init();
	#endif

	ad_init( 0 );							/*	Initializing A/D conversion module and startign the conversion on Continuous scan mode		*/

	ad_set_range(6, 5.0, 5.0);//7,8番ピン初期化
	ad_set_range(7, 5.0, 5.0);//5,6番ピン初期化

	timer0_init( 1000 );					/* Initialization of Timer0 and setting interrupt period										*/
											/* After the initialization, Timer0 is stopped,													*/
											/* and Timer0 interrupt is inactive.															*/

	timer0_init_vector( getHallValue );		/* Timer0 interrupt routine definition															*/
	timer0_start();							/* Starting Timer0 counter																		*/
	timer0_enable_int();					/* Enabling Timer0 interrupt																	*/

	int_enable();							/* Enabling all interrupts																		*/

	while(1)
	{
		#if !defined(F28335LMT)
		watch_data();						/*	Sending data for viewing waveforms in WAVE													*/
		#endif
	}												/* Repeating endlessly													*/
	/* NOTREACHED */
	return 0;
}
