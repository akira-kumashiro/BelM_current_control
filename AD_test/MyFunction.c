#include "stdafx.h"
#include "MyFunction.h"

FLOAT32 radianLimit(FLOAT32 rad)//-PI～PIに制限
{
	FLOAT32 rad_out = rad;

	if(mwabs(rad) > PI(1))//radの絶対値がPI以上の時
		rad_out = (rad > 0) ? radianLimit(rad - PI(2)) : radianLimit(rad + PI(2));//radが正なら2PI引く、radが負なら2PI足す
	
	return rad_out;
}

FLOAT32 myExp(FLOAT32 x)//指数関数の計算exp(x)
{
	FLOAT32 total = 1.0;
	FLOAT32 a = 1.0;
	INT32 i;

	for (i = 1; i < 10; i++)
	{
		a = a / (FLOAT32)i * x;
		total += a;
	}
	return total;
	//return myExpCalc(x, 1, 1.0, 1.0, 1.0);
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

void adInput(INT32 seq_No, INT32 ad_group, FLOAT32 *signalA, FLOAT32 *signalB)
{
    ad_start(seq_No, ad_group);							//	Starting A/D conversion of channel group6 and setting it to  SEQ1
	while(ad_in_st() != 0){}						//	Checking status of A/D converter device and waiting for completing A/D conversion
	ad_in( seq_No, 0, signalA, signalB );				//	Reading A/D conversion output data of channel group6
}

FLOAT32 getHallRawValue(FLOAT32 _hallA, FLOAT32 _hallB, FLOAT32 _hallC, FLOAT32 _hallD)
{
	FLOAT32 _hallR = 0.0, _hall_theta = 0.0;
	FLOAT32 _hallX = (_hallA - _hallC) / 2;//ホール素子のXを平均化で導出(AとCは理想的には2.5からの差が同じ絶対値で符号が逆)
	FLOAT32 _hallY = (_hallB - _hallD) / 2;//ホール素子のYを平均化で導出(BとDは理想的には2.5からの差が同じ絶対値で符号が逆)

	xy2ra(_hallX, _hallY, &_hallR, &_hall_theta);//直交座標を極座標に変換して回転角度を導出
	return _hall_theta;
}
