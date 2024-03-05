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

/*FLOAT32 myExpCalc(FLOAT32 x, INT32 n, FLOAT32 _numerator, FLOAT32 _denominator, FLOAT32 y)
{
	FLOAT32 denominator = _denominator * (FLOAT32)n;
	FLOAT32 numerator = _numerator * x;
	FLOAT32 a = numerator / denominator;
	if(mwabs(a) <= (FLOAT32)1.0e-5)//最大誤差
		return y;
	else
		return (y + myExpCalc(x, ++n, numerator, denominator, a));
}*/

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

void uvw2dq(FLOAT32 _u, FLOAT32 _w, FLOAT32 _theta, FLOAT32 *_d, FLOAT32 *_q)
{
	FLOAT32 a, b;
	uw2ab(_u, _w, &a, &b);
	ab2dq(a, b, _theta, _d, _q);
}
void dq2uvw(FLOAT32 _d, FLOAT32 _q, FLOAT32 _theta, FLOAT32 *_u, FLOAT32 *_v, FLOAT32 *_w)
{
	FLOAT32 a,b;
	dq2ab(_d, _q, _theta, &a, &b);
	ab2uvw(a, b, _u, _v, _w);
}

void pi_control(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _max, volatile FLOAT32 *_p_out, volatile FLOAT32 *_i_out, volatile FLOAT32 *_itemp)
{
	FLOAT32 err;
	// PI制御 d軸
    _reference = mylimit(_reference, _max);
    // d軸電流 P制御
    err = _reference - _input;
    *_p_out = _p_gain * err;
    // d軸電流 I制御
    *_itemp += err;
    *_itemp = mylimit(*_itemp, 30000.0);
    *_i_out = _i_gain * *_itemp;//なんかバグる
}


FLOAT32 pi_control_release(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _max, volatile FLOAT32 *_itemp)
{
	FLOAT32 err = 0.0, p_out = 0.0, i_out = 0.0;
	// PI制御 d軸
    _reference = mylimit(_reference, _max);
    // d軸電流 P制御
    err = _reference - _input;
    p_out = _p_gain * err;
    // d軸電流 I制御
    *_itemp += err;
    *_itemp = mylimit(*_itemp, 30000.0);
    i_out = _i_gain * *_itemp;//なんかバグる
	return (p_out + i_out);
}
/*void pid_control(FLOAT32 _input, FLOAT32 _reference, FLOAT32 _p_gain, FLOAT32 _i_gain, FLOAT32 _d_gain, FLOAT32 _max, volatile FLOAT32 *_p_out, volatile FLOAT32 *_i_out, volatile FLOAT32 *_itemp, volatile FLOAT32 *_d_out, volatile FLOAT32 *_err_prev)
{
	FLOAT32 err;
	// PI制御 d軸
    _reference = mylimit(_reference, _max);
    // d軸電流 P制御
    err = _reference - _input;
    *_p_out = _p_gain * err;
    // d軸電流 I制御
    *_itemp += err;
    *_itemp = mylimit(*_itemp, 30000.0);
    *_i_out = _i_gain * *_itemp;//なんかバグる
}*/

void pwm_output(FLOAT32 _d_ref, FLOAT32 _q_ref, FLOAT32 _half_Vdc, FLOAT32 _theta, INT32 _inverter_no)
{
	FLOAT32 vu_ref, vv_ref, vw_ref;
	FLOAT32 duty_uph, duty_vph, duty_wph;
	dq2uvw(_d_ref, _q_ref, _theta, &vu_ref, &vv_ref, &vw_ref);
	
	//三相電圧指令値を正規化(-1～1)
    duty_uph = mylimit(vu_ref / _half_Vdc, 1.0);
    duty_vph = mylimit(vv_ref / _half_Vdc, 1.0);
    duty_wph = mylimit(vw_ref / _half_Vdc, 1.0);
	switch(_inverter_no)
	{
		case 0:inverter0_set_uvw( duty_uph, duty_vph, duty_wph );	break;
		case 1:inverter1_set_uvw( duty_uph, duty_vph, duty_wph );	break;
		default: break;
	}
	//インバータ出力
}
