﻿// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： LED
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.4			TKR			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準ライブラリ

#include <hal_led.h>	


//**************************************************
// 定義（define）
//**************************************************
#define	LED0_PIN		(PORT5.PODR.BIT.B5)
#define	LED1_PIN		(PORT5.PODR.BIT.B4)
#define	LED2_PIN		(PORTC.PODR.BIT.B2)
#define	LED3_PIN		(PORTC.PODR.BIT.B3)
#define	LED4_PIN		(PORTC.PODR.BIT.B4)

#define	LED_SYS_PIN		(PORTC.PODR.BIT.B5)
#define LED_BATT_PIN	(PORTC.PODR.BIT.B6)


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************


// *************************************************************************
//   機能		： LEDをONする。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.4			TKR				新規
// *************************************************************************/
PUBLIC void LED_on( enLED_ID en_id ){
	
	switch( en_id ){
		case LED0:			LED0_PIN		= 1;	break;
		case LED1:			LED1_PIN		= 1;	break;
		case LED2:			LED2_PIN		= 1;	break;
		case LED3:			LED3_PIN		= 1;	break;
		case LED4:			LED4_PIN		= 1;	break;
		
		case LED_SYS:		LED_SYS_PIN		= 1;	break;
		case LED_BATT:		LED_BATT_PIN	= 1;	break;
		
		default:
			break;
	}
		
}

// *************************************************************************
//   機能		： LEDを全部ONする。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.4			TKR				新規
// *************************************************************************/
PUBLIC void LED_onAll( void ){
	
	LED_on( LED0 );
	LED_on( LED1 );
	LED_on( LED2 );
	LED_on( LED3 );
	LED_on( LED4 );
	
}

// *************************************************************************
//   機能		： LEDをOFFする。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.4			TKR				新規
// *************************************************************************/
PUBLIC void LED_off( enLED_ID en_id ){
	
	switch( en_id ){
		case LED0:			LED0_PIN		= 0;	break;
		case LED1:			LED1_PIN		= 0;	break;
		case LED2:			LED2_PIN		= 0;	break;
		case LED3:			LED3_PIN		= 0;	break;
		case LED4:			LED4_PIN		= 0;	break;
		
		case LED_SYS:		LED_SYS_PIN		= 0;	break;
		case LED_BATT:		LED_BATT_PIN	= 0;	break;
		
		default:
			break;
	}
		
}

// *************************************************************************
//   機能		： 前面のLEDを全部OFFする。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.4			TKR				新規
// *************************************************************************/
PUBLIC void LED_offAll( void ){
	
	LED_off( LED0 );
	LED_off( LED1 );
	LED_off( LED2 );
	LED_off( LED3 );
	LED_off( LED4 );
	
}


// *************************************************************************
//   機能		： LEDの初期化する。
//   注意		： なし
//   メモ		： 起動後に1回だけ実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.4			TKR				新規
// *************************************************************************/
PUBLIC void LED_init( void ){
	
	LED_offAll();	// 全部OFFにする
	LED_off(LED_SYS);

}	