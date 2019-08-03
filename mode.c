// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： MODE
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.5			TKR			新規
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準ライブラリ

#include <mode.h>			// MODE
#include <hal_led.h>		// LED
#include <hal_spk.h>		// SPK
#include <hal_battery.h>	// バッテリ
#include <hal_sci.h>		// SCI
#include <hal_spi.h>		// SPI
#include <hal_gyro.h>		// ジャイロ
#include <hal_dcm.h>		// DCM
#include <hal_enc.h>		// ENC
#include <hal_dist.h>       // DIST
#include <motion.h>			// motion

#include <parameter.h>		// parameter

//**************************************************
// 定義（define）
//**************************************************


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************
PRIVATE enMODE		en_Mode;		// 現在のモード	

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： モードを実行する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PUBLIC void	MODE_exe( void ){
	
	/* 走行パラメータ設定 */
	PARAM_setCntType( TRUE );
	MOT_setTrgtSpeed( 350.0f );						// 目標速度設定
	PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
	PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
	PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速
	
	/* スラロームデータ生成 */
	// 90度
	// 45度
	// 135度
	// 斜め → 90°→ 斜め

	switch( en_Mode ){
		
		case MODE_0:
			LED_offAll();	
			BAT_Check();
			break;
			
		case MODE_1:
			LED_offAll();
			TIME_wait(1000);
			//GYRO_clrAngle();		// 角度リセット

			while(1){
				printf("AngleSpeed:%f[deg]\r",GYRO_getNowAngleSpeed());
				TIME_wait(100);
			}
			break;
			
		case MODE_2:
			LED_offAll();
			DCM_setDirCw(DCM_L);
			DCM_setDirCw(DCM_R);
			DCM_setPwmDuty(DCM_L,200);
			DCM_setPwmDuty(DCM_R,200);
			//DCM_setPwmDuty(DCM_SUC,100);
			DCM_staMotAll();
			
			
			break;
			
		case MODE_3:
			LED_offAll();
			
			DIST_Check();		// 距離センサデバッグ
			
			break;
			
		case MODE_4:
			LED_offAll();
			TIME_wait(1000);
			GYRO_clrAngle();		// 角度リセット

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( 500.0f );						// 目標速度設定
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速

			MOT_goBlock_FinSpeed(1,0);

			break;
			
		case MODE_5:
			LED_offAll();
			TIME_wait(1000);
			GYRO_clrAngle();		// 角度リセット

			while(1){
				printf("Angle:%f[deg]\r",GYRO_getNowAngle());
				TIME_wait(100);
			}

			break;
			
		case MODE_6:
			LED_offAll();
			
			break;
			
		case MODE_7:
			LED_offAll();		
			break;
			
		case MODE_8:
			LED_offAll();
			break;

			
		default:
			break;
			
	}
}

// *************************************************************************
//   機能		： 予約されたモードに変更する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PRIVATE void MODE_chg( enMODE en_mode ){
	
	LED_offAll();	
	
	switch( en_mode ){
		
		case MODE_0:
			//SPK_on(A4,16.0f,120);
			break;
			
		case MODE_1:
			//SPK_on(C4,16.0f,120);
			LED_on(LED0);
			break;
			
		case MODE_2:
			//SPK_on(D4,16.0f,120);
			LED_on(LED1);
			break;
			
		case MODE_3:
			//SPK_on(E4,16.0f,120);
			LED_on(LED2);
			break;
			
		case MODE_4:
			//SPK_on(F4,16.0f,120);
			LED_on(LED3);
			break;
			
		case MODE_5:
			//SPK_on(G4,16.0f,120);
			LED_on(LED4);
			break;
			
		case MODE_6:
			//SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED1);
			break;
			
		case MODE_7:
			//SPK_on(B4,16.0f,120);
			LED_on(LED0);
			LED_on(LED2);
			break;
			
		case MODE_8:
			//SPK_on(C5,16.0f,120);
			LED_on(LED0);
			LED_on(LED3);
			break;
			
		default:
			break;
			
	}
	
	en_Mode = en_mode;	// 現在のモード			
}

// *************************************************************************
//   機能		： モードを加算変更する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PUBLIC void MODE_inc( void ){
	
	en_Mode++;
	
	/* 最大値チェック */
	if( MODE_MAX == en_Mode ){
		en_Mode = MODE_0;
	}
	
	MODE_chg(en_Mode);		// モード変更
	
}

