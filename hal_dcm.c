// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： LED
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.11			TKR			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>					// 定義
#include <common_define.h>				// 共通定義
#include <iodefine.h>					// I/O
#include <stdio.h>						// 標準入出力

#include <hal_dcm.h>						// LED

//**************************************************
// 定義（define）
//**************************************************
// 入力
#define     DCM_R_IN1           (PORTB.PODR.BIT.B1)
#define     DCM_R_IN2           (PORTB.PODR.BIT.B7)
#define     DCM_L_IN1           (PORTE.PODR.BIT.B5)
#define     DCM_L_IN2           (PORTA.PODR.BIT.B3)

// タイマ開始
#define     DCM_R_TIMER         (MTU.TSTR.BIT.CST0)     // 右:MTU0
#define     DCM_L_TIMER         (MTU.TSTR.BIT.CST4)     // 左:MTU4
#define     DCM_SUC_TIMER       (TPUA.TSTR.BIT.CST5)    // 吸引:TPU5

// ピン出力設定
#define		DCM_R_TIORA			(MTU0.TIORH.BIT.IOA)
#define		DCM_R_TIORB			(MTU0.TIORH.BIT.IOB)
#define		DCM_L_TIORA			(MTU4.TIORH.BIT.IOA)
#define		DCM_L_TIORB			(MTU4.TIORH.BIT.IOB)
#define		DCM_SUC_TIORA		(TPU5.TIOR.BIT.IOA)
#define		DCM_SUC_TIORB		(TPU5.TIOR.BIT.IOB)

// カウント値
#define		DCM_R_TCNT			(MTU0.TCNT)
#define		DCM_L_TCNT			(MTU4.TCNT)
#define     DCM_SUC_TCNT        (TPU5.TCNT)

#define		DCM_R_GRA			(MTU0.TGRA)     // 右周期
#define		DCM_R_GRB			(MTU0.TGRB)     // 右Duty比
#define		DCM_L_GRA			(MTU4.TGRA)     // 左周期
#define		DCM_L_GRB			(MTU4.TGRB)     // 左Duty比
#define		DCM_SUC_GRA			(TPU5.TGRA)     // 吸引周期 
#define		DCM_SUC_GRB			(TPU5.TGRB)     // 吸引Duty比


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
//   機能		： DCMの回転方向をCW（時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.12			TKR			新規
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 	= ON;			// BIN1
		DCM_R_IN2	= OFF;			// BIN2	
	}
	else if( en_id == DCM_L ){		// 左
		DCM_L_IN1 	= OFF;			// AIN1
		DCM_L_IN2	= ON;			// AIN2	
	}else{
	
	}
}


// *************************************************************************
//   機能		： DCMの回転方向をCCW（反時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.12			TKR			新規
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 	= OFF;			// BIN1
		DCM_R_IN2	= ON;			// BIN2	
	}
	else if( en_id == DCM_L ){							// 左
		DCM_L_IN1 	= ON;			// AIN1
		DCM_L_IN2	= OFF;			// AIN2	
	}else{
		
	}
}

// *************************************************************************
//   機能		： DCMを停止する
//   注意		： なし
//   メモ		： PWMのHI出力中に本関数を実行すると、ピンが100%出力状態なるため、関数内でピンをクリア（Lo）する。
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
//		v2.0		2019.5.5			TKR			吸引追加
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = OFF;			// BIN1
		DCM_R_IN2 = OFF;			// BIN2
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else if( en_id == DCM_L ){		// 左
		DCM_L_IN1 = OFF;			// AIN1
		DCM_L_IN2 = OFF;			// AIN2
		DCM_L_TIMER = OFF;			// タイマ停止
		DCM_L_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else if(en_id == DCM_SUC ){
		DCM_SUC_TIMER	= OFF;		// タイマ停止
		DCM_SUC_TIORA	= 1;		// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_SUC_TIORB	= 1;		// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else{
		
	}
}

// *************************************************************************
//   機能		： DCMをブレーキングする
//   注意		： 吸引の場合はブレーキはかけずにstopMotで止める
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = ON;				// BIN1
		DCM_R_IN2 = ON;				// BIN2
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else if( en_id == DCM_L ){							// 左
		DCM_L_IN1 = ON;				// AIN1
		DCM_L_IN2 = ON;				// AIN2
		DCM_L_TIMER = OFF;			// タイマ停止
		DCM_L_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	    DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	
	}else{
	}
}

// *************************************************************************
//   機能		： DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
//		v2.0		2019.5.12			TKR			吸引追加
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* タイマスタート */
	if( en_id == DCM_R ){			// 右
		DCM_R_TIORA = 2;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIMER = ON;			// タイマ開始
	}
	else if( en_id == DCM_L ){							// 左
		DCM_L_TIORA = 2;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	    DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_L_TIMER = ON;			// タイマ開始
	}
	else if( en_id == DCM_SUC ){	//吸引
		DCM_SUC_TIORA = 2;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	    DCM_SUC_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_SUC_TIMER = ON;			// タイマ開始
	
	}else{
	}
}

// *************************************************************************
//   機能		： 全DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			外川			新規
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// 右モータON
	DCM_staMot(DCM_L);									// 左モータON
}

// *************************************************************************
//   機能		： DCMのPWM-Dutyを設定する
//   注意		： 割り込み外から設定すると、ダブルバッファでないと歯抜けになる場合がある。
//   メモ		： 割り込みハンドラから実行すること。Duty0%の場合モータを停止させる（PWMにひげが出る）
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// 周期
	USHORT	us_onReg;							// 設定するON-duty
	
	/* PWM設定 */
	//==== 右 ====//
	if( en_id == DCM_R ){				
	
		if( 0 == us_duty10 ){			// Duty0%設定
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER 	= OFF;			// タイマ停止
			DCM_R_TCNT 		= 0;			// TCNT カウンタをクリア
			DCM_R_GRB 		= 5000;			// タイマ値変更
			DCM_R_TIORA 	= 6;			// TIOCA 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
		    DCM_R_TIORB 	= 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_R_TIMER 	= ON;			// タイマ開始
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_R_GRA;		// 周期
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_R_TIMER 	= OFF;				// タイマ停止
			DCM_R_TCNT 		= 0;				// TCNT カウンタをクリア
			DCM_R_GRB 		= us_onReg;			// onDuty
			DCM_staMot( en_id );				// 回転開始
		}
	}
	
	//==== 左 ====//
	else if( en_id == DCM_L ){							

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER 	= OFF;			// タイマ停止
			DCM_L_TCNT 		= 0;			// TCNT カウンタをクリア
			DCM_L_GRB 		= 5000;			// タイマ値変更
			DCM_L_TIORA 	= 6;			// TIOCA 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
		    DCM_L_TIORB 	= 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_L_TIMER 	= ON;			// タイマ開始
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_L_GRA;		// 周期
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_L_TIMER 	= OFF;			// タイマ停止
			DCM_L_TCNT 		= 0;			// TCNT カウンタをクリア
			DCM_L_GRB 		= us_onReg;		// タイマ値変更
			DCM_staMot( en_id );			// 回転開始
			
			printf("DCM_L_GRB = %d\n\r",DCM_L_GRB);
			printf("us_onReg = %d\n\r",us_onReg);
			
			DCM_staMot( en_id );			// 回転開始
			
		}
		
	//==== 吸引 ====//	
	}else{	
		
		if( 0 == us_duty10 ){			// Duty0%
			DCM_stopMot( en_id );		// 吸引の場合は止めてしまう
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_SUC_TIMER 	= OFF;			// タイマ停止
			DCM_SUC_TCNT 	= 0;			// TCNT カウンタをクリア
			DCM_SUC_GRB 	= 5000;			// タイマ値変更
			DCM_SUC_TIORA 	= 6;			// TIOR(IOA)端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
		    DCM_SUC_TIORB 	= 6;			// TIOR(IOB)端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_SUC_TIMER 	= ON;			// タイマ開始
			us_duty10 		= 1000;
		}
		else{
			us_cycle 		= DCM_SUC_GRA;		// 周期
			us_onReg 		= (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_SUC_TIMER 	= OFF;			// タイマ停止
			DCM_SUC_TCNT 	= 0;			// TCNT カウンタをクリア
			DCM_SUC_GRB 	= us_onReg;		// タイマ値変更
			DCM_staMot( en_id );			// 回転開始
		}
	}	
	
}