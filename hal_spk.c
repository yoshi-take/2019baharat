// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： SCI
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.5			TKR			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準入出力
#include <hal_spk.h>		// SPK
#include <hal_led.h>


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

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： SPKを鳴らす。
//   注意		： なし
//   メモ		： なし
//   引数		： 周波数(frq)，音符(beat)，テンポ(bpm)
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.3.24			TKR				新規
// *************************************************************************/
PUBLIC void SPK_on( int frq, float beat, int bpm  ){

	TPU4.TIOR.BIT.IOA  	= 1;
	TPU4.TIOR.BIT.IOB	= 2;

	if(frq != REST){
		TPU4.TGRA			= (int) ((12000000/frq) - 1);
		TPU4.TGRB			= TPU4.TGRA / 2;
		TPUA.TSTR.BIT.CST4	= 1;						// 0:停止,1:開始
	}
	
	// 60000(msec)÷bpm = 4分音符の長さ(msec)
	// 
	TIME_wait( (int)(60000.0f/bpm*4.0f/beat-10.0f ));
	
	TPUA.TSTR.BIT.CST4 	= 0;		// TPU4 0:停止,1:開始
	TPU4.TIOR.BIT.IOA  	= 1;
	TPU4.TIOR.BIT.IOB	= 1;
	TIME_wait(10);
}

// *************************************************************************
//   機能		： SPKを鳴らす(テストver)。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.3.31			TKR				新規
// *************************************************************************/
PUBLIC void SPK_OnTest(void){

	TPU4.TCNT			= 0;	// カウンタリセット
	TPUA.TSTR.BIT.CST4	= 1;	// TPU4 0:停止,1:開始

}

// *************************************************************************
//   機能		： SPKを止める(テストver)。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.3.31			TKR				新規
// *************************************************************************/
PUBLIC void SPK_Off(void){

	PORTB.PMR.BIT.B5	= 0;	// 周辺機能に切り替え
	TPUA.TSTR.BIT.CST4	= 0;	// TPU4 0:停止,1:開始

}