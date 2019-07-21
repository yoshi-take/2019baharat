// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： GYRO
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規（ファイルのインクルード）
// *************************************************************************/
//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力

#include <hal_gyro.h>                       // GYRO
#include <hal_spi.h>						// SPI
#include <parameter.h>						// parameter

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
/* ジャイロセンサ */
PUBLIC	SHORT s_GyroVal_Lo;								// ジャイロセンサ値(下位)
PUBLIC	SHORT s_GyroVal_Hi;								// ジャイロセンサ値(上位)
PUBLIC FLOAT  f_NowGyroAngle;		 					// ジャイロセンサの現在角度
PUBLIC FLOAT  f_NowGyroAngleSpeed;						// ジャイロセンサの現在角速度	

PUBLIC	SHORT s_WhoamiVal;
PRIVATE LONG  l_GyroRef; 								// ジャイロセンサの基準値

extern PUBLIC	enSPI_STATE		en_SpiState;	// SPI通信状態
extern PUBLIC	SHORT*			p_SpiRcvData;	// SPI受信データ格納アドレス
extern PUBLIC	FUNC_PTR		p_SpiCallBackFunc;	// SPIのIDLE遷移割り込み時に登録されると呼び出される関数


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
extern PUBLIC void TIME_wait(ULONG ul_time);

// *************************************************************************
//   機能		： ジャイロの現在の角度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_NowGyroAngle;
}

// *************************************************************************
//   機能		： ジャイロの現在の角速度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngleSpeed( void )
{
	return f_NowGyroAngleSpeed;
}

// *************************************************************************
//   機能		： ジャイロの累積角度をリセットする
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
//		v2.0		2019.6.2			TKR			角度と角速度の変数変更
// *************************************************************************/
PUBLIC void GYRO_clrAngle( void )
{
	f_NowGyroAngle 		= 0;
	f_NowGyroAngleSpeed	= 0;
}

// *************************************************************************
//   機能		： ジャイロセンサ用ポーリング関数
//   注意		： 周期ハンドラから定期的に呼び出す
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.14			TKR			新規
//		v2.0		2019.6.2			TKR			移動平均を削除(前作)
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	f_NowGyroAngle += f_NowGyroAngleSpeed / 1000;		// 角度更新   (0.001sec毎に加算するため)
}

// *************************************************************************
//   機能		： 君の名は？
//   注意		： コールバック関数は使わない
//   メモ		： ジャイロのデバッグ用
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC	void GYRO_get_WHOAMI( void ){
	
	p_SpiRcvData		= &s_WhoamiVal;		// 上位データ
	SPI_staGetData(SPI_WHO_AM_I);			// 受信処理実行

	printf("Who am I = 0x%x\n\r",s_WhoamiVal);
}

// *************************************************************************
//   機能		： ジャイロ初期設定
//   注意		： SPI_init実行後に実行
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.2			TKR			新規
// *************************************************************************/
PUBLIC void GYRO_init( void ){
	
	SHORT	us_dummy;
	
	/* パワーマネジメント1[No.107] */		 
	SPI_staSetData(SPI_PWR_MGMT_1,0x01);		 
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}

	/* 信号リセット[No.104] */
//	SPI_staSetData(SPI_SIGNAL_RESET,0x03);		// 加速度・温度リセット
//	TIME_wait(100);
//	while(1){
//		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
//	}

	/* コンフィグ[No.26] */
	SPI_staSetData(SPI_CONFIG,0x00);
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}
	
	/* FSYNCコンフィグ[No.54] */
	p_SpiRcvData		= &us_dummy;		// ダミー
	SPI_staGetData(SPI_FSYNC_INT);			// 読み出し用レジスタ
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}
	
	/* INT_PINコンフィグ[No.55] */
	SPI_staSetData(SPI_INT_PIN_COMFIG,0xe8);
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}
	

	/* SPI有効[No.106] */
	SPI_staSetData(SPI_USER_CONTROL,0x01);		// ジャイロ・加速度・温度リセット
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}
	
	/* SPI有効[No.112] */
	SPI_staSetData(SPI_I2C_IF,0x40);			// SPI有効(I2Cを切る)
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}

	/* 精度(ジャイロ)[No.27] */
	SPI_staSetData(SPI_GYRO_CFG,0x18);			// ±2000[dps]
	TIME_wait(100);
	while(1){
		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
	}

	/* ジャイロのオフセット処理[No.24] */
//	SPI_staSetData(SPI_GYRO_CFG,0x18);			// 
//	TIME_wait(100);
//	while(1){
//		if( en_SpiState == SPI_IDLE )break;		// SPI通信完了
//	}

}

// *************************************************************************
//   機能		： ジャイロセンサ値取得用関数(2/2)
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PRIVATE void GYRO_getVal_2nd( void ){

	SHORT	s_count;			// ICM20602から得られた角速度(カウント値)
	FLOAT	f_tempAngleSpeed;	// 角速度[dps]

	/* 初期化 */
	p_SpiRcvData		= NULL;
	p_SpiCallBackFunc	= NULL;

	/* 角速度値 */
	s_count				= (SHORT)(s_GyroVal_Lo | (s_GyroVal_Hi << 8) );		// データ結合
	f_tempAngleSpeed	= (FLOAT)s_count / GYRO_SCALE_FACTOR;				// [カウント]→[dps]に変換

	/* SWフィルタを有効にする(後で書く) */ 
	
	/* 角速度更新 */
	f_NowGyroAngleSpeed		= f_tempAngleSpeed;

}

// *************************************************************************
//   機能		： ジャイロセンサ値取得用関数(1/2)
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PRIVATE void GYRO_getVal_1st( void ){

	p_SpiRcvData		= &s_GyroVal_Hi;		// 上位データ
	p_SpiCallBackFunc	= GYRO_getVal_2nd;
	SPI_staGetData(SPI_GYRO_Z_H);

}

// *************************************************************************
//   機能		： ジャイロセンサの値を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC void GYRO_getVal( void ){

	p_SpiRcvData		= &s_GyroVal_Lo;		// 受信データアドレス登録(下位データ)
	p_SpiCallBackFunc	= GYRO_getVal_1st;		// コールバック関数登録
	SPI_staGetData(SPI_GYRO_Z_L);				// 受信処理実行

}