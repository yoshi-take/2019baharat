// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： ENC
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.11			TKR			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル防止
#ifndef	_HAL_GYRO_H
#define	_HAL_GYRO_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力

//**************************************************
// 定義（define）
//**************************************************
#define     SPI_WHO_AM_I        ( 0x7500 )      // Who am I?

#define		SPI_PWR_MGMT_1		( 0x6b00 )		// [No.107]電源制御1
#define		SPI_CONFIG			( 0x1a00 )		// [No.26]コンフィグ

#define		SPI_FSYNC_INT		( 0x3600 )		// [No.54]FSYNCコンフィグ
#define		SPI_INT_PIN_COMFIG	( 0x3700 )		// [No.55]INT_PINコンフィグ

#define		SPI_SIGNAL_RESET	( 0x6800 )		// [No.104]信号リセット
#define		SPI_USER_CONTROL	( 0x6a00 )		// [No.106]ユーザーコントロール
#define     SPI_I2C_IF          ( 0x7000 )      // [No.112]I2Cの無効
#define		SPI_GYRO_CFG		( 0x1b00 )		// [No.27]ジャイロセンサのコンフィグ
#define		SPI_ACC_CFG			( 0x1c00 )		// [No.28]加速度センサのコンフィグ
#define		SPI_GYRO_OFFSET_L	( 0x1800 )	    // [No.24]ジャイロオフセット（下位）
#define		SPI_ACC_OFFSET_L	( 0x1600 )		// [No.20]加速度オフセット（下位）
#define		SPI_SEN_ENB			( 0x2300 )	    // センサ有効

#define		SPI_GYRO_Z_L		( 0x4800 )		// ジャイロセンサの下位データ
#define		SPI_GYRO_Z_H		( 0x4700 )		// ジャイロセンサの上位データ
#define		SPI_TEMP_L			( 0x4200 )		// 温度センサの下位データ
#define		SPI_TEMP_H			( 0x4100 )		// 温度センサの上位データ
#define		SPI_ACC_L			( 0x3e00 )		// 加速度センサの下位データ
#define		SPI_ACC_H			( 0x3d00 )		// 加速度センサの上位データ

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
PUBLIC FLOAT GYRO_getNowAngle( void );
PUBLIC FLOAT GYRO_getNowAngleSpeed( void );
PUBLIC void GYRO_clrAngle( void );
PUBLIC void GYRO_Pol( void );
PUBLIC	void GYRO_get_WHOAMI( void );
PUBLIC void GYRO_init( void );
PRIVATE void GYRO_getVal_2nd( void );
PRIVATE void GYRO_getVal_1st( void );
PUBLIC void GYRO_getVal(void);
PRIVATE void GYRO_getAccVal_2nd( void );
PRIVATE void GYRO_getAccVal_1st( void );
PUBLIC void GYRO_getAccVal( void );

#endif
