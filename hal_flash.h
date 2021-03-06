// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： FLASH
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.3.27			TKR			新規（ファイルのインクルード）
// *************************************************************************/
#ifndef _HAL_FLASH_H
#define	_HAL_FLASH_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準入出力

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
PUBLIC void FLASH_init();
static void FLASH_PEMode();
static void FLASH_ReadMode();
PUBLIC void FLASH_waitFCU( int timeout );
PUBLIC void FLASH_FcuReset();
PUBLIC void FLASH_Erase(ULONG addr);
PUBLIC void FLASH_WriteEE(ULONG addr, USHORT *data);
PUBLIC void FLASH_Read(USHORT *add, USHORT *data);
PUBLIC void FLASH_CheckError( void );


#endif