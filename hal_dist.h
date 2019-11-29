// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： SCI
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規（ファイルのインクルード）
// *************************************************************************/
#ifndef _HAL_DIST_H
#define	_HAL_DIST_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// ・ｽ・ｽ`
#include <stdio.h>			// ・ｽW・ｽ・ｽ・ｽ・ｽ・ｽC・ｽu・ｽ・ｽ・ｽ・ｽ

//**************************************************
// 定義（define）
//**************************************************

//**************************************************
// 列挙体（enum）
//**************************************************
typedef enum{
	DIST_SEN_R_FRONT = 0,		// ・ｽE・ｽO
	DIST_SEN_L_FRONT,			// ・ｽ・ｽ・ｽO
	//DIST_SEN_R_45,			// ・ｽE45・ｽ・ｽ(・ｽ・ｽ・ｽ・ｽ)
	//DIST_SEN_L_45,			// ・ｽ・ｽ45・ｽ・ｽ(・ｽ・ｽ・ｽ・ｽ)
	DIST_SEN_R_SIDE,			// ・ｽE・ｽ・ｽ
	DIST_SEN_L_SIDE,			// ・ｽ・ｽ・ｽ・ｽ
	DIST_SEN_MAX
}enDIST_SEN_ID;

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id );
PUBLIC void DIST_Pol_Front( void );
PUBLIC void DIST_Pol_Side( void );
PUBLIC void DIST_getErr( LONG* p_err );
PUBLIC void DIST_getErrSkew( LONG* p_err );
PUBLIC void DIST_getErrFront( LONG* p_err );
PUBLIC void DIST_Check( void );
PUBLIC void DIST_adj( void );
PUBLIC BOOL DIST_isWall_FRONT( void );
PUBLIC BOOL DIST_isWall_R_SIDE( void );
PUBLIC BOOL DIST_isWall_L_SIDE( void );
PUBLIC void DIST_LogSta( void );
PUBLIC void DIST_showLog( void );
#endif