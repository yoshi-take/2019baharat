// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： モーター制御
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.11			TKR			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル防止
#ifndef	_HAL_DCM_CTRL_H
#define	_HAL_DCM_CTRL_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>					// 定義
#include <common_define.h>				// 共通定義
#include <iodefine.h>					// I/O
#include <stdio.h>						// 標準入出力


//**************************************************
// 定義（define）
//**************************************************

//**************************************************
// 列挙体（enum）
//**************************************************
/* 制御動作タイプ */
typedef enum{
	CTRL_ACC,				// [00] 加速中(直進)
	CTRL_CONST,				// [01] 等速中(直進)
	CTRL_DEC,				// [02] 減速中(直進)

	CTRL_SKEW_ACC,			// [03] 斜め加速中(直進)
	CTRL_SKEW_CONST,		// [04] 斜め等速中(直進)
	CTRL_SKEW_DEC,			// [05] 斜め減速中(直進)
	

	CTRL_ACC_TURN,			// [06] 加速中(超信地旋回)
	CTRL_CONST_TURN,		// [07] 等速中(超信地旋回)
	CTRL_DEC_TURN,			// [08] 減速中(超信地旋回)
	
	CTRL_HIT_WALL,			// [09] 壁あて制御
	
	CTRL_ACC_SMOOTH,		// [10] 加速中(直進 cos近似)
	CTRL_CONST_SMOOTH,		// [11] 等速中(直進 cos近似)
	CTRL_DEC_SMOOTH,		// [12] 減速中(直進)

	CTRL_ACC_SLA,			// [13] 加速中(スラローム)
	CTRL_CONST_SLA,			// [14] 等速中(スラローム)
	CTRL_DEC_SLA,			// [15] 減速中(スラローム)
	
	CTRL_ENTRY_SLA,			// [16] スラローム前の前進動作(スラローム)
	CTRL_EXIT_SLA,			// [17] スラローム後の前進動作(スラローム)
	
	CTRL_MAX,

}enCTRL_TYPE;

//**************************************************
// 構造体（struct）
//**************************************************
/* 制御データ */
typedef struct{
	enCTRL_TYPE		en_type;		// 動作タイプ
	FLOAT			f_time;			// 目標時間 [sec]
	FLOAT			f_acc;			// [速度制御]   加速度[mm/s2]
	FLOAT			f_now;			// [速度制御]   現在速度[mm/s]
	FLOAT			f_trgt;			// [速度制御]   最終速度[mm/s]
	FLOAT			f_nowDist;		// [距離制御]   現在距離[mm]
	FLOAT			f_dist;			// [距離制御]   最終距離[mm]
	FLOAT			f_accAngleS;	// [角速度制御] 角加速度[rad/s2]
	FLOAT			f_nowAngleS;	// [角速度制御] 現在角速度[rad/s]
	FLOAT			f_trgtAngleS;	// [角速度制御] 最終角速度[rad/s]
	FLOAT			f_nowAngle;		// [角度制御]   現在角度[rad]
	FLOAT			f_angle;		// [角速制御]   最終角度[rad]
}stCTRL_DATA;


//**************************************************
// グローバル変数(extern)
//**************************************************
extern PUBLIC FLOAT			f_Time;							// 動作時間[msec]
extern PUBLIC  volatile FLOAT  f_NowDist;        			// [距離制御]　現在距離             （1[msec]毎に更新される）
extern PUBLIC FLOAT			f_TrgtSpeed;					// [速度制御]   現在速度（DCMCのFB目標速度）	（1[msec]毎に更新される）
extern PUBLIC  volatile FLOAT  f_NowAngle;			       	// [角度制御]　現在角度             （1[msec]毎に更新される）
extern PUBLIC FLOAT			f_TrgtAngleS;					// [角速度制御] 現在角速度（DCMCのFB目標速度）	（1[msec]毎に更新される）

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
//PUBLIC void CTRL_setData(stDCMC_DATA* p_data);
PUBLIC void CTRL_clrData(void);
PUBLIC void CTRL_stop(void);
PUBLIC void CTRL_polCtrl(void);
PUBLIC void CTRL_showLog(void);




#endif