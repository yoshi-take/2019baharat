// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： motion
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規（ファイルのインクルード）
// *************************************************************************/
#ifndef _MOTION_H
#define	_MOTION_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力

#include <parameter.h>

//**************************************************
// 定義（define）
//**************************************************

//**************************************************
// 列挙体（enum）
//**************************************************
/* 旋回コマンドリスト */
typedef enum{
	MOT_R90 =0,					// 右 90度超地信旋回
	MOT_L90,					// 左 90度超地信旋回
	MOT_R180,					// 右180度超地信旋回
	MOT_L180,					// 左180度超地信旋回
	MOT_R360,					// 右360度超地信旋回
	MOT_L360,					// 左360度超地信旋回
	MOT_TURN_CMD_MAX,
}enMOT_TURN_CMD;

/* スラロームコマンドリスト */
typedef enum{
	MOT_R90S =0,				// 右 90度超スラローム
	MOT_L90S,					// 左 90度超スラローム
	MOT_R45S_S2N,				// [斜め用] 右 45度超スラローム、ストレート ⇒ 斜め
	MOT_L45S_S2N,				// [斜め用] 左 45度超スラローム、ストレート ⇒ 斜め
	MOT_R45S_N2S,				// [斜め用] 右 45度超スラローム、斜め ⇒ ストレート
	MOT_L45S_N2S,				// [斜め用] 左 45度超スラローム、斜め ⇒ ストレート
	MOT_R90S_N,					// [斜め用] 右 90度超スラローム、斜め ⇒ 斜め
	MOT_L90S_N,					// [斜め用] 左 90度超スラローム、斜め ⇒ 斜め
	MOT_R135S_S2N,				// [斜め用] 右135度超スラローム、ストレート ⇒ 斜め
	MOT_L135S_S2N,				// [斜め用] 左135度超スラローム、ストレート ⇒ 斜め
	MOT_R135S_N2S,				// [斜め用] 右135度超スラローム、斜め ⇒ ストレート
	MOT_L135S_N2S,				// [斜め用] 左135度超スラローム、斜め ⇒ ストレート
	MOT_SURA_CMD_MAX,
}enMOT_SULA_CMD;

/* マウス進行方向 */
typedef enum{
	MOT_DIR_FRONT = 0,			// 前進
	MOT_DIR_BACK,				// 後退
	MOT_DIR_R,					// 右旋回
	MOT_DIR_L,					// 左旋回
	MOT_DIR_MAX,
}enMOT_DIR;

/*壁切れ補正*/
typedef enum{
	MOT_WALL_EDGE_NONE = 0,		//壁のエッジ検出での補正無し
	MOT_WALL_EDGE_RIGHT,		//右壁のエッジ検出での補正
	MOT_WALL_EDGE_LEFT,			//左壁のエッジ検出での補正
	MOT_WALL_EDGE_MAX,
}enMOT_WALL_EDGE_TYPE;

/* 動作タイプ */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,				// [01] 台形加速
	MOT_ACC_CONST_DEC_CUSTOM,	 	// [02] 台形加速（等速値変更）
	MOT_ACC_CONST,					// [03] 加速＋等速
	MOT_ACC_CONST_CUSTOM,		   	// [04] 加速＋等速（加速値変更）
	MOT_CONST_DEC,					// [05] 等速＋減速
	MOT_CONST_DEC_CUSTOM,			// [06] 等速＋減速（減速値変更）
	
	/* cos近似 */
	MOT_ACC_CONST_DEC_SMOOTH,			// [07] 台形加速
	MOT_ACC_CONST_DEC_SMOOTH_CUSTOM,	// [08] 台形加速（等速値変更）
	MOT_ACC_CONST_SMOOTH,				// [09] 加速＋等速
	MOT_ACC_CONST_SMOOTH_CUSTOM,		// [10] 加速＋等速（加速値変更）
	MOT_CONST_DEC_SMOOTH,				// [11] 等速＋減速
	MOT_CONST_DEC_SMOOTH_CUSTOM,		// [12] 等速＋減速（減速値変更）
	
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* 直進タイプ */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// 通常の直進
	MOT_GO_ST_SKEW,				// 斜めの直進
	MOT_GO_ST_SMOOTH,			// cos近似直進
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
/* マクロ                                          */
//**************************************************
#define IS_R_SLA(a)			( ( (a) % 2 == 0 ) ? (TRUE) : (FALSE))

//**************************************************
// グローバル変数
//**************************************************

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void MOT_setTrgtSpeed( FLOAT f_speed );
PUBLIC void MOT_setNowSpeed( FLOAT f_speed );
PUBLIC void MOT_setSlaStaSpeed( FLOAT f_speed );
PUBLIC FLOAT MOT_getSlaStaSpeed( void );

PUBLIC void MOT_goBlock_FinSpeed(FLOAT num, FLOAT f_fin);
PUBLIC void MOT_goSkewBlock_FinSpeed(FLOAT f_num, FLOAT f_fin);
PUBLIC void MOT_goBlock_Const( FLOAT f_num);
PUBLIC void MOT_goSla( enMOT_SULA_CMD en_type, stSLA *p_sla);
PUBLIC void MOT_goHitBackWall( void );
PUBLIC void MOT_Turn( enMOT_TURN_CMD en_type );
PUBLIC void MOT_goBack_Const( FLOAT f_dist );
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type);
PUBLIC void MOT_turn2( enMOT_TURN_CMD en_type, FLOAT f_trgtAngleS );

PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type );
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void );
PUBLIC void MOT_setWallEdge( BOOL bl_val );
PRIVATE void MOT_Failsafe( BOOL* exists );
PUBLIC void MOT_circuit(FLOAT x,FLOAT y, enMOT_SULA_CMD en_type, int num, FLOAT f_speed);

#endif