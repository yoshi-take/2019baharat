// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： parameter
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力
#include <common_define.h>					// common_define

#include <parameter.h>						// parameter

//**************************************************
// 定義（define）
//**************************************************
/* インデックス演算に使用 */
#define GET_INDEX_ST(i)			( i - PARAM_ST_TOP - 1 )		// 直進用のインデックスを取得
#define GET_INDEX_TURN(i)		( i - PARAM_TURN_TOP - 1 )		// 旋回用のインデックスを取得
#define GET_INDEX_SLA(i)		( i - PARAM_SLA_TOP - 1 )		// スラローム用のインデックスを取得

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_st		= PARAM_NORMAL;		// 直進時の移動速度タイプ
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_turn	= PARAM_NORMAL;		// 旋回時の移動速度タイプ
PRIVATE	enPARAM_MOVE_SPEED	en_Speed_sla	= PARAM_NORMAL;		// スラローム時の移動速度タイプ
PRIVATE	BOOL				bl_cntType		= false;			// カウントタイプ(false:探索、true:最短)

/* ============ */
/*  速度データ  */
/* ============ */

	/* 直進速度データ */
	PRIVATE	CONST stSPEED	f_StSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// 加速度		減速度		角速度		角減速度
		{ 800,			1000,		0,			0			},		// 超低速(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0			},		// 低速(PARAM_SLOW)
		{ 2000,			2500,		0,			0			},		// 通常(PARAM_NORMAL)
		{ 2000,			2000,		0,			0			},		// 高速(PARAM_FAST)
		{ 2000,			2000,		0,			0			}		// 超高速(PARAM_VERY_FAST)
	};
	
	/* 直進速度データ(cos近似) */
	PRIVATE	CONST stSPEED	f_StSpeedData_Smooth[PARAM_MOVE_SPEED_MAX]	= {
		
		// 最大加速度	最大減速度	最大角速度	最大角減速度
		{ 800,			1000,		0,			0			},		// 超低速(PARAM_VERY_SLOW)
		{ 2500,			2500,		0,			0			},		// 低速(PARAM_SLOW)
		{ 2000,			2500,		0,			0			},		// 通常(PARAM_NORMAL)
		{ 3000,			3000,		0,			0			},		// 高速(PARAM_FAST)
		{ 3200,			3200,		0,			0			}		// 超高速(PARAM_VERY_FAST)
	};
	
	/* 旋回速度データ */
	PRIVATE	CONST stSPEED	f_TurnSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// 加速度		減速度		角速度		角減速度
		{ 0,			0,			2000,		3000		},		// 超低速(PARAM_VERY_SLOW)
		{ 0,			0,			1800,		1800		},		// 低速(PARAM_SLOW)
		{ 0,			0,			1800,		1800		},		// 通常(PARAM_NORMAL)
		{ 0,			0,			1800,		1800		},		// 高速(PARAM_FAST)
		{ 0,			0,			1800,		1800		}		// 超高速(PARAM_VERY_FAST)
	};

	/* スラローム速度データ */
	PRIVATE CONST stSPEED f_SlaSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	加速度		減速度		角加速度		角減速度
		{ 1800,			1800,		1800,			1800,		},		// 超低速(PARAM_VERY_SLOW)
		{ 1800,			1800,		1800,			1800,		},		// 低速(PARAM_SLOW)
		{ 1800,			1800,		1800,			1800,		},		// 通常(PARAM_NORMAL)
		{ 3000,			3000,		1800,			1800,		},		// 高速(PARAM_FAST)
		{ 1800,			1800,		1800,			1800,		}		// 超高速(PARAM_VERY_FAST)
	};


/* ============== */
/*  ゲインデータ  */
/* ============== */
	// 【アドバイス】 
	//    もしもゲインのパラメータ数を増やしたい場合は、stGAINのメンバと↓のデータを増やすだけでOKです。
	//    PARAM_getGain()でパラメータのアドレスを取得して、追加したメンバを参照して下さい。

	/* 直進ゲインデータ */
	PRIVATE CONST stGAIN f_StGainData[PARAM_MOVE_SPEED_MAX][PARAM_ST_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.1f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST
			{0.0f,		1.0f,		4.0f,		0.15f,		0.08f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		5.5f,		0.01f,		0.04f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.0f,		0.0f,		0.0f,		0.08f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.0f,		4.0f,		0.15f,		0.08f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 低速(PARAM_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.05f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_CONST
			{0.0f,		2.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.0f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 通常(PARAM_NORMAL) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.1f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 高速(PARAM_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.05f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.2f,		0.0f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 超高速(PARAM_VERY_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.08f,		5.0f,		0.0f,		0.0f,		2.0f,		5.0f,		0.01f,		0.8f,		0.15f,	},		// PARAM_ACC		
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		5.0f,		0.01f,		0.8f,		0.15f,	},		// PARAM_CONST
			{0.0f,		5.0f,		15.0f,		0.0f,		1.0f,		5.0f,		0.01f,		0.4f,		0.15f,	},		// PARAM_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_BACK_DEC
			{0.1f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.7f,		7.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0,			0,			0,			0,			0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.07f,		1.5f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		1.5f,		0.0f,		0.0f,		0.08f,		4.0f,		0.0f,		0.095f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
	};
	
	/* 旋回ゲインデータ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TURN_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角度ki		壁kp		壁kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 低速(PARAM_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角度ki		壁kp		壁kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 通常(PARAM_NORMAL) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角度ki		壁kp		壁kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 高速(PARAM_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角度ki		壁kp		壁kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 超低速(PARAM_VERY_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角度ki		壁kp		壁kd
			{0.06f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.5f,		0.09f,		0.0f,		1.5f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
	};
	
	/* スラロームゲインデータ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SULA_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* 低速(PARAM_SLOW) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		4.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* 通常(PARAM_NORMAL) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		2.0f,		4.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* 高速(PARAM_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.34f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.7f,		0.0f,		0.0f,		0.5f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* 超高速(PARAM_VERY_FAST) */
		{	// FF		速度kp		位置kp		位置ki		角速度kp	角度kp		角速度ki	壁kp		壁kd
			{0.0f,		5.0f,		0.0f,		0.0f,		1.0f,		5.0f,		0.0f,		0.8f,		0.2f	},		// PARAM_ENTRY_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.8f,		5.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		5.0f,		0.0f,		0.0f,		1.0f,		5.0f,		0.0f,		0.8f,		0.2f,	},		// PARAM_EXIT_SLA
		},
	};
	
/* ================= */
/*  スラロームの距離 */
/* ================= */
	PRIVATE	CONST FLOAT f_SlaDistData[PARAM_MOVE_SPEED_MAX][SLA_CORR_DIST_MAX]	= {
		
		//進入距離		退避距離 
		{ 	2,			2,		},		// 超低速(PARAM_VERY_SLOW)
		{ 	2,			2,		},		// 低速(PARAM_SLOW)
		{ 	2,			2,		},		// 通常(PARAM_NORMAL)
		{ 	2,			2,		},		// 高速(PARAM_FAST)
		{ 	-2,			-2,		}		// 超高速(PARAM_VERY_FAST)
	};

// *************************************************************************
//   機能		： 制御方法に対応した動作速度を設定する
//   注意		： 動作前にあらかじめ設定しておく
//   メモ		： 速度値やゲイン値を取得する際に、度の動作速度のパラメータを取得するかを決定するために使用する
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.3.11			吉田			新規
// *************************************************************************/
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed ){
	switch( en_mode ){
		
		case PARAM_ST:
			en_Speed_st = en_speed;
			break;
		
		case PARAM_TURN:
			en_Speed_turn = en_speed;
			break;
		
		case PARAM_SLA:
			en_Speed_sla = en_speed;
			break;
			
		default:
			printf("設定した速度のパラメータタイプがありません \n\r");
			break;
	}
}


// *************************************************************************
//   機能		： 速度データのポインタを取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.4.3			吉田			新規
// *************************************************************************/
PUBLIC	CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode ){
	
	const stSPEED*	p_adr;
	
	switch( en_mode ){
		
		case PARAM_ST:													// 直進
		case PARAM_ACC:													// 加速中(直進)
		case PARAM_CONST:												// 等速中(直進)
		case PARAM_DEC:													// 減速中(直進)
		case PARAM_BACK_ACC:											// 加速中(後進)
		case PARAM_BACK_CONST:											// 等速中(後進)
		case PARAM_BACK_DEC:											// 減速中(後進)
		case PARAM_SKEW_ACC:											// 加速中(斜め)
		case PARAM_SKEW_CONST:											// 等速中(斜め)
		case PARAM_SKEW_DEC:											// 減速中(斜め)
		case PARAM_HIT_WALL:											// 壁あて制御		
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
			
		case PARAM_ACC_SMOOTH:											// 加速中(直進 cos近似)
		case PARAM_CONST_SMOOTH:										// 等速中(直進 cos近似)
		case PARAM_DEC_SMOOTH:											// 減速中(直進 cos近似)
			p_adr = &f_StSpeedData_Smooth[en_Speed_st];
			break;
			
		case PARAM_TURN:												// 旋回
		case PARAM_ACC_TURN:											// 加速中(超地信旋回)
		case PARAM_CONST_TURN:											// 等速中(超地信旋回)
		case PARAM_DEC_TURN:											// 減速中(超地信旋回)
			p_adr = &f_TurnSpeedData[en_Speed_turn];
			break;
			
		case PARAM_SLA:													// スラローム
		case PARAM_ENTRY_SURA:											// スラローム前の前進動作(スラローム)
		case PARAM_ACC_SURA:											// 加速中(スラローム)
		case PARAM_CONST_SURA:											// 等速中(スラローム)
		case PARAM_DEC_SURA:											// 減速中(スラローム)
		case PARAM_EXIT_SURA:											// スラローム後の前進動作(スラローム)
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
			
		default:
			printf("設定した速度タイプがありません\n\r");
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
	}
	
	return p_adr;

}


// *************************************************************************
//   機能		： 速度データのポインタを取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.4.3			吉田			新規
// *************************************************************************/
PUBLIC	CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode ){
	
	const stGAIN*	p_adr;
	
	switch( en_mode ){
		
		case PARAM_ACC:													// 加速中(直進)
		case PARAM_CONST:												// 等速中(直進)
		case PARAM_DEC:													// 減速中(直進)
		case PARAM_BACK_ACC:											// 加速中(後進)
		case PARAM_BACK_CONST:											// 等速中(後進)
		case PARAM_BACK_DEC:											// 減速中(後進)
		case PARAM_SKEW_ACC:											// 加速中(斜め)
		case PARAM_SKEW_CONST:											// 等速中(斜め)
		case PARAM_SKEW_DEC:											// 減速中(斜め)
		case PARAM_HIT_WALL:											// 壁あて制御
		case PARAM_ACC_SMOOTH:											// 加速中(直進 cos近似)
		case PARAM_CONST_SMOOTH:										// 等速中(直進 cos近似)
		case PARAM_DEC_SMOOTH:											// 減速中(直進 cos近似)
			p_adr = &f_StGainData[en_Speed_st][GET_INDEX_ST( en_mode )];
			break;
			
		case PARAM_ACC_TURN:											// 加速中(超地信旋回)
		case PARAM_CONST_TURN:											// 等速中(超地信旋回)
		case PARAM_DEC_TURN:											// 減速中(超地信旋回)
			p_adr = &f_TurnGainData[en_Speed_turn][GET_INDEX_TURN( en_mode )];
			break;
		
		case PARAM_ENTRY_SURA:											// スラローム前の前進動作(スラローム)
		case PARAM_ACC_SURA:											// 加速中(スラローム)
		case PARAM_CONST_SURA:											// 等速中(スラローム)
		case PARAM_DEC_SURA:											// 減速中(スラローム)
		case PARAM_EXIT_SURA:											// スラローム後の前進動作(スラローム)
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
		
		default:														// Err、とりあえず・・・（メモリ破壊を防ぐため）
			printf("設定したゲインタイプがありません \n\r");
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
	}
	
	return p_adr;
}	
			

// *************************************************************************
//   機能		： カウント数を距離[mm]に変換する
//   注意		： 等高線などのオーバーヘッドがあるため、1step数を調整して探索時は区画合わせを行う。
//   メモ		： ADJ_1STEP_SEARCHは現物合わせ
//   引数		： なし
//   返り値		： [mm]
// **************************    履    歴    *******************************
// 		v1.0		2018.3.28			吉田			新規
//		v2.0		2018.9.9			吉田			1717仕様に変更
// *************************************************************************/
PUBLIC FLOAT F_CNT2MM( LONG l_cnt ){
	
	/* 探索 */
	if( bl_cntType == false ){
		return (FLOAT)l_cnt * DIST_1STEP( ADJ_1STEP_SEARCH ) * ENC_CONV;
	
	/* 最短 */
	}else{
		return (FLOAT)l_cnt * DIST_1STEP( ADJ_1STEP_DIRECT ) * ENC_CONV;
	
	}
}


// *************************************************************************
//   機能		： 1stepあたりのカウントを行う場合のタイプを設定する
//   注意		： 等高線などのオーバーヘッドがあるため、1step数を調整して探索時は区画合わせを行う。
//   メモ		： 速度値やゲイン値を取得する際に、度の動作速度のパラメータを取得するかを決定するために使用する
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.3.25			吉田			新規
// *************************************************************************/
PUBLIC void PARAM_setCntType( BOOL bl_type )
{
	bl_cntType = bl_type;
}

// *************************************************************************
//   機能		： スラロームの退避距離と進入距離の補正
//   注意		： なし
//   メモ		： なし
//   引数		： 速度タイプ，進入or退避
//   返り値		： 補正距離[mm]
// **************************    履    歴    *******************************
// 		v1.0		2018.11.30			吉田			新規
// *************************************************************************/
PUBLIC FLOAT PARAM_getSlaCorrDist( enPARAM_MOVE_SPEED en_speed , enSlaCorrDist en_dist)
{
	return f_SlaDistData[en_speed][en_dist];	
}

// *************************************************************************
//   機能		： スラロームの走行パラメータの格納先アドレスを取得する
//   注意		： なし
//   メモ		： なし
//   引数		： スラロームタイプ
//   返り値		： スラロームデータアドレス
// **************************    履    歴    *******************************
// 		v1.0		2014.09.01			外川			新規
// *************************************************************************/
#if 0
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode)
{
	return &st_Sla[en_mode];
}
#endif