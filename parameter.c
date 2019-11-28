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
#include <math.h>							// 数値計算

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
PRIVATE stSLA				st_Sla[SLA_TYPE_MAX];				// スラローム時の走行パラメータ
PRIVATE	BOOL				bl_cntType		= false;			// カウントタイプ(false:探索、true:最短)

/* ============ */
/*  速度データ  */
/* ============ */

	/* 直進速度データ */
	PRIVATE	CONST stSPEED	f_StSpeedData[PARAM_MOVE_SPEED_MAX]	= {
		
		// 加速度		減速度		角速度		角減速度
		{ 2000,			2000,		0,			0			},		// 超低速(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0			},		// 低速(PARAM_SLOW)
		{ 2500,			3500,		0,			0			},		// 通常(PARAM_NORMAL)
		{ 10000,		10000,		0,			0			},		// 高速(PARAM_FAST)
		{ 12000,		10000,		0,			0			}		// 超高速(PARAM_VERY_FAST)
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
		{ 0,			0,			2000,		2000		},		// 超低速(PARAM_VERY_SLOW)
		{ 0,			0,			2200,		2200		},		// 低速(PARAM_SLOW)
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

	/* 斜め走行速度データ */
	PRIVATE	CONST stSPEED f_SkewSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	加速度		減速度		角加速度	 角減速度
		{ 2000,			2000,		0,			0,		},		// 超低速(PARAM_VERY_SLOW)
		{ 2000,			2000,		0,			0,		},		// 低速(PARAM_SLOW)
		{ 2000,			2000,		0,			0,		},		// 通常(PARAM_NORMAL)
		{ 2000,			2000,		0,			0,		},		// 高速(PARAM_FAST)
		{ 2000,			2000,		0,			0,		}		// 超高速(PARAM_VERY_FAST)
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
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.07f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_ACC		
			{0.28f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		1.7f,		0.0f,		1.0f,		0.2f,		0.08f,		0.0f,		3.0f,		0.1f,		0.2f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 低速(PARAM_SLOW) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 通常(PARAM_NORMAL) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.23f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.15f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 高速(PARAM_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.050f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.24f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.13f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
		/* 超高速(PARAM_VERY_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.05f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_ACC		
			{0.0f,		0.25f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_CONST
			{0.0f,		0.1f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.0f,		0.0f,	},		// PARAM_DEC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_ACC
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_CONST
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		5.5f,		0.01f,		0.04f,		0.0f,		8.0f,		0.05f,		0.025f,		0.01f,	},		// PARAM_BACK_DEC
			{0.055f,	0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_ACC
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_CONST
			{0.0f,		0.2f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.07f,		0.0f,	},		// PARAM_SKEW_DEC
			{0.12f,		0.0f,		0.0f,		0.0f,		0,			0.0f,		0,			0,			0,			0.0f,		0,			0,			0,			0,		},		// PARAM_HIT_WALL
			{0.1f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_ACC_SMOOTH		
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.08f,		0.0f,		2.0f,		0.0f,		0.03f,		0.0f,	},		// PARAM_CONST_SMOOTH
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		4.0f,		0.15f,		0.08f,		0.0f,		2.0f,		0.0f,		0.015f,		0.0f,	},		// PARAM_DEC_SMOOTH
		},
	};
	
	/* 旋回ゲインデータ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TURN_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.065f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.5f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.0f,		0.0f,		1.0f,		0.1f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 低速(PARAM_SLOW) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.065f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		2.0f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.5f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.0f,		1.0f,		0.0f,		1.0f,		0.1f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 通常(PARAM_NORMAL) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 高速(PARAM_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
		/* 超低速(PARAM_VERY_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.06f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_ACC_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.7f,		0.0f,		0.0f,		0.0f,		0,			0		},		// PARAM_CONST_TURN
			{0.0f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		0.09f,		0.0f,		1.5f,		0.0f,		0.054f,		0.0f,		0,			0		},		// PARAM_DEC_TURN
		},
	};
	
	/* スラロームゲインデータ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SULA_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.01f,		0.18f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.2f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.2f,		0.0f,		3.0f,		0.5f,		0.0f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* 低速(PARAM_SLOW) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.013f,		0.13f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.20f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0002f,	0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* 通常(PARAM_NORMAL) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_ENTRY_SLA
			{0.0f,		0.3f,		0.013f,		0.13f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.3f,		0.0f,		0.20f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.3f,		0.0002f,	0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.3f,		0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		3.0f,		1.0f,		0.1f,		0.0f,	},		// PARAM_EXIT_SLA
		},
		
		/* 高速(PARAM_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		4.0f,		0.0f,		0.48f,		0.01f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.34f,		0.0f,		12.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		0.7f,		0.0f,		0.0f,		0.0f,		0.5f,		0.0f,		4.0f,		0.0f,		0.48f,		0.01f,	},		// PARAM_EXIT_SLA
		},
		
		/* 超高速(PARAM_VERY_FAST) */
		{// FF_加速度	FF_速度		FF_角加速度	 FF_角速度	  速度kp	  速度ki	  位置kp	  位置ki	  角速度kp	   角速度ki		角度kp		角度ki		 壁kp	 	壁kd
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		5.0f,		0.0f,		0.8f,		0.2f	},		// PARAM_ENTRY_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_ACC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_CONST_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.8f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f	},		// PARAM_DEC_SLA
			{0.0f,		0.0f,		0.0f,		0.0f,		5.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f,		5.0f,		0.0f,		0.8f,		0.2f,	},		// PARAM_EXIT_SLA
		},
	};
	
/* ================= */
/*  スラロームの距離 */
/* ================= */
	PRIVATE	CONST FLOAT f_SlaDistData[PARAM_MOVE_SPEED_MAX][SLA_CORR_DIST_MAX]	= {
		
		//進入距離		退避距離 
		{ 	0,			0,		},		// 超低速(PARAM_VERY_SLOW)
		{ 	0,			0,		},		// 低速(PARAM_SLOW)
		{ 	0,			0,		},		// 通常(PARAM_NORMAL)
		{ 	0,			0,		},		// 高速(PARAM_FAST)
		{ 	0,			0,		}		// 超高速(PARAM_VERY_FAST)
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
		case PARAM_HIT_WALL:											// 壁あて制御		
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
		
		case PARAM_SKEW_ACC:											// 加速中(斜め)
		case PARAM_SKEW_CONST:											// 等速中(斜め)
		case PARAM_SKEW_DEC:											// 減速中(斜め)
			p_adr = &f_SkewSpeedData[en_Speed_st];
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
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode)
{
	return &st_Sla[en_mode];
}

// *************************************************************************
//   機能		： スラロームの走行パラメータを作成する
//   注意		： なし
//   メモ		： なし
//   引数		： 進入速度[mm/s]，角加速度[rad/s^2]，横G[mm/s^2]，スラロームタイプ
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.9.8			TKR			新規
// *************************************************************************/
PUBLIC void PARAM_makeSla( FLOAT f_speed, FLOAT f_angAcc, FLOAT f_g, enSLA_TYPE en_mode, enPARAM_MOVE_SPEED en_speed){
	
	FLOAT	f_start_x;					// 開始x位置 [mm]
	FLOAT	f_start_y;					// 開始y位置 [mm]
	FLOAT	f_final_x;					// 最終x位置 [mm]
	FLOAT	f_final_y;					// 最終y位置 [mm]
	FLOAT	f_final_ang;				// 角減速時の最終角度 [rad]	
	FLOAT	f_maxAngleV		= 0;		// 最大角速度[rad/s]
	FLOAT	f_timeAcc		= 0;		// 加速時間[s]
	FLOAT	f_accAngle		= 0;		// 加速角度[rad]
	FLOAT	f_timeConst		= 0;		// 等速時間[s]
	FLOAT	f_constAngle	= 0;		// 等速角度[rad]
	FLOAT	f_ang			= 0;		// 演算用、角度 [rad]
	FLOAT	f_time			= 0;		// 演算用、時間 [s]
	FLOAT	f_x;						// 演算用x位置 [mm]
	FLOAT	f_y;						// 演算用y位置 [mm]
	USHORT	i = 0;						// ループ用
	stSLA	*p_adr 			= &st_Sla[en_mode];	// 記録する走行データ

	//デバッグ用
	FLOAT	f_x_acc = 0;
	FLOAT	f_y_acc = 0;
	FLOAT	f_x_const = 0;
	FLOAT	f_y_const = 0;
	FLOAT	f_x_dec = 0;
	FLOAT	f_y_dec = 0;

	/* スラロームに応じた設定値からスラロームに必要なパラメータを演算する */
	switch(en_mode){

		case SLA_90:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK;
			f_final_y   = HALF_BLOCK;
			f_final_ang = 90.0f* DEG_TO_RAD;
			break;

		case SLA_45:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 0.75f;
			f_final_y   = BLOCK * 0.75f;
			f_final_ang = 45.0f * DEG_TO_RAD;
			break;
			
		case SLA_N90:
			f_start_x   = HALF_BLOCK * 0.5f * 1.4142f;
			f_start_y   = 0.0f;
			f_final_x   = HALF_BLOCK * 1.4142f;
			f_final_y   = HALF_BLOCK * 0.5f * 1.4142f;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;
			
		case SLA_135:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 1.25f;
			f_final_y   = BLOCK * 0.25;
			f_final_ang = 135.0f * DEG_TO_RAD;
			break;
	}
#ifdef TEST
	printf("f_speed = %f[mm/s]\n\r",f_speed);
	printf("f_angAcc = %f[rad/s]\n\r",f_angAcc);	
	printf("f_g = %f[mm/s]\n\r",f_g);	
#endif

	/* 加減速角度の算出 */
	f_maxAngleV		= f_g / f_speed;								// 最大角速度[rad/s] （ω[rad/s] = g[mm/s^2] / v[mm/s] ）
	f_timeAcc		= f_maxAngleV / f_angAcc;						// 最大の角速度になるまでの加速時間[s]
	f_accAngle		= 0.5f * f_angAcc * f_timeAcc * f_timeAcc;		// 加速をする区間の角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
	f_constAngle	= f_final_ang - f_accAngle * 2;					// 等角速度の区間の角度[rad] (θ[rad] = Total角度 - 加速角度 + 減速角度 )
	f_timeConst		= f_constAngle / f_maxAngleV;					// 最大の角速度で動作する時間[s]（ t[s] = θ[rad] / ω[rad/s] ）
	
#ifdef TEST	
	printf("f_maxAngleV = %f[rad/s]\n\r",f_maxAngleV);
	printf("f_timeAcc = %f[s]\n\r",f_timeAcc);
	printf("f_accAngle = %f[rad]\n\r",f_accAngle);
	printf("f_constAngle = %f[rad]\n\r",f_constAngle);
	printf("f_timeConst = %f[s]\n\r",f_timeConst);
#endif

	/* -------------------------------- */
	/*  スラローム完了時の位置を求める  */
	/* -------------------------------- */
	/* 座標開始位置 */
	f_x		= f_start_x;
	f_y		= f_start_y;

	/* 加速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	=  0.001f * (FLOAT)i;							// 時間[s]
		f_ang	=  0.5f * f_angAcc * f_time * f_time;			// 角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y座標[mm]
	}
	
	f_x_acc = f_x;
	f_y_acc = f_y;
#ifdef TEST	
	printf("f_x_acc = %f\n\r",f_x_acc);
	printf("f_y_acc = %f\n\r",f_y_acc);
#endif
	/* 等速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeConst*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// 時間[s]
		f_ang	 = f_accAngle + f_maxAngleV * f_time;			// 角度[rad] (θ[rad] = ω[rad/s] * t[s] )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y座標[mm]
	}

	f_x_const = f_x ;
	f_y_const = f_y ;
#ifdef	TEST
	printf("f_x_const = %f\n\r",f_x_const);
	printf("f_y_const = %f\n\r",f_y_const);
#endif
	/* 減速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// 時間[s]
		f_ang	 = f_accAngle + f_constAngle +0.5f * f_angAcc * f_time * f_time;	// 角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y座標[mm]
	}
	
	f_x_dec = f_x ;
	f_y_dec = f_y ;
#ifdef TEST	
	printf("f_x_dec = %f\n\r",f_x_dec);
	printf("f_y_dec = %f\n\r",f_y_dec);
#endif
	/* ---------------------------- */
	/*  スラローム用パラメータ作成  */
	/* ---------------------------- */
    	p_adr->f_speed						= f_speed;										// 進入速度[mm/s]
    	p_adr->f_angAcc						= f_angAcc * RAD_TO_DEG ;						// 角加速度[deg/s]
    	p_adr->f_angvel						= f_maxAngleV * RAD_TO_DEG;						// 最大角速度を算出 最大角速度[deg/s]
    	p_adr->us_accAngvelTime				= (USHORT)( f_timeAcc * 1000.0f );				// 角加速時間[msec]
    	p_adr->us_constAngvelTime			= (USHORT)( f_timeConst * 1000.0f );			// 等角速時間[msec]
    	p_adr->f_ang_AccEnd					= f_accAngle * RAD_TO_DEG;						// 角加速完了角度[deg]
    	p_adr->f_ang_ConstEnd				= ( f_accAngle + f_constAngle ) * RAD_TO_DEG;	// 等角速度完了角度[deg]
		p_adr->f_ang_Total					= f_final_ang * RAD_TO_DEG;						// 全移動角度[deg]
#ifdef TEST	
	printf("\n========PARAMETER======\n\r");
	printf("f_speed = %f[mm/s]\n\r",f_speed);
	printf("f_angAcc = %f[deg/s]\n\r",f_angAcc*180.0f/3.1416f);
	printf("us_accAngvelTime = %f[ms]\n\r",f_timeAcc*1000);
	printf("us_constAngvelTime = %f[ms]\n\r",f_timeConst*1000);
	printf("f_ang_AccEnd = %f[deg]\n\r",f_accAngle*180.0f/3.1416f);
	printf("f_ang_ConstEnd = %f[deg/s]\n\r",( f_accAngle + f_constAngle )*180.0f/3.1416f);
	printf("f_ang_Total = %f[deg]\n\r",f_final_ang*180.0f/3.1416f);
#endif	
	/* 必要な進入と退出の距離を算出する */
	switch(en_mode){
		case SLA_90:
			p_adr->f_escapeLen = f_final_x - f_x + PARAM_getSlaCorrDist(en_speed,SLA_ESCAPE_ADD);
			p_adr->f_entryLen  = f_final_y - f_y + PARAM_getSlaCorrDist(en_speed,SLA_ENTRY_ADD);
			break;

		case SLA_45:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y - ( f_final_x - f_x );
			break;

		case SLA_N90:
			p_adr->f_escapeLen = f_final_x - f_x;
			p_adr->f_entryLen  = f_final_y - f_y;
			break;

		case SLA_135:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y + ( f_final_x - f_x );
			break;
	}
	
}