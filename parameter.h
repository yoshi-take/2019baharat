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

// 多重コンパイル抑止
#ifndef _PARAMETER_H
#define _PARAMETER_H

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
//#define FUNC_DIST_AUTO_THRESH											// センサ閾値チューニング機能（定義：有効、未定義：無効（センサの固定値が使用される））

//**************************************************
// 定義（define）	 非調整パラメータ 
//**************************************************
#define PI					( 3.14159f )			// π
#define	ENC_CONV			( 0.2f )				// ピニオン8T，スパー40T
#define DEG_TO_RAD	 	 	( 3.1416f/180.0f );		// 度数→弧度
#define RAD_TO_DEG  		( 180.0f/3.1416f );		// 弧度→度数

/* 迷路寸法 */
#define HALF_BLOCK					( 90.0f )							// 半区間 [mm]
#define BLOCK						( 180.0f )							// １区間 [mm]
#define HALF_BLOCK_SKEW				( 127.28f )							// 斜め半区間 [mm]
#define BLOCK_SKEW					( 254.56f )							// 斜め１区間 [mm]

/* アドレス */
#define	ADR_MAP						( 0x00100000 )						// 迷路バックアップ用アドレス
#define	ADR_SEN						( 0x00101000 )						// センサ用データフラッシュアドレス

/* ジャイロ，温度，加速度のスケール */
#define GYRO_SCALE_FACTOR			(16.4f)								// ±2000[dps]
#define TEMP_SCALE_FACTOR			(333.87f)							// 
#define ACC_SCALE_FACTOR			(8192.0f)							// ±4[g]

//**************************************************
// 定義（define）	チューニングが必要なパラメータ 
//**************************************************
/* 迷路サイズ */
#define GOAL_MAP_X					( 7 )								// ゴールのX区画数（横方向） [区画]
#define GOAL_MAP_Y					( 4 )								// ゴールのY区画数（縦方向） [区画]
#define MAP_X_SIZE					( 16 )								// 迷路のX区画数（横方向） [区画]
#define MAP_Y_SIZE					( 16 )								// 迷路のY区画数（縦方向） [区画]

#define MAP_X_SIZE_REAL				( 16 )								// 迷路の実X区画数（横方向） [区画]
#define MAP_Y_SIZE_REAL				( 16 )								// 迷路の実Y区画数（縦方向） [区画]

#define LIST_NUM					( 4096 )							// コマンド走行のリスト数

/* メカ値 */
#define TIRE_R						( 23.03f )								// タイヤ直径 [mm]
#define ROTATE_PULSE				( 2048 )								// モーター1周のパルス数

#define ADJ_1STEP_SEARCH			( 1 )									// 1stepの調整ゲイン、探索走行用  (分母を上げると沢山進む)
#define ADJ_1STEP_DIRECT			( 1 )									// 1stepの調整ゲイン、Drive走行用  (分母を上げると沢山進む)
#define DIST_1STEP(adj)				( PI * TIRE_R / ROTATE_PULSE * adj)		// 1パルスで進む距離 [mm]

#define MOVE_BACK_DIST				( 0.27f ) 							// 壁当て動作で後退した距離 [区画]
#define MOVE_BACK_DIST_SURA			( 0.27f ) 							// 壁当て動作で後退した距離 [区画]

/* 探索時の尻当て周期 */
#define MAP_SLA_NUM_MAX				( 20 )								// 最大連続スラロームを何回まで許可するか
#define MAP_TURN_NUM_MAX			( 1 )								// 最大連続旋回を何回まで許可するか

/* 迷路中での待ち時間 */
#define MAP_TURN_WAIT				( 100 )								// 超信地旋回探索の動作切り替え待ち時間
#define MAP_SLA_WAIT				( 150 )								// スラローム探索の動作切り替え待ち時間

/* 走行速度 */
#define MAP_SEARCH_SPEED			( 500 ) 							// 探索走行の最大速度[mm/s]
#define MAP_KNOWN_ACC_SPEED			( 1200 )							// 既知区間加速するときの目標速度[mm/s]
#define SEN_BACK_CHK_SPEED			( 180 ) 							// センサチューニングのための移動最大速度[mm/s]

/* フィルタ */
#define SW_GYRO_FILTER_VAL_MIN		( -1.5f )							// フィルタ最小値[dps]（最小値～最大値の間はフィルタがかかる）
#define SW_GYRO_FILTER_VAL_MAX		( 1.5f )							// フィルタ最大値[dps]
#define SW_ACC_FILTER_VAL_MIN		( -0.008f )							// フィルタ最小値[g]（最小値～最大値の間はフィルタがかかる）
#define SW_ACC_FILTER_VAL_MAX		( 0.008f )							// フィルタ最大値[g]

/* 手かざし実行の閾値 */
#define	EXE_THRESH_R				( 1300 )							// 右側の閾値
#define	EXE_THRESH_L				( 800 )							// 左側の閾値

/* フェイルセーフの閾値 */
#define	FAIL_THRESH_ACC				( -35.0f )							// 加速度（この値以下で発動）

/* 距離 */
#define MOT_BACK_SEN_ADJ			( 73.0f )							// 壁～柱+aまで（センサオートチューニングに使用する）
#define MOT_WALL_EDGE_DIST			( 33.0f )							// 壁切れセンサOFF～壁まで

/* 距離センサ(環境変化以外) */
#define SEN_WAIT_CNT				( 175 )								// センサの発光安定待ち（実験値）
#define DIST_NO_WALL_DIV_FILTER		( 30 )								// 壁なしとする差分値
#define DIST_REF_UP					( 30 )								// 壁なしと判断する際に基準値に加算する値
#define DIST_NEAR_WALL				( 800 )								// 起動時のチューニングをする前壁の閾値

/* 距離センサ(環境変化) */
#define R_FRONT_WALL_GAIN			( 1.0f )							// 右前壁ゲイン（基準値に対して）、壁検知値
#define L_FRONT_WALL_GAIN			( 1.0f )							// 左前壁ゲイン（基準値に対して）、壁検知値
//#define R_45_WALL_GAIN				( 1.0f )							// 右45度ゲイン（基準値に対して）、壁検知値
#define R_SIDE_WALL_GAIN			( 1.0f )							// 右横壁ゲイン（基準値に対して）、壁検知値
//#define L_45_WALL_GAIN				( 1.0f )							// 左45度ゲイン（基準値に対して）、壁検知値
#define L_SIDE_WALL_GAIN			( 1.0f )							// 左横壁ゲイン（基準値に対して）、壁検知値
#define R_FRONT_WALL_CTRL_GAIN		( 1.0f )							// 右前壁ゲイン（基準値に対して）、これ以上近いと制御する値
#define L_FRONT_WALL_CTRL_GAIN		( 1.0f )							// 左前壁ゲイン（基準値に対して）、これ以上近いと制御する値
#define R_FRONT_WALL_NO_CTRL_GAIN	( 1.0f )							// 右前壁ゲイン（基準値に対して）、これ以上近いと制御しない値
#define L_FRONT_WALL_NO_CTRL_GAIN	( 1.0f )							// 左前壁ゲイン（基準値に対して）、これ以上近いと制御しない値
#define R_FRONT_WALL_HIT_GAIN		( 1.0f )							// 右前壁ゲイン（基準値に対して）、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）
#define L_FRONT_WALL_HIT_GAIN		( 1.0f )							// 左前壁ゲイン（基準値に対して）、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）
#define R_FRONT_SKEW_ERR1_GAIN		( 0.0f )							// 右前壁、斜め走行時の補正閾値1
#define R_FRONT_SKEW_ERR2_GAIN		( 0.0f )							// 右前壁、斜め走行時の補正閾値2
#define R_FRONT_SKEW_ERR3_GAIN		( 0.0f )							// 右前壁、斜め走行時の補正閾値3
#define L_FRONT_SKEW_ERR1_GAIN		( 0.0f )							// 左前壁、斜め走行時の補正閾値1
#define L_FRONT_SKEW_ERR2_GAIN		( 0.0f )							// 左前壁、斜め走行時の補正閾値2
#define L_FRONT_SKEW_ERR3_GAIN		( 0.0f )							// 左前壁、斜め走行時の補正閾値3

/* ↓のセンサ値は、FUNC_DIST_AUTO_THRESHが有効ならばFALSHのデータで上書きされて、無効ならば正式値として使用される */
#define R_FRONT_REF					( 2450 )							// 右前壁、基準値
#define L_FRONT_REF					( 2280 )							// 左前壁、基準値
//#define R_45_REF					( 580 )							// 右45度、基準値
//#define L_45_REF					( 440 )							// 左45度、基準値
#define R_SIDE_REF					( 928 )							// 右横壁、基準値
#define L_SIDE_REF					( 790 )							// 左横壁、基準値
#define R_FRONT_WALL				( 656 )							// 右前壁、壁検知値
#define L_FRONT_WALL				( 697 )							// 左前壁、壁検知値
//#define R_45_WALL					( 270 )							// 右45度、壁検知値
#define R_SIDE_WALL					( 437 )							// 右横壁、壁検知値
//#define L_45_WALL					( 180 )							// 左45度、壁検知値
#define L_SIDE_WALL					( 560 )							// 左横壁、壁検知値
#define R_FRONT_WALL_CTRL			( 3334 )							// 右前壁、これ以上近いと制御する値
#define L_FRONT_WALL_CTRL			( 3158 )							// 左前壁、これ以上近いと制御する値
#define R_FRONT_WALL_NO_CTRL		( 3386 )							// 右前壁、これ以上近いと制御しない値
#define L_FRONT_WALL_NO_CTRL		( 2598 )							// 左前壁、これ以上近いと制御しない値
#define R_FRONT_WALL_HIT			( 1050 )						// 右前壁、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）
#define L_FRONT_WALL_HIT			( 1550 )						// 左前壁、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）

/* ログ */
#define CTRL_LOG				( 700 )								// 記録する制御ログの個数
#define CTRL_LOG_CYCLE			( 5 )								// ↑の記録周期[msec]（1より小さい値はNG）
#define SET_LOG					( 100 )								// 設定した動作データのログ数
#define DIST_LOG				( 5 )								// 距離センサのログの個数
#define POS_LOG					( 5 )								// 記録する位置情報ログの個数
#define POS_LOG_INTERVAL		( 5 )								// ←msec毎に記録

/* その他 */
#define TIME_THRE_WAIT			(2)									// この時間以上たつと減速ループから抜ける
		


//**************************************************
// グローバル変数
//**************************************************


//**************************************************
// 列挙体（enum）
//**************************************************

/* 制御方法 */
typedef enum{
	
	/* ========================================== */ 
	/*  パラメータを取得する際に使用するシンボル  */ 
	/* ========================================== */ 
	/* ---------- */
	/*  直進制御  */
	/* ---------- */
	PARAM_ST_TOP = 0,				// カウント用
	// ↓ 動作を追加する場合にはこの間に記載

		PARAM_ACC,					// 加速中(直進)
		PARAM_CONST,				// 等速中(直進)
		PARAM_DEC,					// 減速中(直進)
		PARAM_BACK_ACC,				// 加速中(後進)
		PARAM_BACK_CONST,			// 等速中(後進)
		PARAM_BACK_DEC,				// 減速中(後進)
		PARAM_SKEW_ACC,				// 加速中(斜め)
		PARAM_SKEW_CONST,			// 等速中(斜め)
		PARAM_SKEW_DEC,				// 減速中(斜め)
		PARAM_HIT_WALL,				// 壁あて制御
		PARAM_ACC_SMOOTH,			// 加速中(直進 cos近似)
		PARAM_CONST_SMOOTH,			// 等速中(直進 cos近似)
		PARAM_DEC_SMOOTH,			// 減速中(直進 cos近似)

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_ST_BTM,					// カウント用
	
	/* -------- */
	/*  ターン  */
	/* -------- */
	PARAM_TURN_TOP,					// カウント用
	// ↓  動作を追加する場合にはこの間に記載

		PARAM_ACC_TURN,				// 加速中(超地信旋回)
		PARAM_CONST_TURN,			// 等速中(超地信旋回)
		PARAM_DEC_TURN,				// 減速中(超地信旋回)

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_TURN_BTM,					// カウント用
	
	/* ------------ */
	/*  スラローム  */
	/* ------------ */
	PARAM_SLA_TOP,					// カウント用
	// ↓  動作を追加する場合にはこの間に記載

		PARAM_ENTRY_SURA,			// スラローム前の前進動作(スラローム)
		PARAM_ACC_SURA,				// 加速中(スラローム)
		PARAM_CONST_SURA,			// 等速中(スラローム)
		PARAM_DEC_SURA,				// 減速中(スラローム)
		PARAM_EXIT_SURA,			// スラローム後の前進動作(スラローム)

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_SLA_BTM,					// カウント用
	
	
	/* ===================================================================== */ 
	/*  PARAM_setGainType()にてモードを決める際に引数として使用するシンボル  */ 
	/* ===================================================================== */ 
	PARAM_ST,						// 直進制御
	PARAM_TURN,						// 旋回制御
	PARAM_SLA,						// スラローム制御
	
	
	/* ====================================================== */ 
	/*  作成するデータ数をカウントするために使用するシンボル  */ 
	/* ====================================================== */ 
	PARAM_ST_MAX		= PARAM_ST_BTM   - PARAM_ST_TOP,		// 直進最大数
	PARAM_TURN_MAX		= PARAM_TURN_BTM - PARAM_TURN_TOP,		// 旋回最大数
	PARAM_SULA_MAX		= PARAM_SLA_BTM  - PARAM_SLA_TOP,		// スラローム最大数
	
	
	PARAM_NC = 0xff,
	
}enPARAM_MODE;

/* 動作速度 */
typedef enum{
	
	PARAM_VERY_SLOW = 0,	// 超低速
	PARAM_SLOW,				// 低速
	PARAM_NORMAL,			// 通常
	PARAM_FAST,				// 高速
	PARAM_VERY_FAST,		// 超高速
	
	PARAM_MOVE_SPEED_MAX
	
}enPARAM_MOVE_SPEED;

typedef enum{
	SLA_ENTRY_ADD = 0,
	SLA_ESCAPE_ADD,
	SLA_CORR_DIST_MAX,
}enSlaCorrDist;

//**************************************************
// 構造体（struct）
//**************************************************
/* 速度データ */
typedef struct{
	FLOAT			f_acc;					// 加速度（加速時）
	FLOAT			f_dec;					// 加速度（減速時）
	FLOAT			f_accAngle;				// 角加速度（加速時）
	FLOAT			f_decAngle;				// 角加速度（減速時）
}stSPEED;

/* ゲイン */
typedef struct {
	FLOAT			f_FF_speed_acc;			// フィードフォワード
	FLOAT			f_FF_speed;				// フィードフォワード
	FLOAT			f_FF_angleS_acc;		// フィードフォワード
	FLOAT			f_FF_angleS;			// フィードフォワード

	FLOAT 			f_FB_speed_kp;			// フィードバック、速度 比例制御
	FLOAT 			f_FB_speed_ki;			// フィードバック、速度 積分制御
	FLOAT			f_FB_dist_kp;			// フィードバック、距離 比例制御
	FLOAT 			f_FB_dist_ki;			// フィードバック、距離 積分制御
	FLOAT			f_FB_angleS_kp;			// フィードバック、角速度 比例制御
	FLOAT			f_FB_angleS_ki;			// フィードバック、角速度 積分制御
	FLOAT			f_FB_angle_kp;			// フィードバック、角度 比例制御
	FLOAT			f_FB_angle_ki;			// フィードバック、角度 積分制御
	FLOAT			f_FB_wall_kp;			// フィードバック、壁 比例制御
	FLOAT			f_FB_wall_kd;			// フィードバック、壁 微分制御
}stGAIN;

/* スラロームデータ */
typedef struct{
	FLOAT	f_speed;
	FLOAT	f_angAcc;
	FLOAT	f_angvel;
	FLOAT	f_entryLen;
	FLOAT	f_escapeLen;
	USHORT	us_accAngvelTime;
	USHORT	us_constAngvelTime;
	FLOAT	f_ang_AccEnd;
	FLOAT	f_ang_ConstEnd;
	FLOAT	f_ang_Total;
}stSLA;

/* スラロームタイプ */
typedef enum{
	SLA_90,
	SLA_45,	
	SLA_135,
	SLA_N90,				// 斜め → 90°→ 斜め
	SLA_TYPE_MAX
}enSLA_TYPE;

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void PARAM_setCntType( BOOL bl_type );
PUBLIC FLOAT F_CNT2MM( LONG l_cnt );
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode );
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode );
PUBLIC FLOAT PARAM_getSlaCorrDist( enPARAM_MOVE_SPEED en_speed , enSlaCorrDist en_dist);
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed );
PUBLIC stSLA* PARAM_getSra(enSLA_TYPE en_mode);
PUBLIC void PARAM_makeSla( FLOAT f_speed, FLOAT f_angAcc, FLOAT f_g, enSLA_TYPE en_mode, enPARAM_MOVE_SPEED en_speed);

#endif