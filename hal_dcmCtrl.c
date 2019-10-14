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

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>					// 定義
#include <common_define.h>				// 共通定義
#include <iodefine.h>					// I/O
#include <stdio.h>						// 標準入出力
#include <math.h>						// 数値計算

#include <parameter.h>                  // parameter

#include <hal_dcmCtrl.h>				// DCM_CTRL
#include <hal_dcm.h>                    // DCM
#include <hal_enc.h>					// Encoder
#include <hal_battery.h>				// バッテリー
#include <hal_dist.h>					// DIST
#include <hal_gyro.h>					// GYRO
//**************************************************
// 定義（define）
//**************************************************
#define     VCC_MAX             (8.4f)
#define     FF_BALANCE_R        (1.0f)
#define     FF_BALANCE_L        (1.0f)
#define     FF_HIT_BALANCE_R    (1.0f)
#define     FF_HIT_BALANCE_L    (1.0f)

//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
/* ログ */
typedef struct 
{
	FLOAT	f_time;				// 時間
	FLOAT	f_trgtSpeed;		// 速度（理論値）
	FLOAT	f_nowSpeed;			// 速度（実測値）
	FLOAT	f_trgtPos;			// 位置（理論値）
	FLOAT	f_nowPos;			// 位置（実測値）
	FLOAT	f_trgtAngleS;		// 角速度（理論値）
	FLOAT	f_nowAngleS;		// 角速度（実測値）
	FLOAT	f_trgtAngle;		// 角度（理論値）
	FLOAT	f_nowAngle;			// 角度（実測値）
	FLOAT	f_nowAccel;			// 加速度（実測値）
}stDCMC_SET_LOG;



//**************************************************
// 変数
//**************************************************
/* 制御 */
PRIVATE enCTRL_TYPE     en_Type;                    // 制御方式
PUBLIC  FLOAT           f_Time          = 0;        // 動作時間[sec]
PUBLIC  FLOAT           f_TrgtTime      = 1000;     // 動作目標時間[msec]
PRIVATE UCHAR 			uc_CtrlFlag	    = false;	// フィードバック or フィードフォワード 制御有効フラグ（false:無効、1：有効） 宣言時は無効にすること
PRIVATE LONG            l_CntR          = 0;        // 右モーターカウント量
PRIVATE LONG            l_CntL          = 0;        // 左モーターカウント量

/* 速度制御 */
PRIVATE FLOAT           f_Acc           = 3;        // [速度制御]　加速度
PRIVATE FLOAT           f_BaseSpeed     = 10;       // [速度制御]　初速度
PRIVATE FLOAT           f_LastSpeed     = 180;      // [速度制御]　最終目標速度
PRIVATE FLOAT           f_NowSpeed      = 0;        // [速度制御]　現在の速度[mm/s]     （1[msec]毎に更新される）
PUBLIC  FLOAT           f_TrgtSpeed     = 0;        // [速度制御]　目標速度             （1[msec]毎に更新される）
PUBLIC	FLOAT			f_SpeedErrSum	= 0;		// [速度制御]　速度積分偏差			（1[msec]毎に更新される）

/* 距離制御 */
PRIVATE FLOAT           f_BaseDist      = 0;        // [距離制御]　初期位置
PRIVATE FLOAT           f_LastDist      = 0;        // [距離制御]　最終移動距離
PUBLIC  FLOAT           f_TrgtDist      = 0;        // [距離制御]　目標移動距離         （1[msec]毎に更新される）
PUBLIC  volatile FLOAT  f_NowDist       = 0;        // [距離制御]　現在距離             （1[msec]毎に更新される）
PRIVATE FLOAT           f_NowDistR      = 0;        // [距離制御]　現在距離(右)         （1[msec]毎に更新される）
PRIVATE FLOAT           f_NowDistL      = 0;        // [距離制御]　現在距離(左)         （1[msec]毎に更新される）
PUBLIC  FLOAT           f_DistErrSum    = 0;        // [距離制御]　距離積分偏差         （1[msec]毎に更新される）

/*角速度制御*/
PRIVATE FLOAT           f_AccAngleS     = 3;        // [角速度制御]　角加速度
PRIVATE FLOAT           f_BaseAngleS    = 10;       // [角速度制御]　初期角速度
PRIVATE FLOAT           f_LastAngleS    = 180;      // [角速度制御]　最終目標速度
PUBLIC  FLOAT           f_TrgtAngleS    = 0;        // [角速度制御]　目標角速度         （1[msec]毎に更新される）
PUBLIC	FLOAT			f_AngleSpeedErrSum	= 0;	// [角速度制御]　角速度積分			（1[msec]毎に更新される）

/* 角度制御 */
PRIVATE FLOAT           f_BaseAngle     = 0;        // [角度制御]　初期角度
PRIVATE FLOAT           f_LastAngle     = 0;        // [角度制御]　最終目標角度
PUBLIC  FLOAT           f_TrgtAngle     = 0;        // [角度制御]　目標角度             （1[msec]毎に更新される）
PUBLIC  volatile FLOAT  f_NowAngle      = 0;        // [角度制御]　現在角度             （1[msec]毎に更新される）
PUBLIC  FLOAT           f_AngleErrSum   = 0;        // [角度制御]　距離積分距離         （1[msec]毎に更新される）

extern	PUBLIC volatile FLOAT  f_NowGyroAngle;		 					// ジャイロセンサの現在角度
extern	PUBLIC volatile FLOAT  f_NowGyroAngleSpeed;						// ジャイロセンサの現在角速度
extern	PUBLIC volatile FLOAT  f_NowAccel;								// 進行方向の現在加速度

/* 壁制御 */
PRIVATE LONG            l_WallErr       = 0;        // [壁制御]　壁との偏差             （1[msec]毎に更新される）
PRIVATE FLOAT           f_ErrDistBuf    = 0;        // [壁制御]　距離センサの誤差の積分  （1[msec]毎に更新される）

/* ログ */
PRIVATE	stDCMC_SET_LOG	st_Log[CTRL_LOG];			// ログ	
PRIVATE USHORT			us_LogPt		= 0;		// ログ位置
PRIVATE BOOL			bl_log			= false;	// ログ取得を許可（false:禁止　true:許可）


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
// *************************************************************************
//   機能		： 制御方式からパラメータIDに変換する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.4.4			吉田			新規
// *************************************************************************/
PRIVATE enPARAM_MODE Chg_ParamID( enCTRL_TYPE en_type ){
	
	switch( en_type ){
		
		case CTRL_ACC:			return PARAM_ACC;			// 加速中(直進)
		case CTRL_CONST:		return PARAM_CONST;			// 等速中(直進)
		case CTRL_DEC:			return PARAM_DEC;			// 減速中(直進)
		
		case CTRL_HIT_WALL:		return PARAM_HIT_WALL;		// 壁あて制御
		
		case CTRL_SKEW_ACC:		return PARAM_SKEW_ACC;		// 加速中(斜め直進)
		case CTRL_SKEW_CONST:	return PARAM_SKEW_CONST;	// 等速中(斜め直進)
		case CTRL_SKEW_DEC:		return PARAM_SKEW_DEC;		// 等速中(斜め直進)
		
		case CTRL_ACC_SMOOTH:	return PARAM_ACC_SMOOTH;	// 加速中(直進 cos近似)
		case CTRL_CONST_SMOOTH:	return PARAM_CONST_SMOOTH;	// 等速中(直進 cos近似)
		case CTRL_DEC_SMOOTH:	return PARAM_DEC_SMOOTH;	// 減速中(直進 cos近似)
		
		case CTRL_ACC_TURN:		return PARAM_ACC_TURN;		// 加速中(超信地旋回)
		case CTRL_CONST_TURN:	return PARAM_CONST_TURN;	// 等速中(超信地旋回)
		case CTRL_DEC_TURN:		return PARAM_DEC_TURN;		// 減速中(超信地旋回)
		
		case CTRL_ENTRY_SLA:	return PARAM_ENTRY_SURA;	// スラローム前の前進動作
		case CTRL_ACC_SLA:		return PARAM_ACC_SURA;		// 加速中(スラローム)
		case CTRL_CONST_SLA:	return PARAM_CONST_SURA;	// 等速中(スラローム)
		case CTRL_DEC_SLA:		return PARAM_DEC_SURA;		// 減速中(スラローム)
		case CTRL_EXIT_SLA:		return PARAM_EXIT_SURA;		// スラローム後の前進動作
		
		default:				return PARAM_NC;
	}
}


// *************************************************************************
//   機能		： 制御を開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.22			TKR			新規
// *************************************************************************/
PUBLIC  void    CTRL_sta(void){
    uc_CtrlFlag = true;
}

// *************************************************************************
//   機能		： 制御を停止する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.22			TKR			新規
// *************************************************************************/
PUBLIC  void    CTRL_stop(void){
    uc_CtrlFlag = false;
    DCM_brakeMot(DCM_R);        // ブレーキ
    DCM_brakeMot(DCM_L);        // ブレーキ
}

// *************************************************************************
//   機能		： 制御データをクリアする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.22			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_clrData(void){

    ENC_clr();      // ENCモジュール初期化
    l_CntR  = 0;     
    l_CntL  = 0;

    /* 現在値 */
	f_NowDist 		= 0;						// 移動距離リセット
	f_NowDistR 		= 0;
	f_NowDistL 		= 0;
	f_NowSpeed		= 0;						// [速度制御]   現在の速度 [mm/s]				（1[msec]毎に更新される）
	f_NowAngle		= 0;						// [角度制御]   現在角度						（1[msec]毎に更新される）
	GYRO_clrAngle();							// 角度リセット
	
	/* 目標値 */
	f_TrgtSpeed		= 0;						// [速度制御]   目標移動速度 [mm/s]				（1[msec]毎に更新される）
	f_TrgtDist 		= 0;						// [距離制御]   目標移動距離					（1[msec]毎に更新される）
	f_TrgtAngleS	= 0;    					// [角速度制御] 目標角速度 [rad/s]				（1[msec]毎に更新される）
	f_TrgtAngle		= 0;						// [角度制御]   目標角度						（1[msec]毎に更新される）
	
	/* 制御データ */
	f_DistErrSum 	= 0;						// [距離制御]   距離積分制御のサム値			（1[msec]毎に更新される）
	f_AngleErrSum 	= 0;						// [角度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
	f_ErrDistBuf	= 0;						// [壁制御]     距離センサーエラー値のバッファ	（1[msec]毎に更新される）
}

// *************************************************************************
//   機能		： 制御データをセットする
//   注意		： なし
//   メモ		： なし
//   引数		： 制御データ
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC  void    CTRL_setData( stCTRL_DATA *p_data ){

    /* 制御方法 */
	en_Type = p_data->en_type;
    /* 速度制御 */
    f_Acc 					= p_data->f_acc;
	f_BaseSpeed				= p_data->f_now;
	f_LastSpeed				= p_data->f_trgt;

    /* 距離制御 */
    f_BaseDist 				= p_data->f_nowDist;
	f_LastDist 				= p_data->f_dist;

    /* 角速度制御 */
    f_AccAngleS 			= p_data->f_accAngleS;
	f_BaseAngleS			= p_data->f_nowAngleS;
	f_LastAngleS			= p_data->f_trgtAngleS;

    /* 角度制御 */
    f_BaseAngle 			= p_data->f_nowAngle;
	f_LastAngle 			= p_data->f_angle;

    f_Time                  = 0;
    f_TrgtTime              = p_data->f_time;

    CTRL_sta();     // 制御開始

}

// *************************************************************************
//   機能		： 制御データを現在の状態に更新する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void    CTRL_refNow( void ){

    FLOAT f_speedR		= 0;							// 右モータ現在速度 [mm/s]
	FLOAT f_speedL		= 0;							// 左モータ現在速度 [mm/s]
	FLOAT f_r 			= F_CNT2MM(l_CntR);				// 右モータの進んだ距離 [mm]
	FLOAT f_l 			= F_CNT2MM(l_CntL);				// 左モータの進んだ距離 [mm]

	/* 速度更新 */
	f_speedR = f_r * 1000;								// 右モータ速度 [mm/s] ( 移動距離[カウント] * 1パルスの移動量(0.0509[mm]) * 1000(msec→sec) 
	f_speedL = f_l * 1000;								// 左モータ速度 [mm/s] ( 移動距離[カウント] * 1パルスの移動量(0.0509[mm]) * 1000(msec→sec) 
	f_NowSpeed  = ( f_speedR + f_speedL ) / 2;			// マウス（進行方向中心軸） [1mm/s] 
	
	/* 距離更新 */
	f_NowDistR += f_r;									// カウント更新
	f_NowDistL += f_l;									// カウント更新
	f_NowDist  = ( f_NowDistR + f_NowDistL ) / 2;		// 平均値更新
	

}

// *************************************************************************
//   機能		： 制御データを目標値に更新する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC  void    CTRL_refTarget( void ){

    /*動作モードに応じる*/
    switch( en_Type ){
	
		/* 加速中(直進) */
		case CTRL_ACC:
		case CTRL_SKEW_ACC:
			if( f_TrgtSpeed < f_LastSpeed ){												// 加速目標更新区間
				f_TrgtSpeed = f_BaseSpeed + f_Acc * f_Time;									// 目標速度
				f_TrgtDist	= f_BaseSpeed * f_Time + 0.5 * f_Acc * f_Time * f_Time;			// 目標位置
			}
			break;
		
		/* 等速中(直進) */
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
			f_TrgtSpeed = f_BaseSpeed;														// 目標速度
			f_TrgtDist	= f_BaseDist + f_BaseSpeed * f_Time;								// 目標位置
			break;
		
		/* 減速中(直進) */
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
			/* 速度制御 ＋ 位置制御 */
			if( f_TrgtSpeed > f_LastSpeed ){												// 減速目標更新区間
				f_TrgtSpeed = f_BaseSpeed - f_Acc * f_Time;									// 目標速度
				f_TrgtDist  = f_BaseDist + ( f_BaseSpeed + f_TrgtSpeed ) * f_Time / 2;		// 目標距離
			}
			/* 位置制御 */
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;
			
		/* 加速中(直進 cos近似) */
		case CTRL_ACC_SMOOTH:
			if( f_TrgtSpeed < f_LastSpeed ){												// 加速目標更新区間			
				f_TrgtSpeed = f_BaseSpeed + ((f_LastSpeed - f_BaseSpeed) / 2) * (1 - cos( (2 * f_Acc * f_Time) / (f_LastSpeed - f_BaseSpeed) ) );		// 目標速度
			}
			break;

		/* 等速中(直進 cos近似) */
		case CTRL_CONST_SMOOTH:
			f_TrgtSpeed = f_BaseSpeed;														// 目標速度
			break;
		
		/* 減速中(直進 cos近似) */
		case CTRL_DEC_SMOOTH:	
			/* 速度制御 ＋ 位置制御 */
			if( f_TrgtSpeed > f_LastSpeed ){												// 減速目標更新区間
				f_TrgtSpeed = f_BaseSpeed + ( ( f_LastSpeed - f_BaseSpeed ) / 2) * (1 - cos( (2 * f_Acc * f_Time)/(f_LastSpeed - f_BaseSpeed ) ) );		// 目標速度
				f_TrgtDist  = f_BaseDist + ( (f_BaseSpeed - f_LastSpeed) / 2 ) * ( f_Time - ( (f_BaseSpeed - f_LastSpeed) / (2*f_Acc) ) * sin(((2*f_Acc)/(f_BaseSpeed - f_LastSpeed))*f_Time) ) ;		// 目標距離
			}
			/* 位置制御 */
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;
			
		/* 加速中(超信地旋回) */
		case CTRL_ACC_TURN:
			
			/*反時計回り*/
			if((f_LastAngle > 0)&&(f_TrgtAngleS < f_LastAngleS)){
				f_TrgtAngleS = f_BaseAngleS + f_AccAngleS*f_Time;
			}
			/*時計回り*/
			else if((f_LastAngle < 0)&&(f_TrgtAngleS > f_LastAngleS)){
				f_TrgtAngleS = f_BaseAngleS - f_AccAngleS*f_Time;
			}
			break;
		
		/* 等速中(超信地旋回) */
		case CTRL_CONST_TURN:
			f_TrgtAngleS = f_BaseAngleS;
			break;
			
		/* 減速中(超信地旋回) */
		case CTRL_DEC_TURN:
			
			/*反時計回り*/
			if(f_LastAngle > 0){
				
				/* 角速度制御＋角度制御 */
				if(f_TrgtAngleS > f_LastAngleS){								//減速目標更新区間
					f_TrgtAngleS = f_BaseAngleS - f_AccAngleS * f_Time;					//目標角速度
					f_TrgtAngle  = f_BaseAngle  + (f_BaseAngleS + f_TrgtAngleS) * f_Time/2;			//目標角度
				}
				/*角度制御*/
				else{
					f_TrgtAngle = f_LastAngle;								//目標角度
				}
			}
			
			/*時計回り*/
			else{
				/*角速度制御＋角度制御*/
				if( f_TrgtAngleS < f_LastAngleS ){								//減速目標更新区間
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;					//目標角速度
					f_TrgtAngle  = f_BaseAngle  + (f_BaseAngleS + f_TrgtAngleS) * f_Time / 2;		//目標角度
				}
				/*角度制御*/
				else{
					f_TrgtAngle = f_LastAngle;								//目標角度
				}
			}

		/* スラローム前の前進動作(スラローム) */
		case CTRL_ENTRY_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			if(f_TrgtDist <= f_LastDist){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;		//目標距離
			}
			break;
		
		/* 加速中(スラローム) */
		case CTRL_ACC_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			
			if( f_LastAngle > 0 ){
				/* 反時計回り */
				if( f_TrgtAngleS < f_LastAngleS){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//目標速度
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}else{
				/* 時計回り */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//目標速度
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}
			
			/* 位置制御 */
			if( f_LastDist > f_TrgtDist ){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;
			}else{
				f_TrgtDist = f_LastDist;
			}
			
			break;
		
		/* 等速中(スラローム) */
		case CTRL_CONST_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = f_BaseAngleS;		//目標角速度
			
			if( f_LastAngle > 0 ){
				/* 反時計回り */
				if( f_TrgtAngle < f_LastAngle ){
					f_TrgtAngle = f_BaseAngle + f_TrgtAngleS * f_Time;		//目標角度
				}else{
					f_TrgtAngle = f_LastAngle;					//目標角度
				}
			}else{
				/* 時計回り */
				if( f_TrgtAngle > f_LastAngle ){
					f_TrgtAngle = f_BaseAngle + f_TrgtAngleS * f_Time;		//目標角度
				}else{
					f_TrgtAngle = f_LastAngle;					//目標角度
				}
			}
			
			/* 位置制御 */
			if( f_LastDist > f_TrgtDist ){													// 目標更新区間
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// 目標位置
			}
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;
			
		/* 減速中(スラローム) */
		case CTRL_DEC_SLA:
			f_TrgtSpeed = f_BaseSpeed;
			
			if( f_LastAngle > 0 ){
				/* 反時計回り */
				if( f_TrgtAngleS > f_LastAngleS){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//目標速度
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}else{
				/* 時計回り */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;				//目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS +f_TrgtAngleS ) * f_Time / 2;	//目標速度
				}else{
					f_TrgtAngle = f_LastAngle;
				}
			}
			
			/* 速度制御＋位置制御 */
			if( f_LastDist > f_TrgtDist){				//目標更新区間
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;	//目標距離
			}else{
				f_TrgtDist = f_LastDist;
			}
			break;
		
		/* スラローム後の前進動作(スラローム) */
		case CTRL_EXIT_SLA:
			f_TrgtSpeed 	= f_BaseSpeed;
			f_TrgtAngleS	= 0;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist = f_BaseDist + f_TrgtSpeed * f_Time;
			}else{
				f_TrgtDist = f_LastDist;
			}
			break;

		/* 上記以外のコマンド */
		default:
			break;
	}
	
}

// *************************************************************************
//   機能		： フィードフォワード量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： フィードフォワード量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
//		v2.0		2019.9.15			TKR			等速，減速対応
// *************************************************************************/
PUBLIC void CTRL_getFF( FLOAT *p_err ){

    FLOAT   f_ff        = 0.0f;

    /* 動作モードに応じる */
    switch( en_Type ){
	
		/* 加速 */
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_ACC_SMOOTH:
		case CTRL_SKEW_ACC:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;	
			*p_err = f_Acc * f_ff;
			break;
			
		/* 加速（超信地旋回）*/
		case CTRL_ACC_TURN:
		case CTRL_ACC_SLA:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
			*p_err = f_AccAngleS * f_ff;
			break;

		/* 等速 */
		case CTRL_CONST:
		case CTRL_CONST_SMOOTH:
		case CTRL_SKEW_CONST:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed;
			*p_err = f_TrgtSpeed * f_ff;
			break;
		
		/* 等速（超信地旋回）*/
		case CTRL_CONST_TURN:
		case CTRL_CONST_SLA:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS;
			*p_err = f_TrgtAngleS * f_ff;
			break;

		/* 減速 */
		case CTRL_DEC:
		case CTRL_DEC_SMOOTH:
		case CTRL_SKEW_DEC:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;
			*p_err = f_Acc * (-1) * f_ff;
			break;

		/* 減速（超信地旋回）*/
		case CTRL_DEC_TURN:
			f_ff = PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
			*p_err = f_AccAngleS* (-1) * f_ff;
			break;
	
		/* その他 */
		default:
			*p_err = 0;
			break;										// 何もしない
	}

}

// *************************************************************************
//   機能		： フィードフォワード量を取得する（並進方向）
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： フィードフォワード量
// **************************    履    歴    *******************************
// 		v1.0		2019.10.11			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_getFF_Speed( FLOAT *p_err ){

	FLOAT	f_ff_speed_acc		= 0.0f;
	FLOAT	f_ff_speed			= 0.0f;

	f_ff_speed_acc 	= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed_acc;
	f_ff_speed		= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_speed;

	/* 動作モードに応じる */
    switch( en_Type ){
	
		/* 加速 */
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_ACC_SMOOTH:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_SLA:
			*p_err			= f_Acc * f_ff_speed_acc + f_TrgtSpeed * f_ff_speed;
			break;
		
		/* 等速 */
		case CTRL_CONST:
		case CTRL_CONST_SMOOTH:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_SLA:
		case CTRL_ENTRY_SLA:
		case CTRL_EXIT_SLA:
			*p_err			= f_TrgtSpeed * f_ff_speed;
			break;

		
		/* 減速 */
		case CTRL_DEC:
		case CTRL_DEC_SMOOTH:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_SLA:
			*p_err			= f_Acc * f_ff_speed_acc * (-1) + f_TrgtSpeed * f_ff_speed;
			break;

		default:
			*p_err			= 0;
			break;
	}

	
}

// *************************************************************************
//   機能		： フィードフォワード量を取得する（回転方向）
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： フィードフォワード量
// **************************    履    歴    *******************************
// 		v1.0		2019.10.11			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_getFF_Angle( FLOAT *p_err ){

	FLOAT	f_ff_angleS_acc		= 0.0f;
	FLOAT	f_ff_angleS			= 0.0f;

	f_ff_angleS_acc 	= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS_acc;
	f_ff_angleS			= PARAM_getGain( Chg_ParamID( en_Type ) ) -> f_FF_angleS;

	/* 動作モードに応じる */
    switch( en_Type ){
	
		/* 加速 */
		case CTRL_ACC_TURN:
		case CTRL_ACC_SLA:
			*p_err			= FABS(f_AccAngleS) * f_ff_angleS_acc + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
		
		/* 等速 */
		case CTRL_CONST_TURN:
		case CTRL_CONST_SLA:
			*p_err			= FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		
		/* 減速 */
		case CTRL_DEC_TURN:	
		case CTRL_DEC_SLA:
			*p_err			= FABS(f_AccAngleS) * f_ff_angleS_acc * (-1) + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		default:
			*p_err			= 0;
			break;
	}

	
}


// *************************************************************************
//   機能		： 速度フィードバック量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： 速度フィードバック制御量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
//		v2.0		2019.9.15			TKR			I制御対応
// *************************************************************************/
PUBLIC void CTRL_getSpeedFB( FLOAT *p_err ){

    FLOAT	f_speedErr;		// [速度制御] 速度偏差
	FLOAT	f_kp = 0.0f;	// 比例ゲイン
	FLOAT	f_ki = 0.0f;	// 積分ゲイン

	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_speed_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_speed_ki;

	/* 速度制御(PI) */
	f_speedErr  	= f_TrgtSpeed - f_NowSpeed;			// 速度偏差[mm/s]
	f_SpeedErrSum	+= f_speedErr * f_ki;			 
	
	if( f_SpeedErrSum > 10000 ){
		f_SpeedErrSum = 10000;
	}

	*p_err = f_SpeedErrSum + f_speedErr * f_kp;			// PI制御量算出

}

// *************************************************************************
//   機能		： 距離フィードバック量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： 距離フィードバック制御量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_getDistFB( FLOAT *p_err ){

    FLOAT       f_distErr;      // [距離制御]距離偏差
    FLOAT       f_kp    = 0;    // 比例ゲイン
    FLOAT       f_ki    = 0;    // 積分ゲイン

    *p_err = 0;
	
	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_ki;
	
	/* 位置制御(PI) */
	f_distErr = f_TrgtDist - f_NowDist;					// 距離偏差[mm]
	f_DistErrSum += f_distErr * f_ki;					// I成分更新
	
	if( f_DistErrSum > 10000 ){
		f_DistErrSum = 10000;
	}
	
	*p_err = f_distErr * f_kp + f_DistErrSum;		// PI制御量算出
	
	// 後でここにフェールセーフを追加
	
	/* 累積偏差クリア */
	if( FABS( f_TrgtDist - f_NowDist ) < 0.1 ){
		f_DistErrSum = 0;
	}
	
}

// *************************************************************************
//   機能		： 角速度フィードバック量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： 角速度フィードバック量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
//		v2.0		2019.6.2			TKR			角速度をGYRO_getNowAngleSpeed関数から取得
//		v3.0		2019.9.15			TKR			I制御追加
// *************************************************************************/
PUBLIC  void CTRL_getAngleSpeedFB( FLOAT *p_err ){

    FLOAT		f_err;			// [入力] ジャイロセンサーエラー 
	FLOAT       f_kp    = 0;    // 比例ゲイン
    FLOAT       f_ki    = 0;    // 積分ゲイン
	
	f_kp = PARAM_getGain(Chg_ParamID(en_Type)) -> f_FB_angleS_kp;
	f_ki = PARAM_getGain(Chg_ParamID(en_Type)) -> f_FB_angleS_ki;
	
	/* 角速度制御(PI) */
	f_err 				= f_TrgtAngleS - f_NowGyroAngleSpeed;		// 角速度偏差[deg/s]
	f_AngleSpeedErrSum	+= f_err * f_ki;							// I成分更新
	
	*p_err = f_err * f_kp + f_AngleSpeedErrSum;				// PI制御量算出

}

// *************************************************************************
//   機能		： 角度フィードバック量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： 角度フィードバック制御量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC  void    CTRL_getAngleFB( FLOAT *p_err ){

    FLOAT f_err;			// [入力] 角度偏差[deg]
	FLOAT f_kp = 0.0f;		// 比例ゲイン
	FLOAT f_ki = 0.0f;		// 積分ゲイン
	
	*p_err = 0;
	
	f_NowAngle = f_NowGyroAngle;					// 現在角度[deg]

	f_err = f_TrgtAngle - f_NowAngle;
	
	/* 直進時 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) ||
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) ||
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA ) ||
		( en_Type == CTRL_DEC_TURN ) || ( en_Type == CTRL_ACC_SLA ) || ( en_Type == CTRL_CONST_SLA ) || ( en_Type == CTRL_DEC_SLA ) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_angle_ki;
		
		f_AngleErrSum += f_err * f_ki;			// I成分更新
								
		*p_err = f_err * f_kp + f_AngleErrSum;		// PI制御量算出
		
		/* 累積偏差クリア */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.1 ){
			f_AngleErrSum = 0;
		}
					
	}
}

// *************************************************************************
//   機能		： 壁制御のフィードバック量を取得する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： なし
//   返り値		： 壁制御のフィードバック量
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_getSenFB( FLOAT *p_err ){

   	FLOAT f_err		= 0;
	FLOAT f_kp		= 0.0f;			// 比例ゲイン
	FLOAT f_kd		= 0.0f;			// 微分ゲイン
	
	/* 直進時 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) || 
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) || 
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_wall_kp;
		f_kd = PARAM_getGain( Chg_ParamID(en_Type) ) -> f_FB_wall_kd;
						
		/* 偏差取得 */
		DIST_getErr( &l_WallErr );		
		f_err = (FLOAT)l_WallErr;
		
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD制御
		
		f_ErrDistBuf = f_err;			// 偏差をバッファリング
	}
	else if( ( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC ) ){
		
		// DIST_getErrSkew( &l_WallErr);
		f_err = (FLOAT)l_WallErr;
		
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD制御
		*p_err = f_err * f_kp;
	}
}

// *************************************************************************
//   機能		： FF/FB制御量からDCMに出力する
//   注意		： CTRL_polからのみ実行可能
//   メモ		： 1msec毎に実行される
//   引数		： 右duty比，左duty比
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void CTRL_outMot( FLOAT f_duty10_R, FLOAT f_duty10_L ){

    FLOAT   f_temp;     // 計算用

    /* 電圧に応じてPWM出力を変更する */
    f_duty10_R  = f_duty10_R * VCC_MAX /( BAT_getLv() / 1000 );
    f_duty10_L  = f_duty10_L * VCC_MAX /( BAT_getLv() / 1000 );

    /* 右モータ */ 
    if( 20 < f_duty10_R ){									// 前進
		DCM_setDirCcw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_duty10_R );
	}
	else if( f_duty10_R < -20 ){							// 後退
		f_temp = f_duty10_R * -1;
		DCM_setDirCw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_R );								// ブレーキ
	}

	/* 左モータ */
	if( 20 < f_duty10_L ){									// 前進
		DCM_setDirCcw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_duty10_L );
	}
	else if( f_duty10_L < -20 ){							// 後退
		f_temp = f_duty10_L * -1;
		DCM_setDirCw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_L );								// ブレーキ
	}
}

// *************************************************************************
//   機能		： 制御のポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される。1msec毎に割り込み処理を行う。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
//		v2.0		2019.8.6			TKR			ログ機能追加
// *************************************************************************/
PUBLIC void CTRL_pol( void ){

//  FLOAT f_feedFoard			= 0;		// [制御] フィードフォワード制御
	FLOAT f_feedFoard_speed		= 0;		// [制御] フィードフォワード制御（並列方向）
	FLOAT f_feedFoard_angle		= 0;		// [制御] フィードフォワード制御（回転方向）
	FLOAT f_speedCtrl			= 0;		// [制御] 速度制御量
	FLOAT f_distCtrl			= 0;		// [制御] 距離制御量
	FLOAT f_angleSpeedCtrl		= 0;		// [制御] 角速度制御量
	FLOAT f_angleCtrl			= 0;		// [制御] 角度制御量
	FLOAT f_distSenCtrl			= 0;		// [制御] 距離センサー制御量
	FLOAT f_duty10_R;						// [出力] 右モータPWM-DUTY比[0.1%]
	FLOAT f_duty10_L;						// [出力] 左モータPWM-DUTY比[0.1%]
	static UCHAR uc_cycle		= 0;		// ログの記録サイクル

	/* 制御を行うかのチェック */
	if( uc_CtrlFlag != true ){
		return;		// 制御無効状態
	}
    
	/* 各種センサ入力 */
	ENC_GetDiv( &l_CntR, &l_CntL );					// 移動量[カウント値]を取得
	CTRL_refNow();									// 制御に使用する値を現在の状態に更新
	CTRL_refTarget();								// 制御に使用する値を目標値に更新

	/* 制御値取得 */
//	CTRL_getFF( &f_feedFoard );						// [制御] フィードフォワード
	CTRL_getFF_Speed( &f_feedFoard_speed );			// [制御] フィードフォワード制御（並列方向）			
	CTRL_getFF_Angle( &f_feedFoard_angle );			// [制御] フィードフォワード制御（回転方向）

	CTRL_getSpeedFB( &f_speedCtrl );				// [制御] 速度
	CTRL_getDistFB( &f_distCtrl );					// [制御] 距離

	CTRL_getAngleSpeedFB( &f_angleSpeedCtrl );		// [制御] 角速度
	CTRL_getAngleFB( &f_angleCtrl );				// [制御] 角度
	CTRL_getSenFB( &f_distSenCtrl );				// [制御] 壁

#if 1
	/* 走行ログ */
	if(us_LogPt != CTRL_LOG){
		if( bl_log == true ){
			uc_cycle++;
		}

		/* ログ記録 */
		if( uc_cycle == CTRL_LOG_CYCLE ){		// この周期で記録
			uc_cycle			= 0;
			st_Log[us_LogPt].f_time			= f_Time;					// 時間
			st_Log[us_LogPt].f_trgtSpeed	= f_TrgtSpeed;				// 速度（目標値）
			st_Log[us_LogPt].f_nowSpeed		= f_NowSpeed;				// 速度（実測値）
			st_Log[us_LogPt].f_trgtPos		= f_TrgtDist;				// 位置（目標値）
			st_Log[us_LogPt].f_nowPos		= f_NowDist;				// 位置（実測値）
			st_Log[us_LogPt].f_trgtAngleS	= f_TrgtAngleS;				// 角速度（目標値）
			st_Log[us_LogPt].f_nowAngleS	= f_NowGyroAngleSpeed;		// 角速度（実測値）
			st_Log[us_LogPt].f_trgtAngle	= f_TrgtAngle;				// 角度（目標値）
			st_Log[us_LogPt].f_nowAngle		= f_NowAngle;				// 角度（実測値）
			st_Log[us_LogPt].f_nowAccel		= f_NowAccel;				// 加速度（実測値）
			us_LogPt++;
			if(us_LogPt== CTRL_LOG) bl_log = false;
		}
	}
#endif

	/* 直進制御 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) || 
		( en_Type == CTRL_ACC_SMOOTH ) || ( en_Type == CTRL_CONST_SMOOTH ) || ( en_Type == CTRL_DEC_SMOOTH ) || 
		( en_Type == CTRL_ENTRY_SLA ) || ( en_Type == CTRL_EXIT_SLA) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
		){
//		f_duty10_R = f_feedFoard * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// 右モータPWM-DUTY比[0.1%]
//		f_duty10_L = f_feedFoard * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// 左モータPWM-DUTY比[0.1%]
		f_duty10_R = f_feedFoard_speed * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// 右モータPWM-DUTY比[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// 左モータPWM-DUTY比[0.1%]

	}
	
	/* 壁あて制御 */
	else if( en_Type == CTRL_HIT_WALL){
		f_duty10_R = f_feedFoard_speed * FF_HIT_BALANCE_R * (-1);
		f_duty10_L = f_feedFoard_speed * FF_HIT_BALANCE_L * (-1);
	}
	
	/* スラローム制御 */
	else if( ( en_Type == CTRL_ACC_SLA ) || ( en_Type == CTRL_CONST_SLA ) || ( en_Type == CTRL_DEC_SLA ) ){
		if(f_LastAngle > 0){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R		+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//右モータPWM-DUTY比[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L*(-1)+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//左モータPWM-DUTY比[0.1%]
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R	+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L*(-1)		+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//左モータPWM-DUTY比[0.1%]
		}else{
//			f_duty10_R = f_feedFoard * FF_BALANCE_R*(-1)+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//右モータPWM-DUTY比[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L		+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//左モータPWM-DUTY比[0.1%]
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R*(-1)		+ f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl;		//右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L	+ f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl;		//左モータPWM-DUTY比[0.1%]
		}
	}
	
	/*超信地旋回*/
	else{
	
		/*左旋回*/
		if(f_LastAngle > 0){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R	     + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
//			f_duty10_L = f_feedFoard * FF_BALANCE_L*(-1) - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R	   + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L*(-1) - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
		}
		else{
		/*右旋回*/
//			f_duty10_R = f_feedFoard * FF_BALANCE_R*(-1) + f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
//			f_duty10_L = f_feedFoard * FF_BALANCE_L	     - f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R*(-1) 	+ f_angleCtrl + f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L		- f_angleCtrl - f_angleSpeedCtrl +  f_speedCtrl + f_distCtrl;
		}
	}

	CTRL_outMot( f_duty10_R, f_duty10_L );				// モータへ出力
    		
	f_Time += 0.001;
	
#if 0	
	/* 壁切れチェック */
	if( MOT_getWallEdgeType() == MOT_WALL_EDGE_RIGHT ){
		
		/* 壁抜け */
		if( DIST_isWall_R_SIDE() == false){	
			MOT_setWallEdge( true );	//壁の切れ目を検知
		}
		
	}else if( MOT_getWallEdgeType() == MOT_WALL_EDGE_LEFT ){
		
		/* 壁抜け */
		if( DIST_isWall_L_SIDE() == false){	
			MOT_setWallEdge( true );
		}
	}else{

		/* 何もしない */
	
    }
#endif
}

// *************************************************************************
//   機能		： ログの出力
//   注意		： なし
//   メモ		： TeraTermに出力し，csvにして保存し，MATLABに出力
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.8.6			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_showLog( void ){
	
	USHORT	i;
	
	printf("index,TrgtSpeed[mm/s],NowSpeed[mm/s],TrgtPos[mm],NowPos[mm],TrgtAngleS[deg/s],NowAngleS[deg/s],TrgtAngle[deg],NowAngle[deg],NowAccel[m/s^2]\n\r");
	for(i=0; i<CTRL_LOG; i++){
		printf("%4d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r",
				i,st_Log[i].f_trgtSpeed,st_Log[i].f_nowSpeed,st_Log[i].f_trgtPos,st_Log[i].f_nowPos,st_Log[i].f_trgtAngleS,st_Log[i].f_nowAngleS,st_Log[i].f_trgtAngle,st_Log[i].f_nowAngle,st_Log[i].f_nowAccel);
	}

}

// *************************************************************************
//   機能		： ログの変数の初期化
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.8.6			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_Loginit( void ){

	memset( st_Log, 0, sizeof(st_Log) );
}

// *************************************************************************
//   機能		： ログの許可
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.8.12			TKR			新規
// *************************************************************************/
PUBLIC void CTRL_LogSta( void ){
	
	bl_log	= true;

}