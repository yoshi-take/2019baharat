// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： マウスの動作
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.25			TKR			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力
#include <math.h>							// math

#include <hal_dcm.h>						// DCM
#include <hal_dcmCtrl.h>					// DCM_CTRL
#include <hal_gyro.h>						// GYRO

#include <motion.h>                         // motion
#include <parameter.h>						// parameter

#include <mode.h>							// mode
#include <hal_led.h>						// LED
#include <hal_spk.h>						// SPK

//**************************************************
// 定義（define）
//**************************************************
#define A1_MIN						( 20 )			// 第1最低移動角度
#define A2_MIN						( 30 )			// 第2最低移動角度
#define A3_MIN						( 20 )			// 第3最低移動角度

#define ANGLE_OFFSET_R90			( 0 )			// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET_L90			( 0 )			// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET_R180			( 0 )			// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET_L180			( 0 )			// 角度のオフセット値（バッファリングによる誤差を埋めるための値）

#define MOT_MOVE_ST_THRESHOLD		( 45 )			// 直進移動距離の閾値[mm]
#define MOT_MOVE_ST_MIN				( 20 )          // 直進移動距離の最低移動量[mm]

//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
/* 動作情報 */
typedef struct{

	FLOAT			f_time;			// 時間					[msec]

	/* 速度制御 */
	FLOAT			f_acc1;			// 加速度1				[mm/s2]
	FLOAT			f_acc3;			// 加速度3				[mm/s2]
	FLOAT			f_now;			// 現在速度				[mm/s]
	FLOAT			f_trgt;			// 加速後の目標速度		[mm/s]
	FLOAT			f_last;			// 減速後の最終速度		[mm/s]

	/* 距離制御 */
	FLOAT			f_dist;			// 移動距離				[mm]
	FLOAT			f_l1;			// 第1移動距離			[mm]
	FLOAT			f_l1_2;			// 第1+2移動距離		[mm]

	/* 角速度制御 */
	FLOAT			f_accAngleS1;	// 角加速度1			[rad/s2]
	FLOAT			f_accAngleS3;	// 角加速度3			[rad/s2]
	FLOAT			f_nowAngleS;	// 現在角速度			[rad/s]
	FLOAT			f_trgtAngleS;	// 加速後の目標角速度	[rad/s]
	FLOAT			f_lastAngleS;	// 減速後の最終角速度	[rad/s]

	/* 角度制御 */
	FLOAT			f_angle;		// 移動角度				[rad]
	FLOAT			f_angle1;		// 第1移動角度			[rad]
	FLOAT			f_angle1_2;		// 第1+2移動角度		[rad]
}stMOT_DATA;

//**************************************************
// 変数
//**************************************************
/* 動作 */
PRIVATE FLOAT 				f_MotNowSpeed 			= 0.0f;		// 現在速度
PRIVATE FLOAT 				f_MotTrgtSpeed 			= 300.0f;	// 目標速度
PRIVATE FLOAT 				f_MotSlaStaSpeed 		= 0.0f;		// スラローム開始速度
PUBLIC	FLOAT				f_MotTrgtSkewSpeed		= 0.0f;		// 目標速度（斜め用）
PRIVATE	stMOT_DATA 			st_Info;							// シーケンスデータ
PRIVATE BOOL				bl_failsafe				= false;	// フェイルセーフ（TRUE：発動	FALSE：何もなし）

/* 壁切れ関係 */
PRIVATE enMOT_WALL_EDGE_TYPE		en_WallEdge 		= MOT_WALL_EDGE_NONE;	//壁切れ補正
PRIVATE BOOL						bl_IsWallEdge		= false;				//壁切れ検知(true:検知　false:非検知)
PRIVATE FLOAT						f_WallEdgeAddDist	= 0;					//壁切れ補正後の移動距離

/* その他 */
extern PUBLIC	VUCHAR		uc_CntSec;		// 内部時計[sec]

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************


// *************************************************************************
//   機能		： 加速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 加速度[mm/s^2]
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc1( void ){

    return 	PARAM_getSpeed( PARAM_ST ) -> f_acc;

}

// *************************************************************************
//   機能		： 減速度を取得する
//   注意		： 正の値(絶対値)で指定
//   メモ		： なし
//   引数		： なし
//   返り値		： 減速度[mm/s^2]
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc3( void ){

    return 	PARAM_getSpeed( PARAM_ST ) -> f_dec;

}

// *************************************************************************
//   機能		： 加速度を取得する（斜め走行用）
//   注意		： なし
//   メモ		： 後でMOT_getAcc1関数とマージ
//   引数		： なし
//   返り値		： 加速度[mm/s^2]
// **************************    履    歴    *******************************
// 		v1.0		2019.11.28			TKR			新規
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc1Skew( void ){

    return 	PARAM_getSpeed( PARAM_SKEW_ACC ) -> f_acc;

}

// *************************************************************************
//   機能		： 減速度を取得する（斜め走行用）
//   注意		： 正の値(絶対値)で指定
//   メモ		： 後でMOT_getAcc3関数とマージ
//   引数		： なし
//   返り値		： 減速度[mm/s^2]
// **************************    履    歴    *******************************
// 		v1.0		2019.11.28			TKR			新規
// *************************************************************************/
PUBLIC  FLOAT   MOT_getAcc3Skew( void ){

    return 	PARAM_getSpeed( PARAM_SKEW_DEC ) -> f_dec;

}


// *************************************************************************
//   機能		： 加速度を取得する(cos近似)
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 最大加速度[mm/s2]
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			吉田			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc1_Smooth( void )
{	
	return 	PARAM_getSpeed( PARAM_ACC_SMOOTH ) -> f_acc;
}


// *************************************************************************
//   機能		： 減速度を取得する(cos近似)
//   注意		： プラスで指定
//   メモ		： なし
//   引数		： なし
//   返り値		： 最大加速度[mm/s2]
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			吉田			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc3_Smooth( void )
{	
	return	PARAM_getSpeed( PARAM_DEC_SMOOTH ) -> f_dec;
}

// *************************************************************************
//   機能		： 角加速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 角加速度[rad/s2]
// **************************    履    歴    *******************************
// 		v1.0		2017.4.26			吉田			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAccAngle1( void )
{
	return	PARAM_getSpeed( PARAM_TURN )->f_accAngle;
}

// *************************************************************************
//   機能		： 角減速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 角減速度[rad/s2]
// **************************    履    歴    *******************************
// 		v1.0		2018.4.26			吉田			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAccAngle3( void )
{
	return	PARAM_getSpeed( PARAM_TURN ) -> f_decAngle;
}

// *************************************************************************
//   機能		： 直進する
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
//		v2.0		2019.11.3			TKR				減速時にループから抜け出せなくなったときの対策追加		
// *************************************************************************/
PRIVATE void MOT_goBlock_AccConstDec( FLOAT f_fin, enMOT_ST_TYPE en_type, enMOT_GO_ST_TYPE en_goType )
{
	stCTRL_DATA		st_data;					// 制御データ
	
	f_WallEdgeAddDist = 0;
	/* ================ */
	/*      実動作      */
	/* ================ */
	/* ------ */
	/*  加速  */
	/* ------ */
	if( ( en_type != MOT_CONST_DEC ) && ( en_type != MOT_CONST_DEC_CUSTOM ) && ( en_type != MOT_CONST_DEC_SMOOTH ) && ( en_type != MOT_CONST_DEC_SMOOTH_CUSTOM ) ){
			
		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_ACC;
		
		}else if(MOT_GO_ST_SMOOTH == en_goType){
			st_data.en_type		= CTRL_ACC_SMOOTH;
			
		}else{
			st_data.en_type		= CTRL_SKEW_ACC;
		}
		
		st_data.f_acc			= st_Info.f_acc1;			// 加速度指定
		st_data.f_now			= st_Info.f_now;			// 現在速度
		st_data.f_trgt			= st_Info.f_trgt;			// 目標速度
		st_data.f_nowDist		= 0;					// 進んでいない
		st_data.f_dist			= st_Info.f_l1;				// 加速距離
		st_data.f_accAngleS		= 0;					// 角加速度
		st_data.f_nowAngleS		= 0;					// 現在角速度
		st_data.f_trgtAngleS	= 0;						// 目標角度
		st_data.f_nowAngle		= 0;					// 現在角度
		st_data.f_angle			= 0;					// 目標角度
		st_data.f_time 			= 0;					// 目標時間 [sec] ← 指定しない
		CTRL_clrData();								// 設定データをクリア
		CTRL_setData( &st_data );						// データセット
		DCM_staMotAll();							// モータON
			
		while( f_NowDist < st_Info.f_l1 ){					// 指定距離到達待ち
			
		#ifdef	TEST
			/*加速デバッグ*/
			if(f_NowDist>=st_Info.f_l1*0.25){
				LED_on(LED0);
				if(f_NowDist>=st_Info.f_l1*0.5){
					LED_on(LED1);
					if(f_NowDist>=st_Info.f_l1*0.75){
						LED_on(LED2);
					}
				}
			}
		#endif

			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE ){
				return;
			}
			
			MOT_setWallEdgeDIST();		// 壁切れ補正を実行する距離を設定
			
		}
		
	}
	
	/* ------ */
	/*  等速  */
	/* ------ */
	if( MOT_GO_ST_NORMAL == en_goType ){
		st_data.en_type		= CTRL_CONST;
		
	}else if(MOT_GO_ST_SMOOTH == en_goType){
		st_data.en_type		= CTRL_CONST_SMOOTH;
				
	}else{
		st_data.en_type		= CTRL_SKEW_CONST;
	}
	
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_Info.f_trgt;				// 現在速度
	st_data.f_trgt			= st_Info.f_trgt;				// 目標速度
	st_data.f_nowDist		= st_Info.f_l1;					// 現在位置
	st_data.f_dist			= st_Info.f_l1_2;				// 等速完了位置
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS	= 0;						// 目標角度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= 0;						// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない

	if( ( en_type == MOT_CONST_DEC ) || ( en_type == MOT_CONST_DEC_CUSTOM ) || ( en_type == MOT_CONST_DEC_SMOOTH ) || ( en_type == MOT_CONST_DEC_SMOOTH_CUSTOM ) ){
		CTRL_clrData();										// 設定データをクリア
	}
	
	CTRL_setData( &st_data );							// データセット
	
	while( f_NowDist < (st_Info.f_l1_2 ) ){				// 指定距離到達待ち	
		
	#ifdef	TEST
		/*等速デバッグ*/
		/*if(f_NowDist>=st_Info.f_l1){
			LED8=0x04;
			if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.25+st_Info.f_l1)){
				LED8=0x08;
				if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.5+st_Info.f_l1)){
					LED8=0x10;
					if(f_NowDist>=((st_Info.f_l1_2-st_Info.f_l1)*0.75+st_Info.f_l1)){
						LED8=0x20;
					}
				}
			}
		}*/
	#endif	
		
		/*脱出*/
		if(SW_ON == SW_INC_PIN){
			CTRL_stop();				// 制御停止
			break;
		}

		/* フェイルセーフ */
		MOT_Failsafe(&bl_failsafe);
		if( bl_failsafe == TRUE )return;

		MOT_setWallEdgeDIST();	// 壁切れ補正を実行する距離を設定
	}
	
	/* ------ */
	/*  減速  */
	/* ------ */
	if( ( en_type != MOT_ACC_CONST ) && ( en_type != MOT_ACC_CONST_CUSTOM ) && ( en_type != MOT_ACC_CONST_SMOOTH ) && ( en_type != MOT_ACC_CONST_SMOOTH_CUSTOM ) ){	
			
		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_DEC;
		
		}else if(MOT_GO_ST_SMOOTH == en_goType){
			st_data.en_type		= CTRL_DEC_SMOOTH;
	
		}else{
			st_data.en_type		= CTRL_SKEW_DEC;
		}
		
		st_data.f_acc			= st_Info.f_acc3;			// 減速
		st_data.f_now			= st_Info.f_trgt;			// 現在速度
		st_data.f_trgt			= st_Info.f_last;			// 最終速度
		st_data.f_nowDist		= st_Info.f_l1_2;			// 等速完了位置
		st_data.f_dist			= st_Info.f_dist;			// 全移動完了位置
		st_data.f_accAngleS		= 0;						// 角加速度
		st_data.f_nowAngleS		= 0;						// 現在角速度
		st_data.f_trgtAngleS	= 0;						// 目標角度
		st_data.f_nowAngle		= 0;						// 現在角度
		st_data.f_angle			= 0;						// 目標角度
		st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
		CTRL_setData( &st_data );							// データセット

		uc_CntSec	= 0;	// ループにいる時間を数える用					
		while( f_NowDist < ( st_Info.f_dist ) ){		// 怪しい

		#ifdef	TEST
			/*if(f_NowDist>=st_Info.f_l1_2){
				LED8=0x80;
			}*/
			/*減速デバッグ*/
			/*
			if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.25+st_Info.f_l1_2){
				LED8=0x81;
				if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.5+st_Info.f_l1_2){
					LED8=0x82;
					if( f_NowDist < (st_Info.f_dist-st_Info.f_l1_2-0.02)*0.75+st_Info.f_l1_2){
						LED8=0x84;
					}
				}
			}
			*/
		#endif
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			MOT_setWallEdgeDIST();		// 壁切れ補正を実行する距離を設定

			if( uc_CntSec >= TIME_THRE_WAIT )break;		// 一定時間経つとループから抜ける
		}

	}

	/* ------------------ */
	/*  等速(壁の切れ目)  */
	/* ------------------ */
	/* 壁切れがまだ見つからない状態（壁切れ設定をしているのに、エッジを見つけていない） */
#if 1
	if( ( en_WallEdge != MOT_WALL_EDGE_NONE)  && ( bl_IsWallEdge == false) ){
		
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;					// 加速度指定
		st_data.f_now			= st_Info.f_last;		// 現在速度
		st_data.f_trgt			= st_Info.f_last;		// 目標速度
		st_data.f_nowDist		= f_NowDist;			// 現在位置
		st_data.f_dist			= f_NowDist + 180.0f;	// 等速完了位置（180.0f：壁切れをどこまで救うかの距離）
		st_data.f_accAngleS		= 0;					// 角加速度
		st_data.f_nowAngleS		= 0;					// 現在角速度
		st_data.f_trgtAngleS	= 0;					// 目標角速度
		st_data.f_nowAngle		= 0;					// 現在角度
		st_data.f_angle			= 0;					// 目標角度
		st_data.f_time			= 0;					// 目標時間[sec]　←指定しない
		
		CTRL_clrData();									// マウスの現在位置/角度をクリア
		CTRL_setData( &st_data );						// データセット
		
		while( f_NowDist < st_data.f_dist ){			// 指定距離到達待ち
			
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			if( MOT_setWallEdgeDIST_LoopWait() == true ) break;			// 壁切れ補正を実行する距離を設定
			
		}
	}
		
	/* 壁切れまで直進動作を行う */
	if( ( MOT_GO_ST_NORMAL == en_goType) &&
		( f_WallEdgeAddDist != 0.0f ) &&
		( f_fin != 0.0f )
	){
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;					// 加速度指定
		st_data.f_now			= st_Info.f_last;		// 現在速度
		st_data.f_trgt			= st_Info.f_last;		// 目標速度
		st_data.f_nowDist		= 0;					// 現在位置
		st_data.f_dist			= f_WallEdgeAddDist;	// 等速完了位置
		st_data.f_accAngleS		= 0;					// 角加速度
		st_data.f_nowAngleS		= 0;					// 現在角速度
		st_data.f_trgtAngleS	= 0;					// 目標角速度
		st_data.f_nowAngle		= 0;					// 現在角度
		st_data.f_angle			= 0;					// 目標角度
		st_data.f_time			= 0;					// 目標時間[sec]　←指定しない
		
		CTRL_clrData();									// マウスの現在位置/角度をクリア
		CTRL_setData( &st_data );						// データセット
		
		while( f_NowDist < st_data.f_dist ){			// 指定距離到達待ち
			
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

		}
		
	}
#endif

	/* 停止 */
	if( 0.0f == f_fin ){
		TIME_wait(500);				// 安定待ち
	 	CTRL_stop();				// 制御停止
		DCM_brakeMot( DCM_R );	// ブレーキ
		DCM_brakeMot( DCM_L );	// ブレーキ
	}	
	
	f_MotNowSpeed = f_fin;			// 現在速度更新
}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
		
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* 加速度 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の走行
		st_Info.f_acc1 		= MOT_getAcc1();													// 加速度1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3();													// 加速度3[mm/s^2]
	}else{									// 斜め走行
		st_Info.f_acc1 		= MOT_getAcc1Skew();												// 加速度1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// 加速度3[mm/s^2]
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotTrgtSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	st_Info.f_l1		= ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	f_l3			= ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]

#ifdef	TEST
	//printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	//printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	//printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	//printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	//printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	//printf("f_l3%5.4f \n\r",f_l3);
#endif
}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速（等速））
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	FLOAT check;
	check = MOT_MOVE_ST_MIN;
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の走行
		st_Info.f_acc1 		= MOT_getAcc1();													// 加速度1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3();													// 加速度3[mm/s^2]
	}else{									// 斜め走行
		st_Info.f_acc1 		= MOT_getAcc1Skew();												// 加速度1[mm/s^2]
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// 加速度3[mm/s^2]
	}

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_last		= f_fin;		// 最終速度
	
	st_Info.f_trgt		= sqrt( 1 / ( ( st_Info.f_acc3 * -1 ) - st_Info.f_acc1 ) *					// 目標速度
								( 2 * st_Info.f_acc1 * ( st_Info.f_acc3 * -1 ) * ( st_Info.f_dist - MOT_MOVE_ST_MIN ) + 
								 ( st_Info.f_acc3 * -1 ) * f_MotNowSpeed * f_MotNowSpeed - st_Info.f_acc1 * f_fin * f_fin ) );
								 
	st_Info.f_l1		= ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	f_l3				= ( f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) / ( ( st_Info.f_acc3  * -1 ) * 2 );		// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]

#ifdef	TEST	
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("MOT_MOVE_ST_MIN:%5.4f \n\r",check);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("f_now:%5.4f \n\r",st_Info.f_now);
	printf("f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("f_last:%5.4f \n\r",st_Info.f_last);
	printf("f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("f_l1_2 - f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速 cos近似）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
		
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// 最大加速度1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// 最大加速度3[mm/s^2]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotTrgtSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度
	
	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	
	st_Info.f_l1		= ( ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed )* 3.1416f)/ ( 4*st_Info.f_acc1 );			// 第1移動距離[mm]
	f_l3				= ( ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed )* 3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );				// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]
	
#ifdef	TEST	
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif	

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速（等速） cos近似）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	FLOAT check;
	check = MOT_MOVE_ST_MIN;
	
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// 最大加速度1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// 最大加速度3[mm/s^2]


	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_last		= f_fin;															// 最終速度
	
	st_Info.f_trgt		= sqrt( ( 2*st_Info.f_acc1*(st_Info.f_dist - MOT_MOVE_ST_MIN) ) / (3.1416f) + ( ( f_MotNowSpeed * f_MotNowSpeed + f_fin * f_fin ) / 2 ) );					// 目標速度
							
	st_Info.f_l1		= ( ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( st_Info.f_acc1 * 4 );			// 第1移動距離[mm]
	f_l3				= ( (f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) *3.1416f) / ( ( st_Info.f_acc3  * -1 ) * 4 );				// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]
	
#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("MOT_MOVE_ST_MIN:%5.4f \n\r",check);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("f_l3:%5.4f \n\r",f_l3);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の走行
		st_Info.f_acc1 		= MOT_getAcc1();												// 加速度1[mm/s^2]
		st_Info.f_acc3 		= 0;															// 加速度3[mm/s^2](未使用)
	}else{									// 斜め走行
		st_Info.f_acc1 		= MOT_getAcc1Skew();											// 加速度1[mm/s^2]
		st_Info.f_acc3 		= 0;															// 加速度3[mm/s^2]
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速（等速））
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f );	// 加速度1[mm/s^2]（強制的に書き換え）
	st_Info.f_acc3 		= 0;																// 加速度3[mm/s^2](未使用)

	/* 距離 */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速） cos近似
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1_Smooth();													// 最大加速度1[mm/s^2]
	st_Info.f_acc3 		= 0;																// 加速度3[mm/s^2](未使用)

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	st_Info.f_l1		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( st_Info.f_acc1 * 4 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速（等速） cos近似 ）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 4.0f );	// 最大加速度1[mm/s^2]（強制的に書き換え）
	st_Info.f_acc3 		= 0;																// 加速度3[mm/s^2](未使用)

	/* 距離 */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 4 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
	printf("st_Info.f_l1:%5.4f \n\r",st_Info.f_l1);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速＋減速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	if( MOT_GO_ST_NORMAL == en_type ){
		st_Info.f_acc1 		= 0;															// 加速度1[mm/s^2](未使用)
		st_Info.f_acc3 		= MOT_getAcc3();												// 加速度3[mm/s^2]
	}else{
		st_Info.f_acc1 		= 0;															// 加速度1[mm/s^2](未使用)
		st_Info.f_acc3 		= MOT_getAcc3Skew();												// 加速度3[mm/s^2]
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第1-2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速（等速）＋減速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.4.26			TKR			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= 0;																// 加速度1[mm/s^2](未使用)
	st_Info.f_acc3 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f ) * -1;	// 加速度3[mm/s^2]（強制的に書き換え）

	/* 距離 */
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第1-2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速＋減速 cos近似）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_SMOOTH( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= 0;																// 最大加速度1[mm/s^2](未使用)
	st_Info.f_acc3 		= MOT_getAcc3_Smooth();													// 最大加速度3[mm/s^2]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );			// 第1-2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
	printf("st_Info.f_l1_2:%5.4f \n\r",st_Info.f_l1_2);
#endif

}

// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速（等速）＋減速 cos近似）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
//		v1.0		2019.4.26			TKR			cos近似追加
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_SMOOTH_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_SMOOTH == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist;												// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= 0;																// 加速度1[mm/s^2](未使用)
	st_Info.f_acc3 		= ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 4.0f ) * -1;	// 最大加速度3[mm/s^2]（強制的に書き換え）

	/* 距離 */
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) *3.1416f) / ( ( st_Info.f_acc3 * -1 ) * 4 );			// 第1-2移動距離[mm]

#ifdef	TEST
	printf("st_Info.f_acc1:%5.4f \n\r",st_Info.f_acc1);							 
	printf("st_Info.f_acc3:%5.4f \n\r",st_Info.f_acc3);	
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("f_fin:%5.4f \n\r",f_fin);
	printf("st_Info.f_now:%5.4f \n\r",st_Info.f_now);
	printf("st_Info.f_trgt:%5.4f \n\r",st_Info.f_trgt);
	printf("st_Info.f_last:%5.4f \n\r",st_Info.f_last);
	printf("st_Info.f_dist:%5.4f \n\r",st_Info.f_dist);
	printf("st_Info.f_l1_2 - st_Info.f_l1:%5.4f \n\r",st_Info.f_l1_2 - st_Info.f_l1);
#endif

}

// *************************************************************************
//   機能		： 前進のタイプを取得する
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.5.01			TKR			新規
// *************************************************************************/
PRIVATE enMOT_ST_TYPE MOT_getStType( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT f_v1Div;
	FLOAT f_v3Div;
	FLOAT f_acc1;
	FLOAT f_acc3;
	FLOAT f_t1;
	FLOAT f_t3;
	FLOAT f_l1;
	FLOAT f_l3;
	FLOAT f_total;							// 移動距離[mm]
		
	/* 1区画の距離 */
	if( (MOT_GO_ST_NORMAL == en_type) || (MOT_GO_ST_SMOOTH == en_type) ){		// 通常の直進 or cos近似
		f_total	= f_num * BLOCK;

	}
	else{									// 斜めの直進
		f_total	= f_num * BLOCK_SKEW;
	}

	if((MOT_GO_ST_NORMAL == en_type) || (MOT_GO_ST_SKEW == en_type)){			// 通常の直進 or 斜め
		
		/* ================ */
		/*  加速＋等速動作  */
		/* ================ */
		f_v1Div		= f_fin - f_MotNowSpeed;
		
		if( MOT_GO_ST_NORMAL == en_type ){		// 加速度1[mm/s^2]
			f_acc1		= MOT_getAcc1();		// 直進		
		}else{
			f_acc1		= MOT_getAcc1Skew();	// 斜め
		}

		f_t1		= f_v1Div / f_acc1;

		f_l1 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t1;

		/* 加速＋等速動作 */
		if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

			/* 加速が最終速度に対して完了しない */
			if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
	//			printf("パターン4\n\r");
				return MOT_ACC_CONST_CUSTOM;		// パターン4（強制的に加速度を変更する）
			}
			else{
	//			printf("パターン3\n\r");
				return MOT_ACC_CONST;				// パターン3（加速＋等速）
			}
		}

		/* ================ */
		/*  等速＋減速動作  */
		/* ================ */
		f_v3Div		= f_fin - f_MotNowSpeed;

		if( MOT_GO_ST_NORMAL ){		// 加速度3[mm/s^2]
			f_acc3		= MOT_getAcc3();			// 直進
		}else{
			f_acc3		= MOT_getAcc3Skew();		// 斜め
		}

		f_t3		= f_v3Div / ( f_acc3 * -1 );

		f_l3 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t3;

		//printf("f_l3：%5.4f \n\r",f_l3);
		//printf("f_total：%5.4f \n\r",f_total);
		
		/* 等速＋減速動作 */
		if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

			/* 減速が最終速度に対して完了しない */
			if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
	//			printf("パターン6\n\r");
				return MOT_CONST_DEC_CUSTOM;		// パターン6（強制的に加速度を変更する）
			}
			else{
	//			printf("パターン5\n\r");
				return MOT_CONST_DEC;				// パターン5（等速＋減速）
			}
		}

		/* ========== */
		/*  台形動作  */
		/* ========== */
		f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;					// 台形時の速度差
		f_t1		= f_v1Div / f_acc1;
		f_l1		= ( f_MotNowSpeed + f_MotTrgtSpeed ) * 0.5f * f_t1;

		f_v3Div		= f_fin - f_MotTrgtSpeed;							// 台形時の速度差

		if( MOT_GO_ST_NORMAL ){		// 加速度3[mm/s^2]
			f_acc3		= MOT_getAcc3();			// 直進
		}else{
			f_acc3		= MOT_getAcc3Skew();		// 斜め
		}

		f_t3		= -1.0f * f_v3Div / f_acc3;							// 減速時の所要時間
		f_l3		= ( f_MotTrgtSpeed + f_fin ) * 0.5f * f_t3;

		/* 通常の台形動作 */
		if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){

		//printf("%5.4f\n\r",f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN);	
		//printf("パターン1\n\r");
			return MOT_ACC_CONST_DEC;				// パターン1（通常）
		}
		/* 等速値を変更する */
		else{
			//printf("パターン2\n\r");
			return MOT_ACC_CONST_DEC_CUSTOM;		// パターン2（目標速度を変更）
		}
	
	}else if( (MOT_GO_ST_SMOOTH == en_type) ){				// cos近似
	
		/* ========================= */
		/*  加速＋等速動作(cos近似)  */
		/* ========================= */
		f_v1Div		= f_fin - f_MotNowSpeed;
		f_acc1		= MOT_getAcc1_Smooth();						// 最大加速度1[mm/s^2]
		f_t1		= ( 3.1416f * f_v1Div) / (2.0f *f_acc1);	// 加速時間
				
		f_l1 = ( ( f_fin*f_fin - f_MotNowSpeed*f_MotNowSpeed ) * 3.1416f ) / ( 4*f_acc1 );	
		
		//printf("f_total= %f\n\r",f_total);
		
		/* 加速＋等速動作 */
		if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

			/* 加速が最終速度に対して完了しない */
			if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
				//printf("パターン10\n\r");
				return MOT_ACC_CONST_SMOOTH_CUSTOM;		// パターン10（強制的に加速度を変更する）
			}
			else{
				//printf("パターン9\n\r");
				return MOT_ACC_CONST_SMOOTH;				// パターン9（加速＋等速）
			}
		}

		/* ======================== */
		/*  等速＋減速動作(cos近似) */
		/* ======================== */
		f_v3Div		= f_fin - f_MotNowSpeed;
		f_acc3		= MOT_getAcc3_Smooth();				// 最大加速度3[mm/s^2]
		f_t3		= ( 3.1416f * f_v3Div) / ( 2*f_acc3 * -1 );

		f_l3 = ( ( f_fin*f_fin - f_MotNowSpeed*f_MotNowSpeed) * 3.1416f) / ( 4*f_acc3*-1 );

		//printf("f_l3：%5.4f \n\r",f_l3);
		//printf("f_total：%5.4f \n\r",f_total);
		
		/* 等速＋減速動作 */
		if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

			/* 減速が最終速度に対して完了しない */
			if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
				//printf("パターン12\n\r");
				return MOT_CONST_DEC_SMOOTH_CUSTOM;		// パターン12（強制的に加速度を変更する）
			}
			else{
				//printf("パターン11\n\r");
				return MOT_CONST_DEC_SMOOTH;				// パターン11（等速＋減速）
			}
		}

		/* ================== */
		/*  台形動作(cos近似) */
		/* ================== */
		f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;		// 台形時の速度差
		f_acc1		= MOT_getAcc1_Smooth();						// 最大加速度1[mm/s^2]
		f_t1		= ( 3.1416f * f_v1Div) / (2 *f_acc1);	// 加速時の所要時間

		f_l1 		= ( ( f_MotTrgtSpeed*f_MotTrgtSpeed  - f_MotNowSpeed*f_MotNowSpeed ) * 3.1416f ) / ( 4*f_acc1 );

		f_v3Div		= f_fin - f_MotTrgtSpeed;							// 台形時の速度差
		f_acc3		= MOT_getAcc3_Smooth();									// 加速度3[mm/s^2]
		f_t3		= ( 3.1416f * f_v3Div) / ( 2*f_acc3 * -1 );			// 減速時の所要時間
		
		f_l3 		= ( ( f_fin*f_fin - f_MotTrgtSpeed*f_MotTrgtSpeed) * 3.1416f) / ( 4*f_acc3*-1 );

		/* 通常の台形動作 */
		if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){

			//printf("%5.4f\n\r",f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN);	
			//printf("パターン7\n\r");
			return MOT_ACC_CONST_DEC_SMOOTH;				// パターン1（通常）
		}
		/* 等速値を変更する */
		else{
			//printf("パターン8\n\r");
			return MOT_ACC_CONST_DEC_SMOOTH_CUSTOM;		// パターン2（目標速度を変更）
		}
	
	}
	
}

// *************************************************************************
//   機能		： 区画前進
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PRIVATE void MOT_go_FinSpeed( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_goStType )
{
	enMOT_ST_TYPE 		en_type 		= MOT_getStType( f_num, f_fin, en_goStType );			// 動作パターン取得
	

	/* 移動距離と指定値に応じで動作を変える */
	switch( en_type ){
	
		case MOT_ACC_CONST_DEC:			// [01] 台形加速
			MOT_setData_ACC_CONST_DEC( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_DEC_CUSTOM:	// [02] 台形加速（等速）
			MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );		// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST:				// [03] 加速＋等速
			MOT_setData_MOT_ACC_CONST( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_CUSTOM:		// [04] 加速＋等速（等速）
			MOT_setData_MOT_ACC_CONST_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, MOT_GO_ST_NORMAL );			// 動作
			break;

		case MOT_CONST_DEC:				// [05] 等速＋減速
			MOT_setData_MOT_CONST_DEC( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_CONST_DEC_CUSTOM:		// [06] 等速＋減速（減速値変更）
			MOT_setData_MOT_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
			
		/* cos近似 */
		case MOT_ACC_CONST_DEC_SMOOTH:			// [07] 台形加速
			MOT_setData_ACC_CONST_DEC_SMOOTH( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_DEC_SMOOTH_CUSTOM:	// [08] 台形加速（等速）
			MOT_setData_MOT_ACC_CONST_DEC_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );		// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_SMOOTH:				// [09] 加速＋等速
			MOT_setData_MOT_ACC_CONST_SMOOTH( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_SMOOTH_CUSTOM:		// [10] 加速＋等速（等速）
			MOT_setData_MOT_ACC_CONST_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;

		case MOT_CONST_DEC_SMOOTH:				// [11] 等速＋減速
			MOT_setData_MOT_CONST_DEC_SMOOTH( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_CONST_DEC_SMOOTH_CUSTOM:		// [12] 等速＋減速（減速値変更）
			MOT_setData_MOT_CONST_DEC_SMOOTH_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		default:
			break;
	}

}

// *************************************************************************
//   機能		： 区画前進（通常）
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。☆
//   引数		： 区画数、最終速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_NORMAL );		// 通常の直進	
}
	
// *************************************************************************
//   機能		： 区画前進（cos近似）
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。☆
//   引数		： 区画数、最終速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed_Smooth( FLOAT f_num, FLOAT f_fin){
	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SMOOTH );		// cos近似加速
	 
}

// *************************************************************************
//   機能		： 区画前進（斜め走行）
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。☆
//   引数		： 区画数、最終速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.24			TKR			新規
// *************************************************************************/
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin){
	
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SKEW );		// cos近似加速
	 
}

// *************************************************************************
//   機能		： 超信地旋回
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type){
	volatile stMOT_DATA	st_info;			//シーケンスデータ
	stCTRL_DATA		st_data;				//制御データ
	FLOAT			f_angle2	=A2_MIN;	//最低第2移動角度[deg]
	FLOAT			f_angle3;
	FLOAT			us_trgtAngleS;			//目標角速度[deg/s]

	us_trgtAngleS = 300;	
	
	/* -------------- */
	/* 動作データ計算 */
	/* -------------- */
	/*　角加速度　*/
	st_info.f_accAngleS1 = MOT_getAccAngle1();		
	st_info.f_accAngleS3 = MOT_getAccAngle3();		
	
	/* 角速度*/
	st_info.f_nowAngleS = 0;
	st_info.f_trgtAngleS = (FLOAT)us_trgtAngleS;
	st_info.f_lastAngleS = 0;
	
	/* 角度 */
	switch(en_type){	
		case MOT_R90:	
			st_info.f_angle =  -90 + ANGLE_OFFSET_R90;
			break;
			
		case MOT_L90:	
			st_info.f_angle =   90 + ANGLE_OFFSET_L90 ;
			break;
			
		case MOT_R180:
			st_info.f_angle = -180 + ANGLE_OFFSET_R180 ;
			break;
			
		case MOT_L180:
			st_info.f_angle =  180 + ANGLE_OFFSET_L180;	
			break;
			
		case MOT_R360:
			st_info.f_angle = -360 ;
			break;
			
		case MOT_L360:
			st_info.f_angle =  360 ;		
		break;
	}
	
	f_angle3 = (st_info.f_trgtAngleS - st_info.f_lastAngleS) / 2 * (st_info.f_trgtAngleS - st_info.f_lastAngleS) / st_info.f_accAngleS3;	//第3移動角度
	
	if((en_type == MOT_R90) || (en_type == MOT_R180) || (en_type == MOT_R360)){								//-方向
		st_info.f_trgtAngleS 		*= -1;												//回転方向を逆にする
		f_angle2			*= -1;												//回転方向を逆にする
		f_angle3			*= -1;												//回転方向を逆にする
		st_info.f_angle1		= st_info.f_angle - f_angle3 - f_angle2;							//第1移動角度[deg]
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;									//第1+2移動角度[deg]
		
		/* 最小移動距離を上書き */
		if( st_info.f_angle1 > (A1_MIN * -1)){
			st_info.f_angle1 = A1_MIN * -1;
		}
		
	}else{
		st_info.f_angle1	= st_info.f_angle - f_angle3 - f_angle2;
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;
		
		/* 最小移動距離を上書き */
		if( st_info.f_angle < A1_MIN ){
			st_info.f_angle1 = A1_MIN;
		}
		
	}
	
	//GYRO_staErrChkAngle();			//エラー検出開始(まだ)
#ifndef TEST	
	printf("st_info.f_trgtAngleS:%5.4f \n\r",st_info.f_trgtAngleS);
	printf("f_angle3:%5.4f \n\r",f_angle3);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	
	/* ================ */
	/*　　 実動作 　　　*/
	/* ================ */
	/* -----*/
	/* 加速 */
	/* -----*/
	st_data.en_type			= CTRL_ACC_TURN;
	st_data.f_acc			= 0;						//加速度指定
	st_data.f_trgt			= 0;						// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= 0;						// 加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= st_info.f_angle1;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	
	CTRL_clrData();										// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );							// データセット
	DCM_staMotAll();									// モータON	
	//printf("第1+2移動角度：%5.4f \n\r",st_info.f_angle1_2);
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-方向
		while( f_NowAngle > st_info.f_angle1 ){				//指定角度到達待ち(右回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			
			//if( SYS_isOUTOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < st_info.f_angle1){				//指定角度到達待ち(左回転)
		
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}
	
	//printf("加速完了(*´▽｀*)\n\r");
	//LED_SYS = 0;
	/* ---- */
	/* 等速 */
	/* ---- */
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){
		f_angle3		= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS) / st_info.f_accAngleS3;		//第3移動角度
		f_angle3		= -1*f_angle3;
		if( f_angle3 > A3_MIN*-1 ) f_angle3 = A3_MIN*-1;			//減速最低角度に書き換え
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;			//第1+2移動角度[rad]
	}else{
		f_angle3		= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// 第3移動角度
		if( f_angle3 < A3_MIN ) f_angle3 = A3_MIN;					//減速最低角度に書き換え											
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;			//第1+2移動角度[rad]												
	}
	//printf("第1+2移動角度：%5.4f \n\r",st_info.f_angle1_2);
	st_data.en_type			= CTRL_CONST_TURN;
	st_data.f_acc			= 0;					//加速度指定
	st_data.f_now			= 0;					//現在速度
	st_data.f_trgt			= 0;					//目標速度
	st_data.f_nowDist		= 0;					//進んでいない
	st_data.f_dist			= 0;					//等速完了位置
	st_data.f_accAngleS		= 0;					//角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;			//現在角速度
	st_data.f_trgtAngleS	= f_TrgtAngleS;			//目標角速度
	st_data.f_nowAngle		= st_info.f_angle1;		//現在角度
	st_data.f_angle			= st_info.f_angle1_2;	//目標角度
	st_data.f_time			= 0;					//目標時間[sec]←指定しない
	
	CTRL_setData( &st_data );				//データセット
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-方向
		while( f_NowAngle > st_info.f_angle1_2 ){		//指定角度到達待ち(左回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
		
			//if( SYS_isOUTOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < st_info.f_angle1_2){		//指定角度到達待ち(右回転)
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}

#ifndef TEST	
	printf("st_info.f_angle:%5.4f \n\r",st_info.f_angle);
	printf("f_angle3:%5.4f \n\r",f_angle3);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	/* ---- */
	/* 減速 */
	/* ---- */
	st_data.en_type			= CTRL_DEC_TURN;
	st_data.f_acc			= 0;					//加速度指定
	st_data.f_now			= 0;					//現在速度
	st_data.f_trgt			= 0;					//最終速度
	st_data.f_nowDist		= 0;					//等速完了位置
	st_data.f_dist			= 0;					//全移動完了位置
	st_data.f_accAngleS		= st_info.f_accAngleS3;	//角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;			//現在角速度
	st_data.f_trgtAngleS	= 0;					//目標角速度
	st_data.f_nowAngle		= st_info.f_angle1_2;	//現在角度
	st_data.f_angle			= st_info.f_angle;		//目標角度
	st_data.f_time			= 0;					//目標時間[sec]←指定しない
	
	CTRL_setData( &st_data );						// データセット
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){	//-方向
		while( f_NowAngle > ( st_info.f_angle + 1)){		//指定角度到達待ち(右回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
		
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < (st_info.f_angle - 1)){		//指定角度到達待ち(左回転)
		
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	
	}
	//LED8=0xff;
	//printf("旋回完了");
	/* 停止 */
	TIME_wait(200);		//安定待ち
	CTRL_stop();		// 制御停止	
	
}

// *************************************************************************
//   機能		： 超信地旋回（目標角速度変更可能）
//   注意		： なし
//   メモ		： なし
//   引数		： 旋回の種類，目標角速度[deg/s]
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.9.17			TKR			新規
// *************************************************************************/
PUBLIC void MOT_turn2( enMOT_TURN_CMD en_type, FLOAT f_trgtAngleS ){
	volatile stMOT_DATA	st_info;			//シーケンスデータ
	stCTRL_DATA		st_data;				//制御データ
	FLOAT			f_angle3;

	/* -------------- */
	/* 動作データ計算  */
	/* -------------- */
	FLOAT			f_acc1;
	FLOAT			f_acc3;
	FLOAT			f_theta1;
	FLOAT			f_theta3;
	FLOAT			f_total;	

	f_acc1		= MOT_getAccAngle1();
	f_theta1	= f_trgtAngleS * f_trgtAngleS / ( f_acc1 * 2);

	f_acc3		= MOT_getAccAngle3();
	f_theta3	=  f_trgtAngleS * f_trgtAngleS /( f_acc3 * 2 );

	/* 角速度*/
	st_info.f_nowAngleS = 0;
	st_info.f_lastAngleS = 0;

	/* 角度 */
	switch(en_type){	
		case MOT_R90:
			st_info.f_angle	= -90 + ANGLE_OFFSET_R90;
			f_total			= st_info.f_angle * (-1);
			break;
			
		case MOT_L90:
			st_info.f_angle	= 90 + ANGLE_OFFSET_L90;
			f_total 		= st_info.f_angle;
			break;
			
		case MOT_R180:
			st_info.f_angle	= -180 + ANGLE_OFFSET_R180;
			f_total 		= st_info.f_angle * (-1);
			break;
			
		case MOT_L180:
			st_info.f_angle	= 180 + ANGLE_OFFSET_L180;
			f_total 		= st_info.f_angle;	
			break;
			
		case MOT_R360:
			st_info.f_angle	= -360;
			f_total = st_info.f_angle * (-1);
			break;
			
		case MOT_L360:
			st_info.f_angle	= 360;
			f_total = st_info.f_angle;		
			break;
	}

	/* 台形動作の種類判定 */
	if( ( f_total - f_theta1 - f_theta3 -  A2_MIN) >= 0 ){		// 通常の台形動作

		/* 角加速度 */
		st_info.f_accAngleS1		= MOT_getAccAngle1();
		st_info.f_accAngleS3		= MOT_getAccAngle3();

		/* 角速度 */
		st_info.f_nowAngleS 		= 0;
		st_info.f_lastAngleS 		= 0;
		st_info.f_trgtAngleS		= f_trgtAngleS;
		
		/* 角度 */
		st_info.f_angle1	= f_trgtAngleS * f_trgtAngleS /( st_info.f_accAngleS1 * 2 );
		f_angle3			= f_trgtAngleS * f_trgtAngleS /( st_info.f_accAngleS3 * 2 );
		st_info.f_angle1_2	= f_total - f_angle3;

	}else{		// 目標角速度を変更

		/* 角加速度 */
		st_info.f_accAngleS1		= MOT_getAccAngle1();
		st_info.f_accAngleS3		= MOT_getAccAngle3();

		/* 角速度 */
		st_info.f_nowAngleS 		= 0;
		st_info.f_lastAngleS 		= 0;
		st_info.f_trgtAngleS	= sqrt( 1 / ( ( st_info.f_accAngleS3 * -1 ) - st_info.f_accAngleS1 ) *					// 目標角速度を変更
									( 2 * st_info.f_accAngleS1 * ( st_info.f_accAngleS3 * -1 ) * ( f_total - MOT_MOVE_ST_MIN ) ) );
		
		/* 角度 */
		st_info.f_angle1	= st_info.f_trgtAngleS * st_info.f_trgtAngleS /( st_info.f_accAngleS1 * 2 );
		f_angle3			= st_info.f_trgtAngleS * st_info.f_trgtAngleS /( st_info.f_accAngleS3 * 2 );
		st_info.f_angle1_2	= f_total - f_angle3;
	
	}

	/* 符号処理 */
	if((en_type == MOT_R90) || (en_type == MOT_R180) || (en_type == MOT_R360)){
		st_info.f_trgtAngleS 		*= -1;
		st_info.f_angle1			*= -1;
		st_info.f_angle1_2			*= -1;
		f_angle3					*= -1;
	}


#ifndef TEST	
	printf("st_info.f_trgtAngleS:%f \n\r",st_info.f_trgtAngleS);
	printf("f_angle3:%f \n\r",f_angle3);
	printf("st_info.f_angle1:%f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%f \n\r",st_info.f_angle1_2);
#endif
	
	/* ================ */
	/*　　 実動作 　　　*/
	/* ================ */
	/* -----*/
	/* 加速 */
	/* -----*/
	st_data.en_type			= CTRL_ACC_TURN;
	st_data.f_acc			= 0;						//加速度指定
	st_data.f_trgt			= 0;						// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= 0;						// 加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= st_info.f_angle1;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	
	CTRL_clrData();										// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );							// データセット
	DCM_staMotAll();									// モータON	
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-方向
		while( f_NowAngle > st_info.f_angle1 ){				//指定角度到達待ち(右回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			
			//if( SYS_isOUTOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < st_info.f_angle1){				//指定角度到達待ち(左回転)
		
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}
	
	//printf("加速完了(*´▽｀*)\n\r");

	/* ---- */
	/* 等速 */
	/* ---- */
	//printf("第1+2移動角度：%5.4f \n\r",st_info.f_angle1_2);
	st_data.en_type			= CTRL_CONST_TURN;
	st_data.f_acc			= 0;					//加速度指定
	st_data.f_now			= 0;					//現在速度
	st_data.f_trgt			= 0;					//目標速度
	st_data.f_nowDist		= 0;					//進んでいない
	st_data.f_dist			= 0;					//等速完了位置
	st_data.f_accAngleS		= 0;					//角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;			//現在角速度
	st_data.f_trgtAngleS	= f_TrgtAngleS;			//目標角速度
	st_data.f_nowAngle		= st_info.f_angle1;		//現在角度
	st_data.f_angle			= st_info.f_angle1_2;	//目標角度
	st_data.f_time			= 0;					//目標時間[sec]←指定しない
	
	CTRL_setData( &st_data );				//データセット
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){		//-方向
		while( f_NowAngle > st_info.f_angle1_2 ){		//指定角度到達待ち(左回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
		
			//if( SYS_isOUTOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < st_info.f_angle1_2){		//指定角度到達待ち(右回転)
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}

#ifndef TEST	
	printf("st_info.f_angle:%5.4f \n\r",st_info.f_angle);
	printf("st_info.f_angle1:%5.4f \n\r",st_info.f_angle1);
	printf("st_info.f_angle1_2:%5.4f \n\r",st_info.f_angle1_2);
#endif
	/* ---- */
	/* 減速 */
	/* ---- */
	st_data.en_type			= CTRL_DEC_TURN;
	st_data.f_acc			= 0;					//加速度指定
	st_data.f_now			= 0;					//現在速度
	st_data.f_trgt			= 0;					//最終速度
	st_data.f_nowDist		= 0;					//等速完了位置
	st_data.f_dist			= 0;					//全移動完了位置
	st_data.f_accAngleS		= st_info.f_accAngleS3;	//角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;			//現在角速度
	st_data.f_trgtAngleS	= 0;					//目標角速度
	st_data.f_nowAngle		= st_info.f_angle1_2;	//現在角度
	st_data.f_angle			= st_info.f_angle;		//目標角度
	st_data.f_time			= 0;					//目標時間[sec]←指定しない
	
	CTRL_setData( &st_data );						// データセット
	
	if(( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 )){	//-方向
		while( f_NowAngle > ( st_info.f_angle + 1)){		//指定角度到達待ち(右回転)
			
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
		
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	}else{
		while( f_NowAngle < (st_info.f_angle - 1)){		//指定角度到達待ち(左回転)
		
			/*脱出*/
			if(SW_ON == SW_INC_PIN){
				CTRL_stop();				// 制御停止
				break;
			}
			//if( SYS_isOutOfCtrl() == true ) break;		//途中で制御不能になった
		}
	
	}

	//printf("旋回完了");
	/* 停止 */
	TIME_wait(200);		//安定待ち
	CTRL_stop();		// 制御停止	
	
}


// *************************************************************************
//   機能		： 等速区画　前進
//   注意		： なし
//   メモ		： なし
//   引数		： 区画数
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			吉田			新規
// *************************************************************************/
PUBLIC void MOT_goBlock_Const( FLOAT f_num){
	volatile stMOT_DATA	st_info;			//シーケンスデータ
	stCTRL_DATA		st_data;			//制御データ
		
	/*----------------*/
	/* 動作データ計算 */
	/*----------------*/
	/* 距離 */
	st_info.f_dist	= f_num * BLOCK;
	
	/*------*/
	/* 等速 */
	/*------*/
	st_data.en_type		= CTRL_CONST;
	st_data.f_acc		= 0;			//加速度指定
	st_data.f_now		= f_MotNowSpeed;	//現在速度
	st_data.f_trgt		= f_MotNowSpeed;	//目標速度
	st_data.f_nowDist	= 0;			//現在位置
	st_data.f_dist		= st_info.f_dist;	//等速完了位置
	st_data.f_accAngleS	= 0;			//角加速度
	st_data.f_nowAngleS	= 0;			//現在角速度
	st_data.f_trgtAngleS	= 0;			//目標角度
	st_data.f_nowAngle	= 0;			//現在角度
	st_data.f_angle		= 0;			//目標角度
	st_data.f_time		= 0;			//目標時間 [sec] ←指定しない
	
	CTRL_clrData();					//設定データをクリア
	CTRL_setData( &st_data );			//データセット
	f_TrgtSpeed		= f_MotNowSpeed;	//目標速度

	//printf("st_info.f_dist:%5.4f \n\r",st_info.f_dist);

	while( f_NowDist < st_info.f_dist  ){		//指定距離到達待ち
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE ){
				return;
			}

	}
	
}

// *************************************************************************
//   機能		： 壁あて制御
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_goHitBackWall(void){
	
	volatile stMOT_DATA	st_info;			//シーケンスデータ
	stCTRL_DATA		st_data;			//制御データ
	
	/*----------------*/
	/* 動作データ計算 */
	/*----------------*/
	st_info.f_acc1	= 800;
	
	/*--------*/
	/* バック */
	/*--------*/
	st_data.en_type			= CTRL_HIT_WALL;
	st_data.f_acc			= st_info.f_acc1 * 1.0;		// 加速度指定
	st_data.f_now			= 0;				// 現在速度
	st_data.f_trgt			= 0;				// 目標速度
	st_data.f_nowDist		= 0;				// 進んでいない
	st_data.f_dist			= 0;				// 加速距離
	st_data.f_accAngleS		= 0;				// 角加速度
	st_data.f_nowAngleS		= 0;				// 現在角速度
	st_data.f_trgtAngleS	= 0;					// 目標角度
	st_data.f_nowAngle		= 0;				// 現在角度
	st_data.f_angle			= 0;				// 目標角度
	st_data.f_time 			= 0;				// 目標時間 [sec] ← 指定しない
	
	CTRL_clrData();							// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );					// データセット
	DCM_staMotAll();						// モータON
	
	TIME_wait(550);
	
	/* 停止 */
	CTRL_stop();		// 制御停止
	DCM_brakeMot( DCM_R );	// ブレーキ
	DCM_brakeMot( DCM_L );	// ブレーキ
	TIME_wait(300);
	
	f_MotNowSpeed = 0.0f;		//現在速度更新
	
}

// *************************************************************************
//   機能		： スラローム
//   注意		： なし
//   メモ		： なし
//   引数		： スラロームコマンド，スラロームデータ
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_goSla( enMOT_SULA_CMD en_type, stSLA *p_sla){
	
	volatile stMOT_DATA	st_info;			//シーケンスデータ
	stCTRL_DATA		st_data;			//制御データ
	
	FLOAT			f_entryLen;			
	FLOAT			f_escapeLen;
		
	/* -------------- */
	/* 動作データ計算 */
	/* -------------- */
	/* 加速度 */
	st_info.f_acc1 		= 0;					//加速度1[m/s^2]
	st_info.f_acc3		= 0;					//加速度3[m/s^2]
	
	/* 速度 */
	st_info.f_now		= p_sla -> f_speed;			//現在速度
	st_info.f_trgt		= p_sla -> f_speed;			//目標速度
	st_info.f_last		= p_sla -> f_speed;			//最終速度
	
	/* 距離 */
	st_info.f_dist		= 0;
	st_info.f_l1		= 0;
	st_info.f_l1_2		= 0;
	
	/* 角加速度 */
	st_info.f_accAngleS1	= p_sla -> f_angAcc;
	st_info.f_accAngleS3	= p_sla -> f_angAcc;
	
	/* 角速度 */
	st_info.f_nowAngleS 	= 0;
	st_info.f_trgtAngleS 	= p_sla -> f_angvel;
	st_info.f_lastAngleS	= 0;
	
	/* 角度 */
	st_info.f_angle		= p_sla -> f_ang_Total;			//旋回角度[deg]
	st_info.f_angle1	= p_sla -> f_ang_AccEnd;		//第1移動角度[deg]
	st_info.f_angle1_2	= p_sla -> f_ang_ConstEnd;		//第1+2移動角度[deg]


	/* 方向に応じて符号を変更 */
	if( ( en_type == MOT_R90S ) ||
	    ( en_type == MOT_R45S_S2N ) || ( en_type == MOT_R45S_N2S )|| 
	    ( en_type == MOT_R90S_N ) ||
	    ( en_type == MOT_R135S_S2N ) || (en_type == MOT_R135S_N2S)
	){
		st_info.f_accAngleS1	*= -1;
		st_info.f_trgtAngleS	*= -1;
		st_info.f_angle		*= -1;
		st_info.f_angle1	*= -1;
		st_info.f_angle1_2	*= -1;
	}
	else{
		st_info.f_accAngleS3	*= -1;
	}
	
	
	/* 斜めのタイプに応じて、スラローム前の距離とスラローム後の退避距離を入れ替える */
		if( ( en_type == MOT_R45S_N2S ) || ( en_type == MOT_L45S_N2S ) || ( en_type == MOT_R135S_N2S ) || ( en_type == MOT_L135S_N2S ) ){	//逆にするもの
		f_entryLen	= p_sla -> f_escapeLen;
		f_escapeLen	= p_sla -> f_entryLen;
	}
	else{	//通常
	
		f_entryLen	= p_sla -> f_entryLen;
		f_escapeLen	= p_sla -> f_escapeLen;
	}
	
	/* ========== */
	/* 　実動作　 */
	/* ========== */
	/* ---------------------- */
	/* スラローム前の前進動作 */
	/* ---------------------- */
	st_data.en_type			= CTRL_ENTRY_SLA;
	st_data.f_acc			= 0;				// 加速度指定
	st_data.f_now			= st_info.f_now;	// 現在速度
	st_data.f_trgt			= st_info.f_now;	// 目標速度
	st_data.f_nowDist		= 0;				// 進んでいない
	st_data.f_dist			= f_entryLen;		// スラローム前の前進距離
	st_data.f_accAngleS		= 0;				// 角加速度
	st_data.f_nowAngleS		= 0;				// 現在角速度
	st_data.f_trgtAngleS	= 0;				// 目標角度
	st_data.f_nowAngle		= 0;				// 現在角度
	st_data.f_angle			= 0;				// 目標角度
	st_data.f_time 			= 0;				// 目標時間 [sec] ← 指定しない
	
	CTRL_clrData();					// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );			// データセット
	DCM_staMotAll();				// モータON

//	LED_onAll();
	while( f_NowDist < f_entryLen ){				// 指定距離到達待ち
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
	}
	
	
	/* -------- */
	/* 　加速　 */
	/* -------- */
	st_data.en_type			= CTRL_ACC_SLA;	
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen;	
	st_data.f_dist			= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;		//加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= st_info.f_angle1;			// 目標角度
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;		//[msec] → [sec]
	
	CTRL_setData( &st_data );			// データセット
	
	if( IS_R_SLA( en_type ) == true ){	// -方向
		while( ( f_NowAngle > st_info.f_angle1 ) ){
		//while( ( f_NowAngle > st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			
			//break;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1 ) ){
		//while( ( f_NowAngle < st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;

			//break;
		}
	}
	
	
	/* -------- */
	/* 　等速　 */
	/* -------- */
	st_data.en_type			= CTRL_CONST_SLA;	
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;	
	st_data.f_dist			= f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime) * 0.001;		//加速距離
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// 現在角速度
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= st_info.f_angle1;			// 現在角度
	st_data.f_angle			= st_info.f_angle1_2;		// 目標角度
	st_data.f_time			= p_sla->us_constAngvelTime * 0.001;
			
	CTRL_setData( &st_data );				// データセット
	
	if( IS_R_SLA( en_type ) == true ){		// -方向
		while( ( f_NowAngle > st_info.f_angle1_2 ) ){
		//while( ( f_NowAngle > st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			//break;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1_2 ) ){
		//while( ( f_NowAngle < st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
			//break;
		}
	}
	
	
	/* -------- */
	/* 　減速　 */
	/* -------- */
	st_data.en_type			= CTRL_DEC_SLA;	
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime)* 0.001;	
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2) * 0.001;		//加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// 角加速度
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// 現在角速度
	st_data.f_trgtAngleS	= 0;						// 目標角速度
	st_data.f_nowAngle		= st_info.f_angle1_2;		// 現在角度
	st_data.f_angle			= st_info.f_angle;			// 目標角度
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;

	CTRL_setData( &st_data );			// データセット
	
	if( IS_R_SLA( en_type ) == true ){	// -方向
		while( ( f_NowAngle > st_info.f_angle ) ){
		//while( ( f_NowAngle > st_info.f_angle + 2.0f ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
		}
	
	}else{
		while( ( f_NowAngle < st_info.f_angle ) ){
		//while( ( f_NowAngle < st_info.f_angle -1.0f ) || ( f_NowDist < st_data.f_dist ) ){
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
		}
	}
	
	/* ---------------------- */
	/* スラローム後の前進動作 */
	/* ---------------------- */
	st_data.en_type			= CTRL_EXIT_SLA;
	st_data.f_acc			= 0;				//加速度指定
	st_data.f_now			= st_info.f_now;	//現在速度
	st_data.f_trgt			= st_info.f_now;	//目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2) * 0.001;			//進んでいない
	st_data.f_dist			= f_escapeLen + f_entryLen + st_info.f_now * (p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2)*0.001;		//スラローム後の前進距離
	st_data.f_accAngleS		= 0;				// 角加速度
	st_data.f_nowAngleS		= 0;				// 現在角速度
	st_data.f_trgtAngleS	= 0;				// 目標角度
	st_data.f_nowAngle		= 0;				// 現在角度
	st_data.f_angle			= 0;				// 目標角度
	st_data.f_time 			= 0;				// 目標時間 [sec] ← 指定しない
	
	//CTRL_clrData();					// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );			// データセット

	while( f_NowDist < st_data.f_dist ){				// 指定距離到達待ち
			/* フェイルセーフ */
			MOT_Failsafe(&bl_failsafe);
			if( bl_failsafe == TRUE )return;
	}
	LED_offAll();
	f_MotNowSpeed = st_info.f_now;			// 現在速度更新

}

// *************************************************************************
//   機能		： スラローム開始速度を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： 開始速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC	void MOT_setSlaStaSpeed( FLOAT f_speed ) {
	
	f_MotSlaStaSpeed = f_speed;

}

// *************************************************************************
//   機能		： スラローム開始速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： 開始速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC	FLOAT MOT_getSlaStaSpeed( void ) {
	return f_MotSlaStaSpeed;
}

// *************************************************************************
//   機能		： 壁切れ補正のタイプを取得
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 壁切れ補正のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void ){
	return en_WallEdge;
}

// *************************************************************************
//   機能		： 壁切れの検知を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_setWallEdge( BOOL bl_val ){
	
	bl_IsWallEdge = bl_val;
	
}

// *************************************************************************
//   機能		： 壁切れの検知を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type ){
	
	en_WallEdge = en_type;
	bl_IsWallEdge = false;			// 非検知
	
}

// *************************************************************************
//   機能		： 壁の切れ目で補正する距離を算出して設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PRIVATE BOOL MOT_setWallEdgeDIST( void ){
	
	FLOAT f_addDist;
	
	/* 壁の切れ目を検知していない */
	if( ( bl_IsWallEdge == false ) || ( en_WallEdge == MOT_WALL_EDGE_NONE ) ){	// 壁切れ設定されていないか、検出していない場合は処理を抜ける
		
		return false;
		
	}
	
	f_addDist = f_NowDist + MOT_WALL_EDGE_DIST;	//旋回開始位置
	
	/* 多く走る必要がある */
	if( f_addDist > st_Info.f_dist ){
		f_WallEdgeAddDist = f_addDist - st_Info.f_dist;
	}
	
	/* 壁の切れ目補正の変数を初期化 */
	en_WallEdge	= MOT_WALL_EDGE_NONE;		//壁の切れ目タイプ
	bl_IsWallEdge	= false;			//壁の切れ目検知
	
	return true;
}

// *************************************************************************
//   機能		： 壁の切れ目で補正する距離を算出して設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PRIVATE	BOOL MOT_setWallEdgeDIST_LoopWait( void ){
	
	/* 壁の切れ目を検知していない */
	if( bl_IsWallEdge == false ){		// 壁切れ設定されていないか、検出していない場合は処理を抜ける
		return	false;
	}
	
	f_WallEdgeAddDist = MOT_WALL_EDGE_DIST;		// 旋回開始位置
	
	return true;
}

// *************************************************************************
//   機能		： マウスの目標速度を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： 目標速度
//   返り値		： 目標速度
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_setTrgtSpeed( FLOAT f_speed){
	
	f_MotTrgtSpeed = f_speed;

}

// *************************************************************************
//   機能		： マウスの目標速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 目標速度
// **************************    履    歴    *******************************
// 		v1.0		2019.11.28			TKR			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getTrgtSpeed( void ){
	
	return f_MotTrgtSpeed;

}

// *************************************************************************
//   機能		： 斜め走行の目標速度を設定する
//   注意		： f_MotTrgtSpeedに代入していないのに注意
//   メモ		： なし
//   引数		： 目標速度
//   返り値		： 目標速度
// **************************    履    歴    *******************************
// 		v1.0		2019.11.28			TKR			新規
// *************************************************************************/
PUBLIC void MOT_setTrgtSkewSpeed( FLOAT f_speed ){
	
	f_MotTrgtSkewSpeed = f_speed;

}

// *************************************************************************
//   機能		： 斜め走行の目標速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 目標速度
// **************************    履    歴    *******************************
// 		v1.0		2019.11.28			TKR			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getTrgtSkewSpeed( void ){
	
	return f_MotTrgtSkewSpeed;

}

// *************************************************************************
//   機能		： マウスの現在速度を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： 現在速度
//   返り値		： 現在速度
// **************************    履    歴    *******************************
// 		v1.0		2019.5.1			TKR			新規
// *************************************************************************/
PUBLIC void MOT_setNowSpeed( FLOAT f_speed){
	
	f_MotNowSpeed = f_speed;

}

// *************************************************************************
//   機能		： フェイルセーフ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： TRUE：発動	FALSE：何もなし
// **************************    履    歴    *******************************
// 		v1.0		2019.9.22			TKR			新規
// *************************************************************************/
PRIVATE void MOT_Failsafe( BOOL* exists ){
	
	if( f_NowAccel < FAIL_THRESH_ACC ){
		CTRL_stop();
		*exists	= TRUE;

		SPK_on(F4,16.0f,120);
		SPK_on(E4,16.0f,120);
		SPK_on(Eb4,16.0f,120);

		while(1);
	}else{

		*exists	= FALSE;
	}
	
}

// *************************************************************************
//   機能		： サーキット
//   注意		： 右回り：Y	左回り：X
//   メモ		： なし
//   引数		： 区画の広さX，区画の広さY，回る方向，回る回数，速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.10.18			TKR			新規
// *************************************************************************/
PUBLIC void MOT_circuit(FLOAT x,FLOAT y, enMOT_SULA_CMD en_type, int num, FLOAT f_speed){

	int i = 0;

	if( en_type == MOT_R90 ){
		MOT_goBlock_FinSpeed( y-1.5f+MOVE_BACK_DIST, f_speed );	
		for( i = 0; i < num; i++ ){
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( x-2.0f, f_speed );				
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);					
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);				
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);
		}
	}else{

		MOT_goBlock_FinSpeed( x-1.5f+MOVE_BACK_DIST, f_speed );
		for( i = 0; i < num; i++ ){
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( y-2.0f, f_speed );				
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);					
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( y-2.0f, f_speed);				
			MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			// スラローム
			MOT_goBlock_FinSpeed( x-2.0f, f_speed);
		}

	}
	
	MOT_goBlock_FinSpeed(0.5f,0);					// 半区画走行
}