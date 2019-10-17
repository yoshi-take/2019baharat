// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： MODE
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.2.5			TKR			新規
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <iodefine.h>		// I/O
#include <typedefine.h>		// 定義
#include <stdio.h>			// 標準ライブラリ

#include <mode.h>			// MODE
#include <hal_led.h>		// LED
#include <hal_spk.h>		// SPK
#include <hal_battery.h>	// バッテリ
#include <hal_sci.h>		// SCI
#include <hal_spi.h>		// SPI
#include <hal_gyro.h>		// ジャイロ
#include <hal_dcm.h>		// DCM
#include <hal_enc.h>		// ENC
#include <hal_dist.h>       // DIST
#include <motion.h>			// motion
#include <hal_dcmCtrl.h>	// CTRL

#include <parameter.h>		// parameter
#include <map.h>			// map

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
PRIVATE enMODE		en_Mode;		// 現在のモード

extern PUBLIC UCHAR			g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路情報

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： モードを実行する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PUBLIC void	MODE_exe( void ){

	enMAP_HEAD_DIR		en_endDir;
	int i=0;
	
	/* 走行パラメータ設定 */
	PARAM_setCntType( TRUE );
	MOT_setTrgtSpeed( 350.0f );						// 目標速度設定
	PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
	PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
	PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速
	
	
	/* スラロームデータ生成 */
	PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// 90度
	// 45度
	// 135度
	// 斜め → 90°→ 斜め

	switch( en_Mode ){
		
		case MODE_0:	// バッテリーチェックor壁チェック
			LED_offAll();	
			//BAT_Check();
			DIST_Check();
			break;
			
		case MODE_1:		// 探索走行
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1000);
			GYRO_clrAngle();		// 角度リセット

			/* 走行パラメータ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );			// 目標速度設定			
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );	// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速

			/* 迷路探索 */
			MAP_setPos(0,0,NORTH);
			MAP_searchGoal(GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA);

			/* 帰り探索 */
			TIME_wait(1000);

			/* コマンド作成 */
			PARAM_setCntType(TRUE);											// 最短走行
			MAP_setPos(0,0,NORTH);											// 初期座標
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// 等高線マップ作成
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// ドライブコマンド作成
			MAP_makeSuraCmdList();											// スラロームコマンド作成

			break;
			
		case MODE_2:		// マップデータのロード
			LED_offAll();
			TIME_wait(1000);
			LED_onAll();
			MAP_LoadMapData();
			LED_offAll();
			break;
			
		case MODE_3:		// 最短走行
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed(900.0f);
			MOT_setSlaStaSpeed(500.0f);
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );		// [旋回]
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );		// [スラローム]

			/* コマンド作成 */
			MAP_setPos(0,0,NORTH);
			MAP_makeContourMap(GOAL_MAP_X,GOAL_MAP_Y,BEST_WAY);
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y,&en_endDir);
			MAP_makeSuraCmdList();
			MAP_makeSkewCmdList();
			
			/* コマンド走行 */
			MAP_drive(MAP_DRIVE_SURA);
			
			break;
			
		case MODE_4:		// スラローム
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			CTRL_LogSta();			// ログ開始

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( 500.0f );						// 目標速度設定
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速

			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// スラロームデータ生成	

			MOT_goBlock_FinSpeed(1.5f+MOVE_BACK_DIST,500.0f);				// 半区画走行
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// スラローム
			MOT_goBlock_FinSpeed(1.0f,500.0f);				// 半区画走行
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// スラローム
			MOT_goBlock_FinSpeed(1.0f,500.0f);				// 半区画走行
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// スラローム
			MOT_goBlock_FinSpeed(1.0f,500.0f);				// 半区画走行
			MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// スラローム
			MOT_goBlock_FinSpeed(1.0f,500.0f);				// 半区画走行


			LED_onAll();

			MOT_goBlock_FinSpeed(0.5f,0);					// 半区画走行
			break;

			
			break;
			
		case MODE_5:		// 直進&超信地調整			
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			CTRL_LogSta();			// ログ開始

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );						// 目標速度設定
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );	// [直進]
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]

			MOT_goBlock_FinSpeed(5,0);
			//MOT_turn2(MOT_L90,700.0f);
			TIME_wait(100);
			
			LED_onAll();
			
			break;
			
		case MODE_6:	// ログ関係
			LED_offAll();
			TIME_wait(100);
			CTRL_showLog();		// ログの掃き出し
			//MAP_showLog();

			break;
			
		case MODE_7:	// 探索
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1000);
			GYRO_clrAngle();		// 角度リセット

			/* 走行パラメータ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( MAP_SEARCH_SPEED );			// 目標速度設定			
			PARAM_setSpeedType( PARAM_ST, PARAM_VERY_SLOW );	// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_VERY_SLOW );	// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_VERY_SLOW );	// [スラローム]速度低速

			/* 迷路探索 */
			MAP_setPos(0,0,NORTH);
			MAP_searchGoal(GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_TURN);

			/* 帰り探索 */
			TIME_wait(1000);

			/* コマンド作成 */
			PARAM_setCntType(TRUE);											// 最短走行
			MAP_setPos(0,0,NORTH);											// 初期座標
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// 等高線マップ作成
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// ドライブコマンド作成
			MAP_makeSuraCmdList();											// スラロームコマンド作成


			break;
			
		case MODE_8:	// マップデータの消去
			LED_onAll();
			TIME_wait(1000);
			MAP_ClearMapData();
			LED_offAll();
			break;
	
		default:
			break;
			
	}
}

// *************************************************************************
//   機能		： 予約されたモードに変更する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PRIVATE void MODE_chg( enMODE en_mode ){
	
	LED_offAll();	
	
	switch( en_mode ){
		
		case MODE_0:
			SPK_on(A4,16.0f,120);
			break;
			
		case MODE_1:
			SPK_on(C4,16.0f,120);
			LED_on(LED0);
			break;
			
		case MODE_2:
			SPK_on(D4,16.0f,120);
			LED_on(LED1);
			break;
			
		case MODE_3:
			SPK_on(E4,16.0f,120);
			LED_on(LED2);
			break;
			
		case MODE_4:
			SPK_on(F4,16.0f,120);
			LED_on(LED3);
			break;
			
		case MODE_5:
			SPK_on(G4,16.0f,120);
			LED_on(LED4);
			break;
			
		case MODE_6:
			SPK_on(A4,16.0f,120);
			LED_on(LED0);
			LED_on(LED1);
			break;
			
		case MODE_7:
			SPK_on(B4,16.0f,120);
			LED_on(LED0);
			LED_on(LED2);
			break;
			
		case MODE_8:
			SPK_on(C5,16.0f,120);
			LED_on(LED0);
			LED_on(LED3);
			break;
			
		default:
			break;
			
	}
	
	en_Mode = en_mode;	// 現在のモード			
}

// *************************************************************************
//   機能		： モードを加算変更する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.2.5			TKR				新規
// *************************************************************************/
PUBLIC void MODE_inc( void ){
	
	en_Mode++;
	
	/* 最大値チェック */
	if( MODE_MAX == en_Mode ){
		en_Mode = MODE_0;
	}
	
	MODE_chg(en_Mode);		// モード変更
	
}

// *************************************************************************
//   機能		： 前壁(右)が閾値以上だとフラグが立つ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 検知した：true	検知できなかった：false
// **************************    履    歴    *******************************
// 		v1.0		2019.8.16			TKR			新規
// *************************************************************************/
PUBLIC BOOL MODE_DistRightCheck(){
	
	SHORT 	s_rightval;
	BOOL	bl_check;
	
	s_rightval 	= DIST_getNowVal(DIST_SEN_R_FRONT);
	
	if( s_rightval >= EXE_THRESH_R ){
		bl_check = true;
	
	}else{
		bl_check = false;
	
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 前壁(左)が閾値以上だとフラグが立つ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 検知した：true	検知できなかった：false
// **************************    履    歴    *******************************
// 		v1.0		2019.8.16			TKR			新規
// *************************************************************************/
PUBLIC BOOL MODE_DistLeftCheck(){
	
	SHORT 	s_leftval;
	BOOL	bl_check;
	
	s_leftval 	= DIST_getNowVal(DIST_SEN_L_FRONT);
	
	if( s_leftval >= EXE_THRESH_L ){
		bl_check = true;
	
	}else{
		bl_check = false;
	
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 手をかざすと待機状態に入る
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 両方検知：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2019.8.16			TKR			新規
// *************************************************************************/
PUBLIC BOOL MODE_setWaitCheck(){
	
	BOOL bl_check;
	
	if( true == MODE_DistRightCheck() ){	// 右だけ検知
		LED_on_multi(0x18);

	}
	if( true == MODE_DistLeftCheck() ){		// 左だけ検知
		LED_on_multi(0xc0);

	}
	
	if( ( true == MODE_DistRightCheck() ) && ( true == MODE_DistLeftCheck() ) ){
		LED_onAll();
		bl_check = true;
		
	}else if( ( false == MODE_DistRightCheck() ) && ( false == MODE_DistLeftCheck() )){
		LED_offAll();
		bl_check = false;
	
	}else{
		bl_check = false;
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 手をかざすと待機状態に入り、かざしてから離すと実行
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 待機状態から抜け出す：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2018.8.16			吉田			新規
// *************************************************************************/
PUBLIC BOOL MODE_CheckExe(){
	
	BOOL bl_check;
	
	if( true == MODE_setWaitCheck() ){
		TIME_wait(200);
		
		if( false == MODE_setWaitCheck() ){
			LED_offAll();
			TIME_wait(1000);
			bl_check = true;
			
		}else{
			bl_check = false;
		
		}
		
	}else{
		
		bl_check = false;
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 手をかざすと待機状態に入り、かざしてから離すと実行
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 待機状態から抜け出す：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2018.8.16			吉田			新規
// *************************************************************************/
PRIVATE void MODE_wait( void ){

	while(1){
		LED_onAll();

		if(( true == MODE_DistRightCheck() ) && ( true == MODE_DistLeftCheck() )){
			LED_offAll();
			TIME_wait(1000);
			break;
		}	
	}

}