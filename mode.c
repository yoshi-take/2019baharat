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
PRIVATE	USHORT		us_DefaultCnt;	// カウンタ初期値

extern PUBLIC UCHAR			g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路情報

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： モードの初期化。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.10.27			TKR				新規
// *************************************************************************/
PUBLIC void MODE_init( void ){
	us_DefaultCnt 	= ENC_getCnt(ENC_R);
}


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
	MOT_setTrgtSpeed( MAP_SEARCH_SPEED );						// 目標速度設定
	PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
	PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
	PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速
	
	
	/* スラロームデータ生成 */
	PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// 90度
	PARAM_makeSla(500.0f, 100.0f, 2500.0f, SLA_45, PARAM_VERY_SLOW);	// 45度
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
			//MAP_searchGoalKnown(GOAL_MAP_X, GOAL_MAP_Y, SEARCH_SURA);

			/* 帰り探索 */
			TIME_wait(500);
			MAP_searchGoal(0, 0, SEARCH, SEARCH_SURA);

			/* コマンド作成 */
			PARAM_setCntType(TRUE);											// 最短走行
			MAP_setPos(0,0,NORTH);											// 初期座標
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// 等高線マップ作成
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// ドライブコマンド作成
			MAP_makeSuraCmdList();											// スラロームコマンド作成

			break;
			
		case MODE_2:		// マップデータのロード
			LED_offAll();
			TIME_wait(500);
			LED_onAll();
			MAP_LoadMapData();
			LED_offAll();
			MAP_showLog();
			break;
			
		case MODE_3:		// 最短走行
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			/* スラロームデータ生成 */
			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);		// 90度
			PARAM_makeSla(500.0f, 100.0f, 2500.0f, SLA_45, PARAM_SLOW);			// 45度
			PARAM_makeSla(500.0f, 150.0f, 5500.0f, SLA_135, PARAM_SLOW);		// 135度
			PARAM_makeSla(500.0f, 250.0f, 7500.0f, SLA_N90, PARAM_SLOW);		// 斜め → 90°→ 斜め

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed(1000.0f);
			MOT_setSlaStaSpeed(500.0f);
			PARAM_setSpeedType( PARAM_ST, PARAM_NORMAL );		// [直進]
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );		// [旋回]
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );		// [スラローム]

			/* コマンド作成 */
			MAP_setPos(0,0,NORTH);
			MAP_makeContourMap(GOAL_MAP_X,GOAL_MAP_Y,BEST_WAY);
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y,&en_endDir);
			MAP_makeSuraCmdList();
			MAP_makeSkewCmdList();
			MAP_showCmdLog();

			/* コマンド走行 */
			MAP_drive(MAP_DRIVE_SKEW);
			break;
			
		case MODE_4:		// スラローム
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			CTRL_LogSta();			// ログ開始

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( 700.0f );						// 目標速度設定
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );		// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );	// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_NORMAL );	// [スラローム]速度低速

			PARAM_makeSla(700.0f, 300.0f, 9000.0f, SLA_90, PARAM_NORMAL);		// スラロームデータ生成

			MOT_circuit( 3, 3, MOT_L90, 10, 700.0f);	

			SPK_on(Eb4,16.0f,120);
			SPK_on(E4,16.0f,120);
			SPK_on(F4,16.0f,120);

			break;
			
		case MODE_5:		// 直進&超信地調整			
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			CTRL_LogSta();			// ログ開始

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed( 3200.0f );						// 目標速度設定
			PARAM_setSpeedType( PARAM_ST, PARAM_VERY_FAST );	// [直進]
			PARAM_setSpeedType( PARAM_TURN, PARAM_NORMAL );		// [旋回]
			PARAM_setSpeedType( PARAM_SLA, PARAM_NORMAL );		// [スラローム]

			MOT_goBlock_FinSpeed(7,0);
			//MOT_turn2(MOT_L90,700.0f);
			TIME_wait(100);
			
			LED_onAll();
			
			break;
			
		case MODE_6:	// ログ関係
			LED_offAll();
			TIME_wait(100);
			//CTRL_showLog();		// ログの掃き出し
			MAP_showLog();

			break;
			
		case MODE_7:	// 斜め
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1000);
			GYRO_clrAngle();		// 角度リセット

			CTRL_LogSta();			// ログ開始

			/* スラロームデータ生成 */
			PARAM_makeSla(500.0f, 150.0f, 5300.0f, SLA_90, PARAM_SLOW);					// 90度
			PARAM_makeSla(500.0f, 100.0f, 2500.0f, SLA_45, PARAM_SLOW);					// 45度
//			PARAM_makeSla(800.0f, 250.0f, 13000.0f, SLA_135, PARAM_SLOW);				// 135度			
			PARAM_makeSla(500.0f, 150.0f, 5500.0f, SLA_135, PARAM_SLOW);				// 135度
			PARAM_makeSla(500.0f, 250.0f, 7500.0f, SLA_N90, PARAM_SLOW);				// 斜め → 90°→ 斜め
			
			/* 走行パラメータ */
			PARAM_setCntType( FALSE );
			MOT_setTrgtSpeed( 500.0f );				// 目標速度設定			
			PARAM_setSpeedType( PARAM_ST, PARAM_SLOW );			// [直進]速度低速
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );		// [旋回]速度低速
			PARAM_setSpeedType( PARAM_SLA, PARAM_SLOW );	// [スラローム]速度低速

			/* 走行(45) */
#if 0			
			MOT_goBlock_FinSpeed(1.0f+MOVE_BACK_DIST,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_R45S_S2N,PARAM_getSra( SLA_45 ));
			MOT_goSkewBlock_FinSpeed(3.5f,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_L45S_N2S,PARAM_getSra( SLA_45 ));
			MOT_goBlock_FinSpeed(0.5,0);
#endif			
#if 1
			/* 走行(135) */
			MOT_setTrgtSpeed(900.0f);
			MOT_goBlock_FinSpeed(3.5f+MOVE_BACK_DIST,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_R135S_S2N,PARAM_getSra( SLA_135 ));
			MOT_goSkewBlock_FinSpeed(3.5f,0);
//			MOT_goSla(MOT_L135S_N2S,PARAM_getSra( SLA_135 ));
//			MOT_goBlock_FinSpeed(0.5,0);
#endif
#if 0
			/* 走行(N90) */
			MOT_goBlock_FinSpeed(3.5f+MOVE_BACK_DIST,500.0f);
			MOT_goSla(MOT_R135S_S2N,PARAM_getSra( SLA_135 ));
			MOT_goSkewBlock_FinSpeed(0.5f,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_L90S_N,PARAM_getSra( SLA_N90 ));
			MOT_goSkewBlock_FinSpeed(0.5f,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_R90S_N,PARAM_getSra( SLA_N90 ));
			MOT_goSkewBlock_FinSpeed(0.5f,MAP_SEARCH_SPEED);
			MOT_goSla(MOT_L45S_N2S,PARAM_getSra( SLA_45 ));
			MOT_goBlock_FinSpeed(0.5f,0);
			LED_onAll();
#endif
			break;
			
		case MODE_8:
			LED_offAll();
			printf("g_sysMap:%x\n\r",sizeof(g_sysMap));	
			break;

		case MODE_9:
			LED_offAll();
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
			MAP_searchGoalKnown(GOAL_MAP_X, GOAL_MAP_Y, SEARCH_SURA);

			/* 帰り探索 */
			TIME_wait(500);
			MAP_searchGoalKnown(0, 0, SEARCH_SURA);

			/* コマンド作成 */
			PARAM_setCntType(TRUE);											// 最短走行
			MAP_setPos(0,0,NORTH);											// 初期座標
			MAP_makeContourMap(GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY);			// 等高線マップ作成
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y, &en_endDir);	// ドライブコマンド作成
			MAP_makeSuraCmdList();											// スラロームコマンド作成


			break;

		case MODE_10:
			LED_offAll();
			MAP_knowndebug();
			break;

		case MODE_11:
			LED_offAll();
			MAP_SaveMapData();
			LED_onAll();
			break;

		case MODE_12:		// 最短走行
			LED_offAll();
			MODE_wait();			// 手かざし待機
			TIME_wait(1500);
			GYRO_clrAngle();		// 角度リセット
			
			/* スラロームデータ生成 */
			PARAM_makeSla(600.0f, 200.0f, 7000.0f, SLA_90, PARAM_NORMAL);		// 90度
			PARAM_makeSla(600.0f, 100.0f, 4000.0f, SLA_45, PARAM_NORMAL);		// 45度
			PARAM_makeSla(600.0f, 200.0f, 8500.0f, SLA_135, PARAM_NORMAL);		// 135度
			PARAM_makeSla(600.0f, 350.0f, 10000.0f, SLA_N90, PARAM_NORMAL);		// 斜め → 90°→ 斜め

			/* 走行パラメータ */
			PARAM_setCntType( TRUE );
			MOT_setTrgtSpeed(3000.0f);
			MOT_setSlaStaSpeed(600.0f);
			PARAM_setSpeedType( PARAM_ST, PARAM_FAST );			// [直進]
			PARAM_setSpeedType( PARAM_TURN, PARAM_SLOW );		// [旋回]
			PARAM_setSpeedType( PARAM_SLA, PARAM_NORMAL );		// [スラローム]

			/* コマンド作成 */
			MAP_setPos(0,0,NORTH);
			MAP_makeContourMap(GOAL_MAP_X,GOAL_MAP_Y,BEST_WAY);
			MAP_makeCmdList(0,0,NORTH,GOAL_MAP_X,GOAL_MAP_Y,&en_endDir);
			MAP_makeSuraCmdList();
			MAP_makeSkewCmdList();
			MAP_showCmdLog();

			/* コマンド走行 */
			MAP_drive(MAP_DRIVE_SKEW);
			break;

		case MODE_13:
			SPK_debug();
			break;

		case MODE_14:
			break;
	
		case MODE_15:			// マップデータの消去
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
			SPK_on(D6,16.0f,120);
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

		case MODE_9:
			SPK_on(D5,16.0f,120);
			LED_on(LED0);
			LED_on(LED4);
			break;

		case MODE_10:
			SPK_on(E5,16.0f,120);
			LED_on(LED1);
			LED_on(LED2);
			break;

		case MODE_11:
			SPK_on(F5,16.0f,120);
			LED_on(LED1);
			LED_on(LED3);
			break;

		case MODE_12:
			SPK_on(G5,16.0f,120);
			LED_on(LED1);
			LED_on(LED4);
			break;

		case MODE_13:
			SPK_on(A5,16.0f,120);
			LED_on(LED2);
			LED_on(LED3);
			break;

		case MODE_14:
			SPK_on(B_5,16.0f,120);
			LED_on(LED2);
			LED_on(LED4);
			break;
	
		case MODE_15:
			SPK_on(C6,16.0f,120);
			LED_on(LED3);
			LED_on(LED4);
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
//   機能		： モードを減算変更する。
//   注意		： なし
//   メモ		： エンコーダでモード変更できるように追加
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.10.27			TKR				新規
// *************************************************************************/
PUBLIC void MODE_dec( void ){
		
	/* 最大値チェック */
	if( MODE_0 == en_Mode ){
		en_Mode = MODE_MAX;
	}
	
	en_Mode--;

	MODE_chg(en_Mode);		// モード変更
	
}


// *************************************************************************
//   機能		： モードの切り替えチェック
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
//		v1.0		2019.10.27			TKR				新規
// *************************************************************************/
PUBLIC void MODE_chkMode( void ){

	if (( us_DefaultCnt - MODE_CHG_COUNT ) > ENC_getCnt(ENC_R) ){
		ENC_clr();
		MODE_inc();
	}else if( ( us_DefaultCnt + MODE_CHG_COUNT ) < ENC_getCnt(ENC_R) ){
		ENC_clr();
		MODE_dec();
	}

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