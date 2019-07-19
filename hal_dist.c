// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： dist
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
#include <string.h>							// 文字列操作（memset用）
#include <common_define.h>					// common_define

#include <hal_led.h>		// LED
#include <hal_dist.h>		// DIST
#include <hal_sci.h>		// SCI
#include <parameter.h>		// PARAMETER


//**************************************************
// 定義（define）
//**************************************************
#define	SEN_FLASH_SIZE		( 64 )												// センサ用データフラッシュのデータ量

/* 距離センサ */
#define		LED_DIST_RF		( PORTA.PODR.BIT.B4 )			// ビットアクセス
#define		LED_DIST_RS		( PORTA.PODR.BIT.B6 )			// ビットアクセス
#define		LED_DIST_LF		( PORTA.PODR.BIT.B1 )			// ビットアクセス
#define		LED_DIST_LS		( PORT4.PODR.BIT.B6 )			// ビットアクセス


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
/* 距離センサ情報（前壁のみ、データフラッシュ用構造体としても使用する） */
typedef struct{
	SHORT		s_wallHit;					///< @var : 壁に当たっていてもおかしくない値         ( AD 値 ) （前壁とマウス間が約2mmの時の値）
	SHORT		s_skewErr1;					///< @var : 斜め走行時の補正閾値1                    ( AD 値 )
	SHORT		s_skewErr2;					///< @var : 斜め走行時の補正閾値2                    ( AD 値 )
	SHORT		s_skewErr3;					///< @var : 斜め走行時の補正閾値3                    ( AD 値 )
}stDIST_FRONT_SEN;

/* 距離センサ情報（全センサ共通） */
typedef struct{
	SHORT		s_now;						// LED 点灯中の距離センサの現在値           ( AD 値 )
	SHORT		s_old;						// LED 点灯中の距離センサの1つ前の値        ( AD 値 )
	SHORT		s_limit;					// 距離センサの閾値                         ( AD 値 ) ( この値より大きい場合、壁ありと判断する )
	SHORT		s_ref;						// 区画の中心に置いた時の距離センサの基準値 ( AD 値 )
	SHORT		s_offset;					// LED 消灯中の距離センサの値               ( AD 値 )
	SHORT		s_ctrl;						// 制御有効化する際の閾値                   ( AD 値 ) 主に前壁で使用
	SHORT		s_noCtrl;					// 壁に近すぎるため制御無効化する際の閾値   ( AD 値 ) 主に前壁で使用
}stDIST_SEN;

/* 距離センサ情報（全センサ共通、データフラッシュ用構造体のみに使用） */
typedef struct{
	SHORT		s_ref;						///< @var : 区画の中心に置いた時の距離センサの基準値 ( AD 値 )
	SHORT		s_limit;					///< @var : 距離センサの閾値                         ( AD 値 ) ( この値より大きい場合、壁ありと判断する )
	SHORT		s_ctrl;						///< @var : 制御有効化する際の閾値                   ( AD 値 ) 主に前壁で使用
	SHORT		s_noCtrl;					///< @var : 壁に近すぎるため制御無効化する際の閾値   ( AD 値 ) 主に前壁で使用
}stDIST_SEN_DATA;

#if 0
/* データフラッシュのバックアップ or 復帰用 */
typedef struct{
	stDIST_SEN_DATA		st_common[DIST_SEN_MAX];			// センサデータ(全センサ共通)
	stDIST_FRONT_SEN	st_front[DIST_SEN_L_FRONT+1];		// センサデータ(前壁のみ)
}stDIST_FLASH;
#endif

//**************************************************
// グローバル変数
//**************************************************
PRIVATE stDIST_SEN			st_sen[DIST_SEN_MAX];				// 距離センサ
PRIVATE	stDIST_FRONT_SEN	st_senF[DIST_SEN_L_FRONT+1];		// 距離センサ(前壁のみ)
//PRIVATE	USHORT				us_Log[DIST_LOG];					// ログ
//PRIVATE	USHORT				us_LogPt = 0;						// ログ位置
//PRIVATE UCHAR				us_LogSta = 0;						// ログ開始/停止（0:停止、1:開始）
PRIVATE BOOL				bl_NoChkErr = false;				// エラー量のチェックをするか（FALSE:エラー量チェックする、TRUE：エラー量チェックしない）

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
extern PUBLIC void TIME_wait(ULONG ul_time);
extern PUBLIC void TIME_waitFree(ULONG ul_cnt);

// *************************************************************************
//   機能		： 距離センサを初期化する
//   注意		： Storage_Saveを使うためには、1度読み込んでおく必要があるため、同じアドレスを起動時にStorage_Loadしている。(今後この処理を削除する改善の余地あり)
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DIST_init( void ){
	
	//stDIST_FLASH	st_dummy;
	
	//Storage_Load( (const void*)&st_dummy, SEN_FLASH_SIZE, ADR_SEN );	// データロード(dummy) これがないと次の１発目のsaveがうまくいかない
	
	/* 内部変数の初期化 */
	memset( st_sen, 0, sizeof(st_sen) );				// 距離センサ（全センサ共通）
	memset( st_senF, 0, sizeof(st_senF) );				// 距離センサ（前壁のみ）
	//memset( us_Log, 0, sizeof(us_Log[DIST_LOG]) );
}

// *************************************************************************
//   機能		： 全センサの閾値を設定する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DIST_setThresh_AllSenData( void )
{	
	// Flashと抱き合わせ。後で書く
}

// *************************************************************************
//   機能		： 全センサの閾値を移動して設定する
//   注意		： なし
//   メモ		： データフラッシュと抱き合わせ
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
#if 0
PUBLIC void DIST_setThresh_AllSenData_Move( void ){
	
	/* 手を離す時間を設ける */
	TIME_wait(1000);
	
	/* 移動 */
	bl_NoChkErr = true;			// エラー量をチェックしない
	
	MOT_setTrgtSpeed( (FLOAT)SEN_BACK_CHK_SPEED );			// 目標速度設定を低くする
	PARAM_setSpeedType( PARAM_ST, PARAM_VERY_SLOW );		// [直進]速度超低速
	
	//MOT_goBack_Const( MOT_BACK_SEN_ADJ );	※後で作る
	
	MOT_setTrgtSpeed( (FLOAT)MAP_SEARCH_SPEED );			// 目標速度設定を低くする
	PARAM_setSpeedType( PARAM_ST, PARAM_NORMAL );		// [直進]速度普通
	
	bl_NoChkErr = false;		// エラー量をチェックする
	
	DIST_setThresh_AllSenData();

}
#endif

// *************************************************************************
//   機能		： 距離センサをチューニングする
//   注意		： なし
//   メモ		： main関数で実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DIST_adj( void ){
	
//	stDIST_FLASH st_FlashData;					// データフラッシュのバックアップデータ
	
	/* 初期値として、まずは固定値を代入する */
	st_sen[DIST_SEN_R_FRONT].s_ref       = R_FRONT_REF;
	st_sen[DIST_SEN_L_FRONT].s_ref       = L_FRONT_REF;
//	st_sen[DIST_SEN_R_45].s_ref          = R_45_REF;
//	st_sen[DIST_SEN_L_45].s_ref          = L_45_REF;
	st_sen[DIST_SEN_R_SIDE].s_ref        = R_SIDE_REF;
	st_sen[DIST_SEN_L_SIDE].s_ref        = L_SIDE_REF;
	
	st_sen[DIST_SEN_R_FRONT].s_limit     = R_FRONT_WALL;
	st_sen[DIST_SEN_L_FRONT].s_limit     = L_FRONT_WALL;
//	st_sen[DIST_SEN_R_45].s_limit        = R_45_WALL;
//	st_sen[DIST_SEN_L_45].s_limit        = L_45_WALL;
	st_sen[DIST_SEN_R_SIDE].s_limit      = R_SIDE_WALL;
	st_sen[DIST_SEN_L_SIDE].s_limit      = L_SIDE_WALL;
	
	st_sen[DIST_SEN_R_FRONT].s_ctrl      = R_FRONT_WALL_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_ctrl      = L_FRONT_WALL_CTRL;
	
	st_sen[DIST_SEN_R_FRONT].s_noCtrl    = R_FRONT_WALL_NO_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_noCtrl    = L_FRONT_WALL_NO_CTRL;
	
	/* 前壁のみ */
	st_senF[DIST_SEN_R_FRONT].s_wallHit  = R_FRONT_WALL_HIT;
	st_senF[DIST_SEN_L_FRONT].s_wallHit  = L_FRONT_WALL_HIT;
	
	st_senF[DIST_SEN_R_FRONT].s_skewErr1 = R_FRONT_REF;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1 = L_FRONT_REF;
	
	st_senF[DIST_SEN_R_FRONT].s_skewErr2 = R_FRONT_REF;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2 = L_FRONT_REF;

	st_senF[DIST_SEN_R_FRONT].s_skewErr3 = R_FRONT_REF;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3 = L_FRONT_REF;

	
//#ifdef FUNC_DIST_AUTO_THRESH

	/* 起動時に前壁が近ければチューニングモーションを取る */
	/*
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > DIST_NEAR_WALL ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > DIST_NEAR_WALL )
	){
		
		DIST_setThresh_AllSenData_Move();		// チューニングを行う
		printf("チューニング \n\r" );
	}
	*/
	/* 起動時に前壁なければ、データフラッシュから以前に保存したセンサの値をロードする */
	//else{
		
		/* データフラッシュからの値をロード */
//		Storage_Load( (const void*)&st_FlashData, SEN_FLASH_SIZE, ADR_SEN );			// データロード
//		TIME_wait(10);
		
		/* 全センサ共通 */
//		st_sen[DIST_SEN_R_FRONT].s_ref       = st_FlashData.st_common[DIST_SEN_R_FRONT].s_ref;
//		st_sen[DIST_SEN_L_FRONT].s_ref       = st_FlashData.st_common[DIST_SEN_L_FRONT].s_ref;
//		st_sen[DIST_SEN_R_45].s_ref          = st_FlashData.st_common[DIST_SEN_R_45].s_ref;
//		st_sen[DIST_SEN_L_45].s_ref          = st_FlashData.st_common[DIST_SEN_L_45].s_ref;
//		st_sen[DIST_SEN_R_SIDE].s_ref        = st_FlashData.st_common[DIST_SEN_R_SIDE].s_ref;
//		st_sen[DIST_SEN_L_SIDE].s_ref        = st_FlashData.st_common[DIST_SEN_L_SIDE].s_ref;
		
//		st_sen[DIST_SEN_R_FRONT].s_limit     = st_FlashData.st_common[DIST_SEN_R_FRONT].s_limit;
//		st_sen[DIST_SEN_L_FRONT].s_limit     = st_FlashData.st_common[DIST_SEN_L_FRONT].s_limit;
////		st_sen[DIST_SEN_R_45].s_limit        = st_FlashData.st_common[DIST_SEN_R_45].s_limit;
////		st_sen[DIST_SEN_L_45].s_limit        = st_FlashData.st_common[DIST_SEN_L_45].s_limit;
//		st_sen[DIST_SEN_R_SIDE].s_limit      = st_FlashData.st_common[DIST_SEN_R_SIDE].s_limit;
//		st_sen[DIST_SEN_L_SIDE].s_limit      = st_FlashData.st_common[DIST_SEN_L_SIDE].s_limit;
		
//		st_sen[DIST_SEN_R_FRONT].s_ctrl      = st_FlashData.st_common[DIST_SEN_R_FRONT].s_ctrl;
//		st_sen[DIST_SEN_L_FRONT].s_ctrl      = st_FlashData.st_common[DIST_SEN_L_FRONT].s_ctrl;
		
//		st_sen[DIST_SEN_R_FRONT].s_noCtrl    = st_FlashData.st_common[DIST_SEN_R_FRONT].s_noCtrl;
//		st_sen[DIST_SEN_L_FRONT].s_noCtrl    = st_FlashData.st_common[DIST_SEN_L_FRONT].s_noCtrl;
		
		/* 前壁のみ */
//		st_senF[DIST_SEN_R_FRONT].s_wallHit  = st_FlashData.st_front[DIST_SEN_R_FRONT].s_wallHit;
//		st_senF[DIST_SEN_L_FRONT].s_wallHit  = st_FlashData.st_front[DIST_SEN_L_FRONT].s_wallHit;
		
//		st_senF[DIST_SEN_R_FRONT].s_skewErr1 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr1;
//		st_senF[DIST_SEN_L_FRONT].s_skewErr1 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr1;
		
//		st_senF[DIST_SEN_R_FRONT].s_skewErr2 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr2;
//		st_senF[DIST_SEN_L_FRONT].s_skewErr2 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr2;

//		st_senF[DIST_SEN_R_FRONT].s_skewErr3 = st_FlashData.st_front[DIST_SEN_R_FRONT].s_skewErr3;
//		st_senF[DIST_SEN_L_FRONT].s_skewErr3 = st_FlashData.st_front[DIST_SEN_L_FRONT].s_skewErr3;
//		printf("通常ロード \n\r" );
//	}

//#else
//	st_FlashData = st_FlashData;		// バカよけ
//#endif
//#endif
}

// *************************************************************************
//   機能		： 距離センサの値を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id )
{
	return st_sen[en_id].s_now;
}

// *************************************************************************
//   機能		： 距離センサの値を表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　	： hal_dist
//**************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DIST_Check( void ){
	
	LED_offAll();	// インジゲータ消灯

	while(1){

		printf(" 距離センサ [R_F]%5d [L_F]%5d [R_S]%5d [L_S]%5d \r", 
			(int)DIST_getNowVal(DIST_SEN_R_FRONT),
			(int)DIST_getNowVal(DIST_SEN_L_FRONT),
			(int)DIST_getNowVal(DIST_SEN_R_SIDE),
			(int)DIST_getNowVal(DIST_SEN_L_SIDE)
			);
		
		TIME_wait( 300 );
	}
}


// *************************************************************************
//   機能		： 前壁の有無を判断する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　	： hal_dist
//**************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_FRONT( void )
{
	BOOL bl_res 		= false;
	
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_limit ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_limit )
	){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 右壁の有無を判断する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　	： hal_dist
//**************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_R_SIDE( void )
{
	BOOL bl_res 		= false;
	
	if( st_sen[DIST_SEN_R_SIDE].s_now > st_sen[DIST_SEN_R_SIDE].s_limit ){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 左壁の有無を判断する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　	： hal_dist
//**************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_L_SIDE( void )
{
	BOOL bl_res 		= false;
	
	if( st_sen[DIST_SEN_L_SIDE].s_now > st_sen[DIST_SEN_L_SIDE].s_limit ){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 前壁に接近しているか
//   注意		： なし
//   メモ		： フェールセーフ用？
//   引数		： なし
//   返り値		： なし
//   その他　　	： hal_dist
//**************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC BOOL DIST_isNearWall_FRONT( void )
{
	BOOL bl_res 		= false;
	
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_wallHit ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_wallHit )
	){
		bl_res = true;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 基準値と現在値の偏差を取得する
//   注意		： なし
//   メモ		： 壁の切れ目補正対応実施
//   引数		： なし
//   返り値		： 偏差
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void DIST_getErr( LONG* p_err )
{
	VSHORT	s_threshold_R = 0;		// 右センサの閾値
	VSHORT	s_threshold_L = 0;		// 左センサの閾値
	SHORT	s_temp;

#if 0
	/* エラーチェックしない */
	if( bl_NoChkErr == TRUE ){
		*p_err = 0;		// クリア
		return SUCCESS;
	}
#endif	
	/* ---------- */
	/*  右壁制御  */
	/* ---------- */
	/* 壁の切れ目対策 */
	// 急激にセンサの値が変化した場合は、壁の有無の基準値を閾値に変更する
	s_temp = st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_ref + DIST_REF_UP;		// 基準値＋αを壁の存在する閾値にする
		
	}
	else{
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_limit;		// 通常通り
		
	}

	/* ---------- */
	/*  左壁制御  */
	/* ---------- */
	/* 壁の切れ目対策 */
	// 急激にセンサの値が変化した場合は、壁の有無の基準値を閾値に変更する
	s_temp = st_sen[DIST_SEN_L_SIDE].s_now - st_sen[DIST_SEN_L_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_ref + DIST_REF_UP;		// 基準値＋αを壁の存在する閾値にする
		
	}
	else{
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_limit;		// 通常通り
	}
	
	/* ------------ */
	/*  制御値算出  */
	/* ------------ */
	*p_err = 0;		// クリア
	
	/* 前壁がものすごく近い時 */
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_noCtrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_noCtrl )
	){
//		printf("[Val]%6d 前壁がものすごい近い 	\n\r", *p_err);
		*p_err = 0;
	}
	/* 前壁 */
	else if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_ctrl ) &&
			 ( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_ctrl )
	){
		*p_err = ( st_sen[DIST_SEN_L_FRONT].s_now - st_sen[DIST_SEN_L_FRONT].s_ref ) -
				 ( st_sen[DIST_SEN_R_FRONT].s_now - st_sen[DIST_SEN_R_FRONT].s_ref );
		
		
		//printf("[Val]%6d 前壁制御 	\n\r", *p_err);
	}
	/* 右壁と左壁あり */
	else if( ( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ) && ( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now )
	){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) + 
				 ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now );
		//printf("[Val]%6d 両壁制御 	\n\r", *p_err);
	}
	/* 右壁あり */
	else if( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) * 2;
		//printf("[Val]%6d 右壁制御 	\n\r", *p_err);
	}
	
	/* 左壁あり */
	else if( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now ){
		
		*p_err = ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now ) * 2;
		//printf("[Val]%6d 左壁制御 	\n\r", *p_err);
	}
	
}


// *************************************************************************
//   機能		： 距離センサ用（前壁）ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.5.5			TKR			新規
// *************************************************************************/
PUBLIC void DIST_Pol_Front( void )
{
	/* 無発光時の値取得 */
	S12AD.ADANS0.WORD 		= 0x06;			// AN1/2 変換対象設定
	S12AD.ADCSR.BIT.ADST	= 1;			// AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_FRONT].s_offset = (SHORT)S12AD.ADDR2;
	st_sen[DIST_SEN_L_FRONT].s_offset = (SHORT)S12AD.ADDR1;
	
	/* 前壁LED点灯 */
	LED_DIST_RF = ON;
	LED_DIST_LF = ON;
	
	/* 発光安定待ち */
	TIME_waitFree( SEN_WAIT_CNT );

	/* 発光時の値と無発光時の値で差分を取得 */
	S12AD.ADANS0.WORD 		= 0x06;			// AN1/2 変換対象設定
	S12AD.ADCSR.BIT.ADST	= 1;			// AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_FRONT].s_old = st_sen[DIST_SEN_R_FRONT].s_now;		// バッファリング
	st_sen[DIST_SEN_L_FRONT].s_old = st_sen[DIST_SEN_L_FRONT].s_now;		// バッファリング
	st_sen[DIST_SEN_R_FRONT].s_now = (SHORT)S12AD.ADDR2 - st_sen[DIST_SEN_R_FRONT].s_offset;		// 現在値書き換え
	st_sen[DIST_SEN_L_FRONT].s_now = (SHORT)S12AD.ADDR1 - st_sen[DIST_SEN_L_FRONT].s_offset;		// 現在値書き換え
	
	/* 前壁LED消灯 */
	LED_DIST_RF = OFF;
	LED_DIST_LF = OFF;
}

// *************************************************************************
//   機能		： 距離センサ用（横壁）ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC void DIST_Pol_Side( void )
{
	/* 無発光時の値取得 */
	S12AD.ADANS0.WORD 		= 0x09;			// AN0/3 変換対象設定
	S12AD.ADCSR.BIT.ADST	= 1;			// AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_SIDE].s_offset = (SHORT)S12AD.ADDR3;
	st_sen[DIST_SEN_L_SIDE].s_offset = (SHORT)S12AD.ADDR0;
	
	/* 横壁LED点灯 */
	LED_DIST_RS = ON;
	LED_DIST_LS = ON;
	
	/* 発光安定待ち */
	TIME_waitFree( SEN_WAIT_CNT );

	/* 発光時の値と無発光時の値で差分を取得 */
	/* 無発光時の値取得 */
	S12AD.ADANS0.WORD 		= 0x09;			// AN0/3 変換対象設定
	S12AD.ADCSR.BIT.ADST	= 1;			// AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_SIDE].s_old = st_sen[DIST_SEN_R_SIDE].s_now;		// バッファリング
	st_sen[DIST_SEN_L_SIDE].s_old = st_sen[DIST_SEN_L_SIDE].s_now;		// バッファリング
	st_sen[DIST_SEN_R_SIDE].s_now = (SHORT)S12AD.ADDR3 - st_sen[DIST_SEN_R_SIDE].s_offset;		// 現在値書き換え
	st_sen[DIST_SEN_L_SIDE].s_now = (SHORT)S12AD.ADDR0 - st_sen[DIST_SEN_L_SIDE].s_offset;		// 現在値書き換え
	
	/* 横壁LED消灯 */
	LED_DIST_RS = OFF;
	LED_DIST_LS = OFF;
}

