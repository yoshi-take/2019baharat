// *************************************************************************
//   ロボット名	： ティウンティウン2014
//   概要		： 迷路探索、二次走行
//   注意		： なし
//   メモ		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
#ifdef __cplusplus
    extern "C"{
#endif


//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <stdio.h>							// 標準入出力
#include <string.h>							// 文字列操作（memset用）
#include <iodefine.h>						// I/O

#include <hal_dcm.h>						// DCM
#include <hal_dcmCtrl.h>					// DCM_CTRL
#include <hal_dist.h>						// DIST
#include <hal_led.h>						// LED
#include <hal_spk.h>						// SPK

#include <motion.h>							// モーション
#include <parameter.h>						// [機種固有] パラメータ（メカ仕様/ゲイン/調整値）
#include <map.h>							// マップ＋二次走行
#include <hal_flash.h>						// FLASH


//**************************************************
// 定義（define）
//**************************************************
#define MAP_SMAP_MAX_VAL		( MAP_X_SIZE * MAP_Y_SIZE ) 			// 等高線mapの最大値
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				// 等高線mapの優先度最大値


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
/* シミュレーション */
typedef struct{
	enMAP_CMD	en_cmd;			// コマンド
	FLOAT		f_x0_x1;		// [0]/[1]のX座標加算値
	FLOAT		f_y0_y1;		// [0]/[1]のy座標加算値
	FLOAT		f_x2_x3;		// [2]/[3]のX座標加算値
	FLOAT		f_y2_y3;		// [2]/[3]のy座標加算値
	FLOAT		f_x4_x5;		// [4]/[5]のX座標加算値
	FLOAT		f_y4_y5;		// [4]/[5]のy座標加算値
	FLOAT		f_x6_x7;		// [6]/[7]のX座標加算値
	FLOAT		f_y6_y7;		// [6]/[7]のy座標加算値
	SHORT		s_dir;			// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
}stMAP_SIM;


//**************************************************
// グローバル変数
//**************************************************
/* ---------- */
/*  迷路探索  */
/* ---------- */
PRIVATE volatile enMAP_HEAD_DIR	en_Head;										// マウスの進行方向 0:N 1:E 2:S 3:W
PUBLIC UCHAR			my;												// マウスのＸ座標
PUBLIC UCHAR			mx;												// マウスのＹ座標
PUBLIC USHORT			us_cmap[MAP_Y_SIZE][MAP_X_SIZE];				// 等高線 データ
PUBLIC UCHAR			g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路情報
PRIVATE FLOAT			f_MoveBackDist;									// 壁当て動作で後退した距離[区画]
PRIVATE UCHAR			uc_TurnCnt = 0;									// 超信地連続回数
PRIVATE UCHAR			uc_SlaCnt = 0;									// スラローム連続回数
PRIVATE UCHAR			uc_back[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路データ
PRIVATE USHORT			us_DashCmdIndex = 0;							// 既知コマンドインデックス

/* -------------- */
/*  コマンド走行  */
/* -------------- */
/* コマンドリスト */
PUBLIC	UCHAR		dcom[LIST_NUM];					// 超地信旋回用
PUBLIC	UCHAR		scom[LIST_NUM];					// スラローム用
PUBLIC	UCHAR		tcom[LIST_NUM];					// 斜め走行用
PUBLIC	UCHAR		mcom[LIST_NUM];					// 既知コマンド走行用
PRIVATE	USHORT		us_totalCmd;					// トータルコマンド量
PRIVATE	FLOAT		f_PosX;							// X座標
PRIVATE	FLOAT		f_PosY;							// Y座標
PRIVATE	SHORT		s_PosDir;						// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）

/* コマンドに応じた座標更新データ */
PRIVATE CONST stMAP_SIM st_PosData[] = {
	
	//	コマンド	[0]/[1]のX	[0]/[1]のY	[2]/[3]のX	[2]/[3]のY	[4]/[5]のX	[4]/[5]のY	[6]/[7]のX	[6]/[7]のY	方向
	{ R90,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [0]
	{ L90,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [1]
	{ R90S,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [2]
	{ L90S,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [3]
	{ RS45N,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		+1 },		// [4]
	{ LS45N,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-1 },		// [5]
	{ RS135N,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+3 },		// [6]
	{ LS135N,		-0.75,		0.25,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-3 },		// [7]
	{ RN45S,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+1 },		// [8]
	{ LN45S,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		-1 },		// [9]
	{ RN135S,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		0.25,		0.75,		+3 },		// [10]
	{ LN135S,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-3 },		// [11]
	{ RN90N,		0.5,		0,			0,			-0.5,		-0.5,		0,			0,			0.5,		+2 },		// [12]
	{ LN90N,		0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			-2 },		// [13]
	{ GO1,			0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			0  },		// [14]
	{ NGO1,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		0  },		// [15]
	{ MAP_CMD_MAX,	0,			0,			0,			0,			0,			0,			0,			0,			0  },
};

/* ログ */
PRIVATE FLOAT f_LogPosX[30];
PRIVATE FLOAT f_LogPosY[30];
PRIVATE USHORT us_LogIndex = 0;
PRIVATE USHORT us_LogWallCut[30];
PRIVATE USHORT us_LogIndexWallCut = 0;


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
extern PUBLIC void TIME_wait( ULONG ul_time );
//extern PUBLIC BOOL SYS_isOutOfCtrl( void );
PRIVATE void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir );



// *************************************************************************
//   機能		： 迷路データをクリアする
//   注意		： RAMの方をクリア
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_clearMap( void )
{
	USHORT	x, y;
	UCHAR	uc_data;

	/* すべてのマップデータを未探索状態にする */
	for ( y = 0; y < MAP_Y_SIZE; y++){

		for( x = 0; x < MAP_X_SIZE; x++){

			uc_data = 0x00;
			if ( ( x == 0) && ( y == 0 ) ) uc_data = 0xfe;
			else if ( ( x == 1 ) && ( y == 0 ) ) uc_data = 0xcc;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == 0 ) ) uc_data = 0x66;
			else if ( ( x == 0 ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x99;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x33;
			else if ( x == 0 ) uc_data = 0x88;
			else if ( x == (MAP_X_SIZE-1) ) uc_data = 0x22;
			else if ( y == 0 ) uc_data = 0x44;
			else if ( y == (MAP_Y_SIZE-1) ) uc_data = 0x11;
			g_sysMap[y][x] = uc_data;

		}
	}
}


// *************************************************************************
//   機能		： 迷路モジュールを初期化する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_init( void )
{
	/* 座標、向き、迷路情報を初期化 */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();
	
	/* 走行用のパラメータ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
	/* バックアップ迷路の復帰 */
//	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// バックアップデータを復帰する
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// データがあったら
		
		printf("\n\r　　　　　　　　　　　　＼　│　／");
		printf("\n\r　　　　　　　　　　　　　／￣＼　　／￣￣￣￣￣￣￣￣￣");
		printf("\n\r　　　　　　　　　　　─（ﾟ ∀ ﾟ）＜　迷路データをぉぉぉ");
		printf("\n\r　　　　　　　　　　　　　＼＿／　　＼＿＿＿＿＿＿＿＿＿");
		printf("\n\r　　　　　　　　　　　　／　│　＼");
		printf("\n\r　　　　　　　　　　　　　　　 ∩ ∧　∧∩ ／￣￣￣￣￣￣￣￣￣￣");
		printf("\n\r　￣￣￣￣￣￣￣￣＼ ∩∧ ∧∩ ＼（ ﾟ∀ﾟ）＜　復帰だ復帰だ復帰だ！");
		printf("\n\r　　復帰だ～～～！ ＞（ﾟ∀ﾟ）/ 　｜　　　/　＼＿＿＿＿＿＿＿＿＿＿");
		printf("\n\r　＿＿＿＿＿＿＿＿／ ｜　　〈　　｜　　 ｜");
		printf("\n\r　　　　　　　　　　 /　／＼_」　/　／＼」");
		printf("\n\r　　　　　　　　　　 ￣　　　　 / ／");
		printf("\n\r　　　　　　　　　　　　　　　  ￣");

		//MAP_LoadMapData();		// 迷路データを復帰
	}
}


// *************************************************************************
//   機能		： バックアップ迷路情報を実データに反映する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_LoadMapData( void )
{
	SHORT	i;
	USHORT	*map_add;
	map_add	= (USHORT*)&g_sysMap;

	// マップデータをRAMにコピー
	for(i=0; i<128; i++){
		FLASH_Read( (USHORT*)(ADR_MAP+i*2), map_add );
		map_add++;
	}
}


// *************************************************************************
//   機能		： バックアップ迷路情報をバックアップする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.09.8			TKR			新規
// *************************************************************************/
PUBLIC void MAP_SaveMapData( void )
{

	SHORT	i;
	USHORT	*map_add;
	map_add	= (USHORT*)g_sysMap;

	// データフラッシュのイレース
	for( i=0; i<8; i++ ){
		FLASH_Erase((ULONG)(ADR_MAP+i*32));
	}

	// MAPデータをデータフラッシュに書き込み
	for( i=0; i<128; i++){
		FLASH_WriteEE((ULONG)(ADR_MAP+i*2),map_add);
		map_add++;
	}

}


// *************************************************************************
//   機能		： バックアップ迷路情報をクリアする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_ClearMapData( void )
{
	SHORT	i;
	
	/* 座標、向き、迷路情報を初期化 */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();		// RAMの方をクリア
	
	/* 走行用のパラメータ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
	/* データフラッシュイレース */
	for( i=0; i<8; i++){
		FLASH_Erase((ULONG)(ADR_MAP+i*32));
	}
}


// *************************************************************************
//   機能		： バックアップ迷路データを表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showLog_BackUp( void )
{
	volatile SHORT	x,y;
	CHAR	c_data;
	
	memset( uc_back, 0, sizeof(uc_back) );				// 時間
	
	/* ------------------ */
	/*  バックアップ迷路  */
	/* ------------------ */
	printf("\n\r  /* ------------------ */   ");
	printf("\n\r  /*  バックアップ迷路  */   ");
	printf("\n\r  /* ------------------ */   ");
//	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );			// データロード(dummy) これがないと次の１発目のsaveがうまくいかない
	
	/* エラーチェック */
	if( ( uc_back[0][0] & 0xf0 ) != 0xf0  ){
		
		printf("\n\r  表示する情報がありません   ");
		printf("\n\r");
		
		return;		// 実行結果
	}
	
	printf("\n\r     ");

	
	for( x=0; x<MAP_X_SIZE; x++){
		printf("._");
	}
	printf(".\n\r");
	
	for( y=MAP_Y_SIZE-1; y>-1; y-- ){
		
		printf("   %2d",y);
		for( x=0; x<MAP_X_SIZE; x++){
			c_data = (UCHAR)uc_back[y][x];
			if ( ( c_data & 0x08 ) == 0 ){
				printf(".");
			}
			else{
				printf("|");
			}
			if ( ( c_data & 0x04 ) == 0 ){
				printf(" ");
			}
			else{
				printf("_");
			}
		}
		printf("|   ");
		
		for( x=0; x<MAP_X_SIZE; x++ ){
			c_data = uc_back[y][x];
			c_data = (UCHAR)( (c_data >> 4) & 0x0f );
			printf("%x", c_data);
		}
		
		printf("\n\r");
	}
	
	printf("     ");
	for( x=0; x<MAP_X_SIZE; x++){
		printf("%2d",x%10);
	}
	printf("\n\r");
}


// *************************************************************************
//   機能		： 座標位置を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： x座標、y座標、向いている方向
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_setPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	mx			= uc_x;
	my			= uc_y;
	en_Head		= en_dir;
	
	MAP_setCmdPos( uc_x, uc_y, en_dir );
}


// *************************************************************************
//   機能		： 迷路データを表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showLog( void )
{
	SHORT	x, y;
	CHAR	c_data;

	/* ---------- */
	/*  通常迷路  */
	/* ---------- */
	printf("\n\r  /* ---------- */   ");
	printf("\n\r  /*  通常迷路  */   ");
	printf("\n\r  /* ---------- */   ");

	printf("\n\r     ");
	for (x = 0; x < MAP_X_SIZE; x++) {
		printf("._");
	}
	printf(".\n\r");

	for (y = MAP_Y_SIZE - 1; y > -1; y--) {

		printf("   %2d", y);
		for (x = 0; x < MAP_X_SIZE; x++) {
			c_data = (UCHAR)g_sysMap[y][x];
			if ((c_data & 0x08) == 0) {
				printf(".");
}
			else {
				printf("|");
			}
			if ((c_data & 0x04) == 0) {
				printf(" ");
			}
			else {
				printf("_");
			}
		}
		printf("|   ");

		for (x = 0; x < MAP_X_SIZE; x++) {
			c_data = g_sysMap[y][x];
			c_data = (UCHAR)((c_data >> 4) & 0x0f);
			printf("%x", c_data);
		}

		printf("\n\r");
	}

	printf("     ");
	for (x = 0; x < MAP_X_SIZE; x++) {
		printf("%2d", x % 10);
	}
	printf("\n\r");
}


// *************************************************************************
//   機能		： センサ情報から壁情報を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 壁情報
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE UCHAR MAP_getWallData( void )
{
	UCHAR	uc_wall;
	LED_offAll();	// debug

	/* センサ情報から壁情報作成 */
	uc_wall = 0;
	if( TRUE == DIST_isWall_FRONT() ){
		uc_wall = uc_wall | 0x11;
		LED_on(LED2);				// debug
	}
	if( TRUE == DIST_isWall_L_SIDE() ){
		uc_wall = uc_wall | 0x88;
		LED_on(LED0);			// debug
	}
	if( TRUE == DIST_isWall_R_SIDE() ){
		uc_wall = uc_wall | 0x22;
		LED_on(LED4);			// debug
	}

	/* マウスの進行方向にあわせてセンサデータを移動し壁データとする */
	if		( en_Head == EAST ){
		uc_wall = uc_wall >> 3;
	}
	else if ( en_Head == SOUTH ){
		uc_wall = uc_wall >> 2;
	}
	else if ( en_Head == WEST ){
		uc_wall = uc_wall >> 1;
	}

	/*	探索済みフラグを立てる */
	return ( uc_wall | 0xf0 );
}


// *************************************************************************
//   機能		： 壁情報を作成する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_makeMapData( void )
{
	UCHAR uc_wall;

	/*	走行時の壁情報を迷路情報に書込 */
	if ( ( mx == 0 ) && ( my == 0 ) ){
		uc_wall = 0xfe;
	}
	else{
		uc_wall = MAP_getWallData();
	}
	g_sysMap[my][mx] = uc_wall;

	/*	隣の区間のＭＡＰデータも更新する */
	if ( mx != (MAP_X_SIZE-1) ){
		g_sysMap[my][mx+1] = ( g_sysMap[my][mx+1] & 0x77 ) | 0x80 | ( ( uc_wall << 2 ) & 0x08 );
	}
	if ( mx !=  0 ){
		g_sysMap[my][mx-1] = ( g_sysMap[my][mx-1] & 0xdd ) | 0x20 | ( ( uc_wall >> 2 ) & 0x02 );
	}
	if ( my != (MAP_Y_SIZE-1) ){
		g_sysMap[my+1][mx] = ( g_sysMap[my+1][mx] & 0xbb ) | 0x40 | ( ( uc_wall << 2 ) & 0x04 );
	}
	if ( my !=  0 ){
		g_sysMap[my-1][mx] = ( g_sysMap[my-1][mx] & 0xee ) | 0x10 | ( ( uc_wall >> 2 ) & 0x01 );
	}
}

// *************************************************************************
//   機能		： 等高線マップを作成する（探索）
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeContourMap_search( void )
{
	USHORT		x, y;			// ループ変数
	UCHAR		uc_dase;		// 基準値
	UCHAR		uc_new;			// 新値
	UCHAR		uc_level;		// 等高線
	UCHAR		uc_wallData;	// 壁情報

	/* 等高線マップを作成 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					if ( ( ( uc_wallData & 0x01 ) == 0x00 ) && ( y != (MAP_Y_SIZE-1) ) ){
						if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y+1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x02 ) == 0x00 ) && ( x != (MAP_X_SIZE-1) ) ){
						if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x+1] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x04 ) == 0x00 ) && ( y != 0 ) ){
						if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y-1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x08 ) == 0x00 ) && ( x != 0 ) ){
						if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x-1] = uc_new;
							uc_level++;
						}
					}
				}
			}
		}
		uc_dase = uc_dase + 1;
	}
	while( uc_level != 0 );
	
#if 0
	/* debug */
	for( x=0; x<4; x++ ){
		
		for( y=0; y<4; y++ ){
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif
}


// *************************************************************************
//   機能		： 等高線マップを作成する（最短）
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeContourMap_bestWay( void )
{
	USHORT		x, y;			// ループ変数
	UCHAR		uc_dase;		// 基準値
	UCHAR		uc_new;			// 新値
	UCHAR		uc_level;		// 等高線
	UCHAR		uc_wallData;	// 壁情報

	/* 等高線マップを作成 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					if ( ( ( uc_wallData & 0x11 ) == 0x10 ) && ( y != (MAP_Y_SIZE-1) ) ){
						if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y+1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x22 ) == 0x20 ) && ( x != (MAP_X_SIZE-1) ) ){
						if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x+1] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x44 ) == 0x40 ) && ( y != 0 ) ){
						if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y-1][x] = uc_new;
							uc_level++;
						}
					}
					if ( ( ( uc_wallData & 0x88 ) == 0x80 ) && ( x != 0 ) ){
						if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
							us_cmap[y][x-1] = uc_new;
							uc_level++;
						}
					}
				}
			}
		}
		uc_dase = uc_dase + 1;
	}
	while( uc_level != 0 );
	
#if 0
	/* debug */
	for( x=0; x<4; x++ ){
		
		for( y=0; y<4; y++ ){
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif
}


// *************************************************************************
//   機能		： 等高線マップを作成する
//   注意		： なし
//   メモ		： 等高線マップを初期化する方法は4通りある。
//   引数		： ゴールX座標、ゴールY座標、計算方法（探索or最短）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeContourMap( UCHAR uc_goalX, UCHAR uc_goalY, enMAP_ACT_MODE en_type )
{
	USHORT		i;			// ループ変数
//	USHORT		j;			// ループ変数
//	USHORT*		p_adr;		// ループ変数

	/* ------------ */
	/*  初期化処理  */
	/* ------------ */
	/* 等高線マップを初期化する　～やり方1～ */
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		us_cmap[ i / MAP_Y_SIZE][ i & (MAP_X_SIZE-1) ] = MAP_SMAP_MAX_VAL - 1;
	}
	
#if 0
	/* 等高線マップを初期化する　～やり方2～ */
	LED4(0x01);
	for ( i = 0; i < 32; i++ ){
		for ( j = 0; j < 32; j++ ){
			us_cmap[i][j] = MAP_SMAP_MAX_VAL - 1;
		}
	}
	LED4(0x00);

	/* 等高線マップを初期化する　～やり方3～ */
	LED4(0x01);
	p_adr = &us_cmap[0][0];
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		*p_adr = MAP_SMAP_MAX_VAL - 1;
		p_adr++;
	}
	LED4(0x00);
	
	/* 等高線マップを初期化する　～やり方4～ */
	LED4(0x01);
	p_adr = &us_cmap[0][0];
	p_adrEnd = &us_cmap[31][31];
	while(1){
		
		if( p_adr == p_adrEnd ) break;
		*p_adr = MAP_SMAP_MAX_VAL - 1;
		p_adr++;
	}
	LED4(0x00);
#endif
	
	/* 目標地点の等高線を0に設定 */
	us_cmap[uc_goalY][uc_goalX] = 0;
	
	
	/* ------------ */
	/*  等高線作成  */
	/* ------------ */
	/* 探索走行 */
	if( SEARCH == en_type ){
		MAP_makeContourMap_search();
	}
	/* 最短走行 */
	else{
		MAP_makeContourMap_bestWay();
	}
}


// *************************************************************************
//   機能		： マウスの進行方向を決定する
//   注意		： なし
//   メモ		： なし
//   引数		： [in] 計算方法、[out] 進行方向
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_calcMouseDir( enMAP_SEARCH_TYPE en_calcType, enMAP_HEAD_DIR* p_head )
{
	UCHAR		uc_wall;				// 壁情報
	USHORT		us_base;				// 等高線優先度決定値
	USHORT		us_new;
	enMAP_HEAD_DIR	en_tmpHead;

	/* ========== */
	/*  方向計算  */
	/* ========== */
	/* 等高線MAP法 */
	if( CONTOUR_SYSTEM == en_calcType ){
		// 周辺の4区画で一番目的地に近い移動方向を算出する。
		// ただし、移動できる一番近い区間が複数ある場合には、次の順で選択する。
		// ①未探索区間,直進 ②未探索区間,旋回 ③既探索区間,直進 ④既探索区間,旋回
		uc_wall = g_sysMap[my][mx];
		us_base = MAP_SMAP_MAX_PRI_VAL;					// x[区画]×y[区画]×4[方向]

		/* 4方向を比較 */
		//	北方向の区画の確認
		if ( ( uc_wall & 1 ) == 0 ){
			us_new = us_cmap[my+1][mx] * 4 + 4;
			if ( ( g_sysMap[my+1][mx] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == NORTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = NORTH;
			}
		}
		//	東方向の区画の確認
		if ( ( uc_wall & 2 ) == 0 ){
			us_new = us_cmap[my][mx+1] * 4 + 4;
			if ( ( g_sysMap[my][mx+1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == EAST) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = EAST;
			}
		}
		//	南方向の区画の確認
		if ( ( uc_wall & 4 ) == 0 ){
			us_new = us_cmap[my-1][mx] * 4 + 4;
			if ( ( g_sysMap[my-1][mx] & 0xf0 ) != 0xf0) us_new = us_new - 2;
			if ( en_Head == SOUTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = SOUTH;
			}
		}
		//	西方向の区画の確認
		if ( ( uc_wall & 8 ) == 0 ){
			us_new = us_cmap[my][mx-1] * 4 + 4;
			if ( ( g_sysMap[my][mx-1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == WEST ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = WEST;
			}
		}
		
		*p_head = (enMAP_HEAD_DIR)( (en_tmpHead - en_Head) & 3 );		// 移動方向
	}
	// 制御方法指定なし
	else{
		*p_head = (enMAP_HEAD_DIR)0;
	}
}


// *************************************************************************
//   機能		： マウスの座標位置を更新する
//   注意		： なし
//   メモ		： なし
//   引数		： 進行方向
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_refMousePos( enMAP_HEAD_DIR en_head )
{
	switch( en_head ){

		case NORTH:
			my = my + 1;
			break;

		case EAST:
			mx = mx + 1;
			break;

		case SOUTH:
			my = my - 1;
			break;

		case WEST:
			mx = mx - 1;
			break;

		default:
			break;
	}
}


// *************************************************************************
//   機能		： 次の区画に移動する
//   注意		： なし
//   メモ		： なし
//   引数		： 相対進行方向（マウス進行方向を北としている）、前進状態（FALSE: １区間前進状態、TURE:半区間前進状態）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_moveNextBlock( enMAP_HEAD_DIR en_head, BOOL* p_type )
{
	*p_type = TRUE;
	f_MoveBackDist = 0;

	/* 動作 */
	switch( en_head ){

		/* そのまま前進 */
		case NORTH:
			*p_type = FALSE;
			MOT_goBlock_Const( 1 );				// 1区画前進
			break;

		/* 右に旋回する */
		case EAST:
			if( uc_TurnCnt < MAP_TURN_NUM_MAX ){
				MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);					// 右90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt++;
			
			}else{
				MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);					// 右90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt = 0;
				f_MoveBackDist = 0;
				
				/* 壁あて姿勢制御 （左に壁があったらバック＋移動距離を加算する）*/
				if( ( (en_Head == NORTH)&&( (g_sysMap[my][mx] & WALL_WEST ) != 0 ) ) ||
				    ( (en_Head == EAST )&&( (g_sysMap[my][mx] & WALL_NORTH) != 0  ) ) ||
				    ( (en_Head == WEST )&&( (g_sysMap[my][mx] & WALL_SOUTH) != 0 ) ) ||
			      	( (en_Head == SOUTH)&&( (g_sysMap[my][mx] & WALL_EAST ) != 0 ) ) ){
						
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
					TIME_wait( 100 );						// 時間待ち
				}
				*p_type = true;
			}		
			break;

		/* 左に旋回する */
		case WEST:
			if( uc_TurnCnt < MAP_TURN_NUM_MAX ){
				MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);									// 右90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt++;
				
			}else{
				MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);									// 右90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_TurnCnt = 0;
				f_MoveBackDist = 0;
				
				/* 壁あて姿勢制御 （右に壁があったらバック＋移動距離を加算する）*/
				if( ( (en_Head == NORTH)&&( (g_sysMap[my][mx] & WALL_EAST ) != 0 ) ) ||
				    ( (en_Head == EAST )&&( (g_sysMap[my][mx] & WALL_SOUTH) != 0  ) ) ||
				    ( (en_Head == WEST )&&( (g_sysMap[my][mx] & WALL_NORTH) != 0 ) ) ||
			      	( (en_Head == SOUTH)&&( (g_sysMap[my][mx] & WALL_WEST ) != 0 ) ) ){

					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
					TIME_wait( 100 );						// 時間待ち
				}
				*p_type = true;
			}
			break;

		/* 反転して戻る */
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R180);									// 右180度旋回
			TIME_wait( MAP_TURN_WAIT );
			uc_TurnCnt 			= 0;
			f_MoveBackDist 		= 0;

			/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & WALL_NORTH ) != 0 ) )  ||		// 北を向いていて北に壁がある
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & WALL_EAST  ) != 0 ) )  ||		// 東を向いていて東に壁がある
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & WALL_SOUTH ) != 0 ) )  ||		// 南を向いていて南に壁がある
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & WALL_WEST  ) != 0 ) ) 			// 西を向いていて西に壁がある
			){
				MOT_goHitBackWall();					// バックする
				f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
				TIME_wait( MAP_TURN_WAIT );				// 時間待ち
			}
			break;

		default:
			break;
	}
	
#ifndef POWER_RELESASE
	/* 進行方向更新 */
//	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	en_Head = (enMAP_HEAD_DIR)( ((UCHAR)en_Head + (UCHAR)en_head) & (MAP_HEAD_DIR_MAX-1) );
#else
	/* 前進中にパワーリリース機能が働いてレジュームしなければならない */
	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		MOT_goBack_Const( MOT_BACK_POLE );					// １つ前の柱まで後退
		MAP_makeMapData();									// 壁データから迷路データを作成			← ここでデータ作成をミスっている
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
		MAP_moveNextBlock(en_head, p_type);					// もう１度呼び出し（次の区画へ移動）
	}
	else{
		/* 進行方向更新 */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	}
#endif
}


// *************************************************************************
//   機能		： スラロームにて次の区画に移動する
//   注意		： なし
//   メモ		： なし
//   引数		： 相対進行方向（マウス進行方向を北としている）、
//				   前進状態（FALSE: １区間前進状態、TURE:半区間前進状態）
//				   レジューム指定（FALSE: レジューム動作ではない、TURE:レジューム動作）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_Sura( enMAP_HEAD_DIR en_head, BOOL* p_type, BOOL bl_resume )
{
	*p_type = FALSE;
	
	/* 動作 */
	switch( en_head ){

		/* そのまま前進 */
		case NORTH:
			
			/* レジューム動作ではない */
			if( bl_resume == FALSE ){
		
				MOT_goBlock_Const( 1 );					// 1区画前進
				uc_SlaCnt = 0;							// スラロームしていない
			}
			/* レジューム動作 */
			else{
				MOT_goBlock_FinSpeed( 1.0f, MAP_SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
				uc_SlaCnt = 0;										// スラロームしていない
			}
			break;

		/* 右にスラロームする */
		case EAST:
			if( uc_SlaCnt < MAP_SLA_NUM_MAX ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
				uc_SlaCnt++;
			}
			else{
				MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_R90);						// 右90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_SlaCnt = 0;
				/* 壁当て姿勢制御（左に壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 北を向いていて西に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 東を向いていて北に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 南を向いていて東に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// 西を向いていて南に壁がある
				){
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST_SURA;	// バックした分の移動距離[区画]を加算
					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
				}
				*p_type = TRUE;							// 次は半区間（＋バック）分進める
			}
			break;

		/* 左にスラロームする */
		case WEST:
			if( uc_SlaCnt < MAP_SLA_NUM_MAX ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
				uc_SlaCnt++;
			}
			else{
				
				MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
				TIME_wait( MAP_TURN_WAIT );
				MOT_turn(MOT_L90);					// 左90度旋回
				TIME_wait( MAP_TURN_WAIT );
				uc_SlaCnt = 0;
				/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 北を向いていて東に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 東を向いていて南に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 南を向いていて西に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// 西を向いていて北に壁がある
				){
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST_SURA;	// バックした分の移動距離[区画]を加算
					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
				}
				*p_type = TRUE;							// 次は半区間（＋バック）分進める
			}
			break;

		/* 反転して戻る */
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
			TIME_wait( MAP_SLA_WAIT );
			MOT_turn(MOT_R180);									// 右180度旋回
			TIME_wait( MAP_SLA_WAIT );
			uc_SlaCnt = 0;

			/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 北を向いていて北に壁がある
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 東を向いていて東に壁がある
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 南を向いていて南に壁がある
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// 西を向いていて西に壁がある
			){
				MOT_goHitBackWall();					// バックする
				f_MoveBackDist = MOVE_BACK_DIST_SURA;	// バックした分の移動距離[区画]を加算
				TIME_wait( MAP_SLA_WAIT );				// 時間待ち
			}
			*p_type = TRUE;								// 次は半区間＋バック分進める
			break;
			
		default:
			break;
	}
	
#ifndef POWER_RELESASE
	/* 進行方向更新 */
	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );

#else
	/* 前進中にパワーリリース機能が働いてレジュームしなければならない */
	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		TIME_wait(1000);
		MOT_goBack_Const( MOT_BACK_POLE );					// １つ前の柱まで後退
		MAP_makeMapData();									// 壁データから迷路データを作成			← ここでデータ作成をミスっている
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
		MAP_moveNextBlock_Sura(en_head, p_type, TRUE );		// もう１度呼び出し（次の区画へ移動）
	}
	else{
		/* 進行方向更新 */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	}
#endif
}


// *************************************************************************
//   機能		： ゴール時の動作
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_actGoal( void )
{
	MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
	TIME_wait(500);						// 停止安定待ち時間
	MOT_turn(MOT_R180);					// 右180度旋回
	
	TIME_wait(500);						// 停止安定待ち時間
	LED_onAll();						

	//MAP_SaveMapData();					// 迷路情報のバックアップ
	
	en_Head = (enMAP_HEAD_DIR)( (en_Head + 2) & (MAP_HEAD_DIR_MAX-1) );			//	進行方向更新
}


// *************************************************************************
//   機能		： ゴール時の動作
//   注意		： なし
//   メモ		： なし
//   引数		： 目標x座標、目標y座標、探索方法、探索動作
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
//		v1.1		2019.09.22			TKR				スタート座標に戻らない対策（コメントアウト中）
// *************************************************************************/
PUBLIC void MAP_searchGoal( UCHAR uc_trgX, UCHAR uc_trgY, enMAP_ACT_MODE en_type, enSEARCH_MODE en_search )
{
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL			bl_type = TRUE;			// 現在位置、FALSE: １区間前進状態、TURE:半区間前進状態
	
	MOT_setTrgtSpeed(MAP_SEARCH_SPEED);		// 目標速度
	MOT_setNowSpeed( 0.0f );
	
	/* ゴール座標目指すときは尻当て考慮 */
	if( ( uc_trgX == GOAL_MAP_X ) && ( uc_trgY == GOAL_MAP_Y ) ){
		f_MoveBackDist = MOVE_BACK_DIST;		
	}else{
		f_MoveBackDist = 0;
	}

	uc_SlaCnt = 0;

	/* 迷路探索 */
	while(1){
		MAP_refMousePos( en_Head );							// 座標更新
		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );	// 等高線マップを作る

		/* ダミー壁挿入 */
	/*	
		if( (uc_trgX == GOAL_MAP_X) && (uc_trgY == GOAL_MAP_Y) ){
			g_sysMap[0][0]	= 0x01;
			g_sysMap[0][1]	= 0x04;
		}
	*/	
		if( TRUE == bl_type ){
			MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, MAP_SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			f_MoveBackDist = 0;	
			LED_onAll();
		}
		MAP_makeMapData();									// 壁データから迷路データを作成
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出
		
		/* 次の区画へ移動 */
		if(( mx == uc_trgX ) && ( my == uc_trgY )){

			/* ダミー壁削除 */
		/*	
			if( (uc_trgX == GOAL_MAP_X) && (uc_trgY == GOAL_MAP_Y) ){
				g_sysMap[0][0]	&= ~0x01;
				g_sysMap[0][1]	&= ~0x04;
			}
		*/
			MAP_actGoal();		// ゴール時の動作
			return;				// 探索終了
		}
		else{
			/* 超信地旋回探索 */
			if( SEARCH_TURN == en_search ){
				MAP_moveNextBlock(en_head, &bl_type);				// 次の区画へ移動	← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
			}
			/* スラローム探索 */
			else if( SEARCH_SURA == en_search ){
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// 次の区画へ移動	← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
			}
		}
#if 0		
		/* 途中で制御不能になった */
		if( SYS_isOutOfCtrl() == TRUE ){
			
			/* 迷路関連を初期化 */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMCは下位モジュールで既にクリアと緊急停止を行っている。
			break;
		}
#endif		
	}

	TIME_wait(10000);
}


// *************************************************************************
//   機能		： PCIF、現在のマップを表示する
//   注意		： なし
//   メモ		： PCIFから実行する
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showMap_ForPCIF( void )
{
	MAP_showLog();
//	MAP_showLog_BackUp();			// ★
}


// *************************************************************************
//   機能		： 位置を更新する
//   注意		： なし
//   メモ		： なし
//   引数		： 動作コマンド
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_refPos( UCHAR uc_cmd )
{
	UCHAR uc_index = 0;			// テーブルのインデックス番号
	
	/* ------------------------------------------ */
	/*  コマンドからテーブルのインデックスを取得  */
	/* ------------------------------------------ */
	/* 直進 */
	if( ( uc_cmd <=  GO71 ) && ( uc_cmd >=  GO1) ){
		
		uc_index = 14;		// st_PosDataテーブルの直進のインデックス番号
	}
	/* 斜め直進 */
	else if( ( uc_cmd <=  NGO71 ) && ( uc_cmd >=  NGO1) ){
		
		uc_index = 15;		// st_PosDataテーブルの斜め直進のインデックス番号
	}
	/* その他のコマンド */
	else{
		while(1){
			
			if( st_PosData[uc_index].en_cmd == uc_cmd )      break;			// コマンド発見
			if( st_PosData[uc_index].en_cmd == MAP_CMD_MAX ) return;		// コマンド未発見
			uc_index++;
		}
	}
	
	/* 位置更新 */
	switch( s_PosDir ){
		
		/* [0]北 [1]北東 */
		case 0:
		case 1:
		
			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y0_y1 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y0_y1 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x0_x1;
				f_PosY += st_PosData[uc_index].f_y0_y1;
			}
			break;
		
		/* [2]東 [3]南東 */
		case 2:
		case 3:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y2_y3 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y2_y3 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x2_x3;
				f_PosY += st_PosData[uc_index].f_y2_y3;
			}
			break;

		/* [4]南 [5]南西 */
		case 4:
		case 5:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y4_y5 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y4_y5 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x4_x5;
				f_PosY += st_PosData[uc_index].f_y4_y5;
			}
			break;

		/* [6]西 [7]北西 */
		case 6:
		case 7:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y6_y7 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y6_y7 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x6_x7;
				f_PosY += st_PosData[uc_index].f_y6_y7;
			}
			break;
	}
	
	/* 進行方向更新 */
	s_PosDir += st_PosData[uc_index].s_dir;
	if( s_PosDir < 0 ) s_PosDir += 8;				// [0]～[7]にしたい
	else if( s_PosDir > 7 ) s_PosDir -= 8;
	
	f_LogPosX[us_LogIndex] = f_PosX;
	f_LogPosY[us_LogIndex] = f_PosY;
	
	us_LogIndex++;
	us_LogIndex %= 30;
}


// *************************************************************************
//   機能		： コーナー前に壁があったら壁の切れ目補正を行う設定をする
//   注意		： なし
//   メモ		： なし
//   引数		： 動作コマンド
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE BOOL MAP_setWallCut( UCHAR uc_cmd )
{
	UCHAR uc_val = 0;			// 1区画前のコーナー側の壁があるか（0以外なら壁あり）
	UCHAR uc_valPrev = 0;		// 2区画前のコーナー側の壁があるか（0以外なら壁あり）
	BOOL bl_wallCut = FALSE;
	
	/* 位置更新 */
	switch( uc_cmd ){
		
		case R90S:
		case RS135N:
			
			/* 1区画前のコーナー側の壁があるか（0以外ならあり） */
			// s_PosDir：進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
			switch( s_PosDir ){
				
				/* 柱基準で旋回するので、半区画手前が壁の有無を調べたい座標となる（注意：g_sysMapは2次元配列です） */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x02;		// 北を向いているので東側の壁があるか
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x02;		// 北を向いているので東側の壁があるか
					break;	
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x04;		// 東を向いているので南側の壁があるか
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x04;		// 東を向いているので南側の壁があるか
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x08;		// 南を向いているので西側の壁があるか
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x08;		// 南を向いているので西側の壁があるか
					break;
				case 6:
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x01;		// 西を向いているので北側の壁があるか
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x01;		// 西を向いているので北側の壁があるか
					break;
			}
			/* 壁があるため壁切れ補正を行う */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_RIGHT );		// 壁切れ補正を実施する
				bl_wallCut = TRUE;
			}
			break;
			
		case L90S:
		case LS135N:
			/* 1区画前のコーナー側の壁があるか（0以外ならあり） */
			// s_PosDir：進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
			switch( s_PosDir ){
				
				/* 柱基準で旋回するので、半区画手前が壁の有無を調べたい座標となる（注意：g_sysMapは2次元配列です） */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x08;			// 北を向いているので西側の壁があるか
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x08;			// 北を向いているので西側の壁があるか
					break;
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x01;			// 東を向いているので北側の壁があるか
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x01;			// 東を向いているので北側の壁があるか
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x02;			// 南を向いているので東側の壁があるか
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x02;			// 南を向いているので東側の壁があるか
					break;
				case 6: 
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x04;			// 西を向いているので南側の壁があるか
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x04;			// 西を向いているので南側の壁があるか
					break;
			}
			/* 壁があるため壁切れ補正を行う */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_LEFT );		// 壁切れ補正を実施する
				bl_wallCut = TRUE;
			}
			break;
			
		default:
			break;
	}
	
	return bl_wallCut;
}


// *************************************************************************
//   機能		： コマンド走行用の座標位置を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： x座標、y座標、進行方向
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	f_PosX   = (FLOAT)uc_x;
	f_PosX   = (FLOAT)uc_y;
	s_PosDir = (SHORT)(en_dir * 2);	// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）、2倍すると丁度値が合致する
}


// *************************************************************************
//   機能		： 迷路コマンドデータを表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showCmdLog( void )
{
	USHORT i=0;
	
	/* 超信地旋回コマンド */
	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
	}
	i=0;
	
	/* スラロームコマンド */
	while(1){
		
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
		if( scom[i] == CEND ) break;
		i++;
	}
	i=0;

	/* 斜め走行コマンド */
	while(1){
		
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
	}
}


// *************************************************************************
//   機能		： 超地信動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： 開始X座標、開始Y座標、開始時の方向、終了X座標、終了Y座標、[out]終了時の方向
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeCmdList( UCHAR uc_staX, UCHAR uc_staY, enMAP_HEAD_DIR en_staDir, UCHAR uc_endX,	UCHAR uc_endY, enMAP_HEAD_DIR* en_endDir )
{
	UCHAR			uc_goStep;									// 前進のステップ数
	USHORT			us_high;									// 等高線の高さ
	USHORT			us_pt;										// コマンドポインタ
	enMAP_HEAD_DIR	en_nowDir;									// 現在マウスの向いている絶対方向
	enMAP_HEAD_DIR	en_tempDir;									// 相対方向
//	USHORT			i;											// roop
	
	/* 前進ステップ数を初期化する */
	uc_goStep = 0;
	us_pt = 0;

	/* 迷路情報からコマンド作成 */
	while(1){	
		us_high = us_cmap[uc_staY][uc_staX]-1;
		if (en_staDir == NORTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else   while(1);
		}else if (en_staDir == EAST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else   while(1);
		}else if (en_staDir == SOUTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else   while(1);
		}else if (en_staDir == WEST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else   while(1);
		}
		
		en_tempDir = (enMAP_HEAD_DIR)( (en_nowDir - en_staDir) & (enMAP_HEAD_DIR)3 );		// 方向更新
		en_staDir = en_nowDir;

		if (en_tempDir == NORTH){
			uc_goStep = uc_goStep + 2;
		}
		else if (en_tempDir == EAST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R90;
			uc_goStep = 2;
			us_pt++;
		}
		else if (en_tempDir == WEST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = L90;
			uc_goStep = 2;
			us_pt++;
		}
		else{
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R180;
			uc_goStep = 2;
			us_pt++;
		}

		if      (en_nowDir == NORTH) uc_staY = uc_staY + 1;
		else if (en_nowDir == EAST) uc_staX = uc_staX + 1;
		else if (en_nowDir == SOUTH) uc_staY = uc_staY - 1;
		else if (en_nowDir == WEST) uc_staX = uc_staX - 1;
		
		en_staDir = en_nowDir;
		
		if ((uc_staX == uc_endX) &&(uc_staY == uc_endY)) break;
	}
	
	/* 超地信旋回用のコマンドリスト作成 */
	dcom[us_pt] = uc_goStep;
	dcom[++us_pt] = STOP;
	dcom[++us_pt] = CEND;
	us_totalCmd = us_pt+1;			// コマンド総数


	/* 最終的に向いている方向 */
	*en_endDir = en_staDir;
	
#if 0
	/* debug */
	for( i = 0; i < us_totalCmd; i++){
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
	}
#endif

}


// *************************************************************************
//   機能		： スラローム動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeSuraCmdList( void )
{
	USHORT dcom_temp[1024];			// 半区画超信旋回コマンドリスト
	USHORT i=0,j=0;					// roop
	
	/* 超地信旋回コマンドをコピー */
	for( i=0; i<us_totalCmd; i++ ){
		dcom_temp[i] = dcom[i];
	}

	i = 0;

	/* 配列が旋回コマンドかをチェック */
	while(1)
	{
		if( dcom_temp[i] == R90 ){		// 右90°
			dcom_temp[i-1] -= 1;		// 1つ手前を引く
			dcom_temp[i+1] -= 1;		// 1つ手前を引く
			dcom_temp[i] = R90S;		// 右スラローム90°
		}
		else if( dcom_temp[i] == L90 ){	// 左90°
			dcom_temp[i-1] -= 1;		// 1つ手前を引く
			dcom_temp[i+1] -= 1;		// 1つ手前を引く
			dcom_temp[i] = L90S;		// 左スラローム90°
		}
		else{
			if( dcom_temp[i] == CEND ){
				break;
			}
		}
		i++;
	}

	i = j = 0;

	/* スラロームコマンド変換 */
	while(1)
	{
		if( dcom_temp[i+1] == CEND ){
			scom[j] = STOP;
			scom[j+1] = CEND;
			break;
		}
		else
		{
			/* データがストップコマンドだったら */
			if( dcom_temp[i] == 0 ){
				i++;
			}
			
			scom[j] = (UCHAR)dcom_temp[i];
			
			i++;
			j++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
	}
#endif
}


// *************************************************************************
//   機能		： 斜め動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v0.9		2013.09.17			外川			135度のLターンがRターンになっていた不具合修正
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_makeSkewCmdList( void )
{
	USHORT	scom_temp[4096];			// 半区画超信旋回コマンドリスト
	USHORT	i;							// roop
	USHORT	c1, c2, c3, c4;				// 計算用
	USHORT	x;
	USHORT	ct_n=0, ct_st=0;
	USHORT	flag = 0;					//	斜め走行用バッファ  0:複合コマンド　1:斜め  2:S135N → N135S  3:直進
	
	/* 超地信旋回コマンドをコピー */
	for( i=0; i<us_totalCmd; i++ )
	{
		scom_temp[i] = scom[i];
	}

	i=0;

	/* 配列が旋回コマンドかをチェック */
	while(1)
	{
		c1 = scom_temp[ct_st];
		c2 = scom_temp[ct_st+1];
		c3 = scom_temp[ct_st+2];
		c4 = scom_temp[ct_st+3];

		//	直進 → 右45度 → 斜め
		if( (c1<=GO32) && (c2==R90S) && (c3==L90S) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	前の複合コマンドによって直線区間が消えない場合
			tcom[ ct_n++ ] = RS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 0;
		}
		//	直進 → 左45度 → 斜め
		else if( (c1<=GO32) && (c2==L90S) && (c3==R90S) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	前の複合コマンドによって直線区間が消えない場合
			tcom[ ct_n++ ] = LS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 0;
		}

		//	直進 → 右90度 → 直進
		else if( (c1<=GO32) && (c2==R90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = R90S;
			ct_st += 2;
			flag = 3;		//	直進
		}
		//	直進 → 左90度 → 直進
		else if( (c1<=GO32) && (c2==L90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = L90S;
			ct_st += 2;
			flag = 3;		//	直進
		}
		//	直進 → 右135度 → 斜め
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = RS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 2;
		}
		//	直進 → 左135度 → 斜め
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = LS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 2;
		}

		//	直進 → 右180度 → 直進
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = R90S;
			tcom[ ct_n++ ] = R90S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	直進 → 左180度 → 直進
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = (UCHAR)c1;
			tcom[ ct_n++ ] = L90S;
			tcom[ ct_n++ ] = L90S;
			ct_st += 3;
			flag = 3;		//	直進
		}

		//	斜め → 右45度 → 直進
		else if( (c1==R90S) && (c2<=GO32) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)x;
			tcom[ ct_n++ ] = RN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	直線区間を1つ減らす
			ct_st ++;
			flag = 3;		//	直進
		}
		//	斜め → 左45度 → 直進
		else if( (c1==L90S) && (c2<=GO32) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)x;
			tcom[ ct_n++ ] = LN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	直線区間を1つ減らす
			ct_st ++;
			flag = 3;		//	直進
		}
		//	斜め → 右90度 → 斜め
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからRN90N
			else if( flag==1 ) tcom[ ct_n++ ] = (UCHAR)(x+1);
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 1;
		}
		//	斜め → 左90度 → 斜め
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 1;
		}
		//	斜め → 右135度 → 直進
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN135S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	斜め → 左135度 → 直進
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4<=GO32) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN135S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	斜め → 斜め
		else if( (c1==R90S) && (c2==L90S) && ( (c3==R90S) || (c3==L90S) || ( c3<=GO32 ) ) )
		{
			x++;
			ct_st ++;

			flag = 1;		//	斜め走行バッファあり
		}
		else if( (c1==L90S) && (c2==R90S) && ( (c3==L90S) || (c3==R90S) || ( c3<=GO32 ) ) )
		{
			//	コマンド出力
			x++;
			ct_st ++;

			flag = 1;		//	斜め走行バッファあり
		}
		else
		{
			tcom[ ct_n ] = (UCHAR)scom_temp[ct_st];
			if( tcom[ ct_n ] == CEND ) break;
			ct_st ++;
			ct_n ++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
	}
#endif
}


// *************************************************************************
//   機能		： コマンド走行モジュール
//   注意		： なし
//   メモ		： 動作開始の座標位置は、MAP_setPos()で設定する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_drive( enMAP_DRIVE_TYPE en_driveType )
{
	USHORT				us_rp = 0;				// 現在の読み込み位置
	enMOT_TURN_CMD 		en_type;
	BOOL				bl_isWallCut = FALSE;
	
	/* 超信旋回モード*/
	if( en_driveType == MAP_DRIVE_TURN )
	{
		while(1)
		{
			if ( dcom[us_rp] == CEND  ) break;								//	コマンド終了
			
			else if ( dcom[us_rp] == STOP  ){
				CTRL_stop();			// 制御停止
				DCM_stopMot( DCM_R );		// 停止
				DCM_stopMot( DCM_L );		// 停止
			}
			else if ( ( dcom[us_rp] <=  GO71 ) && ( dcom[us_rp] >=  GO1) )
			{
				MOT_goBlock_FinSpeed( (FLOAT)dcom[us_rp]*0.5f, 0 );		// 直線走行コマンド、半区間前進後に停止
			}
			else{
				
				if( dcom[us_rp] == R90 ) en_type = MOT_R90;
				else 					 en_type = MOT_L90;
				
				TIME_wait(500);
				MOT_turn( en_type );		//	旋回
				TIME_wait(500);
			}
			us_rp++;
#if 0			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
		CTRL_stop();			// 制御停止
		DCM_stopMot( DCM_R );		// 停止
		DCM_stopMot( DCM_L );		// 停止
	}
	/* スラロームモード */
	else if( en_driveType == MAP_DRIVE_SURA )
	{
		while(1)
		{
			MAP_refPos( scom[us_rp] );									// 実行されるコマンドが終了した位置に更新

			if ( scom[us_rp] == CEND  ) break;							//	コマンド終了
			
			else if ( scom[us_rp] == STOP  )
			{
				CTRL_stop();			// 制御停止
				DCM_stopMot( DCM_R );		// 停止
				DCM_stopMot( DCM_L );		// 停止
			}
			else if ( ( scom[us_rp] <=  GO71 ) && ( scom[us_rp] >=  GO1) )
			{
				if( scom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, 0 );						// 直線走行コマンド、半区間前進（最終速度なし）
				}
				else{
					
					/* 壁の切れ目補正 */
					if( ( scom[us_rp+1] == R90S )   || ( scom[us_rp+1] == L90S ) ){
						bl_isWallCut = MAP_setWallCut( scom[us_rp+1] );		// コーナー前に壁があったら壁の切れ目補正を行う設定をする
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, MOT_getSlaStaSpeed() );		// 直線走行コマンド、半区間前進（最終速度あり）
				}
			}
			else if( scom[us_rp] == R90S )
			{
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
			}
			else if( scom[us_rp] == L90S )
			{
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
			}
			us_rp++;
#if 0			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
	}
	/* 斜めモード */
	else if( en_driveType == MAP_DRIVE_SKEW )
	{
		while(1)
		{
			MAP_refPos( tcom[us_rp] );									// 実行されるコマンドが終了した位置に更新
			
			if ( tcom[us_rp] == CEND  ) break;							//	コマンド終了

			else if ( tcom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// 制御停止
				DCM_stopMot( DCM_R );		// 停止
				DCM_stopMot( DCM_L );		// 停止
			}
			else if ( ( tcom[us_rp] <=  GO71 ) && ( tcom[us_rp] >=  GO1) )
			{
				if( tcom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, 0 );						// 直線走行コマンド、半区間前進（最終速度なし）
				}
				else{
					
					/* 壁の切れ目補正 */
					if( ( tcom[us_rp+1] == R90S )   || ( tcom[us_rp+1] == L90S )   || 
					 	( tcom[us_rp+1] == RS135N ) || ( tcom[us_rp+1] == LS135N ) 
					 ){
						bl_isWallCut = MAP_setWallCut( tcom[us_rp+1] );		// コーナー前に壁があったら壁の切れ目補正を行う設定をする
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, MOT_getSlaStaSpeed() );		// 直線走行コマンド、半区間前進（最終速度あり）
				}
			}
			else if ( ( tcom[us_rp] <=  NGO71 ) && ( tcom[us_rp] >=  NGO1) )
			{
				MOT_goSkewBlock_FinSpeed( (FLOAT)(tcom[us_rp]-81)*0.5f, MOT_getSlaStaSpeed());	// 斜め直線走行コマンド、半区間前進（最終速度あり）
			}
			else
			{
				switch( tcom[us_rp] )
				{
					/* 直進 → 直進 */
					case R90S:		MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			break;
					case L90S:		MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			break;
					
					/* 直進 → 斜め */
					case RS45N:		MOT_goSla( MOT_R45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case LS45N:		MOT_goSla( MOT_L45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case RS135N:	MOT_goSla( MOT_R135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;
					case LS135N:	MOT_goSla( MOT_L135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;

					/* 斜め → 直進 */
					case RN45S:		MOT_goSla( MOT_R45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case LN45S:		MOT_goSla( MOT_L45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case RN135S:	MOT_goSla( MOT_R135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;
					case LN135S:	MOT_goSla( MOT_L135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;

					/* 斜め → 斜め */
					case RN90N:		MOT_goSla( MOT_R90S_N, PARAM_getSra( SLA_N90 ) ); 		break;
					case LN90N:		MOT_goSla( MOT_L90S_N, PARAM_getSra( SLA_N90 ) );		break;
				}
			}
			us_rp++;
#if 0			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE ){
				break;
			}
#endif			
		}
	}

}


// *************************************************************************
//   機能		： PCIF、コマンドリストを表示（動作したログ）
//   注意		： なし
//   メモ		： PCIFから実行する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showCmdList( void )
{
	USHORT i=0;

	/* 超信地旋回 */
	printf("dcom \n\r");
	while(1){
		if( dcom[i] == 0 ) break;
		printf("%03d\n\r",dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* スラローム */
	i = 0;
	printf("scom \n\r");
	while(1){
		if( scom[i] == 0 ) break;
		printf("%03d\n\r",scom[i]);
		if( scom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* 斜め */
	i = 0;
	printf("tcom \n\r");
	while(1){
		if( tcom[i] == 0 ) break;
		printf("%03d\n\r",tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}

	/* 探索ダッシュ */
	i = 0;
	printf("mcom \n\r");
	while(1){
		if( mcom[i] == 0 ) break;
		printf("%03d\n\r",mcom[i]);
		if( mcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}
	
	printf("[x]%d [y]%d [Head]%d \n\r", mx, my, en_Head);
	
	
	MAP_makeContourMap( 0, 0 , SEARCH);
}


// *************************************************************************
//   機能		： PCIF、Mapから作成したコマンドリストを表示（シミュレーション）
//   注意		： なし
//   メモ		： PCIFから実行する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_showMakeCmdList( void )
{
	USHORT i=0;
	enMAP_HEAD_DIR		en_endDir;
	
	MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// 等高線マップを作る
	MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();

	/* 超信地旋回 */
	printf("dcom \n\r");
	while(1){
		printf("%03d\n\r",dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
		
		if( i== LIST_NUM-1 ) break;
	}

	/* スラローム */
	i = 0;
	printf("scom \n\r");
	while(1){
		printf("%03d\n\r",scom[i]);
		if( scom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}

	/* 斜め */
	i = 0;
	printf("tcom \n\r");
	while(1){
		printf("%03d\n\r",tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;

		if( i== LIST_NUM-1 ) break;
	}
}


// *************************************************************************
//   機能		：PCIF for debug
//   注意		： なし
//   メモ		： PCIFから実行する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PUBLIC void MAP_debug( void )
{
//	USHORT i=0;
//	enMAP_HEAD_DIR		en_endDir;
	
//	UCHAR uc_dummy[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路データ


	MAP_showLog();
	MAP_showLog_BackUp();





	MAP_clearMap();

//	Storage_Load( (const void*)uc_dummy, sizeof(uc_dummy), ADR_MAP );			// データロード(dummy) これがないと次の１発目のsaveがうまくいかない

#if 0
	/* バックアップ迷路の復帰 */
	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// バックアップデータを復帰する
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// データがあったら
		
		printf("\n\r　　　　　　　　　　　　＼　│　／");
		printf("\n\r　　　　　　　　　　　　　／￣＼　　／￣￣￣￣￣￣￣￣￣");
		printf("\n\r　　　　　　　　　　　─（ﾟ ∀ ﾟ）＜　迷路データをぉぉぉ");
		printf("\n\r　　　　　　　　　　　　　＼＿／　　＼＿＿＿＿＿＿＿＿＿");
		printf("\n\r　　　　　　　　　　　　／　│　＼");
		printf("\n\r　　　　　　　　　　　　　　　 ∩ ∧　∧∩ ／￣￣￣￣￣￣￣￣￣￣");
		printf("\n\r　￣￣￣￣￣￣￣￣＼ ∩∧ ∧∩ ＼（ ﾟ∀ﾟ）＜　復帰だ復帰だ復帰だ！");
		printf("\n\r　　復帰だ～～～！ ＞（ﾟ∀ﾟ）/ 　｜　　　/　＼＿＿＿＿＿＿＿＿＿＿");
		printf("\n\r　＿＿＿＿＿＿＿＿／ ｜　　〈　　｜　　 ｜");
		printf("\n\r　　　　　　　　　　 /　／＼_」　/　／＼」");
		printf("\n\r　　　　　　　　　　 ￣　　　　 / ／");
		printf("\n\r　　　　　　　　　　　　　　　  ￣");
		Storage_Load( (const void*)g_sysMap, sizeof(g_sysMap), ADR_MAP );		// バックアップ迷路データで現在の迷路情報を上書き
	}

#endif

	MAP_SaveMapData();














#if 0
	MAP_makeContourMap( 3, 3, BEST_WAY );					// 等高線マップを作る
	MAP_makeCmdList( 0, 0, NORTH, 3, 3, &en_endDir );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();

	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);

		if( ( dcom[i] == CEND ) || ( i ==500 ) ) break;
		
		i++;
	}

#endif	



#if 0
	printf( "0x%x \n\r", g_sysMap[0][0]);
	printf( "0x%x \n\r", g_sysMap[0][1]);
	printf( "0x%x \n\r", g_sysMap[0][2]);
	printf( "0x%x \n\r", g_sysMap[0][3]);
	printf( "0x%x \n\r", g_sysMap[1][0]);
	printf( "0x%x \n\r", g_sysMap[1][1]);
	printf( "0x%x \n\r", g_sysMap[1][2]);
	printf( "0x%x \n\r", g_sysMap[1][3]);
	printf( "0x%x \n\r", g_sysMap[2][0]);
	printf( "0x%x \n\r", g_sysMap[2][1]);
	printf( "0x%x \n\r", g_sysMap[2][2]);
	printf( "0x%x \n\r", g_sysMap[2][3]);
	printf( "0x%x \n\r", g_sysMap[3][0]);
	printf( "0x%x \n\r", g_sysMap[3][1]);
	printf( "0x%x \n\r", g_sysMap[3][2]);
	printf( "0x%x \n\r", g_sysMap[3][3]);
	MAP_clearMap();
	g_sysMap[0][0] = 0xfe;			us_cmap[0][0] = 10;
	g_sysMap[0][1] = 0xfc;			us_cmap[0][1] = 7;
	g_sysMap[0][2] = 0xf4;			us_cmap[0][2] = 6;
	g_sysMap[0][3] = 0xf6;			us_cmap[0][3] = 5;
	g_sysMap[0][4] = 0xfc;			us_cmap[0][4] = 255;
                                           
	g_sysMap[1][0] = 0xf8;			us_cmap[1][0] = 9;
	g_sysMap[1][1] = 0xf2;			us_cmap[1][1] = 8;
	g_sysMap[1][2] = 0xfb;			us_cmap[1][2] = 7;
	g_sysMap[1][3] = 0xfa;			us_cmap[1][3] = 4;
	g_sysMap[1][4] = 0xf8;			us_cmap[1][4] = 255;
                                           
	g_sysMap[2][0] = 0xfa;			us_cmap[2][0] = 10;
	g_sysMap[2][1] = 0xfa;			us_cmap[2][1] = 9;
	g_sysMap[2][2] = 0xfc;			us_cmap[2][2] = 2;
	g_sysMap[2][3] = 0xf3;			us_cmap[2][3] = 3;
	g_sysMap[2][4] = 0xf8;			us_cmap[2][4] = 255;
                                           
	g_sysMap[3][0] = 0xf9;			us_cmap[3][0] = 11;
	g_sysMap[3][1] = 0xf3;			us_cmap[3][1] = 10;
	g_sysMap[3][2] = 0xf9;			us_cmap[3][2] = 1;
	g_sysMap[3][3] = 0xf7;			us_cmap[3][3] = 0;
	g_sysMap[3][4] = 0xf8;			us_cmap[3][4] = 255;
                                           
	g_sysMap[4][0] = 0xfc;			us_cmap[4][0] = 255;
	g_sysMap[4][1] = 0xf4;			us_cmap[4][1] = 255;
	g_sysMap[4][2] = 0xf4;			us_cmap[4][2] = 255;
	g_sysMap[4][3] = 0xf4;			us_cmap[4][3] = 255;


	MAP_showLog();
	
	
	MAP_makeCmdList( 0, 0, NORTH, 3, 3, EAST );
	MAP_makeSuraCmdList();
	MAP_makeSkewCmdList();
	
#endif
	
	
#if 0
//	UCHAR x,z;

	for( z=0; z<20; z++ ){
		printf("\n\r");
		printf("\n\r");
		printf("[%d] \n\r", z);

		printf("%d ",3);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[3][x][z]);
		}
		printf("\n\r");

		printf("%d ",2);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[2][x][z]);
		}
		printf("\n\r");

		printf("%d ",1);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[1][x][z]);
		}
		printf("\n\r");

		printf("%d ",0);
		for( x=0; x<4; x++ ){
			printf(" %4d ",us_Log[0][x][z]);
		}
		printf("\n\r");
	}
#endif
}




#ifdef __cplusplus
    }
#endif



