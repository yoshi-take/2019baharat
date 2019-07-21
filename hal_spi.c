// *************************************************************************
//   ロボット名	： Baharat（バハラット）
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： SPIの設定
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

#include <hal_spi.h>		// SPI

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
PUBLIC	enSPI_STATE		en_SpiState;	// SPI通信状態
PUBLIC	SHORT*			p_SpiRcvData;	// SPI受信データ格納アドレス
PUBLIC	FUNC_PTR		p_SpiCallBackFunc;	// SPIのIDLE遷移割り込み時に登録されると呼び出される関数

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： RSPIの初期化
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.4.2			TKR			新規
// *************************************************************************/
PUBLIC void SPI_init(void){
	
	UCHAR	uc_dummy;
	
    SYSTEM.PRCR.WORD	= 0xA502;
	MSTP(RSPI1)			= 0;
	SYSTEM.PRCR.WORD	= 0xA500;
	
	PORTE.PDR.BIT.B3	= 0;	// PE3：入力に設定(MISO)	
	
    MPC.PWPR.BIT.B0WI   	= 0;
    MPC.PWPR.BIT.PFSWE  	= 1;
    MPC.PE1PFS.BIT.PSEL     = 0x0e; // PE1:RSPCKB
    MPC.PE2PFS.BIT.PSEL     = 0x0e; // PE2:MOSIB
    MPC.PE3PFS.BIT.PSEL     = 0x0d; // PE3:MISOB
    MPC.PWPR.BIT.PFSWE  	= 0;
    MPC.PWPR.BIT.B0WI   	= 1;

	PORTE.PMR.BIT.B1	= 1;	// 周辺機能の設定(CLK)
	PORTE.PMR.BIT.B2	= 1;	// 周辺機能の設定(MOSI)		
	PORTE.PMR.BIT.B3	= 1;	// 周辺機能の設定(MISO)
	PORTE.PDR.BIT.B4	= 1;	// PE4：出力に設定(CS)	
	
	RSPI1.SPDCR.BIT.SPLW	= 0;	// ワードアクセス
	RSPI1.SPBR				= 5;	// 1MHz = 48MHz(PCLK) / ( 2 * ( SPBR + 1 ) * 2 ^ BRDV )
	RSPI1.SPCMD0.BIT.BRDV	= 2;	// ↑SPBR:5,BRDV:2
	RSPI1.SPCMD0.BIT.CPHA	= 1;	// 奇数エッジでデータ変化、偶数エッジでデータサンプル
	RSPI1.SPCMD0.BIT.CPOL	= 1;	// アイドル時のクロックはHighレベル
	RSPI1.SPCMD0.BIT.SPB	= 0x0f;	// データ長16bit
	RSPI1.SPCMD0.BIT.LSBF	= 0;	// 0:MSBファースト 1:LSBファースト
	RSPI1.SPCMD0.BIT.SSLKP	= 0;	// バースト転送無し（SSLはGPIOなので設定しない）
    RSPI1.SPCMD0.BIT.SCKDEN = 0;    // RSPCKを1RSPCK
    RSPI1.SPCMD0.BIT.SLNDEN = 0;    // ネゲート遅延を1RSPCLK 
	RSPI1.SPCMD0.BIT.SPNDEN = 1;    // 次アクセス遅延を1RSPCK+2PCLK
	RSPI1.SPCR.BIT.MSTR		= 1;	// マスターモード
	uc_dummy 				= RSPI1.SPCR.BYTE;		// ダミーリード
	uc_dummy				= uc_dummy;				// warining抑止
	
	ICU.IER[0x05].BIT.IEN2 		= 1;		// SPRI1の割り込みの許可
	ICU.IER[0x05].BIT.IEN3 		= 1;		// SPTI1の割り込みの許可
	ICU.IER[0x05].BIT.IEN4 		= 1;		// SPII1の割り込みの許可
	ICU.IPR[42].BIT.IPR 		= 7;		// SPI1の割り込みレベルの設定
	
}

// *************************************************************************
//   機能		： RSPIの受信
//   注意		： なし
//   メモ		： なし
//   引数		： アドレス(マスク済み)
//   返り値		： 読みだした値
// **************************    履    歴    *******************************
// 		v1.0		2019.4.3			TKR			新規
// *************************************************************************/
PUBLIC USHORT SPI_Recv(USHORT address){

    USHORT  data;
    RSPI1.SPDR.WORD.H   = address;
	
  	while(!RSPI1.SPSR.BIT.IDLNF);	//送信開始を確認
	while(RSPI1.SPSR.BIT.IDLNF);	//RSPI1ｱｲﾄﾞﾙ状態か確認

    data = RSPI1.SPDR.WORD.H;
    return(data);
}

// *************************************************************************
//   機能		： RSPIのデータ送信割り込み処理
//   注意		： なし
//   メモ		： この関数の後にSPI_III()の割り込みを期待している（送信処理時）
//   引数		： なし
//   返り値		： 読みだした値
// **************************    履    歴    *******************************
// 		v1.0		2019.6.1			TKR			新規
// *************************************************************************/
PUBLIC void	SPI_TXI(void){
	
	RSPI1.SPCR.BIT.SPTIE	= 0;	// SPI送信割り込み要求の発生の禁止
	RSPI1.SPCR2.BIT.SPIIE	= 1;	// アイドル割り込み要求の発生を許可

}

// *************************************************************************
//   機能		： RSPIのデータ設定開始
//   注意		： なし
//   メモ		： なし
//   引数		： アドレス，書き込む値
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.1			TKR			新規
// *************************************************************************/
PUBLIC void SPI_staSetData( USHORT us_adr, USHORT us_val ){
	
	USHORT	us_dummy	= 0;
	UCHAR	uc_dummy	= 0;	

	/* IDLE時以外には実行しない */
	if( en_SpiState != SPI_IDLE ){
		printf("SPI送信開始NG [状態]%d \n\r", en_SpiState);
		return;
	}
	
	RSPI1.SPCR.BIT.SPE	= 0;		// SPI機能無効
	en_SpiState			= SPI_SND;	// SPI状態：送信中
	
	/*--------------*/
	/*　転送前処理　*/
	/*--------------*/
	/*　フラグクリア処理　*/
	RSPI1.SPSR.BIT.MODF	= 0;		// モードフォルトエラーなし
	RSPI1.SPSR.BIT.OVRF	= 0;		// オーバーランエラーなし
	RSPI1.SPSR.BIT.PERF	= 0;		// パリティエラーなし
	
	/*　割り込み禁止処理　*/
	RSPI1.SPCR2.BIT.SPIIE	= 0;	// アイドル割り込みの発生を禁止
	
	/* ポート制御処理 */
	PORTE.PODR.BIT.B4		= 0;	// ポートE-4をLo出力(SSL-ON)
	
	/* SPI通信許可 */
	RSPI1.SPCR.BIT.SPE		= 1;	// SPI機能有効
	RSPI1.SPCR.BIT.SPTIE	= 1;	// SPI送信割り込み要求の発生を許可
	RSPI1.SPCR.BIT.SPRIE	= 0;	// SPI受信割り込み要求の発生を禁止
	RSPI1.SPCR.BIT.SPEIE	= 0;	// SPIエラー割り込み要求の発生を禁止
	
	RSPI1.SPDR.WORD.H		= (USHORT)( us_adr | us_val | SPI_W );	// データセット

	/* ダミー処理 */
	us_dummy		= RSPI1.SPDR.WORD.H;	// dummy read
	uc_dummy		= RSPI1.SPCR.BYTE;		// dummy read
	NOUSE( us_dummy );
	NOUSE( uc_dummy );	
	
}

// *************************************************************************
//   機能		： RSPIデータ受信割り込み処理
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC void SPI_RXI( void ){
	
	USHORT	us_rcv;		// データ取得
	
	RSPI1.SPCR.BIT.SPRIE	= 0;		// SPI受信割り込み要求の発生を禁止
	us_rcv	= RSPI1.SPDR.WORD.H;		// データ取得

	/* 格納するアドレスチェック */
	if( p_SpiRcvData != NULL ){
		*p_SpiRcvData	= (SHORT)( us_rcv & 0xff );	// 1byteだけの取り出し
	}
	
}	

// *************************************************************************
//   機能		： RSPIデータ受信開始
//   注意		： 本関数実行前に，データ取得先(p_SpiRcvData)を指定すること．
//				   必要に応じてコールバック関数(p_SpiCallBackFunc)も指定すること
//   メモ		： なし
//   引数		： アドレス
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC void SPI_staGetData( USHORT us_adr ){

	USHORT	us_dummy	= 0;
	UCHAR	uc_dummy	= 0;

	/* IDLE時以外は実行しない */
	if( en_SpiState != SPI_IDLE ){
		printf("SPI取得開始NG[状態]%d \n\r",en_SpiState);
		return;
	}

	RSPI1.SPCR.BIT.SPE	= 0;		// SPI機能無効
	en_SpiState			= SPI_RCV;	// SPI状態：受信中
	
	/*--------------*/
	/*　転送前処理　*/
	/*--------------*/
	/* フラグクリア処理 */
	RSPI1.SPSR.BIT.MODF		= 0;	// モードフォルトエラーフラグ
	RSPI1.SPSR.BIT.OVRF		= 0;	// オーバーランエラーフラグ
	RSPI1.SPSR.BIT.PERF		= 0;	// パリティエラーフラグ

	/* 割り込み禁止処理 */
	RSPI1.SPCR2.BIT.SPIIE	= 0;	// アイドル割り込み要求の発生を禁止

	/* ポート制御 */
	PORTE.PODR.BIT.B4		= 0;	// ポートE-4をLo出力(SSL-ON)
	
	/* SPI通信許可 */
	RSPI1.SPCR.BIT.SPE		= 1;	// SPI機能有効
	RSPI1.SPCR.BIT.SPTIE	= 1;	// SPI送信割り込み要求の発生を許可
	RSPI1.SPCR.BIT.SPRIE	= 1;	// SPI受信割り込み要求の発生を許可
	RSPI1.SPCR.BIT.SPEIE	= 0;	// SPIエラー割り込み要求の発生を許可
	
	RSPI1.SPDR.WORD.H		= (USHORT)( us_adr | SPI_R );	// データセット
	
	/* ダミー処理 */
	us_dummy		= RSPI1.SPDR.WORD.H;	// dummy read
	uc_dummy		= RSPI1.SPCR.BYTE;		// dummy read
	NOUSE( us_dummy );
	NOUSE( uc_dummy );	
}


// *************************************************************************
//   機能		： RSPIアイドル遷移割り込み処理（登録したコールバック関数を実行）
//   注意		： なし
//   メモ		： RSPIがアイドルになった際の割り込み
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC void	SPI_III(void){

	/* SPI終了処理 */
	RSPI1.SPCR2.BIT.SPIIE	= 0;		// アイドル割り込み要求の発生を禁止
	PORTE.PODR.BIT.B4		= 1;		// ポートE-4をHi出力 (CS-OFF)
	RSPI1.SPCR.BIT.SPE		= 0;		// SPI機能無効
	en_SpiState				= SPI_IDLE;	// SPI状態：IDLE

	/* コールバック関数対応 */
	if( p_SpiCallBackFunc != NULL ){
		p_SpiCallBackFunc();
	}
}

// *************************************************************************
//   機能		： RSPIがアイドル状態か確認する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.6.2			TKR			新規
// *************************************************************************/
PUBLIC BOOL SPI_isIdle(void){

	if( en_SpiState == SPI_IDLE ){
		return TRUE;
	}else{
		return FALSE;
	}
}